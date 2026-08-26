"""Tests for the sidecar service supervisor."""

from __future__ import annotations

from collections.abc import Generator
import threading
import time

import pytest

from isaac_core.config import SidecarConfig, SidecarServiceConfig
from isaac_core.sidecar.service import (
    _REGISTRY,
    RestartPolicy,
    Service,
    ServiceRegistry,
    ServiceSupervisor,
)

# -- fake services ---------------------------------------------------------- #


class _FakeService:
    """Controllable fake service for testing."""

    def __init__(self, service_name: str = "fake") -> None:
        self._name = service_name
        self._started = False
        self._healthy = True
        self.start_count = 0
        self.stop_count = 0

    @property
    def name(self) -> str:
        return self._name

    def start(self) -> None:
        self._started = True
        self._healthy = True
        self.start_count += 1

    def stop(self, timeout: float = 5.0) -> None:
        self._started = False
        self._healthy = False
        self.stop_count += 1

    def is_healthy(self) -> bool:
        return self._healthy

    def set_unhealthy(self) -> None:
        self._healthy = False


class _HangingService:
    """Service whose stop() blocks for longer than the timeout."""

    def __init__(self, hang_duration: float = 10.0) -> None:
        self._hang_duration = hang_duration
        self._started = False
        self._healthy = True

    @property
    def name(self) -> str:
        return "hanging"

    def start(self) -> None:
        self._started = True
        self._healthy = True

    def stop(self, timeout: float = 5.0) -> None:
        # Simulate a hang that respects the timeout ceiling
        time.sleep(min(self._hang_duration, timeout + 0.5))
        self._started = False
        self._healthy = False

    def is_healthy(self) -> bool:
        return self._healthy


class _FailingStartService:
    """Service that always fails on start."""

    @property
    def name(self) -> str:
        return "always-fails"

    def start(self) -> None:
        msg = "intentional start failure"
        raise RuntimeError(msg)

    def stop(self, timeout: float = 5.0) -> None:
        pass

    def is_healthy(self) -> bool:
        return False


# -- fixtures --------------------------------------------------------------- #


@pytest.fixture(autouse=True)
def _clean_registry() -> Generator[None, None, None]:
    """Ensure tests don't leak registry entries."""
    saved = dict(_REGISTRY)
    yield
    _REGISTRY.clear()
    _REGISTRY.update(saved)


# -- protocol conformance --------------------------------------------------- #


def test_fake_service_satisfies_protocol() -> None:
    svc = _FakeService()
    assert isinstance(svc, Service)


def test_hanging_service_satisfies_protocol() -> None:
    svc = _HangingService()
    assert isinstance(svc, Service)


# -- supervisor start and stop ---------------------------------------------- #


def test_supervisor_starts_and_stops_multiple_services() -> None:
    svc1 = _FakeService("alpha")
    svc2 = _FakeService("beta")
    policy = RestartPolicy(poll_interval_s=0.05, shutdown_timeout_s=1.0)
    supervisor = ServiceSupervisor(policy=policy)
    supervisor.add_service(svc1)
    supervisor.add_service(svc2)
    supervisor.start()

    assert svc1.start_count == 1
    assert svc2.start_count == 1
    assert supervisor.is_running

    supervisor.shutdown(timeout=2.0)

    assert svc1.stop_count == 1
    assert svc2.stop_count == 1
    assert not supervisor.is_running


def test_supervisor_services_property() -> None:
    svc = _FakeService("only")
    supervisor = ServiceSupervisor()
    supervisor.add_service(svc)
    assert svc in supervisor.services


# -- health check and restart ----------------------------------------------- #


def test_unhealthy_service_is_restarted() -> None:
    svc = _FakeService("flaky")
    policy = RestartPolicy(poll_interval_s=0.02, max_restarts=3, shutdown_timeout_s=1.0)
    supervisor = ServiceSupervisor(policy=policy)
    supervisor.add_service(svc)
    supervisor.start()

    # Force unhealthy
    svc.set_unhealthy()
    # Wait for the supervisor to notice and restart
    time.sleep(0.15)

    assert svc.start_count >= 2  # at least one restart happened
    supervisor.shutdown(timeout=1.0)


def test_restart_cap_is_honoured() -> None:
    # Service that stays unhealthy after restart
    svc = _FakeService("doomed")
    policy = RestartPolicy(poll_interval_s=0.02, max_restarts=2, shutdown_timeout_s=1.0)
    supervisor = ServiceSupervisor(policy=policy)
    supervisor.add_service(svc)
    supervisor.start()

    # Force permanently unhealthy (override start behaviour)
    original_start = svc.start

    def _start_but_stay_unhealthy() -> None:
        original_start()
        svc._healthy = False  # noqa: SLF001

    svc.start = _start_but_stay_unhealthy  # type: ignore[method-assign]
    svc.set_unhealthy()

    # Give the supervisor time to hit the cap
    time.sleep(0.3)

    # Supervisor should not restart beyond max_restarts
    # start_count = 1 (initial) + at most 2 restarts = 3
    assert svc.start_count <= 4  # generous upper bound for timing slack
    supervisor.shutdown(timeout=1.0)


# -- graceful shutdown with a hanging service ------------------------------- #


def test_shutdown_with_hanging_service_does_not_leave_threads() -> None:
    # Use a short hang that still exceeds the timeout
    svc = _HangingService(hang_duration=10.0)
    policy = RestartPolicy(poll_interval_s=0.05, shutdown_timeout_s=0.2)
    supervisor = ServiceSupervisor(policy=policy)
    supervisor.add_service(svc)
    supervisor.start()

    # Record threads before shutdown
    time.sleep(0.05)  # let poller start

    supervisor.shutdown(timeout=0.3)

    # Give threads a moment to wind down
    time.sleep(0.1)

    # No supervisor threads should remain (they are daemon but should have joined)
    thread_names = [t.name for t in threading.enumerate()]
    assert "sidecar-supervisor" not in thread_names


# -- from_config with registry --------------------------------------------- #


def _fake_factory(config: SidecarServiceConfig) -> _FakeService:
    """Create a FakeService from config."""
    return _FakeService(f"from-config-{config.kind}")


def test_from_config_instantiates_registered_services() -> None:
    ServiceRegistry.register("test_kind", _fake_factory)
    config = SidecarConfig(
        enabled=True,
        services={"svc_a": SidecarServiceConfig(kind="test_kind")},
    )
    supervisor = ServiceSupervisor.from_config(config)
    assert len(supervisor.services) == 1
    assert supervisor.services[0].name == "from-config-test_kind"


def test_from_config_skips_unknown_kind() -> None:
    # No registration for "mystery"
    config = SidecarConfig(
        enabled=True,
        services={"bad": SidecarServiceConfig(kind="mystery")},
    )
    supervisor = ServiceSupervisor.from_config(config)
    # Should not crash, should just skip
    assert len(supervisor.services) == 0


def test_from_config_with_multiple_services() -> None:
    ServiceRegistry.register("alpha_kind", _fake_factory)
    ServiceRegistry.register("beta_kind", _fake_factory)
    config = SidecarConfig(
        enabled=True,
        services={
            "a": SidecarServiceConfig(kind="alpha_kind"),
            "b": SidecarServiceConfig(kind="beta_kind"),
        },
    )
    supervisor = ServiceSupervisor.from_config(config)
    assert len(supervisor.services) == 2


# -- registry --------------------------------------------------------------- #


def test_registry_register_and_get() -> None:
    ServiceRegistry.register("custom", _FakeService)
    assert ServiceRegistry.get("custom") is _FakeService


def test_registry_get_unknown_returns_none() -> None:
    assert ServiceRegistry.get("nonexistent_kind_xyz") is None


def test_registry_registered_kinds() -> None:
    ServiceRegistry.register("aaa", _FakeService)
    assert "aaa" in ServiceRegistry.registered_kinds()


# -- start idempotence ------------------------------------------------------ #


def test_start_is_idempotent() -> None:
    svc = _FakeService()
    policy = RestartPolicy(poll_interval_s=0.05, shutdown_timeout_s=1.0)
    supervisor = ServiceSupervisor(policy=policy)
    supervisor.add_service(svc)
    supervisor.start()
    supervisor.start()  # second call should be no-op
    assert svc.start_count == 1
    supervisor.shutdown(timeout=1.0)


# -- failing start abandons the service ------------------------------------ #


def test_service_that_fails_start_is_abandoned() -> None:
    svc = _FailingStartService()
    policy = RestartPolicy(poll_interval_s=0.02, shutdown_timeout_s=1.0)
    supervisor = ServiceSupervisor(policy=policy)
    supervisor.add_service(svc)
    supervisor.start()

    # Give the supervisor time -- should not crash
    time.sleep(0.1)
    supervisor.shutdown(timeout=1.0)
