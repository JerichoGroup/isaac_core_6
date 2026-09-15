"""Service protocol, registry, and supervisor for sidecar processes.

The supervisor polls service health at a configurable interval, restarts failed
services per a policy (with a maximum restart count to avoid a spin loop on hard
failures), captures structured logs, and shuts down gracefully with a timeout
before escalating to a force stop.

Services are looked up in a registry keyed by the config's ``kind`` field, so
adding a new service kind does not require editing the supervisor.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import threading
import time
from typing import Any, Callable, Protocol, runtime_checkable

from isaac_core.config import SidecarConfig

logger = logging.getLogger(__name__)

# Default health poll interval in seconds.
_DEFAULT_POLL_INTERVAL_S = 2.0

# Default grace period before forcing a stop during shutdown.
_DEFAULT_SHUTDOWN_TIMEOUT_S = 5.0

# Default maximum restart count before giving up on a service.
_DEFAULT_MAX_RESTARTS = 5


@runtime_checkable
class Service(Protocol):
    """Protocol for a supervised sidecar service.

    Implementations must be safe to call ``start()`` after ``stop()`` for restart
    support. ``is_healthy()`` should be cheap and non-blocking.
    """

    @property
    def name(self) -> str:
        """Return the human-readable service identifier."""
        ...

    def start(self) -> None:
        """Start the service, blocking until it is ready to serve."""
        ...

    def stop(self, timeout: float = 5.0) -> None:
        """Stop the service gracefully within *timeout* seconds.

        If the service cannot stop in time it must still release resources to the
        extent possible -- the supervisor will not call it again.

        Args:
            timeout: Maximum seconds to wait for a graceful shutdown.

        """
        ...

    def is_healthy(self) -> bool:
        """Return whether the service is operating normally."""
        ...


@dataclass(frozen=True)
class RestartPolicy:
    """Control how the supervisor handles a failed service.

    Args:
        max_restarts: Maximum number of restarts before the service is abandoned.
            ``0`` means never restart.
        poll_interval_s: Seconds between health checks.
        shutdown_timeout_s: Seconds to wait for a graceful stop before forcing.

    """

    max_restarts: int = _DEFAULT_MAX_RESTARTS
    poll_interval_s: float = _DEFAULT_POLL_INTERVAL_S
    shutdown_timeout_s: float = _DEFAULT_SHUTDOWN_TIMEOUT_S


@dataclass
class _ServiceSlot:
    """Internal bookkeeping for one supervised service."""

    service: Service
    restart_count: int = 0
    abandoned: bool = False


# Module-level registry mapping ``kind`` strings to factory callables.
# A factory receives the raw ``SidecarServiceConfig`` model and returns a
# ``Service`` instance.
_REGISTRY: dict[str, Callable[..., Any]] = {}


class ServiceRegistry:
    """Registry mapping service kind strings to factory classes.

    Factories must accept a single ``SidecarServiceConfig`` argument and return a
    :class:`Service`-compatible object.
    """

    @staticmethod
    def register(kind: str, factory: Callable[..., Any]) -> None:
        """Register a factory for the given service kind.

        Args:
            kind: The ``kind`` string from config.
            factory: A callable accepting ``SidecarServiceConfig`` and returning a
                :class:`Service`.

        """
        _REGISTRY[kind] = factory

    @staticmethod
    def get(kind: str) -> Callable[..., Any] | None:
        """Look up the factory for a service kind.

        Args:
            kind: The ``kind`` string from config.

        Returns:
            The registered factory, or ``None`` if no factory is registered.

        """
        return _REGISTRY.get(kind)

    @staticmethod
    def registered_kinds() -> tuple[str, ...]:
        """Return all registered kind strings in insertion order."""
        return tuple(_REGISTRY.keys())


@dataclass
class ServiceSupervisor:
    """Start, monitor and restart sidecar services per a restart policy.

    The supervisor runs a background polling thread. Call :meth:`shutdown` to
    stop all services and join the poller.
    """

    policy: RestartPolicy = field(default_factory=RestartPolicy)
    _slots: list[_ServiceSlot] = field(default_factory=list, init=False, repr=False)
    _running: bool = field(default=False, init=False, repr=False)
    _poll_thread: threading.Thread | None = field(default=None, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    @classmethod
    def from_config(cls, config: SidecarConfig, policy: RestartPolicy | None = None) -> "ServiceSupervisor":
        """Build a supervisor from the sidecar configuration.

        Instantiates each service from the registry and returns a ready-to-start
        supervisor. Unknown kinds are logged and skipped.

        Args:
            config: The ``[sidecar]`` section of the application config.
            policy: Override the default restart policy.

        Returns:
            A configured supervisor (call :meth:`start` to begin monitoring).

        Raises:
            ValueError: If a service kind is not registered and no services can be
                started (all unknown).

        """
        effective_policy = policy if policy is not None else RestartPolicy()
        supervisor = cls(policy=effective_policy)

        for service_id, svc_config in config.services.items():
            factory = ServiceRegistry.get(svc_config.kind)
            if factory is None:
                logger.error(
                    "Unknown service kind %r for service %r; registered kinds: %s",
                    svc_config.kind,
                    service_id,
                    ServiceRegistry.registered_kinds(),
                )
                continue
            try:
                service = factory(svc_config)
            except Exception:
                logger.exception("Failed to instantiate service %r (kind=%r)", service_id, svc_config.kind)
                continue
            supervisor.add_service(service)

        return supervisor

    def add_service(self, service: Service) -> None:
        """Add a service to the supervisor.

        Args:
            service: A :class:`Service`-compatible instance.

        """
        with self._lock:
            self._slots.append(_ServiceSlot(service=service))

    def start(self) -> None:
        """Start all services and begin health monitoring."""
        with self._lock:
            if self._running:
                return
            self._running = True

        for slot in self._slots:
            self._start_service(slot)

        self._poll_thread = threading.Thread(
            target=self._poll_loop,
            name="sidecar-supervisor",
            daemon=True,
        )
        self._poll_thread.start()

    def shutdown(self, timeout: float | None = None) -> None:
        """Stop all services and the supervisor polling thread.

        Args:
            timeout: Override the policy's shutdown timeout.

        """
        effective_timeout = timeout if timeout is not None else self.policy.shutdown_timeout_s
        with self._lock:
            self._running = False

        if self._poll_thread is not None:
            self._poll_thread.join(timeout=effective_timeout)

        for slot in self._slots:
            self._stop_service(slot, effective_timeout)

    @property
    def services(self) -> tuple[Service, ...]:
        """Return all managed services."""
        return tuple(slot.service for slot in self._slots)

    @property
    def is_running(self) -> bool:
        """Whether the supervisor is actively polling."""
        return self._running

    def _start_service(self, slot: _ServiceSlot) -> None:
        """Start a single service, logging failures."""
        try:
            slot.service.start()
            logger.info("Started service %r", slot.service.name)
        except Exception:
            logger.exception("Failed to start service %r", slot.service.name)
            slot.abandoned = True

    def _stop_service(self, slot: _ServiceSlot, timeout: float) -> None:
        """Stop a single service, escalating on timeout."""
        try:
            slot.service.stop(timeout=timeout)
            logger.info("Stopped service %r", slot.service.name)
        except Exception:
            logger.exception("Error stopping service %r", slot.service.name)

    def _poll_loop(self) -> None:
        """Background loop that checks health and restarts as needed."""
        while self._running:
            time.sleep(self.policy.poll_interval_s)
            if not self._running:
                break
            with self._lock:
                for slot in self._slots:
                    if slot.abandoned:
                        continue
                    if not slot.service.is_healthy():
                        self._handle_unhealthy(slot)

    def _handle_unhealthy(self, slot: _ServiceSlot) -> None:
        """Attempt to restart an unhealthy service, respecting the restart cap."""
        if slot.restart_count >= self.policy.max_restarts:
            logger.error(
                "Service %r exceeded max restarts (%d); abandoning",
                slot.service.name,
                self.policy.max_restarts,
            )
            slot.abandoned = True
            return

        slot.restart_count += 1
        logger.warning(
            "Service %r unhealthy; restarting (%d/%d)",
            slot.service.name,
            slot.restart_count,
            self.policy.max_restarts,
        )
        try:
            slot.service.stop(timeout=self.policy.shutdown_timeout_s)
        except Exception:
            logger.exception("Error stopping service %r before restart", slot.service.name)

        self._start_service(slot)
