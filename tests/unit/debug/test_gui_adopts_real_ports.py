"""A GUI tab must send to the port the simulator is really listening on.

A tab's port came from `33333 + tab index`, which matches only the *default* allocation. A config that pins
`udp_port` left the GUI sending where nobody listens, while the readback showed the simulator's held pose --
so the sender looked fine, the camera looked frozen, and the readback line that exists to say which half is
wrong could not.

The resolution rule is asked of the simulator rather than repeated in the GUI, because a second copy of it
is exactly what drifts.
"""

from __future__ import annotations

from typing import Any

import pytest

from isaac_core.debug.pose_sender_gui import PoseSenderController, _adopt_simulator_vehicle

# The shipped defaults, so these are the numbers a real config collides with.
DEFAULT_UDP_BASE = 33333


def _controller(port: int = DEFAULT_UDP_BASE) -> PoseSenderController:
    """Return a controller on a given port, with no transport opened."""
    controller = PoseSenderController()
    controller.port = port
    return controller


def _with_config(monkeypatch: pytest.MonkeyPatch, config: dict[str, Any] | None) -> None:
    """Make the control-plane call return a given resolved config.

    Args:
        monkeypatch: Pytest's patcher.
        config: What `get_config` should return, or ``None`` to simulate no simulator.

    """

    class FakeClient:
        def __init__(self, **kwargs: Any) -> None:
            del kwargs

        def connect(self, timeout: float | None = None) -> None:
            del timeout
            if config is None:
                raise ConnectionRefusedError("nothing listening")

        def call(self, method: str, params: Any = None) -> Any:
            del method, params
            return config

        def close(self) -> None:
            return None

    monkeypatch.setattr("isaac_core.control.client.ControlClient", FakeClient)


def _config_with(vehicles: dict[str, Any]) -> dict[str, Any]:
    """Return a resolved config dict carrying the given vehicles.

    Args:
        vehicles: The vehicles section.

    Returns:
        A full resolved config as `get_config` returns it.

    """
    from isaac_core.config import IsaacCoreConfig

    dumped: dict[str, Any] = IsaacCoreConfig(vehicles=vehicles).model_dump(mode="json")
    return dumped


def test_ports_are_read_from_the_simulator(monkeypatch: pytest.MonkeyPatch) -> None:
    _with_config(monkeypatch, _config_with({"lead": {"camera": {}}, "wing": {"camera": {}}}))
    ports = _controller().readback_vehicle_ports()
    assert ports == {"lead": DEFAULT_UDP_BASE, "wing": DEFAULT_UDP_BASE + 1}


def test_a_pinned_port_is_reported_not_the_arithmetic(monkeypatch: pytest.MonkeyPatch) -> None:
    # The case the arithmetic gets wrong.
    _with_config(monkeypatch, _config_with({"drone_0": {"camera": {}, "udp_port": 34500}}))
    assert _controller().readback_vehicle_ports() == {"drone_0": 34500}


def test_no_simulator_means_no_ports_not_a_crash(monkeypatch: pytest.MonkeyPatch) -> None:
    # The GUI has to open with nothing running, which is the normal way people start it.
    _with_config(monkeypatch, None)
    assert _controller().readback_vehicle_ports() == {}


def test_an_uninterpretable_config_means_no_ports(monkeypatch: pytest.MonkeyPatch) -> None:
    _with_config(monkeypatch, {"not": "a config"})
    assert _controller().readback_vehicle_ports() == {}


def test_adopting_a_vehicle_moves_the_tab_to_its_real_port(monkeypatch: pytest.MonkeyPatch) -> None:
    # The whole point: the tab must end up on 34500, not on the default it was created with.
    _with_config(monkeypatch, _config_with({"drone_0": {"camera": {}, "udp_port": 34500}}))
    controller = _controller(port=DEFAULT_UDP_BASE)
    _adopt_simulator_vehicle(controller, taken=set())
    assert controller.vehicle == "drone_0"
    assert controller.port == 34500, "the tab kept the arithmetic port and would send nowhere"


def test_adopting_leaves_a_default_port_alone(monkeypatch: pytest.MonkeyPatch) -> None:
    _with_config(monkeypatch, _config_with({"drone_0": {"camera": {}}}))
    controller = _controller(port=DEFAULT_UDP_BASE)
    _adopt_simulator_vehicle(controller, taken=set())
    assert controller.port == DEFAULT_UDP_BASE


def test_two_tabs_adopt_two_vehicles_and_two_ports(monkeypatch: pytest.MonkeyPatch) -> None:
    _with_config(
        monkeypatch,
        _config_with({"lead": {"camera": {}, "udp_port": 34500}, "wing": {"camera": {}, "udp_port": 34600}}),
    )
    taken: set[str] = set()
    first, second = _controller(), _controller()
    _adopt_simulator_vehicle(first, taken)
    _adopt_simulator_vehicle(second, taken)
    assert {first.vehicle, second.vehicle} == {"lead", "wing"}
    assert {first.port, second.port} == {34500, 34600}


def test_adopting_with_no_simulator_keeps_the_tab_usable(monkeypatch: pytest.MonkeyPatch) -> None:
    # Nothing listening: the tab keeps its own port so a user can still fly by hand.
    _with_config(monkeypatch, None)
    controller = _controller(port=DEFAULT_UDP_BASE)
    _adopt_simulator_vehicle(controller, taken=set())
    assert controller.port == DEFAULT_UDP_BASE
