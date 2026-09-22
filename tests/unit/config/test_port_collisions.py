"""Two vehicles must not end up on the same port.

A pinned port and a derived one can collide: pinning `lead` to 8555 leaves `wing` deriving 8555 too,
because pinning one vehicle deliberately does not shift the others. Nothing checked it, so the config
loaded, the stage composed, both cameras reported enabled, and the second RTSP server simply failed to
bind. Found live as a refused connection on a stream the capability report said was running.

The README's phrasing invited the mistake: it said ports are allocated as `rtsp_port + vehicle index`,
which reads as though a pinned port is the base for the others. The real rule is per vehicle -- its own
port if set, otherwise the default base plus its index.
"""

from __future__ import annotations

import pytest

from isaac_core.config import IsaacCoreConfig

# The shipped defaults, so these numbers are the ones a real config collides with.
DEFAULT_RTSP_BASE = 8554
DEFAULT_UDP_BASE = 33333


# -- what the resolution rule actually is ---------------------------------------- #


def test_unset_ports_are_allocated_by_vehicle_index() -> None:
    config = IsaacCoreConfig(vehicles={"lead": {"camera": {}}, "wing": {"camera": {}}})
    assert config.resolved_rtsp_port("lead") == DEFAULT_RTSP_BASE
    assert config.resolved_rtsp_port("wing") == DEFAULT_RTSP_BASE + 1
    assert config.resolved_udp_port("lead") == DEFAULT_UDP_BASE
    assert config.resolved_udp_port("wing") == DEFAULT_UDP_BASE + 1


def test_pinning_one_vehicle_does_not_shift_another() -> None:
    # The behaviour the README described wrongly. Pinning lead to 8570 leaves wing on the default base
    # plus its index, not on 8571.
    config = IsaacCoreConfig(vehicles={"lead": {"camera": {"rtsp_port": 8570}}, "wing": {"camera": {}}})
    assert config.resolved_rtsp_port("lead") == 8570
    assert config.resolved_rtsp_port("wing") == DEFAULT_RTSP_BASE + 1


def test_both_pinned_are_both_honoured() -> None:
    config = IsaacCoreConfig(
        vehicles={"lead": {"camera": {"rtsp_port": 8570}}, "wing": {"camera": {"rtsp_port": 8580}}}
    )
    assert config.resolved_rtsp_port("lead") == 8570
    assert config.resolved_rtsp_port("wing") == 8580


# -- collisions are refused ------------------------------------------------------- #


def test_pinning_onto_another_vehicles_derived_rtsp_port_is_refused() -> None:
    # The exact live failure: wing derives 8555, lead is pinned to it, wing's server never binds.
    with pytest.raises(ValueError, match="RTSP port 8555 is used by both") as excinfo:
        IsaacCoreConfig(vehicles={"lead": {"camera": {"rtsp_port": 8555}}, "wing": {"camera": {}}})
    message = str(excinfo.value)
    assert "lead" in message and "wing" in message
    assert "rtsp_port" in message, "the message does not say which key to change"


def test_two_vehicles_pinned_to_the_same_rtsp_port_are_refused() -> None:
    with pytest.raises(ValueError, match="RTSP port 8600 is used by both"):
        IsaacCoreConfig(vehicles={"lead": {"camera": {"rtsp_port": 8600}}, "wing": {"camera": {"rtsp_port": 8600}}})


def test_a_udp_port_collision_is_refused() -> None:
    # Worse than RTSP: two vehicles on one pose port means one silently takes the other's poses.
    with pytest.raises(ValueError, match="UDP pose port 33334 is used by both"):
        IsaacCoreConfig(vehicles={"lead": {"camera": {}, "udp_port": 33334}, "wing": {"camera": {}}})


def test_two_vehicles_pinned_to_the_same_udp_port_are_refused() -> None:
    with pytest.raises(ValueError, match="UDP pose port 34000 is used by both"):
        IsaacCoreConfig(vehicles={"lead": {"camera": {}, "udp_port": 34000}, "wing": {"camera": {}, "udp_port": 34000}})


def test_the_message_says_what_to_do() -> None:
    with pytest.raises(ValueError) as excinfo:
        IsaacCoreConfig(vehicles={"lead": {"camera": {"rtsp_port": 8555}}, "wing": {"camera": {}}})
    message = str(excinfo.value)
    assert "free port" in message
    assert "default base" in message, "the message does not explain the alternative"


# -- valid configurations still load --------------------------------------------- #


def test_a_single_vehicle_is_never_a_collision() -> None:
    IsaacCoreConfig(vehicles={"drone_0": {"camera": {}}})


def test_three_unset_vehicles_are_fine() -> None:
    config = IsaacCoreConfig(vehicles={"a": {"camera": {}}, "b": {"camera": {}}, "c": {"camera": {}}})
    ports = {config.resolved_rtsp_port(name) for name in config.vehicles}
    assert len(ports) == 3, ports


def test_a_pinned_port_far_from_the_base_is_fine() -> None:
    IsaacCoreConfig(vehicles={"lead": {"camera": {"rtsp_port": 9100}}, "wing": {"camera": {}}})


def test_the_shipped_default_config_loads() -> None:
    # The guard must not reject the ordinary case, which is the only one most people ever use.
    from isaac_core.config import load

    load()
