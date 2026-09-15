"""Tests for the gimbal control path: config start angles, commanded slewing and vehicle scope."""

from pathlib import Path

import pytest

from isaac_core.config import IsaacCoreConfig, load
from isaac_core.contracts.gimbal import GimbalAngles
from isaac_core.control import InvalidParamsError
from isaac_core.sim.manifest import load_manifest
from isaac_core.sim.runtime import SimulationRuntime

LAYERS = Path(__file__).resolve().parents[3] / "src" / "isaac_core" / "assets" / "layers"


def test_both_camera_layers_bind_the_gimbal_start_angles() -> None:
    # These config keys were dead for the whole of v1: the offset inputs were connected to a ROS
    # subscriber, and a connected USD attribute ignores authored values. With the subscriber gone
    # the bindings are what finally makes them take effect, so their absence is a silent regression.
    wanted = {"inputs:offset_roll_deg", "inputs:offset_pitch_deg", "inputs:offset_yaw_deg"}
    for layer_id in ("camera_udp", "camera_ros"):
        manifest = load_manifest(LAYERS / layer_id / "layer.toml")
        bound = {b.attribute: b for b in manifest.bindings}
        assert wanted <= set(bound), f"{layer_id} is missing {wanted - set(bound)}"
        assert bound["inputs:offset_roll_deg"].config == "vehicles.{instance}.gimbal.start_roll_deg"


def test_gimbal_config_has_no_topic_field() -> None:
    # Commanding the gimbal is control-plane work (D20) and the ROS subscriber is gone, so a topic
    # setting would configure nothing.
    gimbal = load().vehicles["drone_0"].gimbal
    assert not hasattr(gimbal, "topic")


def test_set_gimbal_is_a_registered_control_method() -> None:
    """Ensure the method exists on the enum and the devkit exposes it."""
    from isaac_core.control.messages import Method
    from isaac_core.devkit.session import SimSession

    assert Method.SET_GIMBAL.value == "set_gimbal"
    assert callable(SimSession.set_gimbal)


def test_set_gimbal_refuses_a_swarm_instead_of_silently_aiming_the_first_vehicle() -> None:
    # Guards a real bug: set_gimbal resolved its vehicle as next(iter(vehicles)), so in a swarm a
    # script asking for wing's gimbal moved lead's and got a success response. Refusing is honest;
    # per-vehicle gimbal state is roadmapped.
    config = IsaacCoreConfig(
        vehicles={"lead": {"cameras": {"eo": {}}}, "wing": {"cameras": {"eo": {}}}},
    )
    runtime = SimulationRuntime.__new__(SimulationRuntime)
    runtime._config = config
    with pytest.raises(InvalidParamsError) as excinfo:
        runtime._require_single_vehicle("set_gimbal")
    message = str(excinfo.value)
    assert "set_gimbal" in message
    assert "lead" in message and "wing" in message


def test_require_single_vehicle_returns_the_only_vehicle() -> None:
    config = IsaacCoreConfig(vehicles={"drone_0": {"cameras": {"eo": {}}}})
    runtime = SimulationRuntime.__new__(SimulationRuntime)
    runtime._config = config
    assert runtime._require_single_vehicle("capture_frame") == "drone_0"


def _bare_runtime(config: IsaacCoreConfig) -> SimulationRuntime:
    """Return a runtime with only the state the gimbal handler touches."""
    runtime = SimulationRuntime.__new__(SimulationRuntime)
    runtime._config = config
    runtime._gimbal_target = None
    runtime._gimbal_current = None
    return runtime


def _single_vehicle(**gimbal: float) -> IsaacCoreConfig:
    """Return a one-vehicle config with the given gimbal start angles."""
    return IsaacCoreConfig(vehicles={"drone_0": {"cameras": {"eo": {}}, "gimbal": gimbal}})


def test_an_omitted_axis_falls_back_to_the_configured_start_angle() -> None:
    # Before anything has moved, the omitted axes must come from config, not zero.
    runtime = _bare_runtime(_single_vehicle(start_roll_deg=5.0, start_pitch_deg=-15.0, start_yaw_deg=20.0))
    result = runtime._handle_set_gimbal({"pitch_deg": -40.0})
    assert result["pitch_deg"] == pytest.approx(-40.0)
    assert result["roll_deg"] == pytest.approx(5.0)
    assert result["yaw_deg"] == pytest.approx(20.0)


def test_an_omitted_axis_holds_where_the_gimbal_actually_is() -> None:
    # Once the gimbal has moved, an omitted axis must hold its CURRENT angle rather than snapping
    # back to the config start -- otherwise commanding pitch would silently undo a previous yaw.
    runtime = _bare_runtime(_single_vehicle(start_roll_deg=0.0, start_pitch_deg=0.0, start_yaw_deg=0.0))
    runtime._gimbal_current = GimbalAngles.from_degrees(3.0, -25.0, 90.0)
    result = runtime._handle_set_gimbal({"pitch_deg": -10.0})
    assert result["pitch_deg"] == pytest.approx(-10.0)
    assert result["roll_deg"] == pytest.approx(3.0)
    assert result["yaw_deg"] == pytest.approx(90.0)


def test_commanding_every_axis_ignores_both_start_and_current() -> None:
    runtime = _bare_runtime(_single_vehicle(start_roll_deg=5.0, start_pitch_deg=5.0, start_yaw_deg=5.0))
    runtime._gimbal_current = GimbalAngles.from_degrees(1.0, 2.0, 3.0)
    result = runtime._handle_set_gimbal({"roll_deg": -1.0, "pitch_deg": -2.0, "yaw_deg": -3.0})
    assert (result["roll_deg"], result["pitch_deg"], result["yaw_deg"]) == pytest.approx((-1.0, -2.0, -3.0))


def test_the_target_is_recorded_for_the_loop_to_act_on() -> None:
    # The handler must not touch USD: it records intent and the simulation loop moves the gimbal.
    runtime = _bare_runtime(_single_vehicle())
    assert runtime._gimbal_target is None
    runtime._handle_set_gimbal({"yaw_deg": 45.0})
    assert runtime._gimbal_target is not None
    assert runtime._gimbal_target.yaw_deg == pytest.approx(45.0)


def test_a_non_numeric_angle_is_rejected() -> None:
    runtime = _bare_runtime(_single_vehicle())
    with pytest.raises(InvalidParamsError):
        runtime._handle_set_gimbal({"pitch_deg": "down"})
