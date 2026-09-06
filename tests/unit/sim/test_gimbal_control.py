"""Tests for the gimbal control path: config start angles and commanded slewing."""

from pathlib import Path

from isaac_core.config import load
from isaac_core.sim.manifest import load_manifest

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
