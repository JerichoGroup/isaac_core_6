"""Tests for isaac_core.devkit.transport: UDP send, fake transport, pace()."""

from collections.abc import Iterator
import socket
import time

import pytest

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.transport import FakePoseTransport, PoseTransport, UdpPoseTransport, pace
from isaac_core.protocol import decode
from isaac_core.vehicle import OrbitTrajectory

# -- helpers ---------------------------------------------------------------- #


def _sample_pose() -> GeodeticPose:
    return GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=500.0),
        orientation=Rpy(roll_r=0.1, pitch_r=0.2, yaw_r=0.3, frame=Frame.NED),
    )


def _n_poses(n: int) -> Iterator[GeodeticPose]:
    pose = _sample_pose()
    for _ in range(n):
        yield pose


# -- PoseTransport protocol ------------------------------------------------- #


def test_udp_transport_satisfies_protocol() -> None:
    assert isinstance(UdpPoseTransport(), PoseTransport)


def test_fake_transport_satisfies_protocol() -> None:
    assert isinstance(FakePoseTransport(), PoseTransport)


# -- UdpPoseTransport ------------------------------------------------------- #


def test_udp_transport_sends_decodable_packet() -> None:
    # Bind an ephemeral UDP port on loopback.
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    port = receiver.getsockname()[1]
    receiver.settimeout(2.0)

    transport = UdpPoseTransport(host="127.0.0.1", port=port)
    try:
        pose = _sample_pose()
        transport.send(pose)

        data, _addr = receiver.recvfrom(1024)
        decoded = decode(data)

        assert decoded.position.lat_deg == pytest.approx(pose.position.lat_deg)
        assert decoded.position.lon_deg == pytest.approx(pose.position.lon_deg)
        assert decoded.position.alt_m == pytest.approx(pose.position.alt_m)
        assert decoded.orientation.roll_r == pytest.approx(pose.orientation.roll_r)
        assert decoded.orientation.pitch_r == pytest.approx(pose.orientation.pitch_r)
        assert decoded.orientation.yaw_r == pytest.approx(pose.orientation.yaw_r)
        assert decoded.orientation.frame is Frame.NED
    finally:
        transport.close()
        receiver.close()


def test_udp_transport_close_is_idempotent() -> None:
    transport = UdpPoseTransport()
    transport.close()
    transport.close()  # should not raise


def test_udp_transport_rejects_enu_pose() -> None:
    transport = UdpPoseTransport(host="127.0.0.1", port=33333)
    enu_pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.ENU),
    )
    with pytest.raises(ValueError, match="NED"):
        transport.send(enu_pose)
    transport.close()


def test_udp_transport_sends_multiple_packets() -> None:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    port = receiver.getsockname()[1]
    receiver.settimeout(2.0)

    transport = UdpPoseTransport(host="127.0.0.1", port=port)
    try:
        for _ in range(5):
            transport.send(_sample_pose())

        for _ in range(5):
            data, _ = receiver.recvfrom(1024)
            assert len(data) == 51
    finally:
        transport.close()
        receiver.close()


# -- FakePoseTransport ------------------------------------------------------ #


def test_fake_transport_records_poses() -> None:
    transport = FakePoseTransport()
    poses = [_sample_pose(), _sample_pose()]
    for p in poses:
        transport.send(p)
    assert len(transport.history) == 2
    assert transport.history[0] is poses[0]


def test_fake_transport_raises_after_close() -> None:
    transport = FakePoseTransport()
    transport.close()
    assert transport.closed
    with pytest.raises(RuntimeError, match="closed"):
        transport.send(_sample_pose())


def test_fake_transport_captures_orbit_trajectory() -> None:
    # Drive a full orbit trajectory through the fake transport.
    orbit = OrbitTrajectory(
        center_lat_deg=32.0,
        center_lon_deg=35.0,
        radius_m=100.0,
        height_m=500.0,
        speed_mps=10.0,
        orbit_duration_s=1.0,
    )
    transport = FakePoseTransport()
    rate_hz = 10.0
    expected_count = int(orbit.duration_s * rate_hz)

    for pose in orbit.poses(rate_hz):
        transport.send(pose)

    assert len(transport.history) == expected_count
    # All poses should be valid (NED frame, valid LLA).
    for pose in transport.history:
        assert pose.orientation.frame is Frame.NED
        assert -90.0 <= pose.position.lat_deg <= 90.0


# -- pace() ----------------------------------------------------------------- #


def test_pace_sends_all_poses() -> None:
    transport = FakePoseTransport()
    count = pace(_n_poses(5), transport, rate_hz=1000.0)
    assert count == 5
    assert len(transport.history) == 5


def test_pace_rejects_non_positive_rate() -> None:
    transport = FakePoseTransport()
    with pytest.raises(ValueError, match="rate_hz"):
        pace(iter([]), transport, rate_hz=0.0)
    with pytest.raises(ValueError, match="rate_hz"):
        pace(iter([]), transport, rate_hz=-1.0)


def test_pace_does_not_drift_over_many_iterations() -> None:
    # Verify that accumulated timing error stays bounded.
    # Use a high rate (1000 Hz) for 100 iterations so the test finishes fast.
    n_poses = 100
    rate_hz = 1000.0
    expected_duration = n_poses / rate_hz  # 0.1 seconds

    transport = FakePoseTransport()
    start = time.perf_counter()
    pace(_n_poses(n_poses), transport, rate_hz=rate_hz)
    elapsed = time.perf_counter() - start

    # Allow 50ms tolerance (generous for OS scheduling jitter).
    assert elapsed >= expected_duration * 0.8
    assert elapsed < expected_duration + 0.05
    assert len(transport.history) == n_poses


def test_pace_handles_empty_iterator() -> None:
    transport = FakePoseTransport()
    count = pace(iter([]), transport, rate_hz=100.0)
    assert count == 0
    assert len(transport.history) == 0


def test_pace_with_slow_rate() -> None:
    # 2 poses at 20 Hz = 0.1s total. Verify timing is close.
    transport = FakePoseTransport()
    start = time.perf_counter()
    count = pace(_n_poses(2), transport, rate_hz=20.0)
    elapsed = time.perf_counter() - start

    assert count == 2
    # Should take about 0.05s (first pose immediate, second after 0.05s).
    assert elapsed >= 0.03
    assert elapsed < 0.15
