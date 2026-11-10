"""E2.2-C contract: PX4 attitude for the onboard ESKF and faster IMU / flow streams.

Plan/E2_2_YEU_CAU_ESKF_ONBOARD_CHONG_RUNG_BANG_PHAN_MEM.md, section 5. Pure functions only (no ROS).
"""
import math
from pathlib import Path
import re
from types import SimpleNamespace

from uwb_px4_logger import imu_bridge as bridge

PACKAGE = Path(__file__).resolve().parents[1]


def imu(**changes):
    fields = dict(time_usec=1_000_000, fields_updated=0x3F, xacc=0., yacc=0., zacc=-9.81, xgyro=0., ygyro=0., zgyro=0.)
    fields.update(changes)
    return SimpleNamespace(**fields)


def odometry(**changes):
    fields = dict(time_usec=1_010_000, q=[1.0, 0.0, 0.0, 0.0])
    fields.update(changes)
    return SimpleNamespace(**fields)


def test_clock_maps_other_px4_times_without_changing_its_state():
    clock = bridge.ImuSampleClock()
    assert clock.to_stamp(1_000_000) is None                       # no IMU yet: no offset to use
    stamp, _, _ = bridge.highres_imu_sample(imu(), 10_000_000_000, 5_000_000_000, clock)
    before = (clock.offset_ns, clock.last_us, clock.last_mono_ns, clock.resets)
    assert clock.to_stamp(1_010_000) == stamp + 10_000_000          # same PX4 boot clock as HIGHRES_IMU
    assert clock.to_stamp(900_000) == stamp - 100_000_000           # older samples map too (no reorder check)
    assert (clock.offset_ns, clock.last_us, clock.last_mono_ns, clock.resets) == before
    assert clock.to_stamp(-1) is None
    assert clock.to_stamp(1.5) is None


def test_odometry_quaternion_is_stamped_with_device_time_and_normalized():
    clock = bridge.ImuSampleClock()
    assert bridge.odometry_attitude(odometry(), clock) is None              # before the first IMU sample
    stamp, _, _ = bridge.highres_imu_sample(imu(), 10_000_000_000, 5_000_000_000, clock)
    half = math.sqrt(0.5)
    result = bridge.odometry_attitude(odometry(q=[1.5 * half, 0.0, 0.0, 1.5 * half]), clock)   # 90 deg yaw, scaled by 1.5
    assert result is not None
    attitude_stamp, q = result
    assert attitude_stamp == stamp + 10_000_000
    assert all(abs(a - b) < 1e-12 for a, b in zip(q, (half, 0.0, 0.0, half)))     # MAVLink order w, x, y, z kept


def test_invalid_attitude_is_not_published():
    clock = bridge.ImuSampleClock()
    bridge.highres_imu_sample(imu(), 10_000_000_000, 5_000_000_000, clock)
    for q in ([0.0, 0.0, 0.0, 0.0], [float('nan'), 0.0, 0.0, 1.0], [1e-3, 0.0, 0.0, 0.0], [3.0, 0.0, 0.0, 0.0]):
        assert bridge.odometry_attitude(odometry(q=q), clock) is None
    assert bridge.odometry_attitude(odometry(time_usec=-5), clock) is None


def _rates(text):
    return dict(item.split(':') for item in re.findall(r'"([A-Z_]+:[0-9.]+)"', text))


def test_streams_needed_by_the_onboard_estimator_are_requested():
    for path in (PACKAGE / 'uwb_px4_logger' / 'px4_link.py', PACKAGE / 'config' / 'logger.yaml'):
        rates = _rates(path.read_text(encoding='utf-8'))
        assert float(rates['HIGHRES_IMU']) >= 200, path              # averaged to 50 Hz inside the estimator
        assert float(rates['ODOMETRY']) >= 50, path                  # attitude source
        assert float(rates['OPTICAL_FLOW_RAD']) >= 50, path          # recorded for later study (tlog), not fused
        assert float(rates['DISTANCE_SENSOR']) >= 50, path
        for name in ('ATTITUDE_QUATERNION', 'LOCAL_POSITION_NED', 'ESTIMATOR_STATUS', 'VIBRATION', 'EXTENDED_SYS_STATE'):
            assert name in rates, (path, name)                       # existing streams are kept
