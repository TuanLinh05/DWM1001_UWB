from types import SimpleNamespace
from uwb_px4_logger.imu_bridge import ImuSampleClock, highres_imu_sample


def message(**changes):
    fields = dict(time_usec=1_000_000, fields_updated=0x3F, xacc=1., yacc=2., zacc=-9.81,
                  xgyro=.1, ygyro=.2, zgyro=.3)
    fields.update(changes)
    return SimpleNamespace(**fields)


def test_device_intervals_survive_batched_reads_and_frd_is_preserved():
    clock = ImuSampleClock()
    first = highres_imu_sample(message(), 10_000_000_000, 5_000_000_000, clock)
    second = highres_imu_sample(message(time_usec=1_020_000), 10_001_000_000, 5_001_000_000, clock)
    assert second[0] - first[0] == 20_000_000
    assert first[1] == (1., 2., -9.81)
    assert highres_imu_sample(message(time_usec=1_020_000), 10_002_000_000, 5_002_000_000, clock) is None


def test_clock_reanchors_after_device_restart_or_link_gap():
    clock = ImuSampleClock()
    clock.stamp(5_000_000, 10_000_000_000, 5_000_000_000)
    assert clock.stamp(10_000, 10_020_000_000, 5_020_000_000) == 10_020_000_000
    assert clock.resets == 1
    assert clock.stamp(20_000, 11_000_000_000, 6_000_000_000) == 11_000_000_000
    assert clock.resets == 2


def test_invalid_or_partial_imu_is_not_published_as_fresh():
    clock = ImuSampleClock()
    assert highres_imu_sample(message(fields_updated=0), 1, 1, clock) is None
    assert highres_imu_sample(message(xgyro=float('nan')), 1, 1, clock) is None
    assert highres_imu_sample(message(time_usec=-1), 1, 1, clock) is None
