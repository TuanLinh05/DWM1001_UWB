"""HIGHRES_IMU extraction; FRD values stay FRD until the estimator converts them.

The header clock preserves device sample intervals, anchored to UP7000 receive
time. This is not a hardware time synchronization guarantee. Device time is
published separately for offline alignment with the existing TIMESYNC log.
"""
import math


class ImuSampleClock:
    def __init__(self):
        self.offset_ns = None
        self.last_us = None
        self.last_mono_ns = None
        self.resets = 0

    def stamp(self, device_us, receive_ns, monotonic_ns):
        if not isinstance(device_us, int) or device_us < 0:
            return None
        reset = self.last_mono_ns is not None and monotonic_ns - self.last_mono_ns > 500_000_000
        if self.last_us is not None and device_us <= self.last_us and not reset:
            if self.last_us - device_us < 1_000_000:
                return None  # duplicate or reordered sample, not a device reboot
            reset = True
        predicted = device_us * 1000 + (self.offset_ns or 0)
        if self.offset_ns is not None and not -500_000_000 <= predicted - receive_ns <= 100_000_000:
            reset = True
        if self.offset_ns is None or reset:
            if self.offset_ns is not None:
                self.resets += 1
            self.offset_ns = receive_ns - device_us * 1000
        self.last_us, self.last_mono_ns = device_us, monotonic_ns
        return device_us * 1000 + self.offset_ns


def highres_imu_sample(msg, receive_ns, monotonic_ns, clock):
    # Bits 0..5 report fresh x/y/z acceleration and angular velocity.
    if int(msg.fields_updated) & 0x3F != 0x3F:
        return None
    values = tuple(float(getattr(msg, field)) for field in ('xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro'))
    if not all(math.isfinite(value) for value in values):
        return None
    stamp = clock.stamp(msg.time_usec, receive_ns, monotonic_ns)
    if stamp is None:
        return None
    return stamp, values[:3], values[3:]
