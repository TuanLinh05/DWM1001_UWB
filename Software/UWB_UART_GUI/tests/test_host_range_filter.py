"""Regression checks for the host-side range filter port."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from host_range_filter import (  # noqa: E402
    HostRangeFilter,
    Median3,
)


class HostRangeFilterTests(unittest.TestCase):
    def test_median_three_rejects_single_spike(self) -> None:
        median = Median3()
        self.assertEqual(median.update(1000), 1000)
        self.assertEqual(median.update(3000), 3000)
        self.assertEqual(median.update(1010), 1010)
        self.assertEqual(median.update(1020), 1020)

    def test_first_sample_initializes_without_zero_transient(self) -> None:
        filter_state = HostRangeFilter()
        self.assertEqual(filter_state.update(1270, -9607), 1270)

    def test_single_spike_does_not_move_output(self) -> None:
        filter_state = HostRangeFilter()
        now_ms = 20
        for _ in range(10):
            output = filter_state.update(1000, -9700, now_ms)
            now_ms += 20
        before_spike = output
        spike_output = filter_state.update(5000, -9700, now_ms)
        now_ms += 20
        recovery_output = filter_state.update(1000, -9700, now_ms)

        self.assertLessEqual(abs(spike_output - before_spike), 1)
        self.assertLessEqual(abs(recovery_output - before_spike), 1)

    def test_step_converges_smoothly_without_snap(self) -> None:
        filter_state = HostRangeFilter()
        now_ms = 20
        output = 1000
        for _ in range(10):
            output = filter_state.update(1000, -9700, now_ms)
            now_ms += 20

        outputs: list[int] = []
        for _ in range(50):
            output = filter_state.update(3000, -9700, now_ms)
            outputs.append(output)
            now_ms += 20

        steps = [abs(b - a) for a, b in zip([1000] + outputs, outputs)]
        self.assertLessEqual(max(steps), 200)
        self.assertGreaterEqual(outputs[-1], 2950)
        self.assertNotIn(3000, outputs[:5])

    def test_fast_ramp_enters_tracking_and_keeps_up(self) -> None:
        filter_state = HostRangeFilter()
        now_ms = 20
        for _ in range(10):
            filter_state.update(1000, -9700, now_ms)
            now_ms += 20

        output = 1000
        for index in range(1, 31):
            output = filter_state.update(1000 + 160 * index, -9700, now_ms)
            now_ms += 20

        self.assertGreater(output, 5400)
        self.assertLess(5800 - output, 300)

    def test_stale_reacquire_remains_continuous(self) -> None:
        filter_state = HostRangeFilter()
        filter_state.update(1000, -9700, 20)
        filter_state.update(1000, -9700, 40)
        before_gap = filter_state.update(1000, -9700, 60)

        first = filter_state.update(3000, -9700, 700)
        second = filter_state.update(3010, -9700, 720)
        third = filter_state.update(2990, -9700, 740)
        latest = third
        for index in range(32):
            latest = filter_state.update(
                3000 + (index % 3 - 1) * 10,
                -9700,
                760 + index * 20,
            )

        self.assertLessEqual(first - before_gap, 100)
        self.assertLess(first, second)
        self.assertLess(second, third)
        self.assertLess(third, 1500)
        self.assertGreaterEqual(latest, 2950)
        self.assertEqual(filter_state.stale_reacquire_count, 1)

    def test_weak_fpp_does_not_create_a_one_metre_ceiling(self) -> None:
        filter_state = HostRangeFilter()
        output = 500
        for index in range(41):
            output = filter_state.update(
                500 + 50 * index,
                -9700,
                20 + index * 20,
            )

        self.assertGreater(output, 2350)
        self.assertLess(2500 - output, 100)

    def test_stationary_noise_stays_smooth(self) -> None:
        noise = [0, 20, -15, 35, -30, 10, -5, 25, -20, 0] * 20
        filter_state = HostRangeFilter()
        outputs = [
            filter_state.update(1000 + value, -9700, 20 + index * 20)
            for index, value in enumerate(noise)
        ]

        self.assertLessEqual(max(outputs[50:]) - min(outputs[50:]), 10)


if __name__ == "__main__":
    unittest.main()
