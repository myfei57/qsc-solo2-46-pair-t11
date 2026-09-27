"""Unit tests for the position-segmented trolley speed profile."""

from __future__ import annotations

import unittest

from cranefleet.audit import Audit
from cranefleet.errors import ConfigError, HardLimitHeld, HookNotLowered
from cranefleet.hoist import Hoist, lower as hoist_lower
from cranefleet.limit import AXIS_TROLLEY, Limit
from cranefleet.profile import (
    SEGMENT_MAX_BUFFER,
    SEGMENT_MIDDLE,
    SEGMENT_MIN_BUFFER,
    STATE_HARD_HELD,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_SOFT_HELD,
    STATE_STOPPING,
    TrolleyDrive,
    TrolleyProfile,
    command,
    drive_view,
    request_stop,
    reset,
    segment_at,
    step,
    target_speed,
)
from cranefleet.trolley import Trolley


class ProfileTests(unittest.TestCase):
    def test_defaults_pass_validation(self) -> None:
        profile = TrolleyProfile()
        self.assertEqual(profile.hard_min(), -1.0)
        self.assertEqual(profile.hard_max(), 61.0)

    def test_invalid_geometry_is_rejected(self) -> None:
        with self.assertRaises(ConfigError):
            TrolleyProfile(track_min=60.0, track_max=0.0)
        with self.assertRaises(ConfigError):
            TrolleyProfile(buffer=0.0)
        with self.assertRaises(ConfigError):
            TrolleyProfile(buffer=35.0)
        with self.assertRaises(ConfigError):
            TrolleyProfile(hard_margin=0.0)
        with self.assertRaises(ConfigError):
            TrolleyProfile(creep_speed=2.0, rated_speed=1.0)
        with self.assertRaises(ConfigError):
            TrolleyProfile(deceleration=0.0)


class TargetSpeedTests(unittest.TestCase):
    def setUp(self) -> None:
        self.profile = TrolleyProfile()

    def test_middle_segment_allows_rated_speed(self) -> None:
        self.assertAlmostEqual(target_speed(self.profile, 30.0, 1), 1.0)
        self.assertAlmostEqual(target_speed(self.profile, 30.0, -1), 1.0)

    def test_buffer_segments_cap_at_creep(self) -> None:
        self.assertAlmostEqual(target_speed(self.profile, 55.0, 1), 0.15)
        self.assertAlmostEqual(target_speed(self.profile, 5.0, -1), 0.15)

    def test_soft_limits_command_zero(self) -> None:
        self.assertEqual(target_speed(self.profile, 60.0, 1), 0.0)
        self.assertEqual(target_speed(self.profile, 61.5, 1), 0.0)
        self.assertEqual(target_speed(self.profile, 0.0, -1), 0.0)
        self.assertEqual(target_speed(self.profile, 30.0, 0), 0.0)

    def test_braking_curve_pulls_the_ceiling_to_zero(self) -> None:
        near = target_speed(self.profile, 59.99, 1)
        self.assertLess(near, self.profile.creep_speed)
        self.assertAlmostEqual(near, (2 * self.profile.deceleration * 0.01) ** 0.5)
        self.assertGreater(target_speed(self.profile, 59.99, 1), target_speed(self.profile, 59.999, 1))
        self.assertAlmostEqual(target_speed(self.profile, 58.0, 1), self.profile.creep_speed)

    def test_segments(self) -> None:
        self.assertEqual(segment_at(self.profile, 0.0), SEGMENT_MIN_BUFFER)
        self.assertEqual(segment_at(self.profile, 6.0), SEGMENT_MIN_BUFFER)
        self.assertEqual(segment_at(self.profile, 30.0), SEGMENT_MIDDLE)
        self.assertEqual(segment_at(self.profile, 54.0), SEGMENT_MAX_BUFFER)
        self.assertEqual(segment_at(self.profile, 60.0), SEGMENT_MAX_BUFFER)


class DriveTests(unittest.TestCase):
    def setUp(self) -> None:
        self.profile = TrolleyProfile()
        self.drive = TrolleyDrive(self.profile)
        self.trolley = Trolley("crane-1")
        self.hoist = Hoist("crane-1")
        self.limit = Limit()
        self.audit = Audit(50)
        hoist_lower(self.hoist)

    def travel(self, seconds: float) -> None:
        step(self.drive, self.trolley, self.limit, seconds, self.audit)

    def test_command_requires_lowered_hook(self) -> None:
        self.hoist = Hoist("crane-1")
        with self.assertRaises(HookNotLowered):
            command(self.drive, self.trolley, self.hoist, 1, self.audit)

    def test_buffer_creep_then_rated_then_soft_stop(self) -> None:
        command(self.drive, self.trolley, self.hoist, 1, self.audit)
        self.assertEqual(self.drive.state_now(), STATE_RUNNING)

        # Leaving the near end the trolley may run at rated speed; the
        # buffer only caps the speed while approaching the far end.
        self.travel(5.0)
        self.assertAlmostEqual(self.drive.speed_now(), self.profile.rated_speed)

        self.travel(45.0)
        self.assertEqual(segment_at(self.profile, self.trolley.position_now()), SEGMENT_MIDDLE)
        self.assertAlmostEqual(self.drive.speed_now(), self.profile.rated_speed)

        self.travel(15.0)
        self.assertEqual(segment_at(self.profile, self.trolley.position_now()), SEGMENT_MAX_BUFFER)
        self.assertAlmostEqual(self.drive.speed_now(), self.profile.creep_speed)

        result = step(self.drive, self.trolley, self.limit, 45.0, self.audit)
        self.assertTrue(result.soft_stopped)
        self.assertFalse(result.hard_tripped)
        self.assertEqual(self.drive.state_now(), STATE_SOFT_HELD)
        self.assertAlmostEqual(self.trolley.position_now(), self.profile.track_max, delta=0.1)
        self.assertEqual(self.drive.speed_now(), 0.0)
        self.assertFalse(self.trolley.is_moving())
        self.assertFalse(self.limit.is_tripped(AXIS_TROLLEY))
        actions = [entry.action for entry in self.audit.snapshot()]
        self.assertIn("trolley-soft-limit", actions)

    def test_speed_never_exceeds_the_profile(self) -> None:
        command(self.drive, self.trolley, self.hoist, 1, self.audit)
        for _ in range(140):
            result = step(self.drive, self.trolley, self.limit, 1.0)
            self.assertLessEqual(result.speed, self.profile.rated_speed + 1e-9)
            if result.state == STATE_SOFT_HELD:
                break
        self.assertEqual(self.drive.state_now(), STATE_SOFT_HELD)

    def test_operator_stop_works_anywhere(self) -> None:
        command(self.drive, self.trolley, self.hoist, 1, self.audit)
        self.travel(50.0)
        self.assertAlmostEqual(self.drive.speed_now(), self.profile.rated_speed)
        position_at_stop = self.trolley.position_now()

        request_stop(self.drive, self.trolley, self.audit)
        self.assertEqual(self.drive.state_now(), STATE_STOPPING)
        self.travel(5.0)
        self.assertEqual(self.drive.state_now(), STATE_IDLE)
        self.assertEqual(self.drive.speed_now(), 0.0)
        self.assertFalse(self.trolley.is_moving())
        stopped_at = self.trolley.position_now()
        self.assertGreater(stopped_at, position_at_stop)
        self.assertLess(stopped_at, self.profile.track_max - self.profile.buffer)

    def test_reversal_waits_for_a_standstill(self) -> None:
        command(self.drive, self.trolley, self.hoist, 1, self.audit)
        self.travel(50.0)
        self.assertEqual(self.drive.direction_now(), 1)

        command(self.drive, self.trolley, self.hoist, -1, self.audit)
        self.assertEqual(self.drive.state_now(), STATE_STOPPING)
        self.assertEqual(self.drive.direction_now(), 1)

        self.travel(2.0)
        self.assertLess(self.drive.speed_now(), self.profile.rated_speed)
        self.assertEqual(self.drive.direction_now(), 1)
        peak = self.trolley.position_now()

        self.travel(10.0)
        self.assertEqual(self.drive.state_now(), STATE_RUNNING)
        self.assertEqual(self.drive.direction_now(), -1)
        self.assertLess(self.trolley.position_now(), peak - 5.0)

    def test_soft_limit_holds_outward_and_releases_inward(self) -> None:
        command(self.drive, self.trolley, self.hoist, 1, self.audit)
        self.travel(160.0)
        self.assertEqual(self.drive.state_now(), STATE_SOFT_HELD)
        held_position = self.trolley.position_now()

        command(self.drive, self.trolley, self.hoist, 1, self.audit)
        self.assertEqual(self.drive.state_now(), STATE_SOFT_HELD)
        self.travel(1.0)
        self.assertAlmostEqual(self.trolley.position_now(), held_position)
        details = [entry.detail for entry in self.audit.snapshot()]
        self.assertTrue(any("held-by-soft-limit" in detail for detail in details))

        command(self.drive, self.trolley, self.hoist, -1, self.audit)
        self.assertEqual(self.drive.state_now(), STATE_RUNNING)
        self.travel(10.0)
        self.assertLess(self.trolley.position_now(), held_position)

    def test_inward_journey_soft_stops_at_the_near_end(self) -> None:
        command(self.drive, self.trolley, self.hoist, 1, self.audit)
        self.travel(160.0)
        command(self.drive, self.trolley, self.hoist, -1, self.audit)
        self.travel(160.0)
        self.assertEqual(self.drive.state_now(), STATE_SOFT_HELD)
        self.assertAlmostEqual(self.trolley.position_now(), self.profile.track_min, delta=0.1)

    def test_hard_limit_is_the_backstop(self) -> None:
        self.trolley.position = self.profile.hard_max() + 0.5
        result = step(self.drive, self.trolley, self.limit, 0.5, self.audit)
        self.assertTrue(result.hard_tripped)
        self.assertEqual(self.drive.state_now(), STATE_HARD_HELD)
        self.assertFalse(self.trolley.is_moving())
        self.assertTrue(self.limit.is_tripped(AXIS_TROLLEY))
        actions = [entry.action for entry in self.audit.snapshot()]
        self.assertIn("trolley-hard-limit", actions)

        with self.assertRaises(HardLimitHeld):
            command(self.drive, self.trolley, self.hoist, 1, self.audit)
        with self.assertRaises(HardLimitHeld):
            command(self.drive, self.trolley, self.hoist, -1, self.audit)

        self.assertTrue(reset(self.drive, self.trolley, self.audit))
        self.assertEqual(self.drive.state_now(), STATE_IDLE)
        self.assertFalse(reset(self.drive, self.trolley, self.audit))

        command(self.drive, self.trolley, self.hoist, -1, self.audit)
        self.travel(5.0)
        self.assertLess(self.trolley.position_now(), self.profile.hard_max())
        self.assertEqual(self.drive.state_now(), STATE_RUNNING)
        self.assertFalse(drive_view(self.drive, self.trolley)["recovery"])

    def test_drive_view_shape(self) -> None:
        view = drive_view(self.drive, self.trolley)
        for key in (
            "state",
            "direction",
            "requested",
            "speed",
            "target",
            "recovery",
            "segment",
            "track_min",
            "track_max",
            "buffer",
            "hard_min",
            "hard_max",
            "creep_speed",
            "rated_speed",
        ):
            self.assertIn(key, view)
        self.assertEqual(view["state"], STATE_IDLE)
        self.assertEqual(view["segment"], SEGMENT_MIN_BUFFER)


if __name__ == "__main__":
    unittest.main()
