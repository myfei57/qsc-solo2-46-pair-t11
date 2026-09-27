"""Position-segmented speed control for trolley travel.

The trolley used to run the whole track at one speed, so operators judged
the buffer zones by eye and jogged the brake. This module splits the track
into a full-speed middle segment and a creep-speed buffer zone at each end,
decelerates into the soft limits before stopping, and keeps the hard limits
as the final backstop. An operator stop ramps the trolley down wherever it
is, and a reversal is only adopted after a controlled standstill.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

from .audit import Audit
from .errors import ConfigError, HardLimitHeld, HookNotLowered
from .hoist import Hoist
from .limit import AXIS_TROLLEY, Limit, trip
from .trolley import Trolley

STATE_IDLE = "idle"
STATE_RUNNING = "running"
STATE_STOPPING = "stopping"
STATE_SOFT_HELD = "soft-held"
STATE_HARD_HELD = "hard-held"

SEGMENT_MIN_BUFFER = "min-buffer"
SEGMENT_MIDDLE = "middle"
SEGMENT_MAX_BUFFER = "max-buffer"

SCAN_STEP = 0.1


@dataclass(frozen=True)
class TrolleyProfile:
    """Track geometry and speed limits of one trolley axis."""

    track_min: float = 0.0
    track_max: float = 60.0
    buffer: float = 6.0
    hard_margin: float = 1.0
    rated_speed: float = 1.0
    creep_speed: float = 0.15
    acceleration: float = 0.3
    deceleration: float = 0.4

    def __post_init__(self) -> None:
        if not self.track_min < self.track_max:
            raise ConfigError("track_min must be below track_max")
        if self.buffer <= 0 or 2 * self.buffer >= self.track_max - self.track_min:
            raise ConfigError("buffer must be positive and shorter than half the track")
        if self.hard_margin <= 0:
            raise ConfigError("hard_margin must be positive")
        if not 0 < self.creep_speed <= self.rated_speed:
            raise ConfigError("creep_speed must be positive and not exceed rated_speed")
        if self.acceleration <= 0 or self.deceleration <= 0:
            raise ConfigError("ramp rates must be positive")

    def hard_min(self) -> float:
        return self.track_min - self.hard_margin

    def hard_max(self) -> float:
        return self.track_max + self.hard_margin


class TrolleyDrive:
    """Mutable motion state of the position-segmented trolley drive."""

    def __init__(self, profile: TrolleyProfile) -> None:
        self._lock = threading.Lock()
        self.profile = profile
        self.state = STATE_IDLE
        self.direction = 0
        self.requested = 0
        self.speed = 0.0
        self.target = 0.0
        self.recovery = False

    def state_now(self) -> str:
        with self._lock:
            return self.state

    def speed_now(self) -> float:
        with self._lock:
            return self.speed

    def direction_now(self) -> int:
        with self._lock:
            return self.direction


@dataclass(frozen=True)
class StepResult:
    """The outcome of advancing the drive, used by the console handler."""

    position: float
    speed: float
    state: str
    segment: str
    target: float
    hard_tripped: bool
    soft_stopped: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "position": self.position,
            "speed": self.speed,
            "state": self.state,
            "segment": self.segment,
            "target": self.target,
            "hard_tripped": self.hard_tripped,
            "soft_stopped": self.soft_stopped,
        }


def segment_at(profile: TrolleyProfile, position: float) -> str:
    if position <= profile.track_min + profile.buffer:
        return SEGMENT_MIN_BUFFER
    if position >= profile.track_max - profile.buffer:
        return SEGMENT_MAX_BUFFER
    return SEGMENT_MIDDLE


def target_speed(profile: TrolleyProfile, position: float, direction: float) -> float:
    """The speed ceiling at ``position`` while travelling in ``direction``.

    The middle segment allows the rated speed and each buffer zone caps the
    speed at creep; a braking curve then pulls the ceiling to zero exactly
    at the soft limit so the trolley decelerates before it stops.
    """

    if direction > 0:
        remaining = profile.track_max - position
    elif direction < 0:
        remaining = position - profile.track_min
    else:
        return 0.0
    if remaining <= 0:
        return 0.0
    cap = profile.creep_speed if remaining <= profile.buffer else profile.rated_speed
    braking = math.sqrt(2.0 * profile.deceleration * remaining)
    return min(cap, braking)


def command(
    drive: TrolleyDrive,
    trolley: Trolley,
    hoist: Hoist,
    direction: float,
    recorder: Audit | None = None,
) -> None:
    """Command travel in ``direction``; the profile shapes the speed."""

    with drive._lock, trolley._lock:  # noqa: SLF001 - module-level functions share drive state
        if drive.state == STATE_HARD_HELD:
            raise HardLimitHeld("hard limit holds the trolley; reset required")
        if not hoist.hook_lowered():
            raise HookNotLowered("hook is not lowered")
        requested = 1 if direction > 0 else -1
        drive.requested = requested
        detail = f"direction={requested}"
        if drive.speed <= 0.0:
            if target_speed(drive.profile, trolley.position, requested) > 0.0:
                drive.direction = requested
                drive.state = STATE_RUNNING
                trolley.moving = True
            else:
                detail += " held-by-soft-limit"
        elif requested == drive.direction:
            drive.state = STATE_RUNNING
            trolley.moving = True
        else:
            # Reverse only after a controlled standstill, never by plugging.
            drive.state = STATE_STOPPING
    if recorder is not None:
        recorder.record(trolley.id, "trolley-command", detail)


def request_stop(drive: TrolleyDrive, trolley: Trolley, recorder: Audit | None = None) -> None:
    """Ramp the trolley to a standstill wherever it is on the track."""

    with drive._lock, trolley._lock:  # noqa: SLF001
        drive.requested = 0
        if drive.state == STATE_RUNNING:
            drive.state = STATE_STOPPING
        drive.target = 0.0
        if drive.state == STATE_STOPPING and drive.speed <= 0.0:
            _settle(drive, trolley)
    if recorder is not None:
        recorder.record(trolley.id, "trolley-stop", "controlled stop")


def reset(drive: TrolleyDrive, trolley: Trolley, recorder: Audit | None = None) -> bool:
    """Release the hard-limit latch; returns False when nothing was held.

    Recovery stays armed until the trolley is jogged back inside the hard
    limits, so the backstop cannot re-latch while it is being cleared.
    """

    with drive._lock:  # noqa: SLF001
        if drive.state != STATE_HARD_HELD:
            return False
        drive.state = STATE_IDLE
        drive.recovery = True
    if recorder is not None:
        recorder.record(trolley.id, "trolley-reset", "hard limit latch released")
    return True


def step(
    drive: TrolleyDrive,
    trolley: Trolley,
    limit: Limit,
    seconds: float,
    recorder: Audit | None = None,
) -> StepResult:
    """Advance the drive by ``seconds`` of track time in scan sub-steps."""

    remaining = max(0.0, seconds)
    hard_tripped = False
    soft_stopped = False
    while remaining > 1e-9:
        dt = min(remaining, SCAN_STEP)
        remaining -= dt
        tripped, stopped, active = _scan(drive, trolley, limit, dt, recorder)
        hard_tripped = hard_tripped or tripped
        soft_stopped = soft_stopped or stopped
        if not active:
            break
    return _result(drive, trolley, hard_tripped, soft_stopped)


def drive_view(drive: TrolleyDrive, trolley: Trolley) -> dict[str, object]:
    """Flatten the drive and profile state for the console view."""

    with drive._lock, trolley._lock:  # noqa: SLF001
        profile = drive.profile
        return {
            "state": drive.state,
            "direction": drive.direction,
            "requested": drive.requested,
            "speed": drive.speed,
            "target": drive.target,
            "recovery": drive.recovery,
            "segment": segment_at(profile, trolley.position),
            "track_min": profile.track_min,
            "track_max": profile.track_max,
            "buffer": profile.buffer,
            "hard_min": profile.hard_min(),
            "hard_max": profile.hard_max(),
            "creep_speed": profile.creep_speed,
            "rated_speed": profile.rated_speed,
        }


def _scan(
    drive: TrolleyDrive,
    trolley: Trolley,
    limit: Limit,
    dt: float,
    recorder: Audit | None,
) -> tuple[bool, bool, bool]:
    """One scan cycle: check the backstop, ramp the speed, integrate."""

    hard_tripped = False
    soft_stopped = False
    event: tuple[str, str] | None = None
    with drive._lock, trolley._lock:  # noqa: SLF001
        profile = drive.profile
        breach = _hard_breach(profile, trolley.position)
        if breach == 0:
            drive.recovery = False
        motion = drive.direction if drive.state in (STATE_RUNNING, STATE_STOPPING) else 0
        if breach != 0 and drive.state != STATE_HARD_HELD and not (drive.recovery and motion != breach):
            drive.speed = 0.0
            drive.target = 0.0
            drive.direction = 0
            drive.requested = 0
            drive.state = STATE_HARD_HELD
            trolley.moving = False
            trip(limit, AXIS_TROLLEY)
            hard_tripped = True
            event = ("trolley-hard-limit", f"position={trolley.position:.2f}")
        elif drive.state == STATE_RUNNING:
            ceiling = target_speed(profile, trolley.position, drive.direction)
            drive.target = ceiling
            rate = profile.deceleration if ceiling < drive.speed else profile.acceleration
            drive.speed = _walk(drive.speed, ceiling, rate * dt)
            trolley.position += drive.direction * drive.speed * dt
            trolley.moving = True
            if ceiling == 0.0 and drive.speed == 0.0:
                drive.direction = 0
                drive.requested = 0
                drive.state = STATE_SOFT_HELD
                trolley.moving = False
                soft_stopped = True
                event = ("trolley-soft-limit", f"position={trolley.position:.2f}")
        elif drive.state == STATE_STOPPING:
            drive.target = 0.0
            drive.speed = _walk(drive.speed, 0.0, profile.deceleration * dt)
            trolley.position += drive.direction * drive.speed * dt
            if drive.speed == 0.0:
                _settle(drive, trolley)
        active = drive.state in (STATE_RUNNING, STATE_STOPPING)
    if event is not None and recorder is not None:
        recorder.record(trolley.id, event[0], event[1])
    return hard_tripped, soft_stopped, active


def _settle(drive: TrolleyDrive, trolley: Trolley) -> None:
    """Adopt the requested direction once the trolley stands still."""

    if drive.requested != 0:
        ceiling = target_speed(drive.profile, trolley.position, drive.requested)
        if ceiling > 0.0:
            drive.direction = drive.requested
            drive.requested = 0
            drive.state = STATE_RUNNING
            trolley.moving = True
            return
        drive.requested = 0
        drive.direction = 0
        drive.state = STATE_SOFT_HELD
        trolley.moving = False
        return
    drive.direction = 0
    drive.state = STATE_IDLE
    trolley.moving = False


def _result(
    drive: TrolleyDrive,
    trolley: Trolley,
    hard_tripped: bool,
    soft_stopped: bool,
) -> StepResult:
    with drive._lock, trolley._lock:  # noqa: SLF001
        return StepResult(
            position=trolley.position,
            speed=drive.speed,
            state=drive.state,
            segment=segment_at(drive.profile, trolley.position),
            target=drive.target,
            hard_tripped=hard_tripped,
            soft_stopped=soft_stopped,
        )


def _hard_breach(profile: TrolleyProfile, position: float) -> int:
    if position > profile.hard_max():
        return 1
    if position < profile.hard_min():
        return -1
    return 0


def _walk(speed: float, target: float, max_delta: float) -> float:
    if speed < target:
        return min(speed + max_delta, target)
    return max(speed - max_delta, target)
