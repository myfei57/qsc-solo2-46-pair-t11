"""Trolley travel with position-zoned speed control and a speed sampler.

The travel range is split into segments measured inward from each hard
limit: a buffer zone where the drive may only creep, and the normal zone
that allows the rated speed. A soft limit just inside each hard limit ramps
the drive down to a full stop, while the hard limit stays as the independent
backstop that clamps the position and kills the motion outright. An
operator stop takes effect at any point of the travel.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

from .audit import Audit
from .errors import HookNotLowered, InvalidTravelZones
from .hoist import Hoist, resume_lower
from .store import Store

DIRECTION_IN = -1
DIRECTION_OUT = 1

HOLD_NONE = ""
HOLD_SOFT = "soft"
HOLD_HARD = "hard"

SEGMENT_NORMAL = "normal"
SEGMENT_BUFFER = "buffer"
SEGMENT_SOFT = "soft"
SEGMENT_HARD = "hard"


@dataclass(frozen=True)
class TravelZones:
    """Speed zones along the jib, from the root hard limit to the tip hard limit.

    The soft limits sit ``soft_margin`` inside the hard limits and the buffer
    zones extend ``buffer`` inward from the soft limits. Speeds are m/s and
    ``decel`` is the ramp rate (m/s²) used both to accelerate and to brake.
    """

    hard_minimum: float = 0.0
    hard_maximum: float = 60.0
    soft_margin: float = 0.5
    buffer: float = 3.0
    rated_speed: float = 1.0
    creep_speed: float = 0.2
    decel: float = 0.5

    def __post_init__(self) -> None:
        if self.hard_maximum <= self.hard_minimum:
            raise InvalidTravelZones("hard limits are inverted")
        if self.soft_margin < 0.0:
            raise InvalidTravelZones("soft margin must not be negative")
        if self.soft_maximum <= self.soft_minimum:
            raise InvalidTravelZones("soft limits are inverted")
        if self.buffer <= 0.0:
            raise InvalidTravelZones("buffer must be positive")
        if not 0.0 < self.creep_speed < self.rated_speed:
            raise InvalidTravelZones("creep speed must be below the rated speed")
        if self.decel <= 0.0:
            raise InvalidTravelZones("decel must be positive")

    @property
    def soft_minimum(self) -> float:
        return self.hard_minimum + self.soft_margin

    @property
    def soft_maximum(self) -> float:
        return self.hard_maximum - self.soft_margin


class Trolley:
    """Mutable state of the trolley that moves the load along the jib."""

    def __init__(self, trolley_id: str) -> None:
        self._lock = threading.Lock()
        self.id = trolley_id
        self.position = 0.0
        self.speed = 0.0
        self.direction = DIRECTION_OUT
        self.moving = False
        self.hold = HOLD_NONE

    def is_moving(self) -> bool:
        with self._lock:
            return self.moving

    def position_now(self) -> float:
        with self._lock:
            return self.position

    def speed_now(self) -> float:
        with self._lock:
            return self.speed

    def direction_now(self) -> int:
        with self._lock:
            return self.direction

    def hold_now(self) -> str:
        with self._lock:
            return self.hold


class Speedometer:
    """Derives trolley speed from consecutive position samples."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_position = 0.0
        self._last_speed = 0.0

    def sample(self, position: float, seconds: float) -> float:
        if seconds <= 0:
            return 0.0
        with self._lock:
            speed = (position - self._last_position) / seconds
            self._last_position = position
            self._last_speed = speed
            return speed

    def last_speed(self) -> float:
        with self._lock:
            return self._last_speed


def target_speed(zones: TravelZones, position: float, direction: int) -> float:
    """The fastest speed allowed at ``position`` while heading ``direction``.

    The buffer zone caps the speed at the creep speed; the soft limit adds a
    braking profile so the drive can always stop before the limit. Only the
    end the trolley is approaching restricts the speed, so backing away from
    a limit is never throttled.
    """

    if direction == DIRECTION_OUT:
        remaining = zones.soft_maximum - position
        hard_remaining = zones.hard_maximum - position
    elif direction == DIRECTION_IN:
        remaining = position - zones.soft_minimum
        hard_remaining = position - zones.hard_minimum
    else:
        return 0.0
    if hard_remaining <= 0.0 or remaining <= 0.0:
        return 0.0
    cap = zones.creep_speed if remaining <= zones.buffer else zones.rated_speed
    stoppable = math.sqrt(2.0 * zones.decel * remaining)
    return min(cap, stoppable)


def segment_at(zones: TravelZones, position: float) -> str:
    """Name the segment the position falls in, from either end of the track."""

    if position < zones.hard_minimum or position > zones.hard_maximum:
        return SEGMENT_HARD
    if position < zones.soft_minimum or position > zones.soft_maximum:
        return SEGMENT_SOFT
    if position - zones.soft_minimum <= zones.buffer:
        return SEGMENT_BUFFER
    if zones.soft_maximum - position <= zones.buffer:
        return SEGMENT_BUFFER
    return SEGMENT_NORMAL


def step(trolley: Trolley, zones: TravelZones, dt: float) -> float:
    """Advance one control cycle and return the new (signed) speed.

    The speed walks toward the zoned target at the decel rate and the
    position integrates it. The soft limit holds the drive once it ramped
    to a standstill; the hard limit clamps the position and stops the drive
    immediately, whatever the speed law allowed.
    """

    with trolley._lock:  # noqa: SLF001 - module-level functions share trolley state
        target = 0.0
        if trolley.moving:
            target = trolley.direction * target_speed(
                zones, trolley.position, trolley.direction
            )
        trolley.speed = _walk(trolley.speed, target, zones.decel * dt)
        trolley.position += trolley.speed * dt
        if trolley.position > zones.hard_maximum:
            trolley.position = zones.hard_maximum
            _hold_hard(trolley)
        elif trolley.position < zones.hard_minimum:
            trolley.position = zones.hard_minimum
            _hold_hard(trolley)
        elif (
            trolley.moving
            and trolley.speed == 0.0
            and target == 0.0
            and _past_soft_limit(zones, trolley.position, trolley.direction)
        ):
            trolley.moving = False
            trolley.hold = HOLD_SOFT
        return trolley.speed


def travel(trolley: Trolley, zones: TravelZones, seconds: float, dt: float = 0.1) -> None:
    """Drive for at most ``seconds`` in ``dt`` slices, stopping early on a hold."""

    steps = int(seconds / dt)
    for _ in range(steps):
        step(trolley, zones, dt)
        if not trolley.is_moving() and trolley.speed_now() == 0.0:
            break


def move(trolley: Trolley, hoist: Hoist, direction: int = DIRECTION_OUT) -> None:
    """Move the trolley only after the hook has been lowered."""

    with trolley._lock:  # noqa: SLF001 - module-level functions share trolley state
        if not hoist.hook_lowered():
            raise HookNotLowered("hook is not lowered")
        trolley.moving = True
        trolley.direction = DIRECTION_IN if direction < 0 else DIRECTION_OUT
        trolley.hold = HOLD_NONE


def move_resume(trolley: Trolley, hoist: Hoist, store: Store) -> None:
    """Resume a trolley move by reloading the hoist snapshot first."""

    resume_lower(hoist, store)
    move(trolley, hoist)


def stop(trolley: Trolley, recorder: Audit | None = None) -> None:
    with trolley._lock:  # noqa: SLF001
        trolley.moving = False
        trolley.speed = 0.0
    if recorder is not None:
        recorder.record(trolley.id, "trolley-stop", "stopped")


def limit_stop(trolley: Trolley) -> None:
    """Hold the trolley after its axis trips a limit."""

    with trolley._lock:  # noqa: SLF001
        _hold_hard(trolley)


def _walk(speed: float, target: float, amount: float) -> float:
    if abs(target - speed) < amount:
        return target
    if target > speed:
        return speed + amount
    return speed - amount


def _past_soft_limit(zones: TravelZones, position: float, direction: int) -> bool:
    if direction == DIRECTION_OUT:
        return position >= zones.soft_maximum
    if direction == DIRECTION_IN:
        return position <= zones.soft_minimum
    return False


def _hold_hard(trolley: Trolley) -> None:
    trolley.moving = False
    trolley.speed = 0.0
    trolley.hold = HOLD_HARD
