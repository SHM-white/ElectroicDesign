from __future__ import annotations

import math
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass
from typing import Protocol, TypeVar


class MissionCancelled(RuntimeError):
    pass


class MissionTimeout(RuntimeError):
    pass


class CancellableGoal(Protocol):
    def cancel_goal_async(self) -> object:
        ...


class CancellableFuture(Awaitable[object], Protocol):
    def cancel(self) -> None:
        ...

    def cancelled(self) -> bool:
        ...


class Timer(Protocol):
    def cancel(self) -> None:
        ...


@dataclass(slots=True)
class ActiveGoals:
    flight: CancellableGoal | None = None
    planner: CancellableGoal | None = None

    def cancel_active(self) -> None:
        if self.flight is not None:
            self.flight.cancel_goal_async()
        if self.planner is not None:
            self.planner.cancel_goal_async()


@dataclass(frozen=True, slots=True)
class MissionDeadline:
    deadline_sec: float | None

    @classmethod
    def from_limits(cls, *, now_sec: float, limits: Sequence[float]) -> MissionDeadline:
        positive_limits = tuple(
            limit for limit in limits if math.isfinite(limit) and limit > 0.0
        )
        if not positive_limits:
            return cls(deadline_sec=None)
        return cls(deadline_sec=now_sec + min(positive_limits))

    def remaining_sec(self, now_sec: float) -> float | None:
        if self.deadline_sec is None:
            return None
        return max(0.0, self.deadline_sec - now_sec)


def steady_now_sec() -> float:
    from rclpy.clock import Clock, ClockType

    return Clock(clock_type=ClockType.STEADY_TIME).now().nanoseconds / 1_000_000_000.0


def deadline_from_timeout(timeout_sec: float) -> MissionDeadline:
    return MissionDeadline.from_limits(now_sec=steady_now_sec(), limits=(timeout_sec,))


def _create_steady_timer(node: object, timeout_sec: float, callback: Callable[[], None]) -> Timer:
    from rclpy.clock import Clock, ClockType

    clock = Clock(clock_type=ClockType.STEADY_TIME)
    try:
        return node.create_timer(timeout_sec, callback, clock=clock)
    except TypeError:
        return node.create_timer(timeout_sec, callback)


Result = TypeVar("Result")


async def wait_with_deadline(
    node: object,
    future: CancellableFuture,
    deadline: MissionDeadline,
    on_timeout: Callable[[], None],
) -> Result:
    remaining_sec = deadline.remaining_sec(steady_now_sec())
    if remaining_sec is not None and remaining_sec <= 0.0:
        on_timeout()
        future.cancel()
        raise MissionTimeout("action deadline expired")
    timed_out = False

    def timeout() -> None:
        nonlocal timed_out
        if timed_out:
            return
        timed_out = True
        on_timeout()
        future.cancel()

    timer = None
    if remaining_sec is not None:
        timer = _create_steady_timer(node, remaining_sec, timeout)
    try:
        result = await future
    finally:
        if timer is not None:
            timer.cancel()
            node.destroy_timer(timer)
    if future.cancelled():
        if timed_out:
            raise MissionTimeout("action deadline expired")
        raise MissionCancelled("action future canceled")
    return result
