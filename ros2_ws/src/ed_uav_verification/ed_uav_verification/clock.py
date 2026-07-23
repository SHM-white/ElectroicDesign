"""Virtual monotonic time used by deterministic offline scenarios."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class VirtualMonotonicClock:  # noqa: MUTABLE_OK
    """A mutable clock whose advancement is explicit and independent of wall time."""

    now_ns: int
    tick_duration_ns: int

    def timestamp_for_tick(self, tick: int) -> int:
        """Return the deterministic source acquisition timestamp for one tick."""
        return self.now_ns + tick * self.tick_duration_ns

    def elapsed_ns(self, earlier_ns: int) -> int:
        """Measure elapsed time on this monotonic test clock."""
        return self.now_ns - earlier_ns
