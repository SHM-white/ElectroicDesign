"""In-memory GPIO and laser fake with no device or flight-side effects."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class GpioLaserSnapshot:
    """Observable simulated GPIO/laser output state."""

    laser_enabled: bool
    sequence: int


class FakeGpioLaser:  # noqa: MUTABLE_OK
    """A deliberately mutable test output state machine with no hardware backend."""

    def __init__(self) -> None:
        self._laser_enabled = False
        self._sequence = 0

    def set_laser(self, enabled: bool) -> GpioLaserSnapshot:
        """Set the synthetic laser state and return its new deterministic snapshot."""
        self._laser_enabled = enabled
        self._sequence += 1
        return self.snapshot()

    def snapshot(self) -> GpioLaserSnapshot:
        """Return the current in-memory output state without touching a GPIO device."""
        return GpioLaserSnapshot(laser_enabled=self._laser_enabled, sequence=self._sequence)
