"""Payload trigger plugin — laser/LED actuation via FlightCommand."""

from __future__ import annotations

from ed_uav_mission.mission_model import PayloadParams


class PayloadPlugin:
    """Map a payload params block to a single trigger action.

    The executor translates the returned params into one or more
    ``FlightCommand`` goals with ``COMMAND_SET_MODE`` or similar
    auxiliary-channel commands.  This plugin never imports serial,
    GPIO, or camera APIs.
    """

    def generate(self, params: PayloadParams) -> PayloadParams:
        return params
