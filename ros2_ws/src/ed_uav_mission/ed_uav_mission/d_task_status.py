"""Map internal D-task phases to the operator-visible MissionStatus contract."""

from ed_uav_interfaces.msg import MissionStatus
from ed_uav_mission.d_task_model import DTaskPhase


def mission_status_state(phase: DTaskPhase) -> int:
    mapping = {
        DTaskPhase.PRE_ARM: MissionStatus.STATE_PRE_ARM,
        DTaskPhase.WAITING_START: MissionStatus.STATE_PRE_ARM,
        DTaskPhase.TAKEOFF: MissionStatus.STATE_TAKEOFF,
        DTaskPhase.STABILIZING: MissionStatus.STATE_TAKEOFF,
        DTaskPhase.ACQUIRING: MissionStatus.STATE_SEARCHING,
        DTaskPhase.ESCORTING: MissionStatus.STATE_ACCOMPANYING,
        DTaskPhase.TRACKING: MissionStatus.STATE_ACCOMPANYING,
        DTaskPhase.RELEASING: MissionStatus.STATE_PAYLOAD_DROP,
        DTaskPhase.DESCENDING: MissionStatus.STATE_LANDING_ON_VEHICLE,
        DTaskPhase.VEHICLE_DWELL: MissionStatus.STATE_VEHICLE_DWELL,
        DTaskPhase.RETAKEOFF: MissionStatus.STATE_RETURNING_HOME,
        DTaskPhase.RETURNING_HOME: MissionStatus.STATE_RETURNING_HOME,
        DTaskPhase.LANDING_HOME: MissionStatus.STATE_LANDING_HOME,
        DTaskPhase.SAFE_HOVER: MissionStatus.STATE_RETURNING_HOME,
        DTaskPhase.SAFE_RETURN: MissionStatus.STATE_RETURNING_HOME,
        DTaskPhase.SAFE_LAND: MissionStatus.STATE_LANDING_HOME,
        DTaskPhase.SUCCEEDED: MissionStatus.STATE_SUCCEEDED,
        DTaskPhase.ABORTED: MissionStatus.STATE_ABORTED,
    }
    return mapping[phase]
