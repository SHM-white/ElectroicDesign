"""Typed pre-arm selection contracts for D-task mission branches."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from ed_uav_mission.d_task_model import DTaskKind, DTaskSelection
from ed_uav_mission.mission_model import CompetitionParams


class DTaskSelectionRequest(Protocol):
    contract_version: int
    mission_id: str
    field_profile_id: str
    mission_profile_id: str
    deployment_preset_id: str
    target_revision: str
    task: int


@dataclass(frozen=True, slots=True)
class DTaskSelectionContract:
    mission_id: str
    field_profile_id: str
    mission_profile_id: str
    deployment_preset_id: str
    target_revision: str
    allowed_tasks: frozenset[DTaskKind]

    @classmethod
    def for_competition(
        cls,
        mission_id: str,
        field_profile_id: str,
        params: CompetitionParams,
    ) -> DTaskSelectionContract:
        return cls(
            mission_id=mission_id,
            field_profile_id=field_profile_id,
            mission_profile_id=params.mission_profile_id,
            deployment_preset_id=params.deployment_preset_id,
            target_revision=params.target_revision,
            allowed_tasks=frozenset(
                (DTaskKind.PAYLOAD_DROP, DTaskKind.DYNAMIC_LANDING)
            ),
        )

    def rejection_reason(self, request: DTaskSelectionRequest, contract_version: int) -> str:
        if request.contract_version != contract_version:
            return "unsupported selection contract"
        if request.mission_id != self.mission_id:
            return "selection mission_id does not match loaded mission"
        if request.field_profile_id != self.field_profile_id:
            return "selection field_profile_id does not match loaded field"
        if request.mission_profile_id != self.mission_profile_id:
            return "selection mission profile does not match loaded profile"
        if request.deployment_preset_id != self.deployment_preset_id:
            return "selection deployment preset does not match loaded preset"
        if request.target_revision != self.target_revision:
            return "selection target revision does not match loaded revision"
        if request.task not in {int(task) for task in self.allowed_tasks}:
            return "selection task is unsupported"
        return ""


def selection_from_request(request: DTaskSelectionRequest, committed_at_s: float) -> DTaskSelection:
    return DTaskSelection(
        mission_id=str(request.mission_id),
        mission_profile_id=str(request.mission_profile_id),
        deployment_preset_id=str(request.deployment_preset_id),
        target_revision=str(request.target_revision),
        task=DTaskKind(int(request.task)),
        committed_at_s=committed_at_s,
    )


def is_committed_task3_selection(selection: DTaskSelection | None) -> bool:
    return selection is not None and selection.task is DTaskKind.STABILITY_TEST
