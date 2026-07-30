"""ROS output adapter for typed observations and annotated camera frames."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol
from typing_extensions import assert_never

from cv_bridge import CvBridge, CvBridgeError
from ed_uav_interfaces.msg import TargetObservation
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from ed_uav_perception.target_annotation import (
    AnnotationFrame,
    render_target_observation,
)
from ed_uav_perception.target_input import stamp_seconds
from ed_uav_perception.target_message import (
    InvalidPoseMessageError,
    to_target_observation,
)
from ed_uav_perception.target_types import (
    AcceptedObservation,
    ObservationResult,
    RejectReason,
    RejectedObservation,
)


class ObservationPublisher(Protocol):
    """Publisher contract for typed target observations."""

    def publish(self, message: TargetObservation) -> None:
        ...


class ImagePublisher(Protocol):
    """Publisher contract for annotated image messages."""

    def publish(self, message: Image) -> None:
        ...


class ParameterSetter(Protocol):
    """Subset of the node parameter API needed for diagnostics."""

    def __call__(
        self, parameters: list[Parameter]
    ) -> Sequence[SetParametersResult]:
        ...


@dataclass(frozen=True, slots=True)
class ObservationOutputContext:
    """Live ROS bindings and sequence context for one output operation."""

    bridge: CvBridge
    observation_publisher: ObservationPublisher
    annotated_publisher: ImagePublisher
    set_parameters: ParameterSetter
    target_revision: str
    source_sequence: int


def _record(
    set_parameters: ParameterSetter, result: ObservationResult
) -> None:
    match result:
        case AcceptedObservation():
            values = (
                result.candidate_count,
                result.reprojection_rms_px,
                result.quality,
                "",
            )
        case RejectedObservation():
            values = (
                result.candidate_count,
                result.reprojection_rms_px
                if math.isfinite(result.reprojection_rms_px)
                else -1.0,
                0.0,
                result.reject_reason.value,
            )
        case unreachable:
            assert_never(unreachable)
    set_parameters(
        [
            Parameter("last_candidate_count", value=values[0]),
            Parameter("last_reprojection_rms_px", value=values[1]),
            Parameter("last_quality", value=values[2]),
            Parameter("last_reject_reason", value=values[3]),
        ]
    )


def _rejection(
    context: ObservationOutputContext, image: Image, reason: RejectReason
) -> RejectedObservation:
    return RejectedObservation(
        stamp_seconds(image.header.stamp),
        context.source_sequence,
        image.header.frame_id or "camera_unknown",
        context.target_revision,
        reason,
    )


def emit_observation(
    context: ObservationOutputContext,
    result: ObservationResult,
    image: Image,
    frame: AnnotationFrame | None = None,
) -> ObservationResult:
    """Publish typed observation and, when decodable, its annotation."""
    final = result
    try:
        message_out = to_target_observation(final, image.header.stamp)
    except InvalidPoseMessageError:
        final = _rejection(context, image, RejectReason.INVALID_INPUT)
        message_out = to_target_observation(final, image.header.stamp)
    _record(context.set_parameters, final)
    context.observation_publisher.publish(message_out)
    annotation_frame = frame
    if annotation_frame is None:
        try:
            annotation_frame = AnnotationFrame(
                context.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            )
        except CvBridgeError:
            annotation_frame = None
    if annotation_frame is not None:
        annotated_message = context.bridge.cv2_to_imgmsg(
            render_target_observation(annotation_frame, final), encoding="bgr8"
        )
        annotated_message.header = image.header
        context.annotated_publisher.publish(annotated_message)
    return final


def reject_observation(
    context: ObservationOutputContext,
    image: Image,
    reason: RejectReason,
    frame: AnnotationFrame | None = None,
) -> ObservationResult:
    """Publish a typed rejection and annotate it when the image is decodable."""
    return emit_observation(context, _rejection(context, image, reason), image, frame)
