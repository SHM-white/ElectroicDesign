"""Optional real rosbag2 conversion for a completed deterministic event report."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .artifacts import ArtifactExistsError, IncompleteScenarioError
from .model import ScenarioReport


@dataclass(frozen=True, slots=True)
class Rosbag2UnavailableError(Exception):
    """Raised when a static host lacks the ROS runtime required to write rosbag2."""

    def __str__(self) -> str:
        return "rosbag2_py and rclpy serialization are required to write a rosbag2 fixture"


@dataclass(frozen=True, slots=True)
class RosbagFixtureBuilder:
    """Write event evidence as a standard SQLite rosbag2 topic on a ROS host."""

    root: Path

    def write(self, report: ScenarioReport) -> Path:
        """Create `/verification/events` as serialized `std_msgs/msg/String` records."""
        if not report.completed:
            raise IncompleteScenarioError()
        if self.root.exists():
            raise ArtifactExistsError(self.root)
        try:
            from rclpy.serialization import serialize_message
            from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
            from std_msgs.msg import String
        except ImportError as error:
            raise Rosbag2UnavailableError() from error
        writer = SequentialWriter()
        writer.open(StorageOptions(uri=str(self.root), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
        writer.create_topic(
            TopicMetadata(
                name="/verification/events",
                type="std_msgs/msg/String",
                serialization_format="cdr",
                offered_qos_profiles="",
            )
        )
        for event in report.events:
            message = String()
            message.data = json.dumps(event.as_json_value(), separators=(",", ":"), sort_keys=True)
            writer.write("/verification/events", serialize_message(message), event.simulated_time_ns)
        del writer
        return self.root
