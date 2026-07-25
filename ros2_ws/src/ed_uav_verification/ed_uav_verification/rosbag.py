"""Optional real rosbag2 conversion for a completed deterministic event report."""

from __future__ import annotations

import json
import os
import shutil
import sys
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

    @staticmethod
    def _add_ros_python_paths() -> None:
        """Recover ROS Python paths when a caller supplies a package-only PYTHONPATH."""
        prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        patterns = ("local/lib/python*/dist-packages", "lib/python*/site-packages", "lib/python*/dist-packages")
        for prefix in prefixes:
            for pattern in patterns:
                for candidate in Path(prefix).glob(pattern):
                    candidate_text = str(candidate)
                    if candidate_text not in sys.path:
                        sys.path.insert(0, candidate_text)

    def validate_runtime(self) -> None:
        """Require the ROS serialization runtime before other artifacts are written."""
        self._add_ros_python_paths()
        try:
            from rclpy.serialization import serialize_message
            from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
            from std_msgs.msg import String
        except ImportError as error:
            raise Rosbag2UnavailableError() from error
        _ = (serialize_message, ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata, String)

    def write(self, report: ScenarioReport) -> Path:
        """Create `/verification/events` as serialized `std_msgs/msg/String` records."""
        if not report.completed:
            raise IncompleteScenarioError()
        if self.root.exists() or self.root.is_symlink():
            raise ArtifactExistsError(self.root)
        partial = self.root.with_name(f"{self.root.name}.partial")
        if partial.exists() or partial.is_symlink():
            raise ArtifactExistsError(partial)
        self._add_ros_python_paths()
        try:
            from rclpy.serialization import serialize_message
            from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
            from std_msgs.msg import String
        except ImportError as error:
            raise Rosbag2UnavailableError() from error
        partial.parent.mkdir(parents=True, exist_ok=True)
        writer = SequentialWriter()
        succeeded = False
        try:
            writer.open(StorageOptions(uri=str(partial), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
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
            writer.close()
            partial.replace(self.root)
            succeeded = True
        finally:
            if not succeeded:
                shutil.rmtree(partial, ignore_errors=True)
        return self.root
