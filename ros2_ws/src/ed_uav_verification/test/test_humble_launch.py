from __future__ import annotations

import pytest


launch_testing = pytest.importorskip("launch_testing")
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    """Given the ROS harness, when launched, then it exits after virtual replay."""
    harness = Node(
        package="ed_uav_verification",
        executable="ed-uav-verify-ros",
        arguments=["--seed", "19", "--duration-seconds", "1", "--rate-hz", "20"],
        output="screen",
    )
    return LaunchDescription([harness, ReadyToTest()]), {"harness": harness}


class TestVerificationHarnessLaunch:
    def test_virtual_replay_completes(self, proc_output, harness) -> None:
        """Given an active launch, when virtual replay ends, then it reports green."""
        proc_output.assertWaitFor("ROS SCENARIO: GREEN", process=harness, timeout=15)


@launch_testing.post_shutdown_test()
class TestVerificationHarnessShutdown:
    def test_process_exits_cleanly(self, proc_info, harness) -> None:
        """Given a completed launch, when it shuts down, then the harness exits zero."""
        launch_testing.asserts.assertExitCodes(proc_info)
