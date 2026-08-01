"""Process lifecycle for the vehicle bridge ROS node.

Daemon contract: the ground-station communication module must NEVER exit on
errors.  Any unexpected exception — including node construction failure
(UDP bind, HMAC key file, ROS graph) — is logged with a full traceback, the
node is rebuilt, and spinning resumes after a short backoff.  Only SIGINT /
SIGTERM / rclpy shutdown stop the process.
"""

from __future__ import annotations

import sys
import time
import traceback

import rclpy
from rclpy.executors import ExternalShutdownException

from .node import VehicleBridgeNode

_RESPAWN_BACKOFF_SECONDS = 1.0
_RESPAWN_BACKOFF_MAX_SECONDS = 30.0


def _respawn_sleep(backoff: float) -> float:
    time.sleep(backoff)
    return min(backoff * 2.0, _RESPAWN_BACKOFF_MAX_SECONDS)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    backoff = _RESPAWN_BACKOFF_SECONDS
    while True:
        node = None
        try:
            node = VehicleBridgeNode()
            rclpy.spin(node)
            # spin() returned cleanly (SIGINT / SIGTERM / shutdown) — exit.
            return
        except (KeyboardInterrupt, ExternalShutdownException):
            return
        except Exception:  # noqa: BLE001 - daemon contract: never exit on errors
            message = (
                f"vehicle_bridge 异常退出, {backoff:.1f}s 后重建节点"
                "(守护契约: 进程不退出)"
            )
            if node is not None:
                node.get_logger().error(message, exc_info=True)
            else:
                print(message, file=sys.stderr)
                traceback.print_exc()
            backoff = _respawn_sleep(backoff)
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:  # noqa: BLE001
                    pass
            if rclpy.ok():
                rclpy.shutdown()
        # Re-initialize a fresh rclpy context for the next node incarnation.
        try:
            rclpy.init(args=args)
        except Exception:  # noqa: BLE001
            print("rclpy 重新初始化失败, 重试", file=sys.stderr)
            backoff = _respawn_sleep(backoff)
            continue
        backoff = _RESPAWN_BACKOFF_SECONDS


if __name__ == "__main__":
    main()
