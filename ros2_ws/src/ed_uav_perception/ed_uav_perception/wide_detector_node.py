"""广角相机检测节点入口"""

from ed_uav_perception.single_camera_detector_node import main


def main_wide(args=None):
    """广角相机检测节点主函数"""
    main(args=args, camera_role='wide')


if __name__ == '__main__':
    main_wide()
