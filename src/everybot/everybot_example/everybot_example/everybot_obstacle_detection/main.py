#!/usr/bin/env python3

import rclpy

from everybot_example.everybot_obstacle_detection.everybot_obstacle_detection \
    import EverybotObstacleDetection


def main(args=None):
    rclpy.init(args=args)
    everybot_obstacle_detection = EverybotObstacleDetection()
    rclpy.spin(everybot_obstacle_detection)

    everybot_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
