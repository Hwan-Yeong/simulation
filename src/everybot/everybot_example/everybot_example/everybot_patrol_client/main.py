#!/usr/bin/env python3

import rclpy

from everybot_example.everybot_patrol_client.everybot_patrol_client \
    import EverybotPatrolClient


def main(args=None):
    rclpy.init(args=args)
    everybot_patrol_client = EverybotPatrolClient()
    rclpy.spin(everybot_patrol_client)


if __name__ == '__main__':
    main()
