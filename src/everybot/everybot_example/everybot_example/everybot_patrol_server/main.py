#!/usr/bin/env python3

import rclpy

from everybot_example.everybot_patrol_server.everybot_patrol_server \
    import EverybotPatrolServer


def main(args=None):
    rclpy.init(args=args)
    everybot_patrol_server = EverybotPatrolServer()
    rclpy.spin(everybot_patrol_server)

    everybot_patrol_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
