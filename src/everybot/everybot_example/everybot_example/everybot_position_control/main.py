#!/usr/bin/env python3

import rclpy

from everybot_example.everybot_position_control.everybot_position_control \
    import EverybotPositionControl


def main(args=None):
    rclpy.init(args=args)
    everybot_position_control = EverybotPositionControl()
    rclpy.spin(everybot_position_control)

    everybot_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
