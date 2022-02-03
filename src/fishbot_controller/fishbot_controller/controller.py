# -*- coding: utf-8 -*-
# from fishbo base_control import BaseControl
import rclpy
from fishbot_controller.driver.base.base_control import BaseControl
from fishbot_controller.driver.laser.laser_driver import LaserDriver
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    base_control_node =  BaseControl()
    laser_node = LaserDriver(type="tcp",tcp_port=8848)
    executor.add_node(base_control_node)
    executor.add_node(laser_node)
    executor.spin()
    rclpy.shutdown()