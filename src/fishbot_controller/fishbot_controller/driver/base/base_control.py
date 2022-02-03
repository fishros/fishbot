# -*- coding: utf-8 -*-
import math
from queue import Queue
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from .udp_server import UdpServer
from .base_frame import BaseFrame
from .pack_control import PackControl
import transforms3d as tfs


class BaseControl(Node):
    # 运动控制板
    MOTION_CONTROL = 0x01
    LEFT = 0
    RIGHT = 1

    def __init__(self) -> None:
        super().__init__("base_control")
        self.queue_frame = Queue() #frame queue
        self.handler = {}
        self.speed = 0
        self.init()


    def init(self):
        """
        初始化函数：
        """
        # 1.初始化里程计Odom:Publisher
        self.odom_pub = self.create_publisher(Odometry,"odom",10)
        # 2.初始化IMU:Publisher
        self.imu_pub = self.create_publisher(Imu,"imu",10)
        # 3.初始化参数PID
        self.declare_parameter("fishbot_pid_p",1.2)
        self.declare_parameter("fishbot_pid_i",1.0)
        self.declare_parameter("fishbot_pid_d",0.001)
        # 4.初始化数据帧服务
        self.server =  UdpServer(queue=self.queue_frame,port=3334)
        self.handler[BaseControl.MOTION_CONTROL] = PackControl(self)
        # 5.订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,10)
        self.wheel_speed_cmd_ = [0,0] #左右轮速度
        self.wheel_seperation_ =  0.1543 # 两个轮子的间距
        # start thread
        self.handle_thread = threading.Thread(target=self.handleSpin,name="handle_spin")
        self.handle_thread.start()


    def cmd_vel_callback(self,cmd_vel_msg):
        # print(cmd_vel)
        self.last_cmd_vel_time_ = self.get_clock().now()
        self.goal_linear_velocity_ = cmd_vel_msg.linear.x
        self.goal_angular_velocity_ = cmd_vel_msg.angular.z
        self.wheel_speed_cmd_[self.LEFT] = int(
                                            (self.goal_linear_velocity_ - (self.goal_angular_velocity_ * self.wheel_seperation_ / 2.0))
                                            *1000
                                            )
        self.wheel_speed_cmd_[self.RIGHT] = int(
                            (self.goal_linear_velocity_ + (self.goal_angular_velocity_ * self.wheel_seperation_ / 2.0))
                            *1000)
    
    def update_odom(self):
        wheel_l, wheel_r = 0.0 ,0.0 # rotation value of wheel [rad]
        delta_s, delta_theta= 0.0 ,0.0 
        v ,w   = [0.0,0.0],[0.0,0.0]
        # step_time = duration.nanoseconds() / 1e9; 


        # // v = translational velocity [m/s]
        # // w = rotational velocity [rad/s]
        v[self.LEFT] = self.wheel_speed_cmd_[self.LEFT] 
        w[self.LEFT] = v[self.LEFT] / self.wheel_radius_  #;  // w = v / r
        v[self.RIGHT] = self.wheel_speed_cmd_[self.RIGHT]
        w[self.RIGHT] = v[self.RIGHT] / self.wheel_radius_

        self.last_velocity_[self.LEFT] = w[self.LEFT];
        self.last_velocity_[self.RIGHT] = w[self.RIGHT];

        wheel_l = w[self.LEFT] * self.step_time;
        wheel_r = w[self.RIGHT] * self.step_time;


        self.last_position_[self.LEFT] += wheel_l;
        self.last_position_[self.RIGHT] += wheel_r;

        delta_s = self.wheel_radius_ * (wheel_r + wheel_l) / 2.0
        delta_theta = self.wheel_radius_ * (wheel_r - wheel_l) / self.wheel_seperation_

        # // compute odometric pose
        self.odom_pose_[0] += delta_s * math.cos(self.odom_pose_[2] + (delta_theta / 2.0));
        self.odom_pose_[1] += delta_s * math.sin(self.odom_pose_[2] + (delta_theta / 2.0));
        self.odom_pose_[2] += delta_theta;

        # // compute odometric instantaneouse velocity
        self.odom_vel_[0] = delta_s / self.step_time;    # // v
        self.odom_vel_[1] = 0.0;
        self.odom_vel_[2] = delta_theta / self.step_time; # // w

        self.odom_.pose.pose.position.x = self.odom_pose_[0]
        self.odom_.pose.pose.position.y = self.odom_pose_[1]
        self.odom_.pose.pose.position.z = 0

        # tf2::Quaternion q;
        # q.setRPY(0, 0, odom_pose_[2]);
        quat = tfs.euler.euler2quat(0,0,self.odom_pose_[2],"sxyz")

        self.odom_.pose.pose.orientation.w = quat[0]
        self.odom_.pose.pose.orientation.x = quat[1]
        self.odom_.pose.pose.orientation.y = quat[2]
        self.odom_.pose.pose.orientation.z = quat[3]

        # // We should update the twist of the odometry
        self.odom_.twist.twist.linear.x = self.odom_vel_[0]
        self.odom_.twist.twist.angular.z = self.odom_vel_[2]

        

    def handleSpin(self):
        """
        数据处理循环
        1.从队列中取出一帧数据，对数据进行校验及存入队列
        """
        while rclpy.ok():
            if not self.queue_frame.qsize()>0: continue
            raw_frame = self.queue_frame.get()
            base_frame = BaseFrame.GenFrame(raw_frame)
            if base_frame == None: continue
            
            self.handler[BaseControl.MOTION_CONTROL].send_sppeds(self.wheel_speed_cmd_)
            print("send target spped ", self.wheel_speed_cmd_)

            # print(base_frame.target,base_frame.command,base_frame.data_len,base_frame.data_len)
            if base_frame.target in self.handler.keys():
                self.handler[base_frame.target].handle_frame(base_frame)
            else:
                print("no target handler ", base_frame.target)