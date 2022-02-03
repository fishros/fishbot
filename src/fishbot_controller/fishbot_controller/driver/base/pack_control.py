# -*- coding: utf-8 -*-
"""
  运动控制板解包和打包数据
  
 |起始(1)  |目标编号(1)| 指令(1) | 数据长度(1) | 数据(N) | 校验位(1) | 帧结束(1)|
 | 0X7D |  0X01（运动控制板）|  0X01（上报电机速度） | 0X04（左右电机四个字节）|0X00 0XC8 0X00 0XC8 |   0X90|0X7E |
 如数据为为空，一个数据帧的最短长度为：1+1+1+2+0+1+1=6
"""

import struct
import math
import time

import transforms3d as tfs
import tf2_ros
import rclpy

from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from .base_frame import BaseFrame

class PackControl():
  def __init__(self,base_control,target=0x01) -> None:
    self.target = target  # 运动控制板
    self.parser = {}

    self.tf_br = tf2_ros.TransformBroadcaster(base_control)

    self.base_control = base_control
    self.server = base_control.server
    self.__init_parser()

    self.odom_pose_  = [0,0,0]
    self.last_time = time.time()
    

  def __init_parser(self):
    # add speed command parse
    self.parser[0x01] = self.parse_speed
    self.parser[0x02] = self.parse_imu

  def send_pid(self,pid):
    pass

  def send_sppeds(self,speeds):
    data_len = len(speeds)*2
    data = struct.pack("hh",speeds[0],speeds[1])
    base_frame =  BaseFrame.GenBaseFrame(self.target,0x01,data_len,data)
    self.server.send_frame(base_frame.getFrame())


  def handle_frame(self,base_frame:BaseFrame):
    if base_frame.command in self.parser.keys():
      self.parser[base_frame.command](base_frame)
    else:
      print("no parser error",base_frame.target,base_frame.command)


  def parse_speed(self,base_frame:BaseFrame):
    '''
    描述: 解析速度
    返回值: 无
    param {BaseFrame} frame
    '''    
    speed_left = struct.unpack('h',base_frame.data[0:2])[0]/1000.0
    speed_right = struct.unpack('h',base_frame.data[2:4])[0]/1000.0

    angular_speed = (speed_right-speed_left)/self.base_control.wheel_seperation_
    linear_speed = (speed_right+speed_left)/2.0

    self.update_odom(linear_speed,angular_speed)


  def update_odom(self,v,w):
    self.time_step = time.time()-self.last_time
    self.odom_pose_[2] += self.time_step*w
   
    if self.odom_pose_[2]>math.pi:
      self.odom_pose_[2] -= 2*math.pi

    self.odom_pose_[0] += self.time_step*v*math.cos(self.odom_pose_[2])
    self.odom_pose_[1] += self.time_step*v*math.sin(self.odom_pose_[2])
    self.last_time = time.time()


    translation = (self.odom_pose_[0],self.odom_pose_[1],0.0)
    orientation = tfs.euler.euler2quat(0,0,self.odom_pose_[2],"sxyz")
    
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = 'base_link'
    odom.header.stamp = self.base_control.get_clock().now().to_msg()

    odom.pose.pose.position.x = translation[0]
    odom.pose.pose.position.y = translation[1]
    odom.pose.pose.position.z = translation[2]

    odom.pose.pose.orientation.w = orientation[0]
    odom.pose.pose.orientation.x = orientation[1]
    odom.pose.pose.orientation.y = orientation[2]
    odom.pose.pose.orientation.z = orientation[3]

    odom.twist.twist.angular.z = w
    odom.twist.twist.linear.x = v
    
    self.base_control.odom_pub.publish(odom)
    # 发布TF变换
    self.send_transform(self.base_control.get_clock().now().to_msg(),
                        translation=translation,
                        rotation=orientation,
                        parent="odom",
                        child="base_link")

    # print('odom',self.odom_pose_)


    # wheel_l, wheel_r = 0.0 ,0.0 # rotation value of wheel [rad]
    # delta_s, delta_theta= 0.0 ,0.0 
    # v ,w   = [0.0,0.0],[0.0,0.0]
    # # step_time = duration.nanoseconds() / 1e9; 


    # # // v = translational velocity [m/s]
    # # // w = rotational velocity [rad/s]
    # v[self.LEFT] = self.wheel_speed_cmd_[self.LEFT] 
    # w[self.LEFT] = v[self.LEFT] / self.wheel_radius_  #;  // w = v / r
    # v[self.RIGHT] = self.wheel_speed_cmd_[self.RIGHT]
    # w[self.RIGHT] = v[self.RIGHT] / self.wheel_radius_

    # self.last_velocity_[self.LEFT] = w[self.LEFT];
    # self.last_velocity_[self.RIGHT] = w[self.RIGHT];

    # wheel_l = w[self.LEFT] * self.step_time;
    # wheel_r = w[self.RIGHT] * self.step_time;


    # self.last_position_[self.LEFT] += wheel_l;
    # self.last_position_[self.RIGHT] += wheel_r;

    # delta_s = self.wheel_radius_ * (wheel_r + wheel_l) / 2.0
    # delta_theta = self.wheel_radius_ * (wheel_r - wheel_l) / self.wheel_seperation_

    # # // compute odometric pose
    # self.odom_pose_[0] += delta_s * math.cos(self.odom_pose_[2] + (delta_theta / 2.0));
    # self.odom_pose_[1] += delta_s * math.sin(self.odom_pose_[2] + (delta_theta / 2.0));
    # self.odom_pose_[2] += delta_theta;

    # # // compute odometric instantaneouse velocity
    # self.odom_vel_[0] = delta_s / self.step_time;    # // v
    # self.odom_vel_[1] = 0.0;
    # self.odom_vel_[2] = delta_theta / self.step_time; # // w

    # self.odom_.pose.pose.position.x = self.odom_pose_[0]
    # self.odom_.pose.pose.position.y = self.odom_pose_[1]
    # self.odom_.pose.pose.position.z = 0

    # # tf2::Quaternion q;
    # # q.setRPY(0, 0, odom_pose_[2]);
    quat = tfs.euler.euler2quat(0,0,self.odom_pose_[2],"sxyz")

    # self.odom_.pose.pose.orientation.w = quat[0]
    # self.odom_.pose.pose.orientation.x = quat[1]
    # self.odom_.pose.pose.orientation.y = quat[2]
    # self.odom_.pose.pose.orientation.z = quat[3]

    # # // We should update the twist of the odometry
    # self.odom_.twist.twist.linear.x = self.odom_vel_[0]
    # self.odom_.twist.twist.angular.z = self.odom_vel_[2]

  def parse_imu(self,base_frame:BaseFrame):
    """
    IMU数据解析：0x02
    """
    if len(base_frame.data)<18: return    
    # print(base_frame.data_len)
    data = []
    for i in range(0,18,2):
      data.append(struct.unpack('h',base_frame.data[i:i+2])[0]/32768)

    acc = [data[i]*16 for  i in range(3)]
    gy = [data[i]*2000 for  i in range(3,6)]
    angle = [math.radians(data[i]*180) for  i in range(6,9)]
    print("IMU:",angle[2])
    # 发布imu数据
    imu_data = Imu()
    imu_data.header.frame_id = "imu_link"
    imu_data.header.stamp = self.base_control.get_clock().now().to_msg()
    imu_data.linear_acceleration.x = acc[0]
    imu_data.linear_acceleration.y = acc[1]
    imu_data.linear_acceleration.z = acc[2]
    imu_data.angular_velocity.x = gy[0]
    imu_data.angular_velocity.y = gy[1]
    imu_data.angular_velocity.z = gy[2]
    quat = tfs.euler.euler2quat(angle[0],angle[1],angle[2],"sxyz")
    imu_data.orientation.w = quat[0]
    imu_data.orientation.x = quat[1]
    imu_data.orientation.y = quat[2]
    imu_data.orientation.z = quat[3]
    self.base_control.imu_pub.publish(imu_data)
    # 发布TF变换
    # self.send_transform(self.base_control.get_clock().now().to_msg(),translation=(float(self.odom_pose_[0]),float(self.odom_pose_[1]),0.05),rotation=quat,parent="odom",child="imu_link")
    # self.send_transform(self.base_control.get_clock().now().to_msg(),translation=(float(self.odom_pose_[0]),float(self.odom_pose_[1]),0.05),rotation=,parent="base_link",child="imu_link")


  def send_transform(self,stamp,translation=(0,0,0),rotation=(0,0,0,1),parent="world",child="imu"):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.header.stamp = stamp 
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.w = rotation[0]
    transform.transform.rotation.x = rotation[1]
    transform.transform.rotation.y = rotation[2]
    transform.transform.rotation.z = rotation[3]
    self.tf_br.sendTransform(transform)
  