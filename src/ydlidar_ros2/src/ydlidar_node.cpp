/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "CYdLidar.h"
#include "timer.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.4.5"


using namespace ydlidar;

std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;

  while (std::getline(ss, number, delim)) {
    elems.push_back(atoi(number.c_str()));
  }

  return elems;
}



bool fileExists(const std::string filename) {
  return 0 == _access(filename.c_str(), 0x00);  // 0x00 = Check for existence only!
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_node");
  std::string port = "/dev/ttyUSB0";
  int baudrate = 230400;
  std::string frame_id = "laser_frame";
  bool reversion = true;
  bool resolution_fixed = true;
  bool auto_reconnect = true;
  double angle_max = 180;
  double angle_min = -180;
  int samp_rate = 9;
  std::string list = "";
  double max_range = 64.0;
  double min_range = 0.01;
  double frequency = 10.0;
  bool m_singleChannel = false;
  bool m_isToFLidar = false;
  bool m_Inverted = true;

  node->declare_parameter<std::string>("port");
  node->get_parameter("port", port);

  node->declare_parameter<std::string>("frame_id");
  node->get_parameter("frame_id", frame_id);

  node->declare_parameter<std::string>("ignore_array");
  node->get_parameter("ignore_array", list);

  node->declare_parameter<int>("baudrate");
  node->get_parameter("baudrate", baudrate);

  node->declare_parameter<int>("samp_rate");
  node->get_parameter("samp_rate", samp_rate);

  node->declare_parameter<bool>("resolution_fixed");
  node->get_parameter("resolution_fixed", resolution_fixed);

  node->declare_parameter<bool>("singleChannel");
  node->get_parameter("singleChannel", m_singleChannel);

  node->declare_parameter<bool>("auto_reconnect");
  node->get_parameter("auto_reconnect", auto_reconnect);

  node->declare_parameter<bool>("reversion");
  node->get_parameter("reversion", reversion);

  node->declare_parameter<bool>("isToFLidar");
  node->get_parameter("isToFLidar", m_isToFLidar);

  node->declare_parameter<double>("angle_max");
  node->get_parameter("angle_max", angle_max);

  node->declare_parameter<double>("angle_min");
  node->get_parameter("angle_min", angle_min);

  node->declare_parameter<double>("max_range");
  node->get_parameter("max_range", max_range);

  node->declare_parameter<double>("min_range");
  node->get_parameter("min_range", min_range);

  node->declare_parameter<double>("frequency");
  node->get_parameter("frequency", frequency);




  std::vector<float> ignore_array = split(list, ',');

  if (ignore_array.size() % 2) {
    RCLCPP_ERROR(node->get_logger(), "ignore array is odd need be even");
  }

  for (uint16_t i = 0 ; i < ignore_array.size(); i++) {
    if (ignore_array[i] < -180 && ignore_array[i] > 180) {
      RCLCPP_ERROR(node->get_logger(), "ignore array should be between -180 and 180");
    }
  }



  CYdLidar laser;

  if (frequency < 3) {
    frequency = 7.0;
  }

  if (frequency > 16) {
    frequency = 16;
  }

  if (angle_max < angle_min) {
    double temp = angle_max;
    angle_max = angle_min;
    angle_min = temp;
  }

  laser.setSerialPort(port);
  laser.setSerialBaudrate(baudrate);
  laser.setInverted(m_Inverted);
  laser.setMaxRange(max_range);
  laser.setMinRange(min_range);
  laser.setMaxAngle(angle_max);
  laser.setMinAngle(angle_min);
  laser.setReversion(reversion);
  laser.setFixedResolution(resolution_fixed);
  laser.setAutoReconnect(auto_reconnect);
  laser.setSingleChannel(m_singleChannel);
  laser.setScanFrequency(frequency);
  laser.setSampleRate(samp_rate);
  laser.setLidarType(m_isToFLidar ? TYPE_TOF : TYPE_TRIANGLE);
  laser.setIgnoreArray(ignore_array);


  printf("[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());
  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  }

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());



  rclcpp::WallRate loop_rate(8);

  while (ret && rclcpp::ok()) {

    bool hardError;
    LaserScan scan;//

    if (laser.doProcessSimple(scan, hardError)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);
      for(int i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(*scan_msg);


    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
  fflush(stdout);
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
