<!--
 * @作者: 小鱼
 * @公众号: 鱼香ROS
 * @QQ交流群: 2642868461
 * @描述: README
-->
# 基于ROS2实现的Fishbot机器人仿真及实体机器人


## 1.介绍

- fishbot_description  fishbot的URDF定义 
- fishbot_controller   fishbot与真实机器人通信节点
- fishbot_cartographer fishbot使用cartographer建图配置
- fishbot_navigation2  fishbot使用navigation2导航配置 
- nav_core             nav2导航部分源码

## 2.安装运行

### 2.1 下载编译


```
git clone https://github.com/fishros/fishbot.git
cd fishbot
colcon build
```

### 2.2 运行测试

#### 在RVIZ中显示机器人模型

```
source install/setup.bash
ros2 launch fishbot_description display_rviz2.launch.py
```



## 作者
- [鱼香ROS](https://fishros.com)-小鱼

欢迎大家关注公众号鱼香ROS,加入技术交流群