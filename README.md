<!--
 * @作者: 小鱼
 * @公众号: 鱼香ROS
 * @QQ交流群: 2642868461
 * @描述: README
-->
# fishbot

## 安装依赖
首先你需要一个安装好ROS2的环境，如果没有请访问fishros.com使用一键安装进行安装。
接着打开终端运行下面的指令。

```
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-joint-state-publisher
```

## 下载编译


```
git clone https://github.com/fishros/fishbot.git
cd fishbot
colcon build
```


## 运行测试

### 在RVIZ中显示机器人模型

```
source install/setup.bash
ros2 launch fishbot_description display_rviz2.launch.py
```



