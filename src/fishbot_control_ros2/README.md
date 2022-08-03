# fishbot-control-ros2

 FishBot的Ros2驱动.

## 1.安装依赖

手动安装

安装fish_protocol

```
sudo apt install libboost-dev libgtest-dev
git clone https://gh.api.99988866.xyz/https://github.com/fishros/fish_protocol.git 
cd fish_protocol && mkdir build  && cd build
cmake .. && sudo make install # 将安装到系统库
```
安装fishbot_motion_driver

```
git clone https://gh.api.99988866.xyz/https://github.com/fishros/fishbot-motion-driver.git -b v1.0.0.20220717
cd fishbot-motion-driver && mkdir build  && cd build
cmake .. && sudo make install
```

## 2.编译运行

```
colcon build
```

