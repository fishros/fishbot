FROM fishros2/ros:humble-desktop

COPY src/fishbot_laser_driver /workspace/src/
WORKDIR /workspace/

RUN apt update && \
    apt install ros-humble-cartographer-ros -y

RUN colcon build && \
    echo "source /opt/ros/humble/setup.bash" > /root/.bashrc && \
    echo "source /workspace/install/setup.bash" > /root/.bashrc

RUN echo "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch fishbot_laser_driver cartographer_pure_laser.launch.py"   > /scripts/start_slam.sh

RUN echo '#!/bin/bash' > /scripts/startup.sh && \
    echo 'echo "欢迎鱼香ROS使用雷达驱动系统，注意当前版本为ROS2版本驱动哦~"' >> /scripts/startup.sh && \
    echo 'select i in 驱动雷达 建图测试 退出' >> /scripts/startup.sh && \
    echo 'do' >> /scripts/startup.sh && \
    echo '    case $i in' >> /scripts/startup.sh && \
    echo '        驱动雷达)' >> /scripts/startup.sh && \
    echo '        source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run fishbot_laser_driver laser_x2' >> /scripts/startup.sh && \
    echo '        ;;' >> /scripts/startup.sh && \
    echo '        建图测试)' >> /scripts/startup.sh && \
    echo '        source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch fishbot_laser_driver cartographer_pure_laser.launch.py' >> /scripts/startup.sh && \
    echo '        ;;' >> /scripts/startup.sh && \
    echo '        退出)' >> /scripts/startup.sh && \
    echo '        echo "欢迎关注公众号鱼香ROS～Bay～"' >> /scripts/startup.sh && \
    echo '        exit' >> /scripts/startup.sh && \
    echo '        ;;' >> /scripts/startup.sh && \
    echo '    esac'>> /scripts/startup.sh && \
    echo 'done' >> /scripts/startup.sh



# xhost + && docker run  -it --rm  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e DISPLAY=unix$DISPLAY -p 8888:8888 fishros2/fishbot_laser