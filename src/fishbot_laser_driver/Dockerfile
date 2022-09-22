FROM fishros2/ros:humble-desktop

COPY src/fishbot_laser_driver /workspace/src/
WORKDIR /workspace/

RUN apt update && \
    apt install ros-humble-cartographer-ros -y

RUN colcon build