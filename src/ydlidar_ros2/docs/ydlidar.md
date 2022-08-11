#　YDLIDAR ROS2 Package Download and Build
##　Step1: create a ROS2 workspace, if there is no workspace, othereise Skip to Step2
####　Linux/OS X
	$mkdir -p ~/ydlidar_ros2_ws/src
	$cd ~/ydlidar_ros2_ws/src
####　Windows
	$md \dev\ydlidar_ros2_ws\src
	$cd \dev\ydlidar_ros2_ws\src
	
##　Step2: clone ydlidar ros2 package
	$git clone https://github.com/YDLIDAR/ydlidar_ros2
	
##　Step3: Build [ydlidar_ros2](https://github.com/YDLIDAR/ydlidar_ros2) package
	$cd ..
	$colcon build --symlink-install
Note: install colcon [see](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon)

##　Step4:Configure LiDAR [paramters](params/ydlidar.yaml)

	ydlidar_node:
  		ros__parameters:
  			port: /dev/ttyUSB0
  			frame_id: laser_frame
   			ignore_array: ""
    			baudrate: 230400
    			samp_rate: 9
    			resolution_fixed: true
    			singleChannel: false    
    			auto_reconnect: true
    			reversion: true
    			isToFLidar: false
    			angle_max: 180.0
    			angle_min: -180.0
    			max_range: 16.0    
    			min_range: 0.01
    			frequency: 10.0

Note: How to configure paramters, see [here](paramters.md)

## Step5: Create serial port Alias[/dev/ydlidar] 
	$chmod 0777 src/ydlidar_ros2/startup/*
	$sudo sh src/ydlidar_ros2/startup/initenv.sh
Note: After completing the previous operation, replug the LiDAR again.
  
##　Step6:Run ydlidar_ros2 node
	$ros2 run ydlidar ydlidar_node

	$ros2 run ydlidar ydlidar_client
or

	$ros2 launch ydlidar ydlidar_launch.py
	
	$ros2 run ydldiar ydlidar_client
	 
or 

	$ros2 topic echo /scan

