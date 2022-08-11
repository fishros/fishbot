![YDLIDAR](sdk/image/YDLidar.jpg  "YDLIDAR")
# YDLIDAR ROS2 PACKAGE V1.4.5
ROS2 node and test application for YDLIDAR

Visit EAI Website for more details about YDLIDAR.

## How to [install ROS2](https://index.ros.org/doc/ros2/Installation)
[ubuntu](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)

[windows](https://index.ros.org/doc/ros2/Installation/Dashing/Windows-Install-Binary/)

## How to Create a ROS2 workspace
[Create a workspace](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace)

## How to build YDLIDAR ros2 package
    1) Clone this project to your ament's workspace src folder 
    2) Running ament to build ydlidar_node and ydlidar_client
    3) Create the name "/dev/ydlidar" for YDLIDAR
    --$ cd workspace/ydlidar_ros2/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh
Note: Download and Build details [here](docs/ydlidar.md)

## How to run YDLIDAR ros2 package

### 1. Run YDLIDAR node and view using test application
	$ros2 run ydlidar ydlidar_node

	$ros2 run ydlidar ydlidar_client

Note: You should see YDLIDAR's scan result in the console

### 2.Run YDLIDAR node and view using test application by launch
	$launch $(ros2 pkg prefix ydlidar)/share/ydlidar/launch/ydlidar.py

	$ros2 run ydldiar ydlidar_client or ros2 topic echo /scan
or

	$ros2 launch ydlidar ydlidar_launch.py

## Dataset
|LIDAR      | Model  |  Baudrate |  SampleRate(K) | Range(m)  		   |  Frequency(HZ) | Intenstiy(bit) | SingleChannel | voltage(V)|
| :-------- |:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| F4        | 1	   |  115200   |   4            |  0.12~12         | 5~12           | false          | false    	  | 4.8~5.2   |
| S4        | 4	   |  115200   |   4            |  0.10~8.0        | 5~12 (PWM)     | false          | false    	  | 4.8~5.2   |
| S4B       | 4/11   |  153600   |   4            |  0.10~8.0        | 5~12(PWM)      | true(8)        | false    	  | 4.8~5.2   |
| S2        | 4/12   |  115200   |   3            |  0.10~8.0     	| 4~8(PWM)       | false          | true    		  | 4.8~5.2   |
| G4        | 5	   |  230400   |   9/8/4        |  0.28/0.26/0.1~16| 5~12        	  | false          | false    	  | 4.8~5.2   |
| X4        | 6	   |  128000   |   5            |  0.12~10     		| 5~12(PWM)      | false          | false    	  | 4.8~5.2   |
| X2/X2L    | 6	   |  115200   |   3            |  0.10~8.0     	| 4~8(PWM)       | false          | true    		  | 4.8~5.2   |
| G4PRO     | 7	   |  230400   |   9/8/4        |  0.28/0.26/0.1~16| 5~12        	  | false          | false    	  | 4.8~5.2   |
| F4PRO     | 8	   |  230400   |   4/6          |  0.12~12         | 5~12        	  | false          | false    	  | 4.8~5.2   |
| R2        | 9	   |  230400   |   5            |  0.12~16     		| 5~12        	  | false          | false    	  | 4.8~5.2   |
| G6        | 13     |  512000   |   18/16/8      |  0.28/0.26/0.1~25| 5~12        	  | false          | false    	  | 4.8~5.2   |
| G2A       | 14	   |  230400   |   5            |  0.12~12         | 5~12      	  | false          | false    	  | 4.8~5.2   |
| G2        | 15		|  230400   |   5            |  0.28~16     		| 5~12      	  | true(8)        | false    	  | 4.8~5.2   |
| G2C       | 16		|  115200   |   4            |  0.1~12        	| 5~12      	  | false      	 | false    	  | 4.8~5.2   |
| G4B       | 17		|  512000   |   10           |  0.12~16         | 5~12        	  | true(10)       | false    	  | 4.8~5.2   |
| G4C       | 18		|  115200   |   4            |  0.1~12		      | 5~12           | false          | false    	  | 4.8~5.2   |
| G1        | 19		|  230400   |   9            |  0.28~16         | 5~12      	  | false          | false    	  | 4.8~5.2   |
| TX8    　 | 100	   |  115200   |   4            |  0.1~8      	   | 4~8(PWM)       | false          | true      	  | 4.8~5.2   |
| TX20    　| 100	   |  115200   |   4            |  0.1~20      	   | 4~8(PWM)       | false          | true     	  | 4.8~5.2   |
| TG15    　| 100	   |  512000   |   20/18/10     |  0.05~15      	| 3~16      	  | false          | false    	  | 4.8~5.2   |
| TG30    　| 101	   |  512000   |   20/18/10     |  0.05~30      	| 3~16      	  | false          | false    	  | 4.8~5.2   |
| TG50    　| 102	   |  512000   |   20/18/10     |  0.05~50      	| 3~16      	  | false          | false    	  | 4.8~5.2   |

   Note: PWM option speed control requires external PWM wave.

## Configuration
path: [ydlidar.yaml](params/ydlidar.yaml)

## ros2-interfaces

<center>

| Topic                | Type                    | Description                                      |
|----------------------|-------------------------|--------------------------------------------------|
| `scan`               | sensor_msgs/LaserScan   | 2D laser scan of the 0-angle ring                |

| Parameter         | Type                    | Description                                         |
|-----------------------|------------------------|-----------------------------------------------------|
| `port`        		| String                 	| port of lidar (ex. /dev/ttyUSB0)                         		|
| `baudrate`     	| int                      	| baudrate of lidar (ex. 230400)           				|
| `frame_id`      	| String                	| TF frame of sensor, default: `laser_frame`    		|
| `singleChannel`  	| bool                     	| Whether LiDAR is a single-channel, default: false	|
| `resolution_fixed` | bool                     	| Fixed angluar resolution, default: true                    	|
| `auto_reconnect` | bool                  	| Automatically reconnect the LiDAR, default: true    	|
| `reversion`     	| bool                  	| Reversion LiDAR, default: true  					|
| `isToFLidar`       	| bool                  	| Whether LiDAR is TOF Type, default: false  		|
| `angle_min`       	| float                 	| Minimum Valid Angle, defalut: -180.0     			|
| `angle_max`       	| float                  	| Maximum Valid Angle, defalut: 180.0      			|
| `range_min`       	| float                  	| Minimum Valid range, defalut: 0.01m      			|
| `range_max`       	| float                  	| Maximum Valid range, defalut: 64.0m      			|
| `ignore_array`      | String                  	| LiDAR filtering angle area, default: ""      			|
| `samp_rate`       	| int                  	| sampling rate of lidar, default: 9      				|
| `frequency`       	| float                  	| scan frequency of lidar,default: 10.0      			|

</center>

## Parameters
port (string, default: /dev/ydlidar)

    serial port name used in your system. 

baudrate (int, default: 230400)

    serial port baud rate. 
    
| LiDAR                					| baudrate               | 
|-----------------------------------------------|-----------------------|
|F4/S2/X2/X2L/S4/TX8/TX20/G4C 		| 115200			|
|X4                   					| 128000			|
|S4B                         				| 153600			|
|G1/G2/R2/G4/G4PRO/F4PRO         	| 230400			|
|G6/TG15/TG30/TG50			 	| 512000			|

frame_id (string, default: laser_frame)

    frame ID for the device. 

singleChannel (bool, default: false)

    indicated whether the LIDAR is single communication(S2) lidar.
    
| LiDAR                							| singleChannel       | 
|-----------------------------------------------------------|-----------------------|
|G1/G2/G4/G6/F4/F4PRO/S4/S4B/X4/R2/G4C 	| false			|
|S2/X2/X2L                   						| true			|
|TG15/TG30/TG50                         				| false			|
|TX8/TX20         							| true			|

resolution_fixed (bool, default: true)

    indicated whether the LIDAR has a fixed angular resolution. 

auto_reconnect (bool, default: true)

    indicated whether the LIDAR auto reconnection. 

reversion (bool, default: false)

    indicated whether the LIDAR data rotation 180°. 
    
| LiDAR                								| reversion              | 
|-----------------------------------------------------------------|-----------------------|
|G1/G2/G4/G6/F4/F4PRO//R2/G4C/TG15/TG30/TG50 	| true			|
|S2/X2/X2L/S4/S4B/X4/TX8/TX20                   			| false			|


isToFLidar (bool, default: false)

    indicated whether the LIDAR is TOF(TX8) lidar. 
    
| LiDAR                									| isToFLidar             | 
|-----------------------------------------------------------------------|-----------------------|
|G1/G2/G4/G6/F4/F4PRO/S4/S4B/X4/R2/G4C/S2/X2/X2L 	| false			|
|TG15/TG30/TG50/TX8/TX20                   				| true			|


angle_min (double, default: -180)

    Min valid angle (°) for LIDAR data. 

angle_max (double, default: 180)

    Max valid angle (°) for LIDAR data. 

range_min (double, default: 0.08)

    Min valid range (m) for LIDAR data. 

range_max (double, default: 32.0)

    Max valid range (m) for LIDAR data. 

ignore_array (string, default: "")

    Set the current angle range value to zero.
    
    Note: ignore 10 to 20 and 50 to 80, ex: "10, 20, 50, 80" 

samp_rate (int, default: 9)

    the LIDAR sampling frequency.
    
| LiDAR                		| samp_rate             | 
|-----------------------------|------------------------|
|G4/F4                    		| 4,8,9			 |
|F4PRO                   		| 4,6   			 |
|G6                         		| 8,16,18			 |
|G1/G2/R2/X4         		| 5				 |
|S4/S4B/G4C/TX8/TX20 	|4			 	 |
|S2                    		| 3			 	 |
|TG15/TG30/TG50           | 10,18,20		 |


frequency (double, default: 10)

    the LIDAR scanning frequency.


Note: Specific LiDAR paramter configuration, refer to [Dataset](#dataset)




## Upgrade Log

2020-02-08 version:1.4.5

   1.Update SDK to 1.4.5

   2.fixed ROS2 Dashing and Eloquent.

2019-12-23 version:1.4.4

   1.support all standards lidar


2018-07-16 version:1.3.6

   1.Update SDK verison to 1.3.9
 
   2.remove imu sync.

2018-07-16 version:1.3.5

   1.Update SDK verison to 1.3.6

   2.add imu sync.

2018-04-16 version:1.3.1

   1.Update SDK verison to 1.3.1

   2.Increase sampling frequency,scan frequency setting.

   3.Unified coordinate system.

   4.Repair X4,S4 LIDAR cannot be opened.

   5.Increased G4 G4C F4Pro LIDAR power-off protection.

   6.Increased S4B LIDAR low optical power setting.

   7.Fix the wait time for closing ros node.
   
   8.Compensate for each laser point timestamp.

   9.Unified profile, automatic correction lidar model.

# 6 Support

You can get support from YDLidar with the following methods:
* Send email to support@ydlidar.com with a clear description of your problem and your setup
* Github Issues

## Contact EAI

![Development Path](sdk/image/EAI.png)

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)






