#　ROS2 Paramters Table

##　Paramters Description
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

##　Baudrate Table

| LiDAR                					| baudrate               | 
|-----------------------------------------------|-----------------------|
|F4/S2/X2/X2L/S4/TX8/TX20/G4C 		| 115200			|
|X4                   					| 128000			|
|S4B                         				| 153600			|
|G1/G2/R2/G4/G4PRO/F4PRO         	| 230400			|
|G6/TG15/TG30/TG50			 	| 512000			|

## SingleChannel Table

| LiDAR                							| singleChannel       | 
|-----------------------------------------------------------|-----------------------|
|G1/G2/G4/G6/F4/F4PRO/S4/S4B/X4/R2/G4C 	| false			|
|S2/X2/X2L                   						| true			|
|TG15/TG30/TG50                         				| false			|
|TX8/TX20         							| true			|

##isToFLidar Table

| LiDAR                									| isToFLidar             | 
|-----------------------------------------------------------------------|-----------------------|
|G1/G2/G4/G6/F4/F4PRO/S4/S4B/X4/R2/G4C/S2/X2/X2L 	| false			|
|TG15/TG30/TG50/TX8/TX20                   				| true			|

## Sampling Rate Table

| LiDAR                		| samp_rate             | 
|-----------------------------|------------------------|
|G4/F4                    		| 4,8,9			 |
|F4PRO                   		| 4,6   			 |
|G6                         		| 8,16,18			 |
|G1/G2/R2/X4         		| 5				 |
|S4/S4B/G4C/TX8/TX20 	| 4			 	 |
|S2                    		| 3			 	 |
|TG15/TG30/TG50           | 10,18,20		 |

##　Frequency Table


| LiDAR                					| frequency             | 
|-----------------------------------------------|------------------------|
|G1/G2/R2/G6/G4/G4PRO/F4/F4PRO	| 5-12Hz			 |
|S4/S4B/S2/TX8/TX20/X4 			| Not Support		 |
|TG15/TG30/TG50           			| 3-16Hz			 |

Note: For unsupported LiDARs, adjusting the scanning frequency requires external access to PWM speed control.
