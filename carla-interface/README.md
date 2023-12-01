# CARLA Interface

Nodes to interface with CARLA topics for Shell Autonomous Programming Competition, which provides a clearer way to relay commands between team nodes and the simulation.

The sensor configuration file (`launch/objects.json`) and `main.launch` are set to the configurations used for 2023's competition.

## Topics
The following ROS topics are available within the simulation:

Published topics:
  * /carla/ego_vehicle/collision [carla_msgs/CarlaCollisionEvent]
  * /carla/ego_vehicle/depth_middle/image [sensor_msgs/Image]
  * /carla/ego_vehicle/depth_middle/camera_info [sensor_msgs/CameraInfo]
  * /carla/ego_vehicle/gnss [sensor_msgs/NavSatFix]
  * /carla/ego_vehicle/imu [sensor_msgs/Imu]
  * /carla/ego_vehicle/lane_invasion [carla_msgs/CarlaLaneInvasionEvent]
  * /carla/ego_vehicle/odometry [nav_msgs/Odometry]
  * /carla/ego_vehicle/speedometer [std_msgs/Float32]
  * /carla/ego_vehicle/rgb_front [sensor_imgs/Image]
  * /carla/ego_vehicle/vehicle_status [carla_msgs/CarlaEgoVehicleStatus]
  * /carla/ego_vehicle/vlp16_1 [sensor_msgs/PointCloud2]
  * /clock [rosgraph_msgs/Clock]
  * /rosout [rosgraph_msgs/Log] 5 publishers
  * /rosout_agg [rosgraph_msgs/Log]
  * /tf [tf2_msgs/TFMessage]

  These topics have data from sensors that can be used to observe the environment:
  * /carla/ego_vehicle/depth_middle/image [sensor_msgs/Image]
  * /carla/ego_vehicle/depth_middle/camera_info [sensor_msgs/CameraInfo]
  * /carla/ego_vehicle/gnss [sensor_msgs/NavSatFix]
  * /carla/ego_vehicle/lane_invasion [carla_msgs/CarlaLaneInvasionEvent]
  * /carla/ego_vehicle/imu [sensor_msgs/Imu]
  * /carla/ego_vehicle/odometry [nav_msgs/Odometry]
  * /carla/ego_vehicle/vlp16_1 [sensor_msgs/PointCloud2]


And messages from team code can be published to these topic to control the vehicle:

  *  /brake_command [std_msgs/Float64]
  Valid values range from 0.0 (no brake) to 1.0 (full brake)
  *  /gear_command [std_msgs/String]
  Valid values are "forward" or "reverse"
  *  /handbrake_command [std_msgs/Bool]
  If set to "true", throttle will be ignored
  *  /steering_command [std_msgs/Float64]
  Valid values range from -1.0 (full left) to 1.0 (full right)
  *  /throttle_command [std_msgs/Float64]
  Valid values range from 0.0 (no throttle) to 1.0 (full throttle)