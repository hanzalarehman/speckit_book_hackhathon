# SensorData.msg
std_msgs/Header header
float32[] lidar_ranges
float32[] depth_image_data
float32[3] imu_angular_velocity
float32[3] imu_linear_acceleration
float32[4] imu_orientation