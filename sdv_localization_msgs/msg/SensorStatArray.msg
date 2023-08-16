#Timestamp of msg
std_msgs/Header header

## This field has fields relevant to the VeetorNav sensor
sdv_localization_msgs/SensorDiag vn_status
bool is_gps_ready
bool is_gps_best_quality

## This field has fields relevant to the Velodyne sensor
sdv_localization_msgs/SensorDiag vl_status


## This field has fields relevant to the Multisense sensor
sdv_localization_msgs/SensorDiag ms_status
