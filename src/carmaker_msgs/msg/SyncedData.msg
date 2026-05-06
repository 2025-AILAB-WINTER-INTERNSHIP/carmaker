std_msgs/Header header

# Diagnostics & Fault Flags
bool is_valid                 # If false, system is in recovery/heartbeat/time-jump mode.
string[] sensor_names         # ["front_cam", "rear_cam", "objects", "dynamic_info", "uaq"] : sensor names
bool[]   sensor_status        # [true, true, false, true, true] : sensor valid status

# CarMaker Specific Payloads
sensor_msgs/Image[]        images
carmaker_msgs/DynamicInfo  dynamic_info
carmaker_msgs/Objects      objects
carmaker_msgs/UAQ_Out      uaq_out
