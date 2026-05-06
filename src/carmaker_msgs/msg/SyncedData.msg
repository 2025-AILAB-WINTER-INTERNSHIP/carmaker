std_msgs/Header header

# Diagnostics & Fault Flags
bool is_valid                 # If false, system is in recovery/heartbeat/time-jump mode.

# CarMaker Specific Payloads
carmaker_msgs/SensorStatus[] sensors # List of non-camera sensors and their sync status
carmaker_msgs/DynamicsInfo   dynamic_info
carmaker_msgs/Objects        objects
carmaker_msgs/UAQ_Out        uaq_out

carmaker_msgs/CameraData[]   cameras  # List of cameras and their sync status
