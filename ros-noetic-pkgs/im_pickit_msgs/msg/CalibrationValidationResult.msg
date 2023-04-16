# Copyright 2021, Pick-It NV.
# All rights reserved.

# This message contains Calibration status information  ##

uint8 STATUS_NONE = 0     # No validation data available
uint8 STATUS_SUCCESS = 1  # Success.
uint8 STATUS_WARNING = 2  # Success with warning.

uint8 status
# distance and angle in plate frame.
float64 distance
float64 angle
# Validation error in robot frame (robot flange if camera on robot or robot base if fixed camera).
geometry_msgs/TransformStamped transform
