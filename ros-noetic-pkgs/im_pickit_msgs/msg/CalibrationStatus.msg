# Copyright 2021, Pick-It NV.
# All rights reserved.

# This message contains Calibration status information  ##
uint8 REQUIRED_POSES_MULTI_POSE = 5

uint8 STATUS_IDLE = 1
uint8 STATUS_COLLECTING_POSE = 2
uint8 STATUS_CALIBRATION_CALCULATED_SUCCESS = 3
uint8 STATUS_CALIBRATION_CALCULATED_WARNING = 4
uint8 STATUS_CALIBRATION_FAILURE = 5

uint8 status
string status_message
uint8 n_poses  # Number of collected poses. Only relevant for multi pose calibration.
uint8 position_score
uint8 tilt_score
uint8 yaw_score
