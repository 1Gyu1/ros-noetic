# Copyright 2021, Pick-It NV.
# All rights reserved.

## This message contains all objects found by the last Pick-it run ##

###### Return status definitions: ######
uint8 STATUS_FAILURE                                 = 0  # General failure.
uint8 STATUS_SUCCESS                                 = 1
uint8 STATUS_INVALID_LICENSE                         = 2  # At least one required module has an invalid license.
uint8 STATUS_CAPTURE_FAILURE                         = 11
uint8 STATUS_EMPTY_ROI                               = 12
uint8 STATUS_CAPTURE_TIMEOUT_EXCEEDED                = 13
uint8 STATUS_NO_ACTIVE_CAMERA                        = 14

uint8 STATUS_NO_TEACH_MODEL                          = 20
uint8 STATUS_NO_ENABLED_TEACH_MODELS                 = 21
uint8 STATUS_NO_TEACHXD_MODEL                        = 22

uint8 STATUS_COLLISION_CHECK_ERROR                   = 30
uint8 STATUS_TEACHXD_CLUSTERING_FAILURE              = 31

uint8 STATUS_BAGS_PALLET_LOCALISATION_FAILED         = 40
uint8 STATUS_BAGS_EMPTY_PALLET                       = 41
uint8 STATUS_BAGS_LAYER_ORIENTATION_DETECTION_FAILED = 42
uint8 STATUS_BAGS_FAILURE                            = 43

uint8 STATUS_TF_LOOKUP_ERROR                         = 50

uint8 STATUS_STATION_2D_NO_ENABLED_MODELS            = 60
uint8 STATUS_STATION_2D_NO_MODELS                    = 61


###### Detection engines: ######
uint8 ENGINE_NONE                                    = 0
uint8 ENGINE_FLEX                                    = 1
uint8 ENGINE_TEACH                                   = 2
uint8 ENGINE_PATTERN                                 = 3
uint8 ENGINE_BAGS                                    = 5
uint8 ENGINE_TEACHXD                                 = 6
uint8 ENGINE_STATION_2D                              = 7

std_msgs/Header                 header
uint8                           status  # Detetion result status.
time                            image_processed_stamp  # Time at which image processing completes.
geometry_msgs/TransformStamped  robot_to_camera_tf
geometry_msgs/TransformStamped  camera_to_reference_tf

uint8                           detection_engine

float64                         cloud_capturing_duration

uint16                          n_rejected_clusters
float64                         clustering_duration

float64                         pickability_check_duration

float64                         calculation_time # Total object detection time

uint16                          n_valid_objects
uint16                          n_rejected_objects

float64                         occupied_roi_volume
float64                         roi_volume

im_pickit_msgs/Object[]         objects
