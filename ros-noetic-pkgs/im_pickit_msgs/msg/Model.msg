# Copyright 2021, Pick-It NV.
# All rights reserved.

# Values that 'model_type' can take.
uint8 FROM_CAMERA   = 1
uint8 FROM_CAD      = 2
uint8 CYLINDER      = 3
uint8 CIRCLE        = 4
uint8 FROM_DXF      = 5

uint32                 id
uint32                 points
bool                   is_enabled
uint8                  model_type
int32                  priority
float64                max_visibility_angle
float64                min_visibility_ratio
int32                  n_mesh_faces # only makes sense for 3D CAD models
bool                   manual_position_tolerance_flag
float64                position_tolerance
float64                min_model_coverage
bool                   auto_cap_pick_frame_flag
float64                auto_cap_pick_frame_min_diameter

# Model dimensions. The contents of the x, y and z elements depend on the
# model type.
# - Models taught from camera or CAD: Size of the model bounding box.
# - Cylinder primitives: [length, diameter, diameter].
geometry_msgs/Vector3  dimensions

# Crop box
im_pickit_msgs/BoundingBox crop_box

# model_T_cbf
geometry_msgs/Transform model_T_cbf

# Pick frames
ObjectPickFrame[] pick_frames

SubModel[] submodels
