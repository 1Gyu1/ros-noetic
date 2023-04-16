# Copyright 2021, Pick-It NV.
# All rights reserved.

# Pick frame symmetry type.
uint8 SYMMETRY_NONE = 0  # No symmetry, the default.
uint8 SYMMETRY_AXIS = 1  # Symmetric about an axis (e.g. a pick frame on a cylinder surface).
uint8 SYMMETRY_POINT = 2  # Symmetric about a point (e.g. a pick frame on a spherical surface).

uint8 type

# Symmetry frame, specified with respect to the object frame.
# Depending on the value of symmetry_type, this frame is interpreted as follows:
# - SYMMETRY_NONE: Ignored.
# - SYMMETRY_AXIS: A symmetry axis parallel to the frame's X-axis, passing through its origin.
# - SYMMETRY_POINT: A symmetry point in the frame's origin. The orientation is unused.
geometry_msgs/Pose symmetry_frame

# Symmetry angular tolerance ranges, in radians.
# Depending on the value of symmetry_type, these values are interpreted as follows:
# - SYMMETRY_NONE: Ignored.
# - SYMMETRY_AXIS: Angular range about the symmetry axis:
#     [angle_1_min, angle_1_max].
# - SYMMETRY_POINT: Angular ranges about:
#    Azimuth angle: [angle_1_min, angle_1_max]
#    Polar angle: [angle_2_min, angle_2_max]
float64 angle_1_min
float64 angle_1_max
float64 angle_2_min
float64 angle_2_max
