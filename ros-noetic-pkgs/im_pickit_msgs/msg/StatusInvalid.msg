# Copyright 2021, Pick-It NV.
# All rights reserved.

# These objects (100 <= status <= 999)
# will get a red colored visualization marker.

### Enums:

uint16 UNRELIABLE_3D_SCENE_COVERAGE_TOO_LOW = 100
uint16 UNRELIABLE_2D_SCENE_COVERAGE_TOO_LOW = 101
uint16 UNRELIABLE_3D_MODEL_COVERAGE_TOO_LOW = 102
uint16 UNRELIABLE_2D_MODEL_CONTOUR_COVERAGE_TOO_LOW = 103
uint16 UNRELIABLE_2D_MODEL_SURFACE_COVERAGE_TOO_LOW = 104
uint16 UNRELIABLE_2D_MODEL_SURFACE_COVERAGE_TOO_HIGH = 105
uint16 UNRELIABLE_XD_SCORE_TOO_LOW = 106
uint16 UNRELIABLE_3D_NORMAL_COVERAGE_TOO_LOW = 107
uint16 UNRELIABLE_3D_SUBMODEL_COVERAGE_TOO_LOW = 108

uint16 WRONG_SIZE_SMALLER_THAN_X_MIN = 300
uint16 WRONG_SIZE_LARGER_THAN_X_MAX = 301
uint16 WRONG_SIZE_SMALLER_THAN_Y_MIN = 302
uint16 WRONG_SIZE_LARGER_THAN_Y_MAX = 303
uint16 WRONG_SIZE_SMALLER_THAN_Z_MIN = 304
uint16 WRONG_SIZE_LARGER_THAN_Z_MAX = 305
uint16 WRONG_SIZE_SMALLER_THAN_MIN_OBJECT_POINTS = 306
uint16 WRONG_SIZE_LARGER_THAN_MAX_OBJECT_POINTS = 307

uint16 WRONG_SURFACE_SIDE_EXTERNAL = 400
uint16 WRONG_SURFACE_SIDE_INTERNAL = 401

uint16 IN_PREVIOUS_IMAGE = 500
