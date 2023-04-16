# Copyright 2021, Pick-It NV.
# All rights reserved.

# These objects (0 <= status <= 99)
# will get an orange colored visualization marker.

### Enums:
uint16 INVALID_PICK_FRAME_DEFINITION = 1

uint16 INVALID_Z_AXIS_ORIENTATION = 10

uint16 CENTER_OUTSIDE_ROI = 20

uint16 FLANGE_OUTSIDE_ALLOWED_REGION = 21

uint16 COLLISION_TOOL_WORLD = 30

uint16 COLLISION_PRE_PICK   = 41
uint16 COLLISION_POST_PICK  = 42

uint16 NO_PICK_FRAME_DEFINED = 50

uint16 NO_PICK_FRAME_ENABLED = 60

uint16 PICK_SURFACE_NOT_FLAT = 70

uint16 OBJECT_NEARBY = 80  # Rejected by object distance filter
