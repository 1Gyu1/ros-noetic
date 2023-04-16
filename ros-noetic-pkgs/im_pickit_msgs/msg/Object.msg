# Copyright 2021, Pick-It NV.
# All rights reserved.

## This message contains all descriptive data for one object found by Pick-it ##

# Highest and lowest possible model priority.
uint8 HIGH_MODEL_PRIORITY = 0
uint8 LOW_MODEL_PRIORITY  = 10

# Object index in list associated to a single detection run.
uint16                                  index

# Object frame expressed with respect to a frame.
geometry_msgs/TransformStamped          object_tf

# Object pick frame expressed with respect to a frame.
geometry_msgs/TransformStamped          object_pick_tf

# Object post pick frame.
geometry_msgs/Pose                      object_post_pick_tf

# Object type constants.
uint8 OBJECT_TYPE_NONE          = 0
uint8 OBJECT_TYPE_SQUARE        = 21
uint8 OBJECT_TYPE_RECTANGLE     = 22
uint8 OBJECT_TYPE_CIRCLE        = 23
uint8 OBJECT_TYPE_ELLIPSE       = 24
uint8 OBJECT_TYPE_CYLINDER      = 32
uint8 OBJECT_TYPE_SPHERE        = 33
uint8 OBJECT_TYPE_POINT_CLOUD   = 35
uint8 OBJECT_TYPE_BLOB          = 50
uint8 OBJECT_TYPE_CONTOUR_TREE  = 60
uint8 OBJECT_TYPE_CLUSTER       = 250

uint8 object_type  # Object type, takes one of the above defined models types as value.

# Number of scene points verifying the object model.
uint16                                  n_points

# Object dimensions. The contents of the x, y and z elements depend on the
# object type:
#
#  |   Type      |    X     |    Y     |    Z     |
#  |-------------|----------|----------|----------|
#  | CYLINDER    | length   | diameter | diameter |
#  | SPHERE      | diameter | diameter | diameter |
#  | RECTANGLE   | length   | width    |    0     |
#  | SQUARE      | length   | length   |    0     |
#  | ELLIPSE     | length   | width    |    0     |
#  | CIRCLE      | diameter | diameter |    0     |
#  | POINT_CLOUD | bbox X   | bbox Y   | bbox Z   |
#  | BLOB        | bbox X   | bbox Y   | bbox Z   |
#  | CONTOURTREE | bbox X   | bbox Y   | alpha    |
#
# where 'bbox X' means the span of the object's bounding box along the X-axis.
geometry_msgs/Vector3                   dimensions

# Contains scoring metrics used to quantify the goodness of a detection.
Reliability                             reliability

# NOTE: There is currently an inconsistency between terms used in the
#       back-end vs. the front-end. The following table summarizes them to
#       prevent confusion:
#
#       BACK-END flag  maps to ->   FRONT-END detection grid column
#
#       is_pickable    maps to ->   Pickable
#       is_detectable  maps to ->   Valid
#       is_valid       maps to ->   <None>
#
#       Notice in particular how the is_valid back-end flag and the Valid
#       front-end detection grid column have different meanings.

# Flag that evaluates to true when an object is both pickable and detectable.
bool                                    is_valid

# Flag that evaluates to true when an object satisfies frame alignment and
# collision constraints.
bool                                    is_pickable

# Flag that evaluates to true when an object satisfies fitting, reliability and
# size constraints.
bool                                    is_detectable

# Flag that indicates if a valid object has been pushed to the end of the object
# list due to pick blacklisting, i.e. it was already sent to the robot but not
# picked for some reason.
bool                                    is_blacklisted

# Object status. Only valid objects can be picked by the robot.
uint8 STATUS_ERROR          = 0
uint8 STATUS_VALID          = 1
uint8 STATUS_UNPICKABLE     = 2
uint8 STATUS_NOT_DETECTABLE = 3
uint8 STATUS_BLACKLISTED    = 4

uint8 status  # Object status, takes one of the above defined status as value.

# Reasons why the object is not valid.
uint32[]                                rejection_list

# If using TEACH, this integer represents the model from which this object was
# matched against. If object was derived from a different engine, this will be set to 0.
uint16                                  model_id

# If using TEACH, this integer represents the priority of the corresponding model.
# The priority has an impact on the order of the objects. Matches with lower priority
# models are sent to the robot after matches with higher priority models.
uint16                                  model_priority

# List of pick frames for this object.
ObjectPickFrame[] pick_frames

# The selected pick frame given as index (starting from 0) of the pick_frames array.
uint32 pick_frame_id
