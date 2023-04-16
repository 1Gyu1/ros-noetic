# Copyright 2021, Pick-It NV.
# All rights reserved.

# Message describing an object pick frame.
# All poses in this message are expressed in a common frame (e.g. pickit/robot_base or pickit/reference)
# that is specified in a higher-level message.

# Special tool IDs.
string TOOL_INHERIT = __inherit__
string TOOL_NONE    = __none__

# Constant representing the highest possible pick frame priority.
uint32 PRIORITY_HIGHEST = 0

# Pick frame ID.
uint32 id

# The reference of this Pick frame given as pick frame ID. Used for pick offset computation.
uint32 reference_id

# Pick frame priority. Zero represents the highest priority.
uint32 priority

# If true, the pick frame will be a candidate when determining an object's pickability.
bool is_enabled

# If true, the pick frame's pickability has been evaluated.
bool was_evaluated

# Pick transform after shape related flexibility has been applied.
geometry_msgs/Pose shape_pick_pose

# Pick transform after tool related flexibility has been applied. This pose will
# be sent to the robot.
geometry_msgs/Pose robot_pick_pose

# Shape pick pose with applied pre-pick offset.
geometry_msgs/Pose shape_pre_pick_pose

# Shape pick pose with applied post-pick offset.
geometry_msgs/Pose shape_post_pick_pose

# Offset wrt. the reference pick frame.
geometry_msgs/Transform offset_wrt_pick_reference

# Pick frame symmetry specification.
ObjectPickFrameSymmetry symmetry

# ID of the tool that is used for collision checking.
string tool_id

# Reasons why the pick frame was rejected.
uint16[] rejection_list

# Points at which the tool is colliding with the collision world if the pick point was rejected due to collisions.
geometry_msgs/Point[] contact_points

# Name of the collision bodies for the contact points above.
string[] contact_bodies

# Name of the non-fixed joints. Empty if the tool has only fixed joints.
string[] joint_names
# Position values of the non-fixed joints. Empty if the tool has only fixed joints.
float64[] joint_positions
