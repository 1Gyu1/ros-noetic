# Copyright 2021, Pick-It NV.
# All rights reserved.

# This message specifies the sources that contribute to the pick frame offset.
# This offset is typically used to drop off a picked object at the same location,
# regardless of whether different forms of tool/pick frame flexibility were used.
#
# If a pick frame specifies a 'reference_id' other than itself, this offset
# source will always be taken into account. It cannot be toggled, so it's not
# listed here. It's 'always-on'.
#
# Set any of the below values to 'true' to enable that source to contribute to
# the pick point offset. If the value is set to 'true' in this message, but the
# tool or pick point lacks that source, it will be ignored.

# From pick frame:
bool symmetry

# From tool:
bool flexible_pick_orientation_tilt
bool flexible_pick_orientation_z_rotation
bool passive_hinge_tilt
