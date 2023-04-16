# Copyright 2021, Pick-It NV.
# All rights reserved.

bool camera_loaded                # If true, the snapshot camera is loaded.
bool configuration_loaded         # If true, the snapshot configuration is loaded.
string snapshot_name              # Name of the loaded snapshot
string system_id                  # Hostname of the machine that generated the snapshot
string pickit_version             # Git hash in case of compilation from source OR pickit version
string robot_type                 # Robot type from the socket interface.
