/*
 * Copyright 2021, Pick-it NV.
 * All rights reserved.
 */
#pragma once

namespace im_pickit_msgs {
  /// Frame id being used for the robot's base frame.
  static const char ROBOT_BASE_FRAME_ID[] = "pickit/robot_base";
  /// Frame id being used for the Pick-it reference frame.
  static const char PICKIT_REFERENCE_FRAME_ID[] = "pickit/reference";
  /// Frame id being used for the robot's end-effector frame.
  static const char ROBOT_EE_FRAME_ID[] = "pickit/robot_ee";
  /// Frame id being used for the active camera's optical frame.
  /// Note that a camera needs to be active for this frame to exist.
  static const char ACTIVE_CAMERA_FRAME_ID[] = "pickit/camera_rgb_optical_frame";
}
