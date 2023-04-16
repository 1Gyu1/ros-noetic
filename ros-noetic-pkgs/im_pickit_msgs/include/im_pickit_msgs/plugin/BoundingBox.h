#define IM_PICKIT_MSGS_MESSAGE_BOUNDINGBOX_PLUGIN_CONSTRUCTOR \
  BoundingBox_(double x_min = 0, double x_max = 0, \
               double y_min = 0, double y_max = 0, \
               double z_min = 0, double z_max = 0) \
    : x_min(x_min), \
      x_max(x_max), \
      y_min(y_min), \
      y_max(y_max), \
      z_min(z_min), \
      z_max(z_max) {}

#define IM_PICKIT_MSGS_MESSAGE_BOUNDINGBOX_PLUGIN_CLASS_BODY \
  bool operator==(const BoundingBox_& rhs) const { \
    const double EPS = 0.0001; \
    return std::abs(x_max - rhs.x_max) < EPS && std::abs(x_min - rhs.x_min) < EPS && \
           std::abs(y_max - rhs.y_max) < EPS && std::abs(y_min - rhs.y_min) < EPS && \
           std::abs(z_max - rhs.z_max) < EPS && std::abs(z_min - rhs.z_min) < EPS; \
  } \
  bool operator!=(const BoundingBox_& rhs) const { \
    return !operator==(rhs); \
  }
