#define IM_PICKIT_MSGS_MESSAGE_COLORRANGERGB_PLUGIN_CLASS_BODY \
  bool operator==(const ColorRangeRGB_& rhs) const { \
    const double EPS = 0.0001; \
    return std::abs<double>(r_min - rhs.r_min) < EPS && std::abs<double>(r_max - rhs.r_max) < EPS && \
           std::abs<double>(g_min - rhs.g_min) < EPS && std::abs<double>(g_max - rhs.g_max) < EPS && \
           std::abs<double>(b_min - rhs.b_min) < EPS && std::abs<double>(b_max - rhs.b_max) < EPS; \
  } \
  bool operator!=(const ColorRangeRGB_& rhs) const { \
    return !operator==(rhs); \
  }
