// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace ouster_ros {
namespace impl {

inline uint16_t display_to_mono16(float value) {
    // SDK auto-exposure leaves raw values unchanged when a sparse scan has
    // too few samples to initialize. Bound them before the integer conversion,
    // which otherwise has undefined behavior outside the destination range.
    if (!std::isfinite(value) || value <= 0.0f) return 0;
    return static_cast<uint16_t>(
        std::min(value, 1.0f) * std::numeric_limits<uint16_t>::max());
}

}  // namespace impl
}  // namespace ouster_ros
