#pragma once

#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace core {

namespace UDPProfileLidarExt {
    constexpr UDPProfileLidar RNG19_RFL8_SIG16_NIR16_R16_G16_B16 = static_cast<UDPProfileLidar>(22);
}

namespace ChanField {

const std::string R16_G16_B16_PROFILE_NAME = "RNG19_RFL8_SIG16_NIR16_R16_G16_B16";

}  // namespace ChanField

void register_r16_g16_b16_profile();

}  // namespace core
}  // namespace sdk
}  // namespace ouster

