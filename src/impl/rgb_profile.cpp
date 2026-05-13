#include "rgb_profile.h"

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster/sensor_http.h"
#include "ouster/impl/profile_extension.h"

using ouster::sdk::core::add_custom_profile;
using ouster::sdk::core::impl::FieldInfo;
using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::LidarScan;
using ouster::sdk::core::SensorInfo;
using namespace ouster::sdk::core::UDPProfileLidarExt;

namespace ChanField = ouster::sdk::core::ChanField;

void ouster::sdk::core::register_r16_g16_b16_profile() {

    std::vector<std::pair<std::string, FieldInfo>> rgb_fields = {
        {ChanField::RANGE, FieldInfo{ChanFieldType::UINT32, 0, 0x0007ffff, 0}},
        {ChanField::REFLECTIVITY, FieldInfo{ChanFieldType::UINT8, 3, 0, 0}},
        {ChanField::SIGNAL, FieldInfo{ChanFieldType::UINT16, 4, 0, 0}},
        {ChanField::NEAR_IR, FieldInfo{ChanFieldType::UINT16, 6, 0, 0}},
        {ChanField::R, FieldInfo{ChanFieldType::UINT32, 8, 0xffff, 0}},
        {ChanField::G, FieldInfo{ChanFieldType::UINT32, 10, 0xffff, 0}},
        {ChanField::B, FieldInfo{ChanFieldType::UINT32, 12, 0xffff, 0}},
    };
    
    add_custom_profile(static_cast<int>(RNG19_RFL8_SIG16_NIR16_R16_G16_B16),
                       ChanField::R16_G16_B16_PROFILE_NAME, rgb_fields, 20);
}
