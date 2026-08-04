#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "ouster_ros/impl/file_util.h"
#include "../src/pinhole_processor.h"

namespace ouster_ros {
namespace {

ouster::sdk::core::SensorInfo load_test_info() {
    const auto metadata =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "ouster-sdk/tests/metadata/3_0_1_os-122246000293-128.json";
    return ouster::sdk::core::SensorInfo(
        impl::read_text_file(metadata.string()));
}

TEST(PinholeProcessorTest, WrapsRoundedColumnAtPanoramaSeam) {
    const auto info = load_test_info();
    const double quarter_column_azimuth =
        0.25 * 2.0 * M_PI /
        static_cast<double>(info.format.columns_per_frame);

    PinholeProcessor::PanelConfig config;
    config.name = "seam";
    config.yaw_rad = quarter_column_azimuth;
    config.width = 2;
    config.height = 1;

    PinholeProcessor processor(info, {config}, "{ns}/{name}", "/lidar0",
                               0.0, nullptr);
    const auto& panel = *processor.panels().at(0);

    // The center ray maps to W - 0.25 columns. Rounding lands on W, which
    // must wrap to column zero rather than clamp to the last column.
    EXPECT_EQ(panel.v_src(0, 1), 0);
    EXPECT_EQ(panel.optical_frame_id, "lidar0/seam");
    EXPECT_EQ(panel.camera_info.width, 2u);
    EXPECT_EQ(panel.camera_info.height, 1u);
}

TEST(PinholeProcessorTest, ClampsAutoHeightBeforeAllocating) {
    const auto info = load_test_info();

    PinholeProcessor::PanelConfig config;
    config.name = "narrow";
    config.hfov_rad = 1e-12;
    config.width = 1;
    config.height = 0;

    PinholeProcessor processor(info, {config}, "{name}", "", 0.0, nullptr);
    const auto& panel = *processor.panels().at(0);

    EXPECT_EQ(panel.camera_info.height, 8192u);
    EXPECT_EQ(panel.r_src.rows(), 8192);
    EXPECT_EQ(panel.r_src.cols(), 1);
}

}  // namespace
}  // namespace ouster_ros
