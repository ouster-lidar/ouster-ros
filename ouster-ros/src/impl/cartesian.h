#pragma once

#include <ouster/lidar_scan.h>

namespace ouster {

// TODO: move to the sdk client

/**
 * This is the same function as the cartesianT method defined in the client but
 * allows the user choose a specific value for range values of zero.
 *
 * @param[in, out] points The resulting point cloud, should be pre-allocated and
 * have the same dimensions as the direction array.
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarScan.
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 * @param[in] min_range minimum lidar range to consider (millimeters).
 * @param[in] max_range maximum lidar range to consider (millimeters).
 * @param[in] invalid the value to assign of an xyz lut when range values are
 * equal to or exceed the min_range and max_range values.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
template <typename T>
void cartesianT(ouster::sdk::core::PointCloudXYZ<T>& points,
                const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>& range,
                const ouster::sdk::core::ArrayX3R<T>& direction,
                const ouster::sdk::core::ArrayX3R<T>& offset,
                uint32_t min_r, uint32_t max_r, T invalid) {
    assert(points.rows() == direction.rows() &&
           "points & direction row count mismatch");
    assert(points.rows() == offset.rows() &&
           "points & offset row count mismatch");
    assert(points.rows() == range.size() &&
           "points and range image size mismatch");

    const auto pts = points.data();
    const auto* const rng = range.data();
    const auto* const dir = direction.data();
    const auto* const ofs = offset.data();

    const auto N = range.size();

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (auto i = 0; i < N; ++i) {
        const auto r = rng[i];
        const auto idx_x = (i * 3) + 0;
        const auto idx_y = (i * 3) + 1;
        const auto idx_z = (i * 3) + 2;
        if (r <= min_r || r >= max_r) {
            pts[idx_x] = pts[idx_y] = pts[idx_z] = invalid;
        } else {
            pts[idx_x] = r * dir[idx_x] + ofs[idx_x];
            pts[idx_y] = r * dir[idx_y] + ofs[idx_y];
            pts[idx_z] = r * dir[idx_z] + ofs[idx_z];
        }
    }
}

}  // namespace ouster