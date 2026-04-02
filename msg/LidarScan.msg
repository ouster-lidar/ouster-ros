# Represents a structured multi-channel lidar scan.
#
# This message captures the native output of modern lidar sensors as a 2D grid
# (height x width) with multiple co-registered data channels. Unlike PointCloud2
# which provides an unstructured flat list of 3D points, LidarScan preserves:
#   - The image-like 2D adjacency structure of the scan
#   - Multiple synchronized data channels (range, signal, reflectivity, etc.)
#   - Per-column timestamps for precise motion compensation
#
# This format is analogous to a multi-spectral image where each pixel
# corresponds to a single lidar beam measurement and all channels are
# guaranteed to be spatially and temporally aligned.
#
# To convert a LidarScan to a PointCloud2 message, use the companion
# LidarInfo message which provides the calibration data needed to project
# range measurements into 3D Cartesian coordinates.

# Message header with timestamp and frame_id
std_msgs/Header header

# Dimensions of the scan grid
# height = number of beams (vertical resolution, e.g. 32, 64, 128)
# width  = number of columns per revolution (horizontal resolution, e.g. 512, 1024, 2048)
uint32 height
uint32 width

# Frame ID from the sensor (monotonically increasing scan counter)
int64 frame_id

# Per-column timestamps in nanoseconds (length = width).
# Each entry is the absolute timestamp (e.g., Unix epoch nanoseconds)
# at which the corresponding column of measurements was captured.
uint64[] timestamp

# The data channels of this scan. Each channel has the same height x width
# dimensions as the scan. Common channels include:
#   "range"         - Range in millimeters (uint32)
#   "range2"        - Second return range (uint32, dual-return sensors)
#   "signal"        - Signal photon count / intensity (uint16 or uint32)
#   "signal2"       - Second return signal (uint16 or uint32)
#   "reflectivity"  - Calibrated reflectivity (uint8 or uint16)
#   "reflectivity2" - Second return reflectivity (uint8 or uint16)
#   "near_ir"       - Near-infrared / ambient photon count (uint16)
#   "flags"         - Per-pixel measurement flags (uint8)
ouster_ros/ChannelField[] channels
