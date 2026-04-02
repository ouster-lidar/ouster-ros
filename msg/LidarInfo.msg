# Describes the calibration and configuration of a lidar sensor.
#
# This message is the lidar equivalent of sensor_msgs/CameraInfo. It provides
# all the information needed to interpret a LidarScan message and project its
# range measurements into 3D Cartesian coordinates.
#
# Publishing convention: publish once on a latched topic (e.g. /lidar_info)
# or alongside every LidarScan for self-contained processing.

# Message header (should match the associated LidarScan header)
std_msgs/Header header

# ============================================================
# Sensor identification
# ============================================================

# Product line (e.g. "OS0-128", "OS1-64", "OS2-128")
string product_line

# Sensor serial number
uint64 serial_number

# Firmware revision string (e.g. "v3.1.1")
string firmware_revision

# ============================================================
# Scan geometry
# ============================================================

# Number of beams (vertical channels). Must match LidarScan.height.
uint32 pixels_per_column

# Number of measurement columns per full scan frame.
# Must match LidarScan.width.
uint32 columns_per_frame

# Frames per second
uint16 fps

# Per-beam azimuth angle offsets in degrees (length = pixels_per_column).
# These are small corrections to the nominal azimuth for each beam.
float64[] beam_azimuth_angles

# Per-beam altitude (elevation) angle in degrees (length = pixels_per_column).
# Positive values point upward from the horizontal plane.
float64[] beam_altitude_angles

# Per-row pixel shift offsets for destaggering (length = pixels_per_column).
# The raw scan data is staggered: measurements in each row are offset by this
# many columns. Apply: destaggered_col = (col + width - pixel_shift[row]) % width
int32[] pixel_shift_by_row

# ============================================================
# Coordinate frame transforms (4x4 homogeneous, row-major)
# All translations are in MILLIMETERS.
# ============================================================

# Transform from beam frame to lidar frame.
# Encodes the lidar_origin_to_beam_origin distance and any beam offsets.
float64[16] beam_to_lidar_transform

# Transform from lidar frame to sensor frame.
# Accounts for the physical mounting of the lidar within the sensor housing.
float64[16] lidar_to_sensor_transform

# Transform from IMU frame to sensor frame.
float64[16] imu_to_sensor_transform

# User-assignable extrinsic transform (e.g., sensor to robot base).
# Defaults to identity if not set.
float64[16] extrinsic

# ============================================================
# Lidar data profile
# ============================================================

# UDP lidar profile identifier string describing the data format.
# Determines which channels are present and their data types.
# Examples: "LEGACY", "RNG19_RFL8_SIG16_NIR16",
#           "RNG19_RFL8_SIG16_NIR16_DUAL", "RNG15_RFL8_NIR8"
string udp_profile_lidar

# Range unit in meters. Multiply raw range values by this to get meters.
# Typically 0.001 (1 mm per count).
float64 range_unit
