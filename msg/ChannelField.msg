# Represents a single data channel from a lidar scan.
#
# Each channel stores one measurement type (e.g., range, signal, reflectivity)
# as a flat row-major array with dimensions matching the parent LidarScan
# (height x width). The data is stored as a byte array with the element type
# indicated by the 'datatype' field.

# Supported datatypes (matching sensor_msgs/PointField conventions)
uint8 UINT8   = 1
uint8 UINT16  = 2
uint8 UINT32  = 3
uint8 UINT64  = 4
uint8 FLOAT32 = 5
uint8 FLOAT64 = 6

# The name of this channel (e.g., "range", "signal", "reflectivity", "near_ir")
string name

# The element datatype (one of the constants above)
uint8 datatype

# The number of bytes per element
uint8 element_size

# The raw channel data stored as a flat byte array in row-major order.
# Total size = height * width * element_size (dimensions from parent LidarScan)
uint8[] data
