# VoxelGrid.msg
# Custom message for transmitting voxel data from Bonxai occupancy map

# Header with timestamp and frame
std_msgs/Header header

# Voxel resolution in meters
float64 resolution

# Voxel coordinates (integer grid coordinates)
# Each voxel at index i has coordinate (x[i], y[i], z[i])
# To convert to world position: pos = coord * resolution
int32[] x
int32[] y
int32[] z

# Total number of voxels
uint32 num_voxels

# Bounding box of voxels (in voxel coordinates)
int32 min_x
int32 min_y
int32 min_z
int32 max_x
int32 max_y
int32 max_z