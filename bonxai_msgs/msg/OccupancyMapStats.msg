# OccupancyMapStats.msg
# Comprehensive statistics about the occupancy map state

# Header
std_msgs/Header header

# Memory usage (bytes)
uint64 total_memory_bytes
uint64 grid_memory_bytes
uint64 buffer_memory_bytes

# Cell counts
uint64 total_active_cells
uint64 occupied_cells
uint64 free_cells
uint64 unknown_cells

# Update statistics
uint64 total_insertions
uint64 hit_points_added
uint64 miss_points_added
uint8 current_update_cycle

# Spatial extent (bounding box in voxel coordinates)
# Only valid if has_data is true
bool has_data
int32 min_x
int32 min_y
int32 min_z
int32 max_x
int32 max_y
int32 max_z

# Probability distribution (log-odds values)
# Only populated if expensive stats are computed
int32 min_probability_log
int32 max_probability_log
int32 mean_probability_log

# Derived metrics (ratios are 0.0 to 1.0)
float32 occupancy_ratio      # occupied / (occupied + free)
float32 exploration_ratio    # (occupied + free) / total_active

# Performance metrics
uint64 point_clouds_processed
uint64 total_points_processed

# Flag indicating if expensive stats were computed
bool expensive_stats_computed