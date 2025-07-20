# Rolling-Bonxai  
**Rolling-Bonxai** is a C++ 17 implementation built on Bonxai:https://github.com/facontidavide/Bonxai. **Rolling Bonxai** maintains a user defined local map that **rolls** with the robot. Unused parts of the maps are serialized and saved to disk and loaded when required. The underlying Bonxai library implements a compact hierarchical data structure that can store and manipulate volumetric data, discretized as Voxel Grids in a manner that is both **sparse** and **unbounded**. Star this repo if you found this useful/interesting :)

## Motivation
Bonxai is a lightweight Voxel Grid implementation, as seen from the benchmarks on the original repository. However, memory will still be a bottleneck as resource constrained robots (like drones) often have to map and navigate in very large areas. In these large areas, **the robot will likely not need other parts of the map besides some local area to do path planning/local planning**. Therefore, these storing unnecessary parts of the map in memory is wasteful, especially when considering other memory intensive tasks like running vision models.

To this end, **Rolling Bonxai** implements a chunking system (like in games) comprising of *NxNxN* voxels each. Each chunk is a `Bonxai::OccupancyGrid` that can be loaded/created or offloaded depending on the robot's position.

## Rolling-Bonxai In Action
### 1. Position-based chunk eviction
![Rolling Map Demo 1](docs/forward_mapping.gif)

Full Video Here: https://youtu.be/gStI7W-x3mM

**Green and Blue semi-transparent cubes are "Dirty" and "Clean" chunks**. Dirty chunks are those that have been updated since the last time it was loaded into memory, while clean ones have not. Chunks that contain voxels are dirty. As the sensor moves, chunks are evicted based on some **policy**. In this case, it is a position based policy, where **chunks that are not one of the 26 neibors of the current chunk are evicted from memory**.

### 2. Position-based chunk loading from disc
![Rolling Map Demo 2](docs/chunk_retrieval.gif)

Full Video Here: https://youtu.be/h4DjMLnnrf4

**Chunks that have been mapped before but not currently in memory are deserialized and loaded into memory** depending on position. **In the short interim period where chunk retrieval (an io process) is in progress, they are Black.** In the above video, the sensor is moving in the opposite direction to where it is facing. As its position changes, its chunk coordinate changes, resulting in chunks it has seen before being loaded into memory.

## Using Rolling-Bonxai
### Setup
1. Pull in Rolling-Bonxai repo into your ROS 2 workspace
2. Build it using `colcon build  --cmake-args -DCMAKE_BUILD_TYPE=Release` preferably
3. Setup the configs (see below)
4. Launch your camera. Make sure it has a valid TF (from map - odom - baselink)
5. `Inside rolling_map.launch.py`, change the remapping of the /cloud_in topic to the topic where your pointcloud is produced.
6. Launch rviz using the `viz_map.rviz` file.
7. Launch your camera/sensor
8. Modify fields related to your sensor in RVIZ.  
9. `ros2 launch rolling-map rolling_map.launch.py`

### Configs

Below are the configurable parameters for running the **Rolling-Bonxai** mapping pipeline:

#### TF Frame Settings

| Parameter      | Description                    | Example                |
|----------------|--------------------------------|------------------------|
| `map_frame`    | Global reference frame         | `"map"`                |
| `sensor_frame` | Sensor's frame ID              | `"realsense_camera_link"`    |
| `robot_frame`  | Robot's base frame ID          | `"realsense_camera_link"`    |

---

#### Map Parameters

| Parameter                   | Description                                    | Value    |
|----------------------------|------------------------------------------------|----------|
| `m_resolution`             | Voxel resolution (in meters)                   | `0.05`   |
| `m_occupancy_min_thresh`   | Minimum occupancy threshold                    | `0.12`   |
| `m_occupancy_max_thresh`   | Maximum occupancy threshold                    | `0.97`   |

---

#### Sensor Model Parameters

| Parameter             | Description                           | Value  |
|----------------------|---------------------------------------|--------|
| `s_max_z`            | Max height of points (in meters)      | `2.0`  |
| `s_min_z`            | Min height of points (in meters)      | `-2.0` |
| `s_max_range`        | Max usable sensor range               | `5.0`  |
| `s_min_range`        | Min usable sensor range               | `0.5`  |
| `s_probability_hit`  | Occupancy update when hitting voxel   | `0.7`  |
| `s_probability_miss` | Occupancy update when missing voxel   | `0.4`  |

---

#### Chunk Management

| Parameter             | Description                                 | Value      |
|----------------------|---------------------------------------------|------------|
| `c_chunk_dim`         | Chunk size in number of voxels per side     | `64`       |
| `c_chunk_folder_path` | Directory path for saving/loading chunks    | `"/home/your_ws/src/Rolling-Bonxai/chunks"` |

---

#### Preprocessing

| Parameter        | Description                        | Value   |
|------------------|------------------------------------|---------|
| `use_sor_filter` | Apply Statistical Outlier Removal  | `false` |

---

#### Visualization Flags

| Parameter     | Description                     | Value   |
|---------------|---------------------------------|---------|
| `viz_occupied`| Show occupied voxels            | `true`  |
| `viz_free`    | Show free (unoccupied) voxels   | `false` |
| `viz_chunks`  | Display chunk boundaries        | `true`  |
| `viz_usage`   | Visualize chunk memory usage    | `false` |


## Code Hot Paths

```mermaid
graph TD;
    A["**rolling_map_node.cpp**<br>rclcpp::spin<br>(RollingMapNode)"] --> B["**rolling_map.cpp**<br>RollingMapNode::<br>cloudCallback()"];
    B --> C["**map_manager.cpp**<br>MapManager::<br>updateMap(...)"];
    C --> D{"ifFirstUpdate"};
    D -->|yes|; E["**chunk_manager.cpp**<br>ChunkManager::<br>initFirstChunks()"];
    D -->|no| F["**chunk_manager.cpp**<br>ChunkManager::<br>updateChunks()"];
    F --> G["**chunk_manager.cpp**<br>ChunkManager::<br>updateAllOccupancy()"];
    G --> H{"ifNewSource"};
    H -->|yes|; I["**chunk_manager.cpp**<br>ChunkManager::<br>updateCache()"];
```

## Chunk Cache
Each chunk is a `Bonxai::OccupancyGrid` class. The chunks are stored as `std::shared_ptr<Bonxai::OccupancyGrid>` in a `std::array` of size 27. Every chunk has **26 neighbors** - **6 Face Neighbors**, **12 Edge Neighbors**, **8 Corner Neighbors**. The neibors and the souce chunk make up the 27 items in the cache. The source chunk refers to the chunk coordinate where the sensor is. This is always placed in index 0 of the cache. The rest of the chunks, depending on their relative position to the source chunk are placed in the array at an index determined by this enum class:

```cpp
enum class ChunkType : size_t
{
    SOURCE = 0,        //+0,+0,+0
    FACE_F = 1,        //1,0,0
    FACE_B = 2,        //-1,0,0
    FACE_L = 3,        //0,1,0
    FACE_R = 4,        //0,-1,0
    FACE_T = 5,        //0,0,1
    FACE_D = 6,        //0,0,-1
    EDGE_B_R = 7,      //-1,-1,0
    EDGE_B_L = 8,      //-1,1,0
    EDGE_F_R = 9,      //1,-1,0
    EDGE_F_L = 10,     //1,1,0
    EDGE_B_D = 11,     //-1,0,-1
    EDGE_B_T = 12,     //-1,0,1
    EDGE_F_D = 13,     //1,0,-1
    EDGE_F_T = 14,     //1,0,1
    EDGE_R_D = 15,     //0,-1,-1
    EDGE_R_T = 16,     //0,-1,1
    EDGE_L_D = 17,     //0,1,-1
    EDGE_L_T = 18,     //0,1,1
    CORNER_B_R_D = 19, //-1,-1,-1
    CORNER_B_R_T = 20, //-1,-1,1
    CORNER_B_L_D = 21, //-1,1,-1
    CORNER_B_L_T = 22, //-1,1,1
    CORNER_F_R_D = 23, // 1,-1,-1
    CORNER_F_R_T = 24, // 1,-1,1
    CORNER_F_L_D = 25, // 1,1,-1
    CORNER_F_L_T = 26, // 1,1,1
    INVALID = 27
};
```

We can simple `<static_cast>` the enum class to its underlying type to get the chunk index. Therefore, for every source chunk, the configuration of the cache will be unique. However, any 2 cache with 2 different source chunks *may* have same chunks. This is because 2 source chunks can have common neighbors, but the common neighbor chunks will be in different indices in the 2 caches.

For each `ChunkType`, we can also get the position-wise neighbor chunk using this monstrosity that is `ChunkManager::getChunkCoord`

## Distance-Neighborhood based eviction policy
When the sensor moves into a new source chunk, we must now update our cache. The updates happen inside `ChunkManager::updateChunks()`. Given the old source chunk coordinate S, and new source chunk coordinate T, we iterate through the cache for each chunk C:
- If ChunkManager::is26Neibor(C,T) or C == T 
    - Reposition C given T as source
- Else
    - If ChunkState of C == `ChunkState::Clean`
        - Destroy this chunk, it has no new information
    - Otherwise, if Chunkstate of C == `ChunkState::Dirty`
        - It has been updated, we serialize and save it to disk using `ChunkManager::writeWorker()`

Now, we have removed chunks that need to be evicted or reordered chunks that need to be persisted. We get a list of new chunks using `ChunkManager::getChunkCoord()` and either
- Load from memory using `ChunkManager::readWorker()`
- Create new map if it doesnt exist in disc using `ChunkManager::readWorker()`

## Whats Next
- [ ] Refactor and code cleanup (priority!)
- [ ] Explore better ways to do async IO for a single producer multiple consumer (readerWorker,writeWorker) model
- [ ] Eviction logic based on memory usage and active cells
- [ ] ROS 2 Service to get occupied voxels in memory
- [ ] ROS 2 Service to query serialized and saved chunks to get occupied voxels
- [ ] Improve `ChunkManager::~ChunkManager` to dump all dirty chunks currently in memory to disc


