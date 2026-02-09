# Rolling-Bonxai (WIP)
**Work in Progress, expect changes over the coming weeks! This will likely be merged with Bonxai in the future.**  
**Rolling-Bonxai** is a C++ 17 implementation built on Bonxai:https://github.com/facontidavide/Bonxai. **Rolling Bonxai** maintains a user defined local map that **rolls** with the robot. Unused parts of the maps are serialized and saved to disk and loaded when required. The underlying Bonxai library implements a compact hierarchical data structure that can store and manipulate volumetric data, discretized as Voxel Grids in a manner that is both **sparse** and **unbounded**. Star this repo if you found this useful/interesting :)

## Motivation
Bonxai is a lightweight Voxel Grid implementation, as seen from the benchmarks on the original repository. However, memory will still be a bottleneck as resource constrained robots (like drones) often have to map and navigate in very large areas. In these large areas, **the robot will likely not need other parts of the map besides some local area to do path planning/local planning**. Therefore, these storing unnecessary parts of the map in memory is wasteful, especially when considering other memory intensive tasks like running vision models.

To this end, **Rolling Bonxai** implements a chunking system (like in games) comprising of *NxNxN* voxels each. Each chunk is a `Bonxai::OccupancyGrid` that can be loaded/created or offloaded depending on the robot's position.

## Rolling-Bonxai In Action
### 1. Position-based chunk eviction
![Rolling Map Demo 1](media/forward_mapping.gif)

Full Video Here: https://youtu.be/gStI7W-x3mM

**Green and Blue semi-transparent cubes are "Dirty" and "Clean" chunks**. Dirty chunks are those that have been updated since the last time it was loaded into memory, while clean ones have not. Chunks that contain voxels are dirty. As the sensor moves, chunks are evicted based on some **policy**. In this case, it is a position based policy, where **chunks that are not one of the 26 neibors of the current chunk are evicted from memory**.

### 2. Position-based chunk loading from disc
![Rolling Map Demo 2](media/chunk_retrieval.gif)

Full Video Here: https://youtu.be/h4DjMLnnrf4

