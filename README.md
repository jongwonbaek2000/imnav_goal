# imnav_goal

A lightweight ROS package that converts GPS coordinates to UTM (Universal Transverse Mercator) coordinates and sends navigation commands to `move_base` for autonomous robot navigation.

## Overview

`imnav_goal` is designed to bridge GPS-based waypoint navigation with ROS's `move_base` navigation stack. The package subscribes to GPS coordinates from external sources (e.g., CSV files or other GPS inputs), converts them to relative UTM coordinates with respect to an origin point, and publishes them as navigation goals to `move_base`.

**Key Features:**
- Converts GPS coordinates (latitude/longitude) to UTM coordinates
- Generates relative waypoint positions based on origin GPS location
- Publishes `move_base_simple/goal` navigation goals
- Deduplicates redundant goals using floating-point comparison
- Supports multi-zone UTM navigation with zone mismatch warnings
- Built with C++17 using ROS and GeographicLib

## Project Structure

```
imnav_goal/
├── CMakeLists.txt          # CMake build configuration
├── package.xml             # ROS package metadata
├── README.md               # This file
├── include/
│   └── imnav_goal/
│       └── imnav_goal.h    # Header file with ImnavGoal class definition
├── src/
│   └── imnav_goal_node.cpp # Main node implementation
├── launch/
│   └── imnav_goal.launch   # ROS launch file
└── txts/
    └── test.txt            # Usage examples and notes
```

## Requirements

### ROS Dependencies
- `roscpp` - C++ ROS client library
- `sensor_msgs` - Message types for GPS data (NavSatFix)
- `geometry_msgs` - Message types for pose and position data

### System Dependencies
- `GeographicLib` - Library for geographic coordinate conversions
  - Install: `sudo apt-get install libgeographic-dev`
- C++17 compatible compiler

## Installation

1. **Clone the repository:**
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/jongwonbaek2000/imnav_goal.git
   cd ..
   ```

2. **Install dependencies:**
   ```bash
   sudo apt-get install libgeographic-dev
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Quick Start

1. **Launch the node:**
   ```bash
   roslaunch imnav_goal imnav_goal.launch
   ```
   or
   ```bash
   rosrun imnav_goal imnav_goal_node
   ```

2. **Publish origin GPS (one-time setup):**
   ```bash
   rostopic pub /origin_gps sensor_msgs/NavSatFix \
     "{latitude: 37.340727102832, longitude: 126.733292534745}" -1
   ```

3. **Publish goal GPS waypoints:**
   ```bash
   rostopic pub /gps_goal_fix sensor_msgs/NavSatFix \
     "{latitude: 37.340652586, longitude: 126.733606751}" -1
   ```

### Integration with External Systems

The node subscribes to two topics:
- **`/origin_gps`** - Origin reference point (only used once, first message accepted)
- **`/gps_goal_fix`** - Destination waypoints (updates trigger new goal computation)

## How It Works

### Coordinate Transformation

1. **GPS Input:** Accepts latitude/longitude from `NavSatFix` messages
2. **UTM Conversion:** Converts GPS coordinates to UTM using GeographicLib
3. **Relative Positioning:** Calculates waypoint position relative to origin
   ```
   x_star = goal_x_utm - origin_x_utm
   y_star = goal_y_utm - origin_y_utm
   ```
4. **Goal Publication:** Publishes as `geometry_msgs/PoseStamped` to `/move_base_simple/goal`

### Goal Deduplication

- Prevents duplicate goal publications by comparing new GPS coordinates with the last published goal
- Uses epsilon comparison (1e-7) for floating-point tolerances
- Ignores identical waypoint commands

### Multi-Zone Support

- Warns if origin and goal are in different UTM zones
- Recommends caution when zones differ due to potential coordinate inaccuracies

## ROS Topics

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/origin_gps` | `sensor_msgs/NavSatFix` | Origin GPS location (reference point) |
| `/gps_goal_fix` | `sensor_msgs/NavSatFix` | Goal GPS location to navigate to |

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Navigation goal for move_base |

## Example Output

When a valid GPS goal is received:
```
[INFO] Origin GPS received: Lat=37.340727, Lon=126.733293
[INFO] Published new goal: x=45.234567, y=-78.123456
```

## Assumptions

- The robot's initial position in ROS frame is (0, 0)
- The robot's orientation is aligned with the UTM coordinate system
- Robot left direction = +Y (North in UTM)
- Robot forward direction = +X (East in UTM)

## Code Structure

### ImnavGoal Class

**Constructor:**
- Initializes ROS subscribers and publisher
- Sets up callbacks for GPS messages

**Public Methods:**
- None (node runs via `ros::spin()`)

**Private Methods:**
- `originCallback()` - Stores and locks the origin GPS point
- `goalCallback()` - Processes incoming goal GPS messages
- `isNewGoal()` - Checks if goal differs from previously published goal
- `processAndPublishGoal()` - Converts GPS to UTM and publishes to move_base
- `gpsToUTM()` - Performs GPS to UTM coordinate conversion

## Building from Source

The build system is configured to:
- Compile with C++17 standard (`-std=c++17`)
- Link against ROS catkin libraries
- Link against GeographicLib

Build output: `imnav_goal_node` executable

## Known Limitations

- Only accepts the first origin GPS message; subsequent origin messages are ignored
- Multi-zone UTM transitions may require careful path planning
- Expects GPS input to be continuous; stale origin causes goal rejections

## Future Enhancements

- Support for dynamic origin updates
- Batch waypoint processing (CSV file input)
- Visualization of waypoints in RViz
- Configurable epsilon for goal deduplication
- Support for different coordinate systems (ECEF, etc.)

## License

Currently unlicensed. See `package.xml` for maintainer information.

## References

- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [GeographicLib Documentation](https://geographiclib.sourceforge.io/)
- [ROS NavSatFix Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
- [ROS PoseStamped Message](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)

## Contributing

For issues, questions, or contributions, please open an issue or pull request on the [GitHub repository](https://github.com/jongwonbaek2000/imnav_goal).
