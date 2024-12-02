octomap_mapping ![CI](https://github.com/OctoMap/octomap_mapping/workflows/CI/badge.svg)
===============

ROS stack for mapping with OctoMap, contains the `octomap_server` package.

The main branch for ROS1 Kinetic, Melodic, and Noetic is `kinetic-devel`.

The main branch for ROS2 Foxy and newer is `ros2`.

### Usage

#### Save octomap
This forked repo contains new octomap_saver_node code which works by running the node.
Note that it will save the octomap as `octomap_full.ot` in the directory the node is called from.
```
ros2 run octomap_server octomap_saver_node
```

These instructions are for the **original code**:
```
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=(path for saving octomap)
```
Note: The extension of octomap path should be `.bt` or `.ot`
