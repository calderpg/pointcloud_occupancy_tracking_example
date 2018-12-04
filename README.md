# pointcloud_occupancy_tracking_example
ROS example using pointcloud voxelization in [voxelized_geometry_tools](github.com/calderpg.voxelized_geometry_tools) for occupancy tracking from pointclouds.

## Dependencies

- [ROS Kinetic or ROS Melodic](ros.org) ROS Melodic on Ubuntu Bionic (18.04) is prefered. Gazebo simulator and ROS plugins are required, these are installed as part of `ros-<version>-desktop-full`.

- [common_robotics_utilities](github.com/calderpg/common_robotics_utilities)

- [voxelized_geometry_tools](github.com/calderpg/voxelized_geometry_tools)

- [generic_gazebo_thruster_plugins](github.com/calderpg/generic_gazebo_thruster_plugins)

## Optional Dependencies

- joystick_drivers Install via `sudo apt install ros-<version>-joystick-drivers`

## GPU-accelerated pointcloud voxelization is available, but must be supported on your system. Instructions are provided for Ubuntu Bionic.

### NVidia CUDA (system packages)

1. Install CUDA

```
sudo apt install nvidia-cuda-dev nvidia-cuda-toolkit
```

2. Set GCC 6 as the active compiler for your Catkin workspace

```
export CC=/usr/bin/gcc-6
export CXX=/usr/bin/g++-6
```

Note that setting this may interfere with DKMS kernel modules, so you will need to clear this setting before updating.

### NVidia CUDA (first-party)

1. Install first-party NVidia CUDA SDK and drivers.

### NVidia OpenCL (not recommended)

1. Install OpenCL SDK

```
sudo apt install nvidia-opencl-dev opencl-headers clinfo
```

### AMD OpenCL

1. Install AMD OpenCL drivers and SDK for your GPU (varies depending on GPU generation).

2. Install OpenCL SDK

```
sudo apt install opencl-headers clinfo
```

### Intel OpenCL

1. Install OpenCL drivers (for Skylake/Kabylake or older)

```
sudo apt install beignet beignet-*
```

2. Install OpenCL SDK

```
sudo apt install opencl-headers clinfo
```

## Run (Manual)

1. Start ROS

```
roscore
```

2. Start Gazebo simulation

```
roslaunch pointcloud_occupancy_tracking_example sensor_simulation.launch
```

Gazebo simulation with a free-flying cube inside a box. 8 RGBD cameras are simulated, one at each corner of the box.

3. Start pointcloud aggregator
```
rosrun pointcloud_occupancy_tracking_example pointcloud_aggregator_node
```

This node aggregates pointclouds from the 8 simulated RGBD cameras and publishes the aggregated pointcliuds and transforms for the occupancy tracker.

4. Start pointcloud occupancy tracker
```
rosrun pointcloud_occupancy_tracking_example pointcloud_occupancy_tracking_node
```

See the node source for more details on node parameters, including which voxelizer to use, how to filter the voxelized occupancy grid, and how large (and where) to track occupancy.

5. Open RViz

File->Open Config->Select /path/to/your/catkin/workspace/src/pointcloud_occupancy_tracking_example/config/occupancy_tracking.rviz

6. (Optional) Start joystick

Plug in joystick (tested with Xbox One controllers connected via USB).

```
rosrun joy joy_node
```

7. (Optional) Start teleop
```
rosrun pointcloud_occupancy_tracking_example joystick_block_control.py
```

This node allows you to control the flying block in the Gazebo simultation.

On the left stick, forward-backwards is X, left-right is Y, and triggers are Z.

Press and hold the `X` button to apply translation forces, or `Y` to apply rotation torques.
