RTABMAP-Odom Benchmark


## Install
```bash
sudo apt install ros-noetic-rtabmap*

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone git@github.com:duyanwei/rtabmap_odom.git
# OR
git clone https://github.com/duyanwei/rtabmap_odom.git

cd rtabmap_odom
catkin build -j4 --this

```


## Benchmark

#### Open-loop EuRoC

```
cd scripts
python Run_EuRoC_Stereo_ROS.py
```

#### Closed-loop Trajectory-Tracking
- Follow the [tutorial](https://github.com/ivalab/meta_ClosedLoopBench/tree/feature/ubuntu20.04)
  - [launch file](launch/gazebo_trajectory_tracking.launch) for `rtabmap`

#### Task-Driven
  - Simulation
```bash
# First disale odom->base_link from gazebo in 
/kobuki_ros/kobuki_desktop/kobuki_gazebo/gazebo_ros_kobuki_updates.cpp
```
- RealWorld
  - Disable odom->base_link on turtlebot2
`set publish_tf to false` at line 23 in the following launch [file](https://github.com/ivalab/task_driven_slam_benchmarking/launch/realworld/minimal.launch).



#### Mapping Visualization with RTABMAP (NO LC)
- To use the rtabmap mapping visualization tool, it is better to publish the `odom/pose` in `odom->base_link`, and provide the `tf` of `base_link` to `camera_optical_frame`.

- Publish pose in camera frame also works, e.g. `visual_map->camera_optical_frame`, however it does not allow to truncate the stitched pointcloud in robot-body aligned frame, where the values are defined in. 

- The mapping launch file disables loop closure detection of RTABMAP, and only stitches rgbd images with provided poses.