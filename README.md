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

- Open-loop EuRoC

```
cd scripts
python Run_EuRoC_Stereo_ROS.py
```

- Closed-loop Trajectory-Tracking

```
```

- Task-Driven
```
```
