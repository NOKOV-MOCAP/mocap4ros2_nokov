# mocap4r2_nokov_driver

[![rolling](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/rolling.yaml/badge.svg)](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/rolling.yaml)

Create workspace:
```
mkdir -p mocap4r2_ws/src && cd mocap4r2_ws/src
```
Download nokov repo:
```
git clone https://github.com/NOKOV-MOCAP/mocap4ros2_nokov.git
```
Install dependencies:
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y python3-vcstool
cd src
vcs import < mocap4ros2_nokov/dependency_repos.repos
```
Compiling workspace:
```
cd .. && colcon build --symlink-install
```
Source workspace:
```
source install/setup.bash
```
Setup your nokov configuration:
```
mocap4r2_ws/src/mocap4ros2_nokov/mocap4r2_nokov_driver/config/mocap4r2_nokov_driver_params.yaml
```
Launch nokov system:
```
ros2 launch mocap4r2_nokov_driver mocap4r2_nokov_driver_launch.py
```
Visualize in rViz:
```
ros2 launch mocap4r2_marker_viz mocap4r2_marker_viz.launch.py mocap4r2_system:=nokov
```