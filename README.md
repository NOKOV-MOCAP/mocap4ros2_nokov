# mocap4r2_nokov_driver

[![rolling](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/rolling.yml/badge.svg?branch=v2.0)](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/rolling.yml) 

[![lyrical](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/lyrical.yml/badge.svg?branch=v2.0)](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/lyrical.yml) 

[![kilted](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/kilted.yml/badge.svg?branch=v2.0)](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/kilted.yml) 

[![jazzy](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/jazzy.yml/badge.svg?branch=v2.0)](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/jazzy.yml) 

[![humble](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/humble.yml/badge.svg?branch=v2.0)](https://github.com/NOKOV-MOCAP/mocap4ros2_nokov/actions/workflows/humble.yml) 

Driver of the Nokov motion capture systems for MOCAP4ROS2.

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
mocap4r2_ws/src/mocap4ros2_nokov/mocap4r2_nokov_driver/config/nokov_driver.yaml
```
Launch nokov system:
```
ros2 launch mocap4r2_nokov_driver nokov_driver.launch.yaml
```
Visualize in rViz:
```
ros2 launch mocap4r2_marker_viz marker_viz.launch.yaml namespace:=/nokov
```

## Configuration

| Parameter | Default | Description |
| --- | --- | --- |
| `server_address` | `10.1.1.198` | Address of the computer running the Nokov software. |
| `timeout` | `10.0` | Seconds to keep trying to reach the Nokov software when the node is configured. |
| `nominal_frame_rate` | `0.0` | Nominal frame rate of the cameras in Hz, 0 when unknown. |
| `up_axis` | `z` | Axis convention of the data. `z` keeps the data as it is, `y` rotates the data into Z-up inside the driver. |
| `frame_id` | `mocap` | Frame of the published messages. |
| `qos.reliability`, `qos.depth` | `reliable`, `10` | QoS of the data topics. |
| `publish_markers`, `publish_rigid_bodies`, `publish_skeletons` | `true` | What the driver publishes. |
| `publish_tf`, `tf_prefix` | `false`, `""` | TF of the tracked rigid bodies. |

The data published by the driver is always in metres and Z-up, the ids are the
ids of the Nokov software, and the rigid bodies and the skeleton segments that
are not tracked are published with `tracking_valid: false`.

## Controlling the system

The node is a lifecycle node, so it can be configured and activated through
the lifecycle services, or through the MOCAP4ROS2 control topic:

```
ros2 lifecycle set /nokov_driver configure
ros2 lifecycle set /nokov_driver activate
```

The launch file exposes the `autostart` argument, which does the same at
startup: `none` leaves the node unconfigured, `configure` configures it and
`activate` configures and activates it.

```
ros2 launch mocap4r2_nokov_driver nokov_driver.launch.yaml autostart:=activate
```
