# ROS package for Terabee Follow-Me and Terabee Robot Positioning System

This package is a ROS wrapper for `positioning_systems_api` contained in [positioning_systems_api](https://github.com/Terabee/positioning_systems_api) package. It enables easy use of Terabee Follow-Me system in ROS environment.

## Supported hardware
This package works with Terabee Follow-Me and Terabee Robot Positioning System.

## Dependencies

This package depends on [positioning_systems_api](https://github.com/Terabee/positioning_systems_api) version 1.5.6 or newer.

Please follow the instructions in positioning_systems_api README file to build and install.

## Building the packages
Clone the repository into your workspace:

*  If you have ssh key setup for your github account:
```
cd ~/ros_ws/src
git clone git@github.com:Terabee/positioning_systems_ros.git
```
*  If you prefer to use https use this set of commands:
```
cd ~/ros_ws/src
git clone https://github.com/Terabee/positioning_systems_ros.git
```
Navigate to your workspace and build:
```
cd ~/ros_ws
catkin build
source devel/setup.bash
```
## Nodes
### follow_me_master_beacon

With this node you can receive data (distance *in meters* and heading *in degrees*) from the system. It is intended to use with master beacon connected to the computer.

It provides following configuration options of the system:
*  switching between text and binary printout modes,
*  swapping beacons,
*  setting *Exponential Moving Average* filter number of samples (window size),
*  setting span between the beacons (in millimeters),
*  settings parameters for RS485 connection (Modbus slave id, baudrate, parity).

#### Running the node
After your workspace is built and sourced, you can run the node (set `_portname` to the actual port name where the master beacon is connected:
```
rosrun positioning_systems_ros follow_me_master_beacon _portname:=/dev/ttyACM0
```
#### Subscribed topics
This node subscribes to the following topics:

*  `/follow_me_master_beacon/follow_me_autocalibrate` : publishing to this topic activates span autocalibration mode
*  `/follow_me_master_beacon/follow_me_config` : publishing to this topic sets parameters: printout mode, swap beacons, EMA filter window, span between the beacons
*  `/follow_me_master_beacon/follow_me_rs485_config` : publishing to this topic sets slave id, baudrate and parity of RS485 interface
*  `/follow_me_master_beacon/follow_me_test_cmd` : publishing to this topic triggers test command which returns actual configuration of the device

Example 1 of usage:
```
rostopic pub /follow_me_master_beacon/follow_me_config positioning_systems_ros/FollowMeDriverConfig "printout_mode: 'Binary'
swap_beacons: true
ema_window: 10
beacons_span: 540"
```
Sets binary printout mode, swaps beacons, sets EMA filter to 10 samples and beacons span to 540 mm.

Example 2 of usage:
```
rostopic pub /follow_me_master_beacon/follow_me_rs485_config positioning_systems_ros/FollowMeDriverRS485Config "rs485_slave_id: 3
rs485_baudrate: 19200
rs485_parity: 2"
```
Sets Modbus RTU slave id to 3, baud rate to 19200 and parity to 2 (Even).

To see list of valid values for each parameter, open respective \*.msg file.

#### Published topics
This node publishes to the topic:
*  `/follow_me_master_beacon/follow_me_polar_point_2d` : provides distance and heading of the remote control with respect to the beacons

### follow_me_remote_control

This node is intended to use with remote control connected to the computer.

It provides following configuration options of the remote control:
*  setting button operation mode (hold, toggle)
*  setting buzzer operation (enabled, disabled)

#### Running the node
After your workspace is built and sourced, you can run the node (set `_portname` to the actual port name where the master beacon is connected:
```
rosrun positioning_systems_ros follow_me_remote_control _portname:=/dev/ttyUSB0
```
#### Subscribed topics
This node subscribes to the following topics:
*  `/follow_me_remote_control/follow_me_get_config` : publishing to this topic triggers command which returns actual configuration of the remote control
*  `/follow_me_remote_control/follow_me_set_config` : publishing to this topic sets remote control configuration (button mode and buzzer state)

Example of usage:
```
rostopic pub /follow_me_remote_control/follow_me_set_config positioning_systems_ros/FollowMeRemoteControlConfig "button_mode: 'Hold'
buzzer_active: false"
```
Sets button mode to *Hold* and deactivates buzzer.

To see list of valid values for each parameter, open respective \*.msg file.

### rtls_configurator_ros

With this node and provided launch file `configure_device.launch` you can configure connected device using one of the provided YAML files, for example:
```
roslaunch positioning_systems_ros configure_device.launch file:=config/rtls_anchor_0.yaml
```
will configure the device as the initiator anchor with priority number set to 0.

### rtls_tracker_node

With this node you can read the tracker position output when the tracker device is connected.

#### Published topics
This node publishes to the topic:

`/rtls_tracker_node/rtls_tracker_frame`

which provides a custom message:
```
RtlsAnchorData[] anchors
geometry_msgs/Point position
bool is_valid_position
```

with list of anchors data as embedded message:
```
uint16 id // anchor id
geometry_msgs/Point position // position of anchor
float64 distance // distance from anchor to tracker
```
position of tracker: `geometry_msgs/Point position`
and validity of message: `bool is_valid_position` set to false if unable to parse tracker output or the device was unable to provide valid tracker position.

#### Transform broadcast
The node broadcasts `tf2` transforms with positions of anchors and tracker, based on the data read from the tracker.

#### Launch file parameters

Provided launch file `tracker_reader.launch` has already predefined following parameters:

- portname - name of serial port device, e.g. `/dev/ttyUSB0`
- publish_tf - whether transforms of anchors and tracker should be broadcasted
- ref_frame - reference frame name for the broadcasted transforms

## Example

A basic subscriber is available in subdirectory `examples`.

In order to use the package in your own node, do the following:

In `CMakeLists.txt` add the package name `positioning_systems_ros` to `find_package` and `catkin_package` commands.

In `package.xml` add:

```
<build_depend>positioning_systems_ros</build_depend>
```
and
```
<exec_depend>positioning_systems_ros</exec_depend>
```
