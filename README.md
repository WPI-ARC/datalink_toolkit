datalink_toolkit
==============

Tools for using ROS over limited bandwidth datalinks

*While this code is freely licensed (2-clause BSD), we do ask that you send [us](mailto:calder.pg@gmail.com) an email so we can see who is using this software.*

Repository structure
--------------------
Unlike earlier Catkinized software we have provided, this repository does not contain a Catkin workspace. As we expect that other teams will be well on their way to migrating to Catkin, the difficulties of managing multiple workspaces do not justify the convenience of distributing these packages in their own workspace. As such, you will need to clone this repository inside the `src/` directory of an existing Catkin workspace.

Please note that this software is primarily written for ROS Hydro, since that is what we are using for active development. Since this software was initially developed on ROS Groovy, most parts are (and should remain) backwards-compatible. Due to the use of Catkin, this software is incompatible with ROS Fuerte and earlier.

This repository is structured around 4 core packages:

1.  `teleop_launch` - Launch files for teleop links using both the nodes in this repository and other packages (namely a custom version of `openni.launch` to support divorced TF trees). The launch files in this package provide examples for how to configure the various link components in this repository.

2.  `opportunistic_link` - This package provides an automatically switched ROS transmit link that allows for a topic to be sent once over the link to multiple subscribers. To reduce data demands, the link stops data flow automatically when no subscribers are connected.

3.  `teleop_msgs` - Message types for teleoperation datalinks, namely to support message aggregation and compression.

4.  `teleop_mux_demux` - Packages provides nodes for message aggregation/multiplexing and message dis-aggregation/demultiplexing. This allows multiple message topics to be combined together and sent over a single ROS publisher->subscriber link.

Stability and development status
--------------------------------
`opportunistic_link` - Package is stable and tested with various messages including `sensor_msgs/PointCloud2` from a Kinect with no appreciable consequence of overhead.

`teleop_msgs` - Package is currently stable with no additional message types planned.

`teleop_launch` - New example and test launch files are added fairly frequently, but all available launch files should be considered stable.

`teleop_mux_demux` - Package is semi-stable and currently finishing development. Awaiting testing to see the effects of processing overhead.

Depencies
---------
1.  Full ROS Groovy installation - on Ubuntu systems: `$ sudo apt-get install ros-groovy-desktop-full`

Build and usage instructions
----------------------------
First, clone this repository:
```
$ cd /your/catkin/workspace/src
$ git clone https://github.com/WPI-ARC/teleop_toolkit.git
$ rospack profile
```
To build all packages in this repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make
```
To build a particular package in the repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make --pkg <package name>
```
To use, you must source the workspace:

```
(in the surrounding Catkin workspace directory)
$ source devel/setup.bash
```

For usage information and instructions on running components of these packages together, see the repository [Wiki](https://github.com/WPI-ARC/teleop_toolkit/wiki).
