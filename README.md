ROSPlan_interface_quadrotor
===========================

This package implements a simple interface for [hector_quadrotor](http://wiki.ros.org/hector_quadrotor) to [ROSPlan](https://github.com/KCL-Planning/ROSPlan/wiki).

It features simple planning domain with three operators `takeoff`, `land` and `flysquare` to demonstrate ROSPlan connection to quadrotor drone.

### Installation

Get the prerequisites:

(for Indigo)
Install ROSPlan according to [instructions](https://github.com/KCL-Planning/ROSPlan#installation).

Install hector quadrotor demo package: 

```sh
sudo apt-get install ros-indigo-hector-quadrotor-demo
```

Get the code:
```sh
cd src/
git clone https://github.com/fairf4x/ROSPlan_interface_quadrotor.git
```
Compile everything:
```sh
source /opt/ros/indigo/setup.bash
catkin_make
```

### Running the demo

Start rviz and gazebo:

```
roslaunch rosplan_interface_quadrotor test_quadrotor_plan.launch
```

Initialize KnowledgeBase:

```
roscd rosplan_interface_quadrotor/common/scripts/
sh init_demo.sh
```

Expected output:

The quadrotor will takeoff, fly square and land.


