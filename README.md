# formation_controller_ros
Formation controller for multi-agent leader-follower in ROS


## How to run
Follow [here](https://docs.px4.io/master/en/simulation/multi_vehicle_simulation_gazebo.html#multiple-vehicles-with-ros-and-gazebo) for PX4 multi-sim setup.

Launch formation_controller node for all UAVs:
```
roslaunch formation_controller formation_flt_Xm_radius_X_uav.launch
```
Take off and publish /uavX/user commands.

## Helper script to generate launch files for
Input the number of agents (<=10) and formation radius accordingly.
```
python launch-gen.py --num_agent 5 --radius 5
```
Two launch files will be generated, where X is the corresponding input:
- ```Xm_radius_X_uav_mavros_sitl.launch```
- ```formation_flt_Xm_radius_X_uav.launch```
