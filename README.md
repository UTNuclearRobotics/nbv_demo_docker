## Set Up Docker Container for NBV Repository

Run the Docker container. Inside the workspace, create a `src` directory and clone the repositories below:
- [nbv_active](https://github.com/UTNuclearRobotics/active_nbv/tree/ros_noetic)
- [spot_manipulation_moveit](https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit/tree/nrg_devel)
- spot_description

```
cd nbv_demo_ws/src
git clone -b ros_noetic --recursive git@github.com:UTNuclearRobotics/active_nbv.git
git clone -b nrg_devel https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit/tree/nrg_devel
```

## Running the Demo on Spot

### Terminals on Spot
All the steps in this section are in it's own terminal inside Spot.
```
ssh spot@192.168.11.151
```

1. ROS Core: run `roscore`.
```
export ROS_MASTER_URI=http://192.168.11.151:11311
export ROS_IP=192.168.11.151
source /opt/ros/noetic/setup.bash
roscore
```

2. Spot Driver: connect to the AugRE51 Wi-Fi network and launch the NRG Spot driver from the `spot_manipulation_driver` package. 
```
export ROS_MASTER_URI=http://192.168.11.151:11311
export ROS_IP=192.168.11.151
source catkin_ws/devel/setup.bash
roslaunch spot_manipulation_driver nrg_spot_driver.launch
```

3. Spot Tele-Op: launch the tele-operation node from the `teleop_twist_joy` package.
```
export ROS_MASTER_URI=http://192.168.11.151:11311
export ROS_IP=192.168.11.151
source /opt/ros/noetic/setup.bash
roslaunch teleop_twist_joy teleop.launch config_filepath:=/home/spot/catkin_ws/atk3.config.yaml
```

### Terminals on Laptop Docker Container
All the steps in this section are run in it's own terminal inside the Docker container.

4. MoveIt: launch the Spot arm MoveIt node from the [spot_arm_moveit_config](https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit/tree/nrg_devel) package.
```
export ROS_MASTER_URI=http://192.168.11.151:11311
export ROS_IP=192.168.11.206
source catkin_ws/devel/setup.bash
roslaunch spot_arm_moveit_config spot_arm_planning_execution.launch
```

5. Arm Pose Script: if ROS is correctly configured on the laptop, then the python MoveIt Commander API should pickup the MoveIt node (`spot_arm_moveit_config`) and start following the arm motions.
```
export ROS_MASTER_URI=http://192.168.11.151:11311
export ROS_IP=192.168.11.206
source nbv_demo_ws/devel/setup.bash
python3 moveit_joint_execute.py
```

The `moveit_joint_execute.py` depends on `moveit_client.py` and some ROS packages in `active_nbv` repository (robot_helpers) and MoveIt Commander itself. The arm poses are pre-computed for different base paths. By default it should be pre-computed for the S-shaped path around the lab (Starting at the red workbench facing Blake's desk and ending at the back of the lab).
