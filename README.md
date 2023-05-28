# DDR-SLAM: Simultaneous Localization and Mapping of Differential Drive Robots

DDR-SLAM is a ROS2-based project that implements Simultaneous Localization and Mapping (SLAM) for differential drive robots. The project uses Gazebo Fortress as a simulation environment to test the SLAM algorithm.

## Installation

To install DDR-SLAM, you need to have ROS2 Humble and Gazebo Fortress installed on your system. You can follow the official ROS2 Humble installation guide [here](https://docs.ros.org/en/humble/Installation.html) and the Gazebo Fortress installation guide [here](https://gazebosim.org/docs/fortress/install).

Once you have ROS2 and Gazebo Fortress installed, create a ROS2 workspace, cd into it and clone the DDR-SLAM repository and build the project:

```
mkdir ros2_workspace
cd ros2_workspace
git clone https://github.com/mouaff25/DDR-SLAM.git
cd DDR-SLAM
git submodule init
git submodule update
cd ..
mv DDR-SLAM src
```

Before building, you should set the Ignition version:
```
export IGNITION_VERSION=fortress
cd ~/ros2_workspace/src
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/ros2_workspace
colcon build
```

## Usage

To launch the DDR-SLAM algorithm in the Gazebo Fortress environment, you can use the provided launch file:

```
source ~/ros2_workspace/install/local_setup.bash
ros2 launch ppp_bot legacy_launch_sim.launch.py
```

The launch file will start Gazebo Fortress with a differential drive robot and launch the DDR-SLAM algorithm. You can then move the robot around in the environment and observe the SLAM output in RViz.

## Results

Here are some sample results of the DDR-SLAM algorithm running in the Gazebo Fortress environment:

![SLAM output in RViz](images/slam_output.png)

## Acknowledgements

This project was inspired by the [gmapping](http://wiki.ros.org/gmapping) ROS package and the [Gazebo Fortress](https://github.com/osrf/gazebo_ros_pkgs/tree/ros2/gazebo_ros_pkgs) simulation environment. Thanks to the ROS2 and Gazebo communities for their contributions to these projects.

## License

DDR-SLAM is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For any questions or feedback, please contact the authors:

Mouafak Dakhlaoui
mouaffak9@gmail.com

Mohamed Yassine Nefzi
ynyassine7@gmail.com

Jawhar Djebbi
Jawhardjebbi@gmail.com
