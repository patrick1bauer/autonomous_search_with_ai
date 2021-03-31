# Autonomous Turtlebot3 Swarm

## About The Project

UCF Senior Design Project - Sponsored by Lockheed Martin
An autonomous drone swarm that uses YOLOv3 object detection and SLAM pathfinding to locate and disable a turtlebot

### Built With

* [ROS](https://www.ros.org/)
* [Gazebo](http://gazebosim.org/)
* [YOLOv3](https://pjreddie.com/darknet/yolo/)
* [SLAM](https://github.com/xdspacelab/openvslam)
* [LabelImg](https://github.com/tzutalin/labelImg)

## Getting Started

### Installation

1. Create an Ubuntu 16.04 LTS VM or dualboot

2. Install ROS-Kinetic & Dependencies
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo dpkg --configure -a
sudo apt-get install libignition-math2-dev
sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install ros-kinetic-hardware-interface
sudo apt-get install ros-kinetic-controller-interface
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-ros-pkgs
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-geographic-msgs
sudo apt-get install ros-kinetic-teleop-twist-keyboard
sudo apt-get install ros-kinetic-turtlebot3-*
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo-7/setup.sh" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/gazebo_models_worlds_collection/models" >> ~/.bashrc
echo "export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
```

CLOSE AND RE-OPEN TERMINAL

```bash
sudo rosdep init
rosdep update
sudo apt-get install python3.8 python3-pip
```

3. Update gazebo
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7
```

4. Create Simulation Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

5. Get ROS Packages
```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/chaolmu/gazebo_models_worlds_collection.git
git clone https://github.com/patrick1bauer/autonomous_turtlebot3_swarm.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

Terminal #1: Launch the simulation
```bash
roslaunch autonomous_turtlebot3_swarm start.launch
```

Terminal #2: Start the turtlebot3 navigation 
```bash
rosrun autonomous_turtlebot3_swarm turtlebot3_navigation.launch
```

Terminal #3: Start the autonomous turtlebot3
```bash
rosrun autonomous_turtlebot3_swarm test.py
```

## Common Issues

If ros commands are not recognized, you might have to start roscore in a separate terminal before running the simulation.
```bash
roscore
```

Did you build the catkin workspace and source the setup.bash files?
If you did not modify the ~/.bashrc file to source the setup.bash files for each new terminal, you have to manually source them!
```bash
cd ~/catkin_ws
catkin_make
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## Roadmap

See the [open issues](https://github.com/patrick1bauer/autonomous_turtlebot3_swarm/issues) for a list of proposed features (and known issues).

## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Authors

Patrick Bauer - Team Lead - patrick1bauer@gmail.com

Nathanel Casagnol - SLAM

Noah Avizemer - Target Elimination

Mark Pedroso - Robot Vision

Pablo Trivino - Robot Vision

## License

[GPLv3](https://github.com/patrick1bauer/autonomous_drone_swarm/blob/main/LICENSE)
