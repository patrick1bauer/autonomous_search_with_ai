# Autonomous Search With AI

## About The Project

UCF Senior Design Project - Sponsored by Lockheed Martin

An autonomous robot that uses YOLOv3 object detection and SLAM pathfinding to locate and identify a bb8 unit

### Built With

* [ROS](https://www.ros.org/)
* [Gazebo](http://gazebosim.org/)
* [YOLOv3](https://pjreddie.com/darknet/yolo/)
* [SLAM](https://github.com/xdspacelab/openvslam)
* [LabelImg](https://github.com/tzutalin/labelImg)

## Getting Started

### Installation
1. Create an Ubuntu 20.04 LTS VM

2. Install ROS-Noetic & Dependencies
```bash
sudo apt-get update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install git
sudo apt-get install ros-noetic-desktop-full
sudo apt-get install ros-noetic-turtlebot3-*
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/gazebo_models_worlds_collection/models" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
```

CLOSE AND RE-OPEN TERMINAL

```bash
sudo rosdep init
rosdep update
```

3. Create Simulation Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

4. Get ROS Packages
```bash
cd ~/catkin_ws/src
git clone https://github.com/chaolmu/gazebo_models_worlds_collection.git
git clone https://github.com/patrick1bauer/autonomous_search_with_ai.git
git clone https://github.com/Tossy0423/darknet_ros.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

Terminal #1: Launch the simulation
```bash
roslaunch autonomous_search_with_ai start.launch
```

Terminal #2: Spawn the bb8 unit
```bash
roslaunch autonomous_search_with_ai start_bb8.launch
```

Terminal #3: Start the turtlebot3 navigation 
```bash
rosrun autonomous_search_with_ai turtlebot3_navigation.launch
```

Terminal #4: Start the object detection
```bash
roslaunch autonomous_search_with_ai yolo_v3.launch
```

Terminal #5: Start the autonomous turtlebot3
```bash
cd ~/catkin_ws/src/autonomous_search_with_ai/src
python3 waypoints.py
```

## Common Issues

If ros commands are not recognized, you might have to source ros whenever you open a new terminal.
```bash
source /opt/ros/noetic/setup.bash
```

Did you build the catkin workspace and source the setup.bash files?
If you did not modify the ~/.bashrc file to source the setup.bash files for each new terminal, you have to manually source them!
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Roadmap

See the [open issues](https://github.com/patrick1bauer/autonomous_search_with_ai/issues) for a list of proposed features (and known issues).

## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Authors

Patrick Bauer - Team Lead / Simulation Environment / Navigation - patrick1bauer@gmail.com

Pablo Trivino - Object Detection / Robot Sensors

Mark Pedroso - Object Detection / AI Training

Noah Avizemer - Simulation Environment / Mapping

Nathanel Casagnol - Navigation

## License

[GPLv3](https://github.com/patrick1bauer/autonomous_search_with_ai/blob/main/LICENSE)
