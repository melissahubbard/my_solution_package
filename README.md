Welcome to my directory. Here I will explain how I get to my project.

- I have also added screenshots and my video via my Google Drive link here: 

https://drive.google.com/drive/folders/1983vn1CMvtaP5NM_10p5eahksVqEPTbn?usp=sharing

(Screenshots of how to get to my turtlesim program)
(Screenshots of previous exercises from Reflect Robotics)

The package follows the standard ROS2 package structure:

src
    └── solution_package
        ├── launch
        │   └── solution_node.launch.py
        ├── package.xml
        ├── README.md
        ├── setup.cfg
        ├── setup.py
        ├── solution_package
        │   ├── __init__.py
        │   └── m_h_tc.py
        └── test
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py

Prerequisites:
- Ubuntu 20.04
- ROS 2 Foxy (or later)

Setup and Build + Run:

1. Clone the Github repository into your ROS workspace's 'src' folder:

cd ~/ros2_ws/src
git clone https://github.com/melissahubbard/solution_package.git

2. After cloning, build the package:

cd ~/ros2_ws
colcon build

3. Run Project

Source the workspace:

source ~/ros2_ws/install/setup.bash

4. Finally, launch

ros2 launch solution_package solution_node.launch.py

or ros2 run solution_package m_h_tc



