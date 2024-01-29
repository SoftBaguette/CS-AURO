Multi-Robot Item Retrieval System using ROS 2 and Navigation
============================================================

Introduction
------------
This document provides instructions for setting up and running the multi-robot item retrieval system with ROS 2, using TurtleBot3 Waffle Pi robots.
This system includes functionalities for autonomous navigation, item collection, obstacle avoidance, and multi-robot coordination, with integration of RViz2 for visualization and navigation for returning home.

Prerequisites
-------------
- ROS 2 Humble Hawksbill installed and sourced
- Gazebo Classic 11 for simulation
- TurtleBot3 Waffle Pi robot packages
- RViz2 for visualization
- Navigation2 package for autonomous navigation
- Python 3.8 or higher
- Basic understanding of ROS 2 concepts and command line interface
- Should work on the lab PC's we used during practicals.

Installation
------------
1. Navigate to the `src` directory of the submission

2. Build the workspace:
    cd <your path goes here>/src
    (example: cd /home/<username>/submission/src | Navigate to wherever you placed the src folder)
    colcon build --symlink-install

3. Source the ROS 2 environment:
    source install/setup.bash


Setting Up the Simulation Environment
-------------------------------------
1. Launch the Gazebo simulation environment with multiple TurtleBot3 robots:
    ros2 launch solution solution_nav2_launch.py

2. Ensure that the simulation includes objects for retrieval, obstacles, and designated area for item delivery. Otherwise it might not have loaded properly

PS: You can change the number of robots in the solution_nav2_launch.py file, as well as the seed for the item generator (spawns items in different locations depending on seed)
solution works well with 1 - 3 robots

Running the Robot Controller
----------------------------
1. To run the solution, execute the following command:
    ros2 launch solution solution_nav2_launch.py

2. The robots will autonomously navigate the environment, detect items, avoid obstacles, and perform item retrieval tasks, returning home using navigation.

PS: - Make sure you're not connected to a VPN otherwise communication might not work
    - make sure you launch solution_nav2_launch.py not solution_launch.py otherwise it wont work properly.

Monitoring and Interacting
--------------------------
- RViz2 for real-time visualization and monitoring of the robot's state, navigation paths, and environment.
- Monitor the logs for debug information, robot states, and interactions.
- Uncomment the lines in the solution_nav2_launch.py "prefix=[...]" appropriately to see logs of each robot in an individual terminal.
- Parameters can be adjusted in the code or via ROS parameters for different behaviors.

Troubleshooting
---------------
- Ensure all dependencies, including RViz2 and Navigation2, are correctly installed and sourced.
- Verify that the Gazebo simulation is properly configured and running.
- Check ROS 2 network configuration if robots are not communicating or navigating as expected.
- If robots don't spawn, make sure you're not using a VPN.
- If you encounter queue is full spam in the terminal, and robots wont move. Make sure you uncomment the "prefix=[(whatever terminal u use)]" in the nav2 launch file.


prefix=['konsole --new-tab -e'], # Opens in new window (exclusive to my workspace)
