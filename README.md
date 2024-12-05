Repository for Homework 3 of Robotics Labs course made by Marco Bartone P38000237, Giacomo Caiazzo P38000236, Matteo De Simone P38000232, Nicola Monetti P38000238.

# Robotics Lab - Homework 3

### Overview
This is a report of the Homework 3 of Robotics Lab course using Docker and ROS2 with Gazebo. The repo contains the steps to download the folders from github and to run the launch file for the simulations of the built manipulator robot with the velocity controller, the joint space inverse dynamic torque controller or the operational space inverse dynamics torque controller in a world with an object to detect (a sphere or an aruco marker) with a fixed camera on the end-effector.

### Usage

Open the terminal, open the container and enter into the directory where you want to download the folder, then download it with:

      $ git clone https://aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.git
            
--------------------------------

It's possible to load two different worlds in Gazebo: new.world, which spawns the manipulator and a blue sphere and empty.world, which spawns the manipulator and an arucotag. To use one of them, open iiwa.launch.py (/ros2_iiwa/iiwa_bringup/launch folder) and change the last path string parameter ("new.world" for the sphere, "empty.world" for the arucotag) of the PathJoinSubstitution in line 328 as showed:

	iiwa_simulation_world = PathJoinSubstitution(
            [FindPackageShare(description_package),
                'gazebo/worlds', 'empty.world'] #empty.world contains arucotag, new.world contains sphere 
    	)


Furthermore, to detect the aruco marker or the sphere (because they are in different positions), open initial_position.yaml (/ros2_iiwa/iiwa_description/config folder) and comment/uncomment the joints initial positions of the aruco marker section or the sphere section.

--------------------------------

To build the packages, enter into the ROS2 workspace and build them with:

      $ colcon build

Atfer this, use the source command:

      $ source install/setup.bash
      

To load the world with the blue sphere and the manipulator with the camera for the detection, run the simulations with:

      $ ros2 launch iiwa_bringup iiwa.launch.py use_vision:=true

After this, open another terminal and use the following command to detect the sphere:
      
      $ ros2 run ros2_opencv ros2_opencv_node



In order to load the velocity controller, launch the simulation with the following command (this section only refers to the empty.world case):

      $ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_vision:=true
      
To load the torque controller, use the previous commands on the terminal with command_interface:="effort" and robot_controller:="effort_controller".

After this, open another terminal and use the command:

      $ ros2 launch aruco_ros simple_single.launch.py
      
Open another terminal and use the following command specifying as node arguments task:= pos | lap | merge and cmd_interface:= velocity | effort :
      
      $ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="" -p task:=""
      
