source install/local_setup.bash
ros2 run more_interfaces publish_address_book

#open another terminal, source the workspace, and call topic echo:

source install/setup.bash
ros2 topic echo /address_book


#Create a package

ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp


#Build and run

rosdep install -i --from-path src --rosdistro iron -y

colcon build --packages-select cpp_parameters

#Open a new terminal, navigate to ros2_ws, and source the setup files:

source install/setup.bash

ros2 run cpp_parameters minimal_param_node


#Change via the console. Make sure the node is running:\

ros2 run cpp_parameters minimal_param_node

#Open another terminal

ros2 param list

ros2 param set /minimal_param_node my_parameter earth

#Open a console and navigate to the root of your workspace, ros2_ws, and build your new package:

colcon build --packages-select cpp_parameters

#Then source the setup files in a new terminal:

source install/setup.bash

ros2 launch cpp_parameters cpp_parameters_launch.py


#Create a package

ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy


#Build and run

rosdep install -i --from-path src --rosdistro iron -y

colcon build --packages-select python_parameters

#Open a new terminal, navigate to ros2_ws, and source the setup files:

source install/setup.bash

ros2 run python_parameters minimal_param_node


#Change via the console

ros2 run python_parameters minimal_param_node

#Open another terminal, source the setup files from inside ros2_ws again, and enter the following line:

ros2 param list

#There you will see the custom parameter my_parameter. To change it, simply run the following line in the console:

ros2 param set /minimal_param_node my_parameter earth

colcon build --packages-select python_parameters

source install/setup.bash

ros2 launch python_parameters python_parameters_launch.py


#check setup

ros2 doctor

#Check a system

ros2 run turtlesim turtlesim_node

#Open another terminal and source ROS 2 to run the teleop controls:

ros2 run turtlesim turtle_teleop_key

ros2 topic echo /turtle1/color_sensor

ros2 topic echo /turtle1/pose

#Get a full report

ros2 doctor --report


#Create the base class package

ros2 pkg create --build-type ament_cmake --dependencies pluginlib --node-name area_node --license Apache-2.0 polygon_base

#Create the Plugin Package

ros2 pkg create --build-type ament_cmake --dependencies polygon_base pluginlib --library-name polygon_plugins --license Apache-2.0 polygon_plugins


#Build and run

colcon build --packages-select polygon_base polygon_plugins

source install/setup.bash

ros2 run polygon_base area_node

