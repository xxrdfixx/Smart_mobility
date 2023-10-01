# Rosdep installation
pip install rosdep

# rosdep operation
sudo rosdep init
rosdep update

# we can run rosdep install to install dependencies
rosdep install --from-paths src -y --ignore-src

# creating an interface package
mkdir -p ~/ros2_ws/src # you can reuse an existing workspace with this naming convention
cd ~/ros2_ws/src
ros2 pkg create --license Apache-2.0 custom_action_interfaces

# defining an action
# Creating an action directory in ROS2 package custom_action_interfaces:
cd custom_action_interfaces
mkdir action

# building an action
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build

# checking if our action built successfully 
source install/local_setup.bash

# Now checking that if action definition exists:
ros2 interface show custom_action_interfaces/action/Fibonacci

# Creating the custom_action_cpp package
cd ~/ros2_ws/src
ros2 pkg create --dependencies custom_action_interfaces rclcpp rclcpp_action rclcpp_components --license Apache-2.0 -- custom_action_cpp

# compile the packag
colcon build

#  Running the action server
ros2 run custom_action_cpp fibonacci_action_server

# In the first shell, start the component container:
ros2 run rclcpp_components component_container

#On the second shell and verify that the container is running via ros2 command line tools:
ros2 component list

# in the second shell load the talker component (see talker source code):
ros2 component load /ComponentManager composition composition::Talker

# load the listener component:
ros2 component load /ComponentManager composition composition::Listener

# inspect the state of the container:
ros2 component list

# Run-time composition using ROS services with a server and client
# In the first shell:
ros2 run rclcpp_components component_container

# in the second shell (see server and client source code):
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client

# Compile-time composition using ROS services
ros2 run composition manual_composition

# Run-time composition using dlopen
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

# Unloading components
# In the first shell, start the component container:
ros2 run rclcpp_components component_container

# Verifying that the container is running via ros2 command line tools:
ros2 component list

# In the second shell load both the talker and listener as we have before:
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener

# Using the unique ID to unload the node from the component container
ros2 component unload /ComponentManager 1 2


# Remapping container name and namespace
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns


# In a second shell, components can be loaded by using the updated container name:
ros2 component load /ns/MyContainer composition composition::Listener

# Remap component names and namespaces
# In the first shell, start the component container:
ros2 run rclcpp_components component_container

# Remaping node name
ros2 component load /ComponentManager composition composition::Talker --node-name talker2

# Remaping namespace
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns

# Remaping both
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2

# using ros2 command line utility:
ros2 component list

# Passing parameter values into components
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true

# Passing additional arguments into components
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true

# Create package
ros2 pkg create --build-type ament_cmake cpp_parameter_event_handler --dependencies rclcpp

# Build and run
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Navigating back to the ros2_ws, and building new package:
colcon build --packages-select cpp_parameter_event_handler

# In new terminal, navigating to ros2_ws, and sourcing the setup files:
. install/setup.bash

# Now run the node:
ros2 run cpp_parameter_event_handler parameter_event_handler

ros2 param set node_with_parameters an_int_param 43

# navigating back to the ros2_ws, and building updated package as before:
colcon build --packages-select cpp_parameter_event_handler

# sourcing the setup files:
. install/setup.bash

# test monitoring of remote parameters, first run the newly-built parameter_event_handler code:
ros2 run cpp_parameter_event_handler parameter_event_handler

# from another teminal, running the parameter_blackboard demo application
ros2 run demo_nodes_cpp parameter_blackboard

#from a third terminal setting a parameter on the parameter_blackboard node
ros2 param set parameter_blackboard a_double_param 3.45



#Creating a launch file

# Creating a new directory to store launch files:
mkdir launch

# to run the launch file, entering into the directory that was created earlier
cd launch
ros2 launch turtlesim_mimic_launch.py

# opening a new terminal and running the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

# Introspect the system with rqt_graph
rqt_graph

# Creating a workspace for the package to live in:
mkdir -p launch_ws/src
cd launch_ws/src

ros2 pkg create py_launch_example --build-type ament_python

# Building and running the launch file
colcon build
ros2 launch py_launch_example my_script_launch.py

# Creating a new package of build_type ament_python called launch_tutorial:
ros2 pkg create launch_tutorial --build-type ament_python

# Inside of that package, creating a directory called launch:
mkdir launch_tutorial/launch

# building the package:
colcon build

# Launching the example_main_launch.py file using the ros2 launch command.
ros2 launch launch_tutorial example_main_launch.py

# Modifying launch arguments
ros2 launch launch_tutorial example_substitutions_launch.py --show-args

# passing the desired arguments to the launch file
ros2 launch launch_tutorial example_substitutions_launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

# Build and run
ros2 launch launch_tutorial launch_turtlesim_launch.py

# Running the demo
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py

# in the second terminal window
ros2 run turtlesim turtle_teleop_k

# creating a diagram of the frames being broadcast by tf2 over ROS
ros2 run tf2_tools view_frames

# reporting the transform between any two frames broadcasted over ROS
ros2 run tf2_ros tf2_echo turtle2 turtle1

# visualization tool that is useful for examining tf2 frames
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz

#  Create a package
ros2 pkg create --build-type ament_python learning_tf2_py

# Write the static broadcaster node
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py

# Build
rosdep install -i --from-path src --rosdistro iron -y

#Still in the root workspace, building new package:
colcon build --packages-select learning_tf2_py

# in a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

# running
ros2 run learning_tf2_py static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0

# checking that the static transform has been published by echoing the tf_static topic
ros2 topic echo /tf_static

# Publishing a static coordinate transform to tf2
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id

# Publishing a static coordinate transform to tf2 using an x/y/z offset in meters and quaternion.
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id

# Writing a broadcaster
# Write the broadcaster node
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py

# Build
rosdep install -i --from-path src --rosdistro iron -y

#Still in the root of your workspace, build your package
colcon build --packages-select learning_tf2_py

# In a new terminal, navigate to the root and source the setup files:
. install/setup.bash

# running the launch file that will start the turtlesim simulation node and turtle_tf2_broadcaster node
ros2 launch learning_tf2_py turtle_tf2_demo_launch.py

# In the second terminal window typing
ros2 run turtlesim turtle_teleop_key

# checking if the turtle pose is actually getting broadcast to tf2:
ros2 run tf2_ros tf2_echo world turtle1

# Write the broadcaster node
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_broadcaster.cpp

# Build
rosdep install -i --from-path src --rosdistro iron -y

#From the root of your workspace, build your updated package:
colcon build --packages-select learning_tf2_cpp

# In a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

# launching file that will start the turtlesim simulation node and turtle_tf2_broadcaster node
ros2 launch learning_tf2_cpp turtle_tf2_demo_launch.py

# In the second terminal window type the following command:
ros2 run turtlesim turtle_teleop_key

# useing the tf2_echo tool to check if the turtle pose is actually getting broadcast to tf2
ros2 run tf2_ros tf2_echo world turtle1



# Writing a listener (Python)

# Write the listener node
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py

# Build
rosdep install -i --from-path src --rosdistro iron -y

# Still in the root of your workspace, build your package
colcon build --packages-select learning_tf2_py

# In a new terminal, navigate to the root of your workspace, and source the setup files
. install/setup.bash

# Run
ros2 launch learning_tf2_py turtle_tf2_demo_launch.py

# In the second terminal window
ros2 run turtlesim turtle_teleop_key

# Write the fixed frame broadcaster
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/fixed_frame_tf2_broadcaster.py

# Running rosdep in the root of your workspace to check for missing dependencies
rosdep install -i --from-path src --rosdistro iron -y

# Still in the root of your workspace, build your package
colcon build --packages-select learning_tf2_py

# in a new terminal, navigate to the root of your workspace, and source the setup files
. install/setup.bash

# running the launch file:
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo_launch.py

# Write the dynamic frame broadcaster
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/dynamic_frame_tf2_broadcaster.py

# Running rosdep in the root of your workspace to check for missing dependencies
rosdep install -i --from-path src --rosdistro iron -y

# Still in the root of your workspace, build package
colcon build --packages-select learning_tf2_py

# in a new terminal, navigate to the root of your workspace, and source the setup files
. install/setup.bash

#run the launch file
ros2 launch learning_tf2_py turtle_tf2_dynamic_frame_demo_launch.py

# Using time (Python)
# Update the listener node
ros2 launch learning_tf2_py turtle_tf2_demo_launch.py

#running the launch file
ros2 launch learning_tf2_py turtle_tf2_demo_launch.py

# Time travel
# Checking the results
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo_launch.py

#Build and run your tests
colcon test --ctest-args tests [package_selection_args]

#Examine Test Results
colcon test-result –all
colcon test-result --all –verbose

#Debugging tests with GDB
colcon build --cmake-clean-cache --mixin debug
gdb -ex run ./build/rcl/test/test_logging

#Special Commands
colcon test --packages-select <name-of-pkg> --pytest-args -k name_of_the_test_function
colcon test --event-handlers console_cohesion+

#One Shape
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/01-myfirst.urdf

#Multiple Shapes
ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf

#Origins
ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf

#Material Girl
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf

#Finishing the Model
ros2 launch urdf_tutorial display.launch.py model:=urdf/05-visual.urdf

#Building a movable robot model
ros2 launch urdf_tutorial display.launch.py model:=urdf/06-flexible.urdf

#Practical Usage
ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macroed.urdf.xacro
Tasks

#Create a package
mkdir -p ~/second_ros2_ws/src
cd ~/second_ros2_ws/src
ros2 pkg create urdf_tutorial_r2d2 --build-type ament_python --dependencies rclpy --license Apache-2.0
cd urdf_tutorial_r2d2

#Create the URDF File
mkdir -p urdf

# Install the package
cd ~/second_ros2_ws
colcon build --symlink-install --packages-select urdf_tutorial_r2d2
source install/setup.bash

#View the results
ros2 launch urdf_tutorial_r2d2 demo_launch.py
rviz2 -d ~/second_ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz






























