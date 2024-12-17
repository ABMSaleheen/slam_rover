# ReadMe
> This is a project where I built a simple four-wheel differential drive robot intending to do SLAM experiments.


## To run the project ------

### Mapping using `Slam_Toolbox` and `Nav2`

- Create a ros2 C++ workspace `mkdir -p ~/ros2_ws/src`.

- Go to ros2_workspace/src folder using `cd ~/ros2_ws/src`.

- Clone the repo `git clone https://github.com/ABMSaleheen/slam_rover.git -b main`.

- Go back to the ros2_workspace folder. Use `cd ..` twice or if workspace in Home folder use `cd ~/ros2_ws`.

- Build Workspace `colcon build --packages-select slam_rover --symlink-instal`.

- Source workspace  `source install/local_setup.bash`

- Launch rover in Gazebo and Rviz `ros2 launch slam_rover 1_rviz.launch.py use_sim_time:=true`.

- Launch slam_toolbox using our launch file `ros2 launch slam_rover online_async_launch.py use_sim_time:=true`.

- Launch nav2 stack using `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true`.

- Run the teleop keybord node to manually map the environment `ros2 run teleop_twist_keyboard teleop_twist_keyboard `.

At this point, mapping can be done by manually driving the rover using the `Teleop_Keyboard`. Save the Map files in the `map` folder within the package. Map files can be saved using the `Slam_Toolbox` plugin as well as the following line for saving the `.pgm` version.

- Save the generated map using `ros2 run nav2_map_server map_saver_cli -f <map_name>`. [Map will be saved whichever     folder this code is run from]

### Navigating in the map using `Nav2`

- Launch gazebo- `ros2 launch slam_rover gazebo_rover.launch.py `.

- Launch nav2- `ros2 launch slam_rover navigation2_rover.launch.py use_sim_time:=true`. This `launch` file contains information of the `<map>.yaml` and the `<nav2_param>.yaml` files.

- Use `2D Pose Estimate` in `rviz` to initialize the pose of the rover.


## Project Flow ------

### Modeling the Robot.

- The rover was designed from scratch using, Solidoworks CAD modeling software. It has the following components:

    * Chassis.

    * 4 Wheels with Gazebo differential drive plugin.

    * 2 Lidars, Front and Back each covering a range of 4 meters in its 180 degrees zone. Purpose is to detect collisions.

    * 2 cameras, Front and Back. Purpose is to do Computer Vision to follow a track as well as observe surroundings.


- Solidoworks' `sw2urdf` extension was used to export the urdf model of the robot from the CAD model. 

- The model components are saved in `.stl` format in a folder which is then referred to inside the `.urdf` file. 

- To implement specific colors on each component, it is necessary to convert the `.stl` into a `.dae` model file using `Blender` or similar software.




