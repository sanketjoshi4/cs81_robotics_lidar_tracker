# cs81_robotics_lidar_tracker
Using LIDAR to detect, track and pursue a moving entity in a world with other moving entities and obstacles

## Instructions
Paste the package (i.e. target_following directory) in a valid catkin workspace.

`catkin_make` in the root workspace dir.

`cd path/to/package/scripts`

`chmod +x robot.py` or target.py, depending on which executable you want to test/run.

Run in different windows:

1. `roscore`

2. `rosrun stage_ros stageros PA1.world`

3. `rosrun map_server map_server archipelago.yaml`

4. `rosrun target_following executable_file_name.py`
