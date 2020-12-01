# cs81_robotics_lidar_tracker

### Archita Harathi, Sanket Joshi, Jospehine Nguyen 
### CS 81, Fall 2020 

Using LIDAR to detect, track, and pursue a moving entity in a world with other moving entities and obstacles.

## Instructions
Paste the package (i.e. target_following directory) in a valid catkin workspace.

`catkin_make` in the root workspace dir.

`cd path/to/package/scripts`

`chmod +x robot.py`, target.py, and obstacle.py, depending on which executable you want to test/run (all of them for the main simulation).

Run in different windows:

1. `roscore`

2. `rosrun stage_ros stageros demo.world`

4. `rosrun target_following target.py`

After this command, you will be prompted for integer input of 1, 2, or 3. Type the integer that corresponds to the target mode you would like to see (1=hardcoded, 2=sinusoidal, 3=randomized) and press Enter.

5. `rosrun target_following robot.py`

6. `rosrun target_following obstacle.py`

Every simulation exports a metrics csv file in /scripts/metrics. Inorder to generate a multiline graph, enter the following command

`python metrics_plot.py <path_to_metrics_file_1>.csv <path_to_metrics_file_2>.csv ...`