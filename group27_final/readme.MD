# Final Project - Group 27

## Run Instructions


1. Place the group27_final package in the src folder 
2. Colcon build the workspace
3. run the line in terminal 'source install/setup.bash'
4. run the following command to launch the gazebo with maze 

```
ros2 launch final_project final_project.launch.py
```
5. Then run the launch file to load the parameters and start the broadcaster and listener

```
ros2 launch group27_final wafflebot.launch.py 

```
6. Run the below script to start the wafflebot navigation
 
```
ros2 run group27_final wafflebot_navigation 

```
All the above files must be run one after the other in different terminals
* To navigate the robot, we have two options. 1. follow_waypoints 2. navthroughposes.
* By default, our launch file runs follow_waypoints node because it is able to reach all the goals.



* Kill all the nodes and rerun if any node is crashing.

* Note: Navthrough poses doesnot follow the waypoints if the current and next goal paths are overlapping. So it will not work properly for the aruco_1 marker waypoints.

