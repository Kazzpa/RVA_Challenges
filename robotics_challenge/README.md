# Robotic Challenge
Robotic challenge in three different scenarios where turtlebot must reach a goal passed as a parameter, planning a path and avoiding obstacles while trying to perform in the shortest time possible.
The code for the three different scenarios are the same, but parameters are different.
-  **Challenge 1:** Room with all obstacles defined in map which are considered in planning phase
- **Challenge 2**: Room with at least an obstacle undefined in map that can't be considered in planning phase
- **Challenge 3:** Robotics labs that could have obstacles.
## Credits.
[Luis Merino](https://github.com/lmercab)  
[David Alejo Teissière](https://github.com/david-alejo)  
[PyOrca Algorithm To avoid obstacles](https://github.com/Muon/pyorca)  
[Python Robotics algorithms](https://github.com/AtsushiSakai/PythonRobotics)  
# Nodes
- [Control Node](#control-node)
- [Orca Node](#orca-node)
- Planner Node
## Control Node:
##### Gestiona la ruta a seguir y la dirección a seguir
#### Topics:
##### Publisher:
 - **Twist** , /cmd_vel/tracker
 - **Twist** , /cmd_vel_mux/input/navi
 - **Marker**, /visualization_marker
##### Subscriber:
 -  **Bool**, /is_goal_within_obstacle
 -  **Path**, /path_plan
##### Parameters:
 -  **Path**, path: List of points the robot must follow,undefined until callback from /path_plan.
 -  **Bool**, path_set: Variable to know if path has already been defined or not.
 -  **Bool**, skipping_goal: Defines if orca had a problem finding a new movement to avoid obstacles, set in callback from /is_goal_within_obstacle
 -  **Float**, skip_goal_max_distance: Max distance where a goal will be skipped if skipping_goal is True
 - **Float**, goal_tolerance: Distance where a goal will be considered reached.
 - **Float**, linear_max: Max linear speed robot will be able to move.
 - **Float**, angular_max: Max angular speed robot will be rotating.
 - **Float**, angle_tolerance: Degrees where the robot will move slower and adjust the rotation.
 - **Float**, linear_turning: Speed the robot will be moving when turning to achieve the degrees to goal defined in angle_tolerance.
##### Functions:
  - **__main__**: It initializes the node and defines the time ratio to which the command function will be executed, by which iterates the points of the path.
 - **command ( Int i )**:  calculates the linear and angular speed to achieve the current goal without exceed the limits defined, also manages if a goal in path should be skipped when called from /is_goal_within_obstacle , if the goal should be skipped it will calculate a new linea and angular velocity . 
Returns the current or next index in path , and a boolean to determine if the current goal was reached.
 - **path_received ( Path path )**: callback of /path_plan which sets the path.
 - **skip_goal ( Bool skipping_goal )**: callback of /is_goal_within_obstacle which sets skipping_goal to true if the goal is unreachable by orca.
 - **robot_closer_next_goal ( Int i )**: Returns the next point of the path that is closer to the current position of turtlebot
 - **publish ( Float lin_vel Float ang_vel )**: publish to the /cmd_vel/tracker topic the  desirable speed which will be sent to Orca node.
 - **publish_navi ( Float lin_vel,Float ang_vel )**: publish to the /cmd_vel_mux/input/navi topic the speed to move, this function will only be called if skipping_goal is set to true, as a way to resolve a local minimun in with orca algorithm.
 - **marker_goal ( Float goalx,Float goaly )**: Publish to /visualization_marker the next goal so we can visualize it in rviz.
## Orca Node:
##### Encargado de evitar los obstaculos recibidos por el radar y por tanto transformar la velocidad que llega de  control node a una en la que evite estos obstaculos, si no es posible enviara una señal a control para resolver el problema.
#### Topics:
##### Publisher:
 - **Twist** , /cmd_vel_mux/input/navi 
 -  **Bool**, /is_goal_within_obstacle
 - **Marker**, /visualization_marker
##### Subscriber:
 - **Twist** , /cmd_vel/tracker 
 - **LaserScan** , /scan
##### Parameters:
 -  **Float**, linear_threshold: Threshold where if isn't exceeded orca won't take action.
 -  **Float**, linear_max: Max linear speed robot will be able to move.
 -  **Float**, radius: Radius of the obstacle, which are named Agents in orca.
 -  **Float**, delta_t: Time between every call to orca, which is defined as rate of sleep.
 - **Float**, max_distance_obstacle: Max distance where a obstacle will be considered by orca to be avoided
 - **Array**, v_orca: Speed considered for orca in vector 
 - **Float**, linear: Linear speed to be published after being calculated
 - **Float**, angular: Angular speed to be published after being calculated
##### Functions:
  - **__main__**: It initializes the node and sets it to sleep while not shutdown has been invoked.
