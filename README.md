# Fundamental-of-Robot
Robot manipulator kinematics
Basically, if I want to navigate the robot moves toward a specific goal point from the origin without colliding with obstacles. I need first to clarify the obstacles’ positions and run the A* algorithm to search for an optimal path and return the list of waypoints. Then Use the obtained waypoints to generate polynomial trajectories for the robot to follow.
