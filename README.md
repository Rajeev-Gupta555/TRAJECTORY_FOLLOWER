I've developed a system designed to guide robots along specific paths using a method called stochastic gradient. This system is meant for controlled environments, where a specialized controller employs data about the robot's movement to determine optimal actions for keeping it on the desired path.

## Contents of the "Scripts" Folder

1. **diff_drive.py:** This file sets up a simulation for a differential drive robot. It contains functions that enable the robot to move using a provided velocity and calculate its position based on the given velocity.

2. **holonomic_drive.py:** Similar to the previous file, this one establishes a simulation for a holonomic robot. It also includes functions to move the robot and update its position.

3. **path_planner_class.py:** Defines a class named "Path_Planner" that's capable of following a trajectory explicitly specified as a function of x. Alternatively, a target point can be provided. The class computes appropriate speeds for the robot to reach the target point as quickly and directly as possible. Once in motion, the class ensures the robot stays on the chosen path.

4. **trajectory_follower_diff_drive.py:** This file implements the "Path_Planner" class to control the movement of a differential drive robot, utilizing functions from "diff_drive.py".

5. **trajectory_follower_holonomic_drive.py:** Similarly, this file controls the movement of a holonomic drive robot using functions from "holonomic_drive.py".

## About the Path Planner

The "Path_Planner" class in the trajectory_follower* files accepts a target point as input, determining the optimal way for the robot to reach that point. It can also be provided with a specific path equation, such as y = f(x), for the robot to follow. The class calculates suitable speeds for the robot to reach the target point efficiently and directly. As the robot moves, the class ensures it remains on the desired path.

To visualize the robot's progress, I've employed the Matplotlib library. When the robot reaches its goal, an image is generated that displays the intended path in green and the actual path the robot followed in red. This provides a visual representation of the robot's path-following performance.

## Getting Started
1. Clone the repository from https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER.git.
2. For controlling a differential drive, utilize the trajectory_follower_diff_drive.py file. The code is straightforward and self-explanatory.
3. Adjust the physical parameters in the Diff_drive object according to your robot's specifications.
4. Modify the physical limits in Path_Planner, such as max_linear_vel, max_angular_vel, and x-y limits for the displayed image.
5. Once adjustments are made, run the Python file. If a target point is provided, the controller will stop the robot upon reaching it. If a trajectory function is given, manually stop the program using Ctrl-C.
6. The terminal will display the robot's X, Y, and YAW positions.

## Output Images
#### *Before Tuning Physical Parameters*:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/result_before_tuning.png)

#### *After Tuning Physical Parameters (e.g., Velocity Multiplication Factor)*

Point 1:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/point.png)

Point 2:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/point2.png)

Parabola:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/parabola.png)

Shifted Parabola:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/parabola_y_shifted.png)

## Contribution
If you find this work valuable, please share your experience. Feel free to contribute to this project to enhance its user-friendliness and reduce the likelihood of runtime errors.
