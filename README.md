# Trajectory Follower System

I've developed a system for guiding robots along specific paths, using a method called stochastic gradient. This system operates within a controlled environment, where a special controller uses data about how the robot is moving to determine the best actions for keeping it on the desired path.

## Files in the "Scripts" Folder

1. **diff_drive.py:** This file contains a simulation setup for a type of robot called a differential drive robot. It has a function that lets the robot move by providing it with a velocity and another function that calculates the robot's position based on the velocity provided.

2. **holonomic_drive.py:** Similar to the previous file, this one sets up a simulation for a holonomic robot. It also has functions for moving the robot and updating its position.

3. **trajectory_follower_diff_drive.py:** This file introduces a class named "Path_Planner" that's responsible for controlling the movement of the differential drive robot created using the functions from "diff_drive.py".

4. **trajectory_follower_holonomic_drive.py:** Similarly, this file includes a "Path_Planner" class to control the movement of the holonomic drive robot created using the functions from "holonomic_drive.py".

## About the Path Planner

The "Path_Planner" class found in the trajectory_follower* files can be given a target point to aim for. It then figures out the best way for the robot to reach that point. You can also provide a specific path equation like y = f(x) for the robot to follow. The class calculates the right speeds for the robot to get to the target point as quickly and directly as possible. Once the robot is on its way, the class keeps it moving along the chosen path.

To help you see the robot's progress, I've used a library called Matplotlib. When the robot reaches its goal, a picture appears that shows the desired path in green and the actual path the robot took in red. This gives a visual representation of how well the robot followed the path.

## Output images:
#### *Without tuning any physical parameter*:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/result_before_tuning.png)

#### *After tuning physical parameters (eg. multiplication factor that drives the velocity from normal velocity to tangential velocity w.r.t the curve...)*

Point1:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/point.png)

Point2:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/point2.png)

Parabola:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/parabola.png)

Parabola_Y_Shifted:
![Alt Text](https://github.com/Rajeev-Gupta555/TRAJECTORY_FOLLOWER/blob/main/trajectory_follower/images/parabola_y_shifted.png)
