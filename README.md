# Robot Rotation Service
This project defines a ROS (Robot Operating System) service that allows a differential-drive robot to rotate by a specified number of degrees. The service takes requests to rotate the robot, calculates the required rotation based on current orientation (yaw), and commands the robot's velocity to achieve the desired turn.
## Features
- A ROS-based service that allows for precise robot rotation using differential drive.
- Utilizes ROS messages like `Twist` for velocity control and `Odometry` for real-time orientation tracking.
- A configurable Gazebo robot model with laser sensor and caster wheels.
---

## How It Works

The robot is controlled using a differential drive controller in Gazebo, allowing it to rotate based on velocity commands. The rotation service calculates the required angular velocity and commands the robot to rotate by a specific number of degrees.

### Key Components:
- **Differential drive controller**: Controls the left and right wheels of the robot.
- **Odometry feedback**: Tracks the robot's current orientation (yaw) for precise turns.
- **Laser sensor**: Provides environmental data (not directly used in rotation but important for overall navigation).

---
## Usage

### Launching the robot and service

1. Start Gazebo with the robot model:
   ```
   roslaunch my_rb1_gazebo my_rb1_robot_warehouse.launch
   ```
2. Run the rotation service node:
   ```
   rosrun my_rb1_ros rb1_rotate_node
   ```
3. Call the rotation service:
   ```
    rosservice call /rotate_robot "{degrees: 90}"
   ```
 This will rotate the robot by 90 degrees.

---  
