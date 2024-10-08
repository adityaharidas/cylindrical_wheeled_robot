# RB1 Robot
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
## Code Explanation

### Differential Drive Controller

This section of the URDF file defines the robot's drive system using the differential drive controller plugin. It controls the left and right wheels independently.

```xml
<gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
        <leftJoint>joint_chassis_left_wheel</leftJoint>
        <rightJoint>joint_chassis_right_wheel</rightJoint>
        <wheelSeparation>0.4</wheelSeparation>
        <wheelDiameter>0.05</wheelDiameter>
        <commandTopic>cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
    </plugin>
</gazebo>
```
### **Laser Sensor Configuration**
The laser sensor provides the robot with environmental information. This sensor uses a ray to scan and detect obstacles within a certain range.

```xml
<gazebo reference="front_laser">
    <sensor type="ray" name="head_hokuyo_sensor">
        <ray>
            <scan>
                <horizontal>
                    <samples>720</samples>
                    <min_angle>-1.57</min_angle>
                    <max_angle>1.57</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.20</min>
                <max>10.0</max>
            </range>
        </ray>
        <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
            <topicName>/scan</topicName>
        </plugin>
    </sensor>
</gazebo>
```
### Rotation Service Logic
The rb1_rotate_node.cpp file defines the logic for rotating the robot using ROS services. It tracks the robot's current orientation using the Odometry data and commands angular velocity to rotate.

```cpp
bool handleRotationRequest(my_rb1_ros::Rotate::Request &request,
                           my_rb1_ros::Rotate::Response &response)
{
    ROS_INFO("Received rotation request for %d degrees", request.degrees);

    float target_angle_radians = request.degrees * (M_PI / 180.0);
    float initial_yaw = current_yaw;
    float target_yaw = initial_yaw + target_angle_radians;

    // Normalize target_yaw and rotate robot
    // ...
}
```
This code calculates the required yaw angle and rotates the robot accordingly, using feedback from the odometry.

