#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h"  
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/transform_datatypes.h>  // To convert Quaternion to Yaw

// Global variables
ros::Publisher velocity_publisher;
float current_yaw = 0.0;

// Function to publish angular velocity to rotate the robot
void rotateRobot(float angular_velocity)
{
    geometry_msgs::Twist velocity_message;
    velocity_message.angular.z = angular_velocity;
    velocity_publisher.publish(velocity_message);
}

// Callback function for Odometry updates to track the robot's current orientation
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Convert the quaternion to a yaw (rotation around the z-axis)
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    current_yaw = yaw;  // Set the current yaw (angle around z-axis)
}

// Function to handle the rotation service request
bool handleRotationRequest(my_rb1_ros::Rotate::Request &request,
                           my_rb1_ros::Rotate::Response &response)
{
    ROS_INFO("Received rotation request for %d degrees", request.degrees);

    // Convert degrees to radians
    float target_angle_radians = request.degrees * (M_PI / 180.0);
    float initial_yaw = current_yaw;

    // Calculate target yaw based on the current orientation
    float target_yaw = initial_yaw + target_angle_radians;

    // Normalize target_yaw to the range [-π, π]
    if (target_yaw > M_PI) target_yaw -= 2 * M_PI;
    else if (target_yaw < -M_PI) target_yaw += 2 * M_PI;

    float acceptable_error = 0.05;  // 5% tolerance for error in yaw

    // Control loop rate
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        float yaw_error = target_yaw - current_yaw;

        // Normalize yaw_error to the range [-π, π]
        if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        else if (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        ROS_INFO("Current yaw: %f | Target yaw: %f | Error: %f", current_yaw, target_yaw, yaw_error);

        // Stop rotating once the error is within the acceptable range
        if (fabs(yaw_error) < acceptable_error)
        {
            rotateRobot(0.0);  // Stop the robot
            ROS_INFO("Rotation complete. Current yaw: %f, Target yaw: %f", 
                      current_yaw * (180.0 / M_PI), target_yaw * (180.0 / M_PI));

            response.result = "Rotation completed successfully";
            return true;
        }

        // Rotate in the appropriate direction based on the yaw error
        if (yaw_error > 0.0)
        {
            rotateRobot(0.1);  // Rotate counterclockwise
        }
        else
        {
            rotateRobot(-0.1);  // Rotate clockwise
        }

        // Continue checking odometry and looping
        ros::spinOnce();
        loop_rate.sleep();
    }

    response.result = "Rotation failed or interrupted";
    return true;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "rb1_rotate_node");
    ros::NodeHandle nh;

    // Advertise the service to rotate the robot
    ros::ServiceServer rotate_service = nh.advertiseService("/rotate_robot", handleRotationRequest);
    ROS_INFO("The Service /rotate_robot is READY");

    // Advertise the publisher to command the robot's velocity
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to the odometry data to track the robot's orientation
    ros::Subscriber odometry_subscriber = nh.subscribe("/odom", 10, odometryCallback);

    // Spin to keep the node alive and responsive
    ros::spin();

    return 0;
}
