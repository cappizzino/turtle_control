#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

class TurtleController
{
private:
    ros::Publisher cmd_vel_pub;
    ros::Subscriber pose_subscriber;  

    turtlesim::Pose current_pose;

    // Velocity commands
    double linear_speed;
    double angular_speed;

    // Desired position
    double x_goal;
    double y_goal;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        // TODO: Control code goes here
        msg.linear.x = linear_speed;
        msg.angular.z = angular_speed;

        return msg;
    }

public:
    TurtleController(){
        // Initialize ROS
        ros::NodeHandle n;

        // Get parameters from the parameter server
        ros::NodeHandle nh("~");
        nh.param("linear_speed", linear_speed, 0.5);
        nh.param("angular_speed", angular_speed, 0.2);

        nh.param("x_goal", x_goal, 8.0);
        nh.param("y_goal", y_goal, 8.0);

        // Create a publisher object, able to push messages
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // Create a subscriber to get the turtle's current pose
        pose_subscriber = n.subscribe("/turtle1/pose", 10, &TurtleController::poseCallback, this);
    }

    void run()
    {
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            cmd_vel_pub.publish(msg);

            // And throttle the loop
            loop_rate.sleep();
        }
    }

    void poseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        current_pose = *msg;
    }

    void moveToGoal()
    {
        geometry_msgs::Twist vel_msg;

        ros::Rate loop_rate(10);
        float distance = std::sqrt(std::pow((x_goal - current_pose.x), 2) +
                                std::pow((y_goal - current_pose.y), 2));

        while (ros::ok() && distance > 0.01)
        {
            float angle_to_goal = std::atan2(y_goal - current_pose.y, x_goal - current_pose.x);
            float angular_error = angle_to_goal - current_pose.theta;

            // control the linear and angular velocities
            vel_msg.linear.x = 1.5 * distance;
            vel_msg.angular.z = 4.0 * angular_error;

            cmd_vel_pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();

            distance = std::sqrt(std::pow((x_goal - current_pose.x), 2) +
                                std::pow((y_goal - current_pose.y), 2));
        }

        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        cmd_vel_pub.publish(vel_msg);
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "controller");

    // Create our controller object and run it
    auto controller = TurtleController();
    controller.moveToGoal();

    // And make good on our promise
    return 0;
}