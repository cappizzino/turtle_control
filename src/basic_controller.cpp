#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"

class TurtleController
{
private:
    ros::Publisher cmd_vel_pub;

    // Velocity commands
    double linear_speed;
    double angular_speed;

    // Service to toggle control
    ros::ServiceServer toggle_service_;
    bool is_active_ = false;

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

        ros::NodeHandle nh("~");
        nh.param("linear_speed", linear_speed, 0.5);
        nh.param("angular_speed", angular_speed, 0.2);

        // Create a publisher object, able to push messages
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

        // Service client for controlling the turtle
        toggle_service_ = n.advertiseService("toggle_control", &TurtleController::toggleControl, this);
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            if (is_active_)
            {
                // Calculate the command to apply
                auto msg = calculateCommand();

                // Publish the new command
                cmd_vel_pub.publish(msg);
            }
             // Needed to process service calls
            ros::spinOnce();
            // And throttle the loop
            loop_rate.sleep();

        }
    }

    // Service client for controlling the turtle
    bool toggleControl(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        is_active_ = !is_active_;
        ROS_INFO("TurtleController %s", is_active_ ? "resumed" : "stopped");
        return true;
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "talker");

    // Create our controller object and run it
    auto controller = TurtleController();
    controller.run();

    // And make good on our promise
    return 0;
}