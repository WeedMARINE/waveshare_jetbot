#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/joy.hpp>

class teleop_joy : public rclcpp::Node
{
    public:
    teleop_joy() : Node("teleop_joy")
    {
        // Initialize the node
        RCLCPP_INFO(this->get_logger(), "Node initialized");
        
        // Create a subscription to the joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&teleop_joy::joy_callback, this, std::placeholders::_1));
        
        // Create a publisher for the velocity command
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // Initialize the velocity command
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
    }
    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        geometry_msgs::msg::Twist cmd_vel_;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            // Check if the joystick is being used
            if (msg->axes[1] != 0.0 || msg->axes[0] != 0.0)
            {
                // Set the linear and angular velocities based on joystick input
                cmd_vel_.linear.x = msg->axes[1];
                cmd_vel_.angular.z = msg->axes[0];
            }
            else
            {
                // Stop the robot if the joystick is not being used
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.angular.z = 0.0;
            }

            // Publish the velocity command
            cmd_vel_pub_->publish(cmd_vel_);
        }
};