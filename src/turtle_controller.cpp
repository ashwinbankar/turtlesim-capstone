#include "rclcpp/rclcpp.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using namespace std::placeholders; // Gives access to _1, _2... for std::bind
using namespace std::chrono_literals; // Allows writing 0.01s instead of chrono::duration(...)

class TurtleControllerNode: public rclcpp::Node // Base Class
{
    public:
    // Constructor: Initializes the node with the name "turtle_controller" to the /turtle1/cmd_vel topic
    // Also initializes class member variables: turtle's name and boolean status flag
    TurtleControllerNode(): Node("turtle_controller"), name_("turtle1"), turtlesim_up_{false} // Derived Class
    {
        // 1. Sends speed (Twist)
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(name_ + "/cmd_vel", 10);

        // 2. Reads position
        pose_subscriber_ = this->create_subscription<turtlesim_msgs::msg::Pose>(
            name_ + "/pose", 10, std::bind(&TurtleControllerNode::callbackPose, this, _1));

        // 3. Timer to run control loop (100Hz or 0.01s)
        control_loop_timer_ = this->create_wall_timer(
            0.01s, std::bind(&TurtleControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Turtle Controller Node has been started.");
    }

    private:
    // Two async callbacks run repeatedly
    // A. callbackPose - Every time new pose arrives
    void callbackPose(const turtlesim_msgs::msg::Pose::SharedPtr pose)
    {
        // Extract the actual location data from the incoming message and copy it into our local variable
        pose_ = *pose.get();
        // Flag that the data is received and control loop can be run
        turtlesim_up_ = true;
    }

    // B. controlLoop - every 0.01s
    void controlLoop()
    {
        // Safety check: Do not calculate commands if we don't know where the turtle is yet
        if (!turtlesim_up_)
        {
            return;
        }

        // Hardcoded target for Step 1 testing
        double target_x = 8.0;
        double target_y = 8.0;

        // Difference between the target and current position
        double dist_x = target_x - pose_.x;
        double dist_y = target_y - pose_.y;

        // Calculate the Euclidean distance to the target using the Pythagorean theorem
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        // Initialize a blank Twist message for velocity commands
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 2 * distance;

        // If the turtle is farther than 0.5 units from the target, keep moving
        if (distance > 0.5)
        {
            // Proportional orientation control: Calculate the absolute angle to the target
            double steering_angle = std::atan2(dist_y, dist_x);
            // Heading error
            double angle_diff = steering_angle - pose_.theta;

            // Normalize angle_diff to keep it within the range of [-pi, pi]
            // This prevents the turtle from doing inefficient full spins to correct its heafing
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }

            // Apply a proportional gain of 6 to the angular error to turn quickly
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // Target reached! Stop all movement.
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        // Publish the calculated velocities to the simulation
        cmd_vel_publisher_->publish(msg);
    }

    // Private member variables
    std::string name_;
    turtlesim_msgs::msg::Pose pose_;
    bool turtlesim_up_;
    
    // Shared pointers for our ROS 2 communication interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2 communication
    rclcpp::init(argc, argv);
    // Create a shared pointer to our custom node
    auto node = std::make_shared<TurtleControllerNode>();
    // Keep node alive, process callbacks
    rclcpp::spin(node);
    // Clean up upon exit (e.g., when the user presses Ctrl+C)
    rclcpp::shutdown();
    return 0;
}