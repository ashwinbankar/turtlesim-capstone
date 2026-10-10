#include "rclcpp/rclcpp.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include <cmath>

using namespace std::placeholders;
using namespace std::chrono_literals;

class TurtleControllerNode: public rclcpp::Node
{
public:
    TurtleControllerNode(): Node("turtle_controller"), name_("turtle1"), turtlesim_up_(false)
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(name_ + "/cmd_vel", 10);
        
        pose_subscriber_ = this->create_subscription<turtlesim_msgs::msg::Pose>(
            name_ + "/pose", 10, std::bind(&TurtleControllerNode::callbackPose, this, _1));
        
        // Subscribe to the dynamic list of turtles
        alive_turtles_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&TurtleControllerNode::callbackAliveTurtles, this, _1));

        // Client to notify the spawner when a catch happens
        catch_turtle_client_ = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");

        control_loop_timer_ = this->create_wall_timer(
            0.01s, std::bind(&TurtleControllerNode::controlLoop, this));
    }

private:
    void callbackPose(const turtlesim_msgs::msg::Pose::SharedPtr pose)
    {
        pose_ = *pose.get();
        turtlesim_up_ = true;
    }

    void callbackAliveTurtles(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        // Update our local targeting list
        alive_turtles_ = msg->turtles;
    }

    void controlLoop()
    {
        // Do nothing if we don't know where we are, or if there are no turtles to catch
        if (!turtlesim_up_ || alive_turtles_.empty())
        {
            return;
        }

        // Lock onto the first turtle in the array
        auto target = alive_turtles_[0];

        double dist_x = target.x - pose_.x;
        double dist_y = target.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            msg.linear.x = 2 * distance;
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            
            if (angle_diff > M_PI) { angle_diff -= 2 * M_PI; }
            else if (angle_diff < -M_PI) { angle_diff += 2 * M_PI; }
            
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // Target reached! Stop and initiate the catch.
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            callCatchTurtleService(target.name);
        }

        cmd_vel_publisher_->publish(msg);
    }

    void callCatchTurtleService(std::string turtle_name)
    {
        if (!catch_turtle_client_->wait_for_service(1s)) { return; }
        
        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;
        
        catch_turtle_client_->async_send_request(request);
        
        // Remove it locally immediately so the controller doesn't spam the service request while waiting for a reply
        alive_turtles_.erase(alive_turtles_.begin());
    }

    std::string name_;
    turtlesim_msgs::msg::Pose pose_;
    bool turtlesim_up_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
    rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControllerNode>());
    rclcpp::shutdown();
    return 0;
}