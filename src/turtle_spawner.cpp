#include "rclcpp/rclcpp.hpp"
#include "turtlesim_msgs/srv/spawn.hpp"
#include "turtlesim_msgs/srv/kill.hpp" // For removing turtles from the GUI
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp" // Our new custom service
#include <cmath>
#include <cstdlib>
#include <vector>

using namespace std::placeholders;
using namespace std::chrono_literals;

class TurtleSpawnerNode: public rclcpp::Node
{
public:
    TurtleSpawnerNode(): Node("turtle_spawner"), turtle_counter_(0)
    {
        this->declare_parameter("turtle_name_prefix", "turtle");
        this->declare_parameter("spawn_frequency", 1.0);

        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();
        spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();

        alive_turtles_publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10);
        spawn_client_ = this->create_client<turtlesim_msgs::srv::Spawn>("spawn");
        kill_client_ = this->create_client<turtlesim_msgs::srv::Kill>("kill");

        // Service Server: Listens for "catch" requests from the controller
        catch_turtle_service_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "catch_turtle", 
            std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this, _1, _2));

        int timer_period_ms = (int)(1000.0 / spawn_frequency_);
        spawn_turtle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));
    }

private:
    double randomDouble() { return double(std::rand()) / (double(RAND_MAX) + 1.0); }

    void publishAliveTurtles()
    {
        auto msg = my_robot_interfaces::msg::TurtleArray();
        msg.turtles = alive_turtles_;
        alive_turtles_publisher_->publish(msg);
    }

    void callbackCatchTurtle(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                             const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        // 1. Find the caught turtle in our array and erase it
        for (auto it = alive_turtles_.begin(); it != alive_turtles_.end(); ++it)
        {
            if (it->name == request->name)
            {
                alive_turtles_.erase(it);
                break;
            }
        }

        // 2. Erase it from the graphical window
        auto kill_request = std::make_shared<turtlesim_msgs::srv::Kill::Request>();
        kill_request->name = request->name;
        kill_client_->async_send_request(kill_request);

        // 3. Broadcast the updated list so the controller knows to pick a new target
        publishAliveTurtles();
        
        response->success = true;
    }

    void spawnNewTurtle()
    {
        turtle_counter_++;
        auto name = turtle_name_prefix_ + std::to_string(turtle_counter_);
        double x = randomDouble() * 11.0;
        double y = randomDouble() * 11.0;
        double theta = randomDouble() * 2 * M_PI;
        callSpawnTurtleService(name, x, y, theta);
    }

    void callSpawnTurtleService(std::string turtle_name, double x, double y, double theta)
    {
        if (!spawn_client_->wait_for_service(1s)) { return; }
        auto request = std::make_shared<turtlesim_msgs::srv::Spawn::Request>();
        request->name = turtle_name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        turtle_to_save_ = my_robot_interfaces::msg::Turtle();
        turtle_to_save_.name = turtle_name;
        turtle_to_save_.x = x;
        turtle_to_save_.y = y;
        turtle_to_save_.theta = theta;

        spawn_client_->async_send_request(request, std::bind(&TurtleSpawnerNode::callbackCallSpawnTurtleService, this, _1));
    }

    void callbackCallSpawnTurtleService(rclcpp::Client<turtlesim_msgs::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();
        if (response->name != "")
        {
            alive_turtles_.push_back(turtle_to_save_);
            publishAliveTurtles();
        }
    }

    std::string turtle_name_prefix_;
    int turtle_counter_;
    double spawn_frequency_;

    my_robot_interfaces::msg::Turtle turtle_to_save_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;

    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::Client<turtlesim_msgs::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim_msgs::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_service_;
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleSpawnerNode>());
    rclcpp::shutdown();
    return 0;
}