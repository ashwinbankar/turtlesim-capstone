#include "rclcpp/rclcpp.hpp"
#include "turtlesim_msgs/srv/spawn.hpp" // Service interface to spawn turtles
#include <cmath>
#include <cstdlib> // Gives access to std::rand() for random numbers

using namespace std::placeholders;
using namespace std::chrono_literals;

class TurtleSpawnerNode: public rclcpp::Node
{
public:
    // Constructor: Initialize the node and sets spawned turtle counter to 0
    TurtleSpawnerNode(): Node("turtle_spawner"), turtle_counter_(0)
    {
        // 1. Declare parameters with default values
        // Allows to change these values from terminal or launch files later
        this->declare_parameter("turtle_name_prefix", "turtle");
        this->declare_parameter("spawn_frequency", 1.0); // Spawn 1 turtle per second

        // 2. Read the parameters in to class variables
        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();
        spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();

        // 3. Create a Service Client to send requests to the /spawn service
        spawn_client_ = this->create_client<turtlesim_msgs::srv::Spawn>("spawn");

        // 4. Create a timer to automatically trigger the spawning function
        int timer_period_ms = (int)(1000.0 / spawn_frequency_);
        spawn_turtle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));
    }

private:
    // Helper function to generate a random decimal number between 0.0 and 1.0
    double randomDouble()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    // Timer callback: Prepares the random data and triggers the service call
    void spawnNewTurtle()
    {
        // Increment the counter to give each turtle a unique name (e.g., turtle2, turtle3)
        turtle_counter_++;
        auto name = turtle_name_prefix_ + std::to_string(turtle_counter_);

        //The Turtlesim window is an 11.0 by 11.0 coordinate grid
        double x = randomDouble() * 11.0;
        double y = randomDouble() * 11.0;
        double theta = randomDouble() * 2 * M_PI;

        // Send this generated data to our service call function
        callSpawnTurtleService(name, x, y, theta);
    }

    // Executes the actual service call asynchronously
    void callSpawnTurtleService(std::string turtle_name, double x, double y, double theta)
    {
        // Safety check: Wait for the simulation to actually offer the /spawn service before requesting
        while (!spawn_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /spawn Service Server to be up...");
        }

        // Create the request payload and fill it with our random data
        auto request = std::make_shared<turtlesim_msgs::srv::Spawn::Request>();
        request->name = turtle_name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        // Send the request and register a callback function to handle the server's reply
        spawn_client_->async_send_request(
            request, std::bind(&TurtleSpawnerNode::callbackCallSpawnTurtleService, this, _1));
    }

    // Callback function triggered the moment the /spawn server replies
    void callbackCallSpawnTurtleService(rclcpp::Client<turtlesim_msgs::srv::Spawn>::SharedFuture future)
    {
        // Extract the response data
        auto response = future.get();
        // If response contains valid name, the spawn was successful
        if (response->name != "")
        {
            RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.", response->name.c_str());
        }
        
    }

    // Private member variables
    std::string turtle_name_prefix_;
    int turtle_counter_;
    double spawn_frequency_;

    // Shared pointers for ROS2 client and timer
    rclcpp::Client<turtlesim_msgs::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}