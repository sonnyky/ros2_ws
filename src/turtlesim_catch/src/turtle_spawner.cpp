#include <random>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim_catch_interfaces/msg/turtle.hpp"
#include "turtlesim_catch_interfaces/msg/turtle_array.hpp"
#include "turtlesim_catch_interfaces/srv/caught_turtle.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner_node"), rng(rd()), counter(2)
    {
        publisherToAliveTurtles = this->create_publisher<turtlesim_catch_interfaces::msg::TurtleArray>("/alive_turtles", 10);

        catchTurtleServer = this->create_service<turtlesim_catch_interfaces::srv::CaughtTurtle>(
            "/catch_turtle",
            std::bind(&TurtleSpawnerNode::CallbackCaughtTurtle, this, std::placeholders::_1, std::placeholders::_2));

        timer = this->create_wall_timer(std::chrono::seconds(10), std::bind(&TurtleSpawnerNode::SpawnTurtle, this));
        RCLCPP_INFO(this->get_logger(), "Turtle spawner node started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer;

    std::random_device rd;
    std::mt19937 rng;
    int counter;
    turtlesim::msg::Pose newPose;
    std::vector<turtlesim_catch_interfaces::msg::Turtle> aliveTurtles;

    rclcpp::Publisher<turtlesim_catch_interfaces::msg::TurtleArray>::SharedPtr publisherToAliveTurtles;
    rclcpp::Service<turtlesim_catch_interfaces::srv::CaughtTurtle>::SharedPtr catchTurtleServer;

    std::string caughtTurtleName;

    void CallbackCaughtTurtle(
        const turtlesim_catch_interfaces::srv::CaughtTurtle::Request::SharedPtr request,
        const turtlesim_catch_interfaces::srv::CaughtTurtle::Response::SharedPtr response)
    {
        response->received_name = request->name;
        std::thread t = std::thread(std::bind(&TurtleSpawnerNode::RequestKillService, this, request->name));
        t.detach();
        RCLCPP_INFO(this->get_logger(), "got request to kill %s and hot response to send to turtle kill service %s", request->name.c_str(), response->received_name.c_str());
    }

    void RequestKillService(const std::string &targetName)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("/kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for turtle kill server to be up");
        }
        auto killRequest = std::make_shared<turtlesim::srv::Kill::Request>();
        killRequest->name = targetName;
        auto future = client->async_send_request(killRequest);
        try
        {
            auto response = future.get();
            UpdateAliveTurtles(targetName);
            RCLCPP_INFO(this->get_logger(), "called kill service");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Kill Service call failed");
        }
    }

    void UpdateAliveTurtles(const std::string &name)
    {
        // Use the erase-remove idiom to remove the turtle with the matching name
        aliveTurtles.erase(
            std::remove_if(aliveTurtles.begin(), aliveTurtles.end(),
                           [&name](const turtlesim_catch_interfaces::msg::Turtle &turtle)
                           {
                               return turtle.name == name;
                           }),
            aliveTurtles.end());
    }

    void RequestSpawnService()
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("/spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server to be up");
        }
        auto spawnRequest = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawnRequest->x = newPose.x;
        spawnRequest->y = newPose.y;
        spawnRequest->theta = 0;
        std::string turtleName = "turtle" + std::to_string(counter);

        spawnRequest->name = turtleName;
        auto newTurtle = turtlesim_catch_interfaces::msg::Turtle();
        newTurtle.x = newPose.x;
        newTurtle.y = newPose.y;
        newTurtle.name = turtleName;

        aliveTurtles.push_back(newTurtle);

        counter++;

        auto future = client->async_send_request(spawnRequest);

        try
        {
            auto response = future.get();
            PublishAliveTurtles();
            RCLCPP_INFO(this->get_logger(), "%s", response->name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Spawn Service call failed");
        }
    }

    void SpawnTurtle()
    {

        newPose = GetRandomPosition();

        RCLCPP_INFO(this->get_logger(), "Turtle spawner running. %f %f", newPose.x, newPose.y);
        std::thread t = std::thread(std::bind(&TurtleSpawnerNode::RequestSpawnService, this));
        t.detach();
    }

    void PublishAliveTurtles()
    {
        auto msg = turtlesim_catch_interfaces::msg::TurtleArray();
        msg.turtles = this->aliveTurtles;
        publisherToAliveTurtles->publish(msg);
    }

    turtlesim::msg::Pose GetRandomPosition()
    {
        auto randomPose = turtlesim::msg::Pose();
        randomPose.x = GetRandomValue(0.5, 10.5);
        randomPose.y = GetRandomValue(0.5, 10.5);

        return randomPose;
    }

    float GetRandomValue(float min, float max)
    {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(this->rng);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}