#include <random>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim_catch_interfaces/msg/turtle.hpp"
#include "turtlesim_catch_interfaces/msg/turtle_array.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner_node"), rng(rd()), counter(2)
    {
        timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&TurtleSpawnerNode::SpawnTurtle, this));
        RCLCPP_INFO(this->get_logger(), "Turtle spawner node started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer;

    std::random_device rd;
    std::mt19937 rng;
    int counter;
    turtlesim::msg::Pose newPose;

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
        spawnRequest->name = "turtle" + std::to_string(counter);

        counter++;

        auto future = client->async_send_request(spawnRequest);

        try
        {
            auto response = future.get();
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

    turtlesim::msg::Pose GetRandomPosition()
    {
        auto randomPose = turtlesim::msg::Pose();
        randomPose.x = GetRandomValue(0, 11);
        randomPose.y = GetRandomValue(0, 11);

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