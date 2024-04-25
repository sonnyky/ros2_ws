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
        publisherToAliveTurtles = this->create_publisher<turtlesim_catch_interfaces::msg::TurtleArray>("/alive_turtles", 10);
        timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&TurtleSpawnerNode::SpawnTurtle, this));
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