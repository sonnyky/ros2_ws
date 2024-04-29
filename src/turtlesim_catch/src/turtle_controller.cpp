#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include "turtlesim_catch_interfaces/msg/turtle.hpp"
#include "turtlesim_catch_interfaces/msg/turtle_array.hpp"
#include "turtlesim_catch_interfaces/srv/caught_turtle.hpp"

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller_node")
    {
        subscriberToMainTurtlePose = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleControllerNode::CallbackTurtlePose, this, std::placeholders::_1));

        subscriberToAliveTurtles = this->create_subscription<turtlesim_catch_interfaces::msg::TurtleArray>(
            "/alive_turtles", 10,
            std::bind(&TurtleControllerNode::CallbackClosestTurtleAsNextTarget, this, std::placeholders::_1));

        publisherToCmdVel = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        nextTargetPose.x = -1;
        nextTargetPose.y = -1;
        moveTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtleControllerNode::MoveToGoal, this));

        RCLCPP_INFO(this->get_logger(), "Turtle pose subscriber started.");
    }

private:
    void CallbackTurtlePose(const turtlesim::msg::Pose::SharedPtr turtle_pose_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "position at %f, %f", turtle_pose_msg->x, turtle_pose_msg->y);
        selfPose.angular_velocity = turtle_pose_msg->angular_velocity;
        selfPose.linear_velocity = turtle_pose_msg->angular_velocity;
        selfPose.x = turtle_pose_msg->x;
        selfPose.y = turtle_pose_msg->y;
        selfPose.theta = turtle_pose_msg->theta;
    }

    turtlesim::msg::Pose selfPose;
    turtlesim::msg::Pose nextTargetPose;
    turtlesim_catch_interfaces::msg::Turtle nextTargetTurtle;
    void CallbackClosestTurtleAsNextTarget(const turtlesim_catch_interfaces::msg::TurtleArray::SharedPtr alive_turtles_msg)
    {
        RCLCPP_INFO(this->get_logger(), "%ld turtles still alive", alive_turtles_msg->turtles.size());
        if (alive_turtles_msg->turtles.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No alive turtles.");
            // Do nothing
        }
        else
        {
            SetClosestTurtleAsTarget(selfPose, alive_turtles_msg->turtles);
        }
    }

    void SetClosestTurtleAsTarget(const turtlesim::msg::Pose &thisTurtlePose,
                                  const std::vector<turtlesim_catch_interfaces::msg::Turtle> &turtles)
    {
        if (turtles.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No turtles to process.");
            return; // Early return if there are no turtles to process
        }

        // Find the closest turtle using std::min_element
        auto closest = std::min_element(turtles.begin(), turtles.end(),
                                        [&thisTurtlePose](const turtlesim_catch_interfaces::msg::Turtle &a,
                                                          const turtlesim_catch_interfaces::msg::Turtle &b)
                                        {
                                            double dist_a = std::sqrt(std::pow(thisTurtlePose.x - a.x, 2) + std::pow(thisTurtlePose.y - a.y, 2));
                                            double dist_b = std::sqrt(std::pow(thisTurtlePose.x - b.x, 2) + std::pow(thisTurtlePose.y - b.y, 2));
                                            return dist_a < dist_b;
                                        });

        // Check if we actually found a turtle
        if (closest != turtles.end())
        {
            nextTargetPose.x = closest->x;
            nextTargetPose.y = closest->y;
            nextTargetTurtle.x = closest->x;
            nextTargetTurtle.y = closest->y;
            nextTargetTurtle.name = closest->name;
        }
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriberToMainTurtlePose;
    rclcpp::Subscription<turtlesim_catch_interfaces::msg::TurtleArray>::SharedPtr subscriberToAliveTurtles;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherToCmdVel;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr moveTimer;

    float EuclideanDistance(const turtlesim::msg::Pose goalPose)
    {
        return std::sqrt(std::pow((goalPose.x - selfPose.x), 2) + pow((goalPose.y - selfPose.y), 2));
    }

    float LinearVel(const turtlesim::msg::Pose goalPose, float constant = 1.5)
    {
        return constant * this->EuclideanDistance(goalPose);
    }

    float SteeringAngle(const turtlesim::msg::Pose goalPose)
    {
        return std::atan2(goalPose.y - selfPose.y, goalPose.x - selfPose.x);
    }

    float AngularVel(const turtlesim::msg::Pose goalPose, float constant = 3)
    {
        return constant * (this->SteeringAngle(goalPose) - selfPose.theta);
    }

    void MoveToGoal()
    {
        auto messageToCmdVel = geometry_msgs::msg::Twist();
        messageToCmdVel.linear.x = 0;
        messageToCmdVel.angular.z = 0;

        if (this->EuclideanDistance(nextTargetPose) >= 0.01)
        {

            if (nextTargetPose.x < 0 || nextTargetPose.y < 0)
            {
            }
            else
            {

                messageToCmdVel.linear.x = this->LinearVel(nextTargetPose);
                messageToCmdVel.angular.z = this->AngularVel(nextTargetPose);
            }
        }
        else
        {

            // Call the /catch_turtle service
        }

        messageToCmdVel.linear.y = 0;
        messageToCmdVel.linear.z = 0;

        messageToCmdVel.angular.x = 0;
        messageToCmdVel.angular.y = 0;
        RCLCPP_INFO(this->get_logger(), "publishing %f, %f", messageToCmdVel.linear.x, messageToCmdVel.angular.z);

        publisherToCmdVel->publish(messageToCmdVel);
    }

    void RequestCaughtTurtleService()
    {
        auto client = this->create_client<turtlesim_catch_interfaces::srv::CaughtTurtle>("/catch_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting catch turtle server to be up");
        }
        auto catchRequest = std::make_shared<turtlesim_catch_interfaces::srv::CaughtTurtle::Request>();

        catchRequest->name = nextTargetTurtle.name;

        auto future = client->async_send_request(catchRequest);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "got response from catch turtle service: %s", response->received_name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Catch Turtle Service call failed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}