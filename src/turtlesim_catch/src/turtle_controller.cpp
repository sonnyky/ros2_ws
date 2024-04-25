#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class TurtleControllerNode: public rclcpp::Node
{
    public:
        TurtleControllerNode() : Node("turtle_controller_node")
        {
            subscriberToMainTurtlePose = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10,
                std::bind(&TurtleControllerNode::CallbackTurtlePose, this, std::placeholders::_1)
            );
            publisherToCmdVel = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            RCLCPP_INFO(this->get_logger(), "Turtle pose subscriber started.");
        }

    private:
        void CallbackTurtlePose(const turtlesim::msg::Pose::SharedPtr turtle_pose_msg)
        {
            RCLCPP_INFO(this->get_logger(), "position at %f, %f", turtle_pose_msg->x, turtle_pose_msg->y);
            selfPose.angular_velocity = turtle_pose_msg->angular_velocity;
            selfPose.linear_velocity = turtle_pose_msg->angular_velocity;
            selfPose.x = turtle_pose_msg->x;
            selfPose.y = turtle_pose_msg->y;
            selfPose.theta = turtle_pose_msg->theta;
        }
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriberToMainTurtlePose;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherToCmdVel;

        turtlesim::msg::Pose selfPose;

        float EuclideanDistance(const turtlesim::msg::Pose goalPose)
        {
            return std::sqrt(std::pow((goalPose.x - selfPose.x), 2) + pow((goalPose.y - selfPose.y), 2));
        }

        float LinearVel(const turtlesim::msg::Pose goalPose, float constant=1.5)
        {
            return constant * this->EuclideanDistance(goalPose);
        }

        float SteeringAngle(const turtlesim::msg::Pose goalPose)
        {
            return std::atan2(goalPose.y - selfPose.y, goalPose.x - selfPose.x);
        }

        float AngularVel(const turtlesim::msg::Pose goalPose, float constant=6)
        {
            return constant * (this->SteeringAngle(goalPose) - selfPose.theta);
        }

        void MoveToGoal(const turtlesim::msg::Pose goalPose, float distTolerance=0.01)
        {
            auto messageToCmdVel =  geometry_msgs::msg::Twist();
            messageToCmdVel.linear.x  = 0;
            messageToCmdVel.angular.z = 0;

            if (this->EuclideanDistance(goalPose) >= distTolerance)
            {
                messageToCmdVel.linear.x = this->LinearVel(goalPose);
                messageToCmdVel.angular.z = this->AngularVel(goalPose);
            }

            messageToCmdVel.linear.y = 0;
            messageToCmdVel.linear.z = 0;

            messageToCmdVel.angular.x = 0;
            messageToCmdVel.angular.y = 0;

            publisherToCmdVel->publish(messageToCmdVel);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}