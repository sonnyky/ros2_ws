#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleControllerNode: public rclcpp::Node
{
    public:
        TurtleControllerNode() : Node("turtle_controller_node")
        {
            subscriberToTurtlePose = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10,
                std::bind(&TurtleControllerNode::callbackTurtlePose, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Turtle pose subscriber started.");
        }

    private:
        void callbackTurtlePose(const turtlesim::msg::Pose::SharedPtr turtle_pose_msg)
        {
            RCLCPP_INFO(this->get_logger(), "position at %f, %f", turtle_pose_msg->x, turtle_pose_msg->y);
        }
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriberToTurtlePose;    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}