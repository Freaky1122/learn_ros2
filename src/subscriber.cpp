#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubNode: public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

        void sub_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
        }
    public:
        SubNode(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "node : %s start!!", name.c_str());
            sub = this->create_subscription<std_msgs::msg::String>("/command", 10, std::bind(&SubNode::sub_callback, this, std::placeholders::_1));
        }
};


int main(int argc, char** argv)
{
    //init rclcpp
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<SubNode>("sub_node");


    rclcpp::spin(node);

    //shutdown the node
    rclcpp::shutdown();

    return 0;

}