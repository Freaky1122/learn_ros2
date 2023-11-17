#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PubNode: public rclcpp::Node
{
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer;
        int num = 0;

        void timer_callback()
        {
            std_msgs::msg::String msg;
            msg.data = "forward";
            RCLCPP_INFO(this->get_logger(), "Timer event: Publishing %s %d", msg.data.c_str(), num);
            pub->publish(msg);
            num += 1;
        }

    public:
        PubNode(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "node : %s start!!", name.c_str());
            pub = this->create_publisher<std_msgs::msg::String>("/command", 10);
            timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PubNode::timer_callback, this));
        }


};


int main(int argc, char** argv)
{
    //init rclcpp
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<PubNode>("pub_node");


    rclcpp::spin(node);

    //shutdown the node
    rclcpp::shutdown();

    return 0;

}