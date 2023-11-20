#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
    private:
        /* data */
    public:
        MyNode(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "node : %s start!!!", name.c_str());
        }
};


int main(int argc, char** argv)
{
    //init rclcpp
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<MyNode>("first_node");


    rclcpp::spin(node);

    //shutdown the node
    rclcpp::shutdown();

    return 0;
}