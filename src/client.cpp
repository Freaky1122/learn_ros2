#include "rclcpp/rclcpp.hpp"

class ClientNode: public rclcpp::Node
{
    private:
        /* data */
    public:
        ClientNode(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "my first node : %s start!!", name.c_str());
        }
};


int main(int argc, char** argv)
{
    //init rclcpp
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<ClientNode>("client_node");


    rclcpp::spin(node);

    //shutdown the node
    rclcpp::shutdown();

    return 0;

}