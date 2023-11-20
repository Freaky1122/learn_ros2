#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServerNode: public rclcpp::Node
{
    private:
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server;
        void server_callback(
            const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Received request");
            response->sum = request->a + request->b;
        }
    public:
        ServerNode(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "node : %s start!!", name.c_str());
            server = this->create_service<example_interfaces::srv::AddTwoInts>(
                "add_two_ints", 
                std::bind(&ServerNode::server_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
};


int main(int argc, char** argv)
{
    //init rclcpp
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<ServerNode>("server_node");


    rclcpp::spin(node);

    //shutdown the node
    rclcpp::shutdown();

    return 0;

}