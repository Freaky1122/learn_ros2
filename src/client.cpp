#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ClientNode: public rclcpp::Node
{
    private:
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
        using ServiceResponseFuture = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
        void response_callback(ServiceResponseFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Received response : %ld", response->sum);
        }
        
    public:
        ClientNode(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "node : %s start!!", name.c_str());
            client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }

        void send_request(int a, int b)
        {
            RCLCPP_INFO(this->get_logger(), "client request : %d + %d", a, b);

            while(!client->wait_for_service(std::chrono::seconds(1)))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "the server is down");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "waiting for server...");
            }

            auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
            request->a = a;
            request->b = b;

            //send async request
            client->async_send_request(request, std::bind(&ClientNode::response_callback, this, std::placeholders::_1));
        }
};


int main(int argc, char** argv)
{
    //init rclcpp
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<ClientNode>("client_node");

    node->send_request(5, 9);
    rclcpp::spin(node);

    //shutdown the node
    rclcpp::shutdown();

    return 0;

}