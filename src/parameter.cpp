#include "rclcpp/rclcpp.hpp"
#include <chrono>


class ParameterNode : public rclcpp::Node
{
private:
    int log_level;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() 
    {
        this->get_parameter("rcl_log_level", log_level); /*获取参数*/
        /*设置日志级别*/
        this->get_logger().set_level((rclcpp::Logger::Level)log_level);
        std::cout<<"======================================================"<<std::endl;
        RCLCPP_DEBUG(this->get_logger(), "我是DEBUG级别的日志，我被打印出来了!");
        RCLCPP_INFO(this->get_logger(), "我是INFO级别的日志，我被打印出来了!");
        RCLCPP_WARN(this->get_logger(), "我是WARN级别的日志，我被打印出来了!");
        RCLCPP_ERROR(this->get_logger(), "我是ERROR级别的日志，我被打印出来了!");
        RCLCPP_FATAL(this->get_logger(), "我是FATAL级别的日志，我被打印出来了!");
    }
public:
    ParameterNode(std::string name ) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node : %s start!!!", name.c_str());
        this->declare_parameter("rcl_log_level", 0);     /*声明参数*/
        this->get_parameter("rcl_log_level", log_level); /*获取参数*/
        /*设置日志级别*/
        this->get_logger().set_level((rclcpp::Logger::Level)log_level);
        using namespace std::literals::chrono_literals;
        timer_ = this->create_wall_timer(
            500ms, std::bind(&ParameterNode::timer_callback, this));
    };

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>("Parameter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
