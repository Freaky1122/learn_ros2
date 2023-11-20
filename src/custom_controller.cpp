#include "learn_ros2/msg/robot_status.hpp"
#include "learn_ros2/srv/move_robot.hpp"
#include "rclcpp/rclcpp.hpp"

class ExampleInterfacesController : public rclcpp::Node {
public:
    ExampleInterfacesController(std::string name) : Node(name) 
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
        /*创建move_robot客户端*/
        client_ = this->create_client<learn_ros2::srv::MoveRobot>(
        "move_robot");
        /*订阅机器人状态话题*/
        robot_status_subscribe_ = this->create_subscription<learn_ros2::msg::RobotStatus>("robot_status", 10, std::bind(&ExampleInterfacesController::robot_status_callback_, this, std::placeholders::_1));
    }


    /**
     * @brief 发送移动机器人请求函数
     * 步骤：1.等待服务上线
     *      2.构造发送请求
     * 
     * @param distance 
     */
    void move_robot(float distance) 
    {
        RCLCPP_INFO(this->get_logger(), "请求让机器人移动%f", distance);

        /*等待服务端上线*/
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
        //等待时检测rclcpp的状态
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
        }

        // 构造请求
        auto request = 
        std::make_shared<learn_ros2::srv::MoveRobot::Request>();
        request->distance = distance;

        // 发送异步请求，然后等待返回，返回时调用回调函数
        client_->async_send_request(
        request, std::bind(&ExampleInterfacesController::result_callback_, this,
                            std::placeholders::_1));
  };

private:
  // 声明客户端
  rclcpp::Client<learn_ros2::srv::MoveRobot>::SharedPtr client_;
  rclcpp::Subscription<learn_ros2::msg::RobotStatus>::SharedPtr robot_status_subscribe_;
  /* 机器人移动结果回调函数 */
  void result_callback_(
    rclcpp::Client<learn_ros2::srv::MoveRobot>::SharedFuture
      result_future) {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "收到移动结果：%f", response->pose);
  }

  /**
   * @brief 机器人状态话题接收回调函数
   * 
   * @param msg 
   */
  void robot_status_callback_(const learn_ros2::msg::RobotStatus::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到状态数据位置：%f 状态：%d", msg->pose ,msg->status);
  }
};


int main(int argc, char** argv)
{
    //init rclcpp
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<ExampleInterfacesController>("controller_node");
    node->move_robot(5.87);


    rclcpp::spin(node);

    //shutdown the node
    rclcpp::shutdown();

    return 0;

}