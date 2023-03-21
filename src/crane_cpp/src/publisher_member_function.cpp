#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"                                    // CHANGE
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("Crane_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);  // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState(); //creating the messsage 

    //filling the message
    message.header.stamp = this->get_clock()->now();
    message.name.push_back("base_to_crane_boom");
    message.position.push_back(0.32);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.position[0] << "'");    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;             // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}