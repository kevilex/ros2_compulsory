#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"                                    
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;

float pos = 0.0;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("Crane_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 200);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  float add = 0.1;
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();  

    //filling the message
    message.header.stamp = this->get_clock()->now();
    message.name.push_back("base_to_crane_boom");
    if(pos > 0.3){
      add = -0.001;
    }else if (pos < -0.3)
    {
      add = 0.001;
    }
    pos = pos + add;
    message.position.push_back(pos);

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.position[0] << "'");   
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;             
  size_t count_;
};
/*
 void timer_callback()
  {
  static const auto start_time = std::chrono::high_resolution_clock::now(); // initialize start time


  auto message = sensor_msgs::msg::JointState();  

  // filling the message
  message.header.stamp = this->get_clock()->now();
  message.name.push_back("base_to_crane_boom");

  // generate sinusoidal velocity control from -1 to 1
  std::chrono::nanoseconds ns_since_start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start_time);
  float time = ns_since_start.count() / 1e9; // get current time in seconds
  float vel = sin(time); // compute velocity using sin() function
  message.velocity.push_back(vel);

  // update joint position using velocity control
  pos += vel * 0.1; // multiply velocity by time step to get position change
  message.position.push_back(pos);

  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: position = " << message.position[0] << ", velocity = " << message.velocity[0]);   
  publisher_->publish(message);
}



  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;             
  size_t count_;
};
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}