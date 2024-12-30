#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <ctime>

class DatetimePublisher : public rclcpp::Node {
public:
  DatetimePublisher() : Node("datetime_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("current_datetime", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DatetimePublisher::publish_datetime, this));
  }

private:
  void publish_datetime() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    auto message = std_msgs::msg::String();
    message.data = std::ctime(&now_time);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatetimePublisher>());
  rclcpp::shutdown();
  return 0;
}
