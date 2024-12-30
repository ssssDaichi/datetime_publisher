#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <gtest/gtest.h>
#include <chrono>

TEST(DatetimePublisherTest, TestMessageFormat) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto subscription = node->create_subscription<std_msgs::msg::String>(
      "current_datetime",
      10,
      [](const std_msgs::msg::String::SharedPtr msg) {
        ASSERT_FALSE(msg->data.empty());
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Run the executor for a few seconds to receive messages
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
    executor.spin_once();
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
