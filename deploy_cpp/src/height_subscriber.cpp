/**
 * @file height_subscriber.cpp
 * @brief Height measurement subscriber implementation.
 */

#include "height_subscriber.h"

#include <algorithm>
#include <functional>

namespace deploy {

HeightSubscriber::HeightSubscriber(const std::string &topic,
                                   float nominal_base_height)
    : Node("height_subscriber") {
  distances_.fill(nominal_base_height);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();

  sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      topic, qos, std::bind(&HeightSubscriber::callback, this,
                            std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribing height measurements: %s",
              topic.c_str());
}

void HeightSubscriber::callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() != NUM_HEIGHT_POINTS) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Height msg has %zu floats, expected %d",
                         msg->data.size(), NUM_HEIGHT_POINTS);
    return;
  }

  if (msg->layout.dim.size() >= 2) {
    const auto &dx = msg->layout.dim[0];
    const auto &dy = msg->layout.dim[1];
    if (dx.size != NUM_HEIGHT_POINTS_X || dy.size != NUM_HEIGHT_POINTS_Y) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Height layout is %u x %u, expected %d x %d",
                           dx.size, dy.size, NUM_HEIGHT_POINTS_X,
                           NUM_HEIGHT_POINTS_Y);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  std::copy_n(msg->data.begin(), NUM_HEIGHT_POINTS, distances_.begin());
  received_.store(true, std::memory_order_release);
  msg_count_.fetch_add(1, std::memory_order_relaxed);
}

std::array<float, NUM_HEIGHT_POINTS> HeightSubscriber::get_distances() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return distances_;
}

} // namespace deploy
