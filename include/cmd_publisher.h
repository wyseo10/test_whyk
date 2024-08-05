#ifndef ROS2_TUTORIAL_CMD_PUBLISHER_H
#define ROS2_TUTORIAL_CMD_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CmdPublisher : public rclcpp::Node {
public:
  CmdPublisher();

private:
  void timer_tf_callback();

  void timer_cmd_callback();

  rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_pose;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

#endif //ROS2_TUTORIAL_CMD_PUBLISHER_H
