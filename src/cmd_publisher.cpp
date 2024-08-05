#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
  // Publisher
  pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("TODO", 10);

  // TF listener
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Timer
  timer_tf = this->create_wall_timer(
            10ms, std::bind(&CmdPublisher::timer_tf_callback, this));
  timer_cmd = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
}

void CmdPublisher::timer_tf_callback() {
  //TODO: implement this!
}

void CmdPublisher::timer_cmd_callback() {
  //TODO: implement this!
}
