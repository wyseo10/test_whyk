#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

const double PI = 3.14159265358979323846;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
  // Publisher
  pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd", 10);

  // TF listener
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Timer
  timer_tf = this->create_wall_timer(
            10ms, std::bind(&CmdPublisher::timer_tf_callback, this));
  timer_cmd = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
}

int a = 0;
double dt = 0.01;

double real_x = 0;
double real_y = 0;
double real_th = 0;
double prev_error_d = 2;
double prev_error_th = 0;

void CmdPublisher::timer_tf_callback() {
  //TODO: implement this!
  geometry_msgs::msg::TransformStamped t;


  geometry_msgs::msg::Quaternion q = t.transform.rotation;
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double theta = std::atan2(siny_cosp, cosy_cosp);

  real_x = t.transform.translation.x;
  real_y = t.transform.translation.y;
  real_th = theta;
}

void CmdPublisher::timer_cmd_callback() {
  //TODO: implement this!
  geometry_msgs::msg::Twist cmd;

  double goal_x, goal_y, goal_th;

  if (a == 0){
    goal_x = 2;
    goal_y = 0;
    goal_th = 0;
  }else if (a == 1){
    goal_x = 2;
    goal_y = 0;
    goal_th = 0.5 * PI;
  }else if (a == 2){
    goal_x = 2;
    goal_y = 2;
    goal_th = 0.5 * PI;
  }else if (a == 3){
    goal_x = 2;
    goal_y = 2;
    goal_th = 1.0 * PI;
  }else if (a == 4){
    goal_x = 0;
    goal_y = 2;
    goal_th = 1.0 * PI;
  }else if (a == 5){
    goal_x = 0;
    goal_y = 2;
    goal_th = 1.5 * PI;
    if (real_th < 0)
      goal_th = -0.5 * PI;
  }else if (a == 6){
    goal_x = 0;
    goal_y = 0;
    goal_th = -0.5 * PI;
  }else if (a == 7){
    goal_x = 0;
    goal_y = 0;
    goal_th = 0;
  }

  double d_x = goal_x - real_x;
  double d_y = goal_y - real_y;

  //p error
  double error_d = sqrt(d_x * d_x + d_y * d_y);
  double error_th = goal_th - real_th;

  //d error
  double d_error_d =  (error_d - prev_error_d) / dt;
  double d_error_th =  (error_th - prev_error_th) / dt;

  //i error
  double i_error_d = (i_error_d + prev_error_d) * dt;
  double i_error_th = (i_error_th + prev_error_th) * dt;

  cmd.linear.x = 2.4 * error_d + 0.07 * d_error_d + 0.001 * i_error_d;
  cmd.angular.z = 2.4 * error_th + 0.07 * d_error_th + 0.001 * i_error_th;

  prev_error_d = error_d;
  prev_error_th = error_th; 

  if ((a%2 == 0 && error_d < 0.01) or (a%2 == 1 && error_th < 0.001))
    a++;
    a = a % 8;
    i_error_d = 0;
    i_error_th = 0;

  pub_cmd->publish(cmd);
}
