#include "simulator.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

Simulator::Simulator() : Node("simulator") {
    // ROS publisher
    pub_pose = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot/pose", 10);

    // ROS subscriber
    sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>(
            "robot/cmd", 10, std::bind(&Simulator::cmd_callback, this, _1));

    // ROS tf publisher
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ROS timer
    timer = this->create_wall_timer(
            10ms, std::bind(&Simulator::timer_callback, this));
}

double dt = 0.01;
double theta = 0;
double state_x, state_y;

void Simulator::timer_callback() {
    update_state();
    publish_marker_pose();
    broadcast_tf();
}

void Simulator::cmd_callback(const geometry_msgs::msg::Twist &msg) {
    //TODO: implement this!
    cmd_vel.v = msg.linear.x;
    cmd_vel.w = msg.angular.z;
}
void Simulator::update_state() {
    //TODO: implement this!
    state_x = state_x + cmd_vel.v * sin(theta) * dt;
    state_y = state_y + cmd_vel.v * cos(theta) * dt;
    state.theta = state.theta + cmd_vel.w * dt;
}

void Simulator::publish_marker_pose() {
    visualization_msgs::msg::MarkerArray msg;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "turtlesim1";
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://ros2_turtlebot_simulator/mesh/quadrotor_3.dae";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = state.x;
    marker.pose.position.y = state.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = cos(state.theta * 0.5);
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = sin(state.theta * 0.5);
    marker.scale.x = robot_scale;
    marker.scale.y = robot_scale;
    marker.scale.z = robot_scale;
    marker.color.a = 1;
    msg.markers.emplace_back(marker);

    pub_pose->publish(msg);
}

void Simulator::broadcast_tf() {
    //TODO: implement this!
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    
    t.header.frame_id = "/world";
    std::string namespace_ = this->get_namespace();
    t.child_frame_id = namespace_;
   
    t.transform.translation.x = state_x;
    t.transform.translation.y = state_y;
    t.transform.translation.z = 0.0;

    //tf2::Quaternion q;
    //q.setRPY(0, 0, theta);
    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = sin(state.theta * 0.5);
    t.transform.rotation.w = cos(state.theta * 0.5);

    tf_broadcaster->sendTransform(t);
}
