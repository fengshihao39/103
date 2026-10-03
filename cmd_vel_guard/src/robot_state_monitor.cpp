#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using geometry_msgs::msg::Pose2D;
using geometry_msgs::msg::Twist;
using std_msgs::msg::String;

class RobotStateMonitor : public rclcpp::Node
{
public:
  RobotStateMonitor() : Node("robot_state_monitor")
  {
    lin_eps_       = declare_parameter("idle_linear_threshold", 0.01);   // m/s
    ang_eps_       = declare_parameter("idle_angular_threshold", 0.01);  // rad/s
    input_timeout_ = declare_parameter("input_timeout", 0.5);            // s
    const double rate = declare_parameter("state_rate", 10.0);           // Hz

    vel_sub_ = create_subscription<Twist>(
      "cmd_vel_safe", 10, [this](const Twist::SharedPtr m) { onVel(*m); });
    event_sub_ = create_subscription<String>(
      "cmd_vel_events", 10, [this](const String::SharedPtr m) { onEvent(m->data); });

    state_pub_  = create_publisher<String>("robot_state", 10);
    status_pub_ = create_publisher<String>("robot_status", 10);
    pose_pub_   = create_publisher<Pose2D>("robot_pose", 10);

    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() { publishState(); });
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(), "robot_state_monitor started");
  }

private:
  void onVel(const Twist & m)
  {
    const rclcpp::Time t = now();
    if (has_vel_) {
      const double dt = (t - last_vel_time_).seconds();
      if (dt > 0.0 && dt < 0.5) {  // 间隔异常大时不积分，避免位姿跳变
        integrate(m, dt);
      }
    }
    last_vel_time_ = t;
    has_vel_ = true;
    vel_ = m;
  }

  // 机器人坐标系速度 → 世界坐标系位姿
  void integrate(const Twist & m, double dt)
  {
    const double c = std::cos(theta_), s = std::sin(theta_);
    x_ += (m.linear.x * c - m.linear.y * s) * dt;
    y_ += (m.linear.x * s + m.linear.y * c) * dt;
    const double th = theta_ + m.angular.z * dt;
    theta_ = std::atan2(std::sin(th), std::cos(th));  // 归一化到 [-pi, pi]
    distance_ += std::hypot(m.linear.x, m.linear.y) * dt;
  }

  void onEvent(const std::string & e)
  {
    if (e == "INVALID") {
      ++invalid_count_;
    } else if (e == "CLAMPED") {
      ++clamped_count_;
    } else if (e == "TIMEOUT") {
      ++timeout_count_;
      input_timed_out_ = true;
    } else if (e == "RECOVERED") {
      input_timed_out_ = false;
    }
  }

  std::string computeState()
  {
    if (!has_vel_ || (now() - last_vel_time_).seconds() > input_timeout_) {
      return "FILTER_LOST";
    }
    if (input_timed_out_) {
      return "TIMEOUT";
    }
    const bool moving   = std::hypot(vel_.linear.x, vel_.linear.y) > lin_eps_;
    const bool rotating = std::abs(vel_.angular.z) > ang_eps_;
    if (moving && rotating) return "TURNING";
    if (moving)             return "MOVING";
    if (rotating)           return "ROTATING";
    return "IDLE";
  }

  void publishState()
  {
    const std::string s = computeState();
    if (s != state_) {
      RCLCPP_INFO(get_logger(), "state: %s -> %s", state_.c_str(), s.c_str());
      state_ = s;
    }
    String msg;
    msg.data = state_;
    state_pub_->publish(msg);

    Pose2D p;
    p.x = x_;
    p.y = y_;
    p.theta = theta_;
    pose_pub_->publish(p);
  }

  void publishStatus()
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "state=" << state_
       << " v=" << vel_.linear.x << "," << vel_.linear.y
       << " w=" << vel_.angular.z
       << " pose=(" << x_ << "," << y_ << "," << theta_ << ")"
       << " dist=" << distance_
       << " invalid=" << invalid_count_
       << " clamped=" << clamped_count_
       << " timeout=" << timeout_count_;
    String msg;
    msg.data = ss.str();
    status_pub_->publish(msg);
  }

  rclcpp::Subscription<Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<String>::SharedPtr event_sub_;
  rclcpp::Publisher<String>::SharedPtr state_pub_, status_pub_;
  rclcpp::Publisher<Pose2D>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_, status_timer_;

  Twist vel_;
  rclcpp::Time last_vel_time_;
  bool has_vel_ = false;
  bool input_timed_out_ = false;
  std::string state_ = "UNKNOWN";

  double x_ = 0.0, y_ = 0.0, theta_ = 0.0, distance_ = 0.0;
  int invalid_count_ = 0, clamped_count_ = 0, timeout_count_ = 0;
  double lin_eps_, ang_eps_, input_timeout_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStateMonitor>());
  rclcpp::shutdown();
  return 0;
}
