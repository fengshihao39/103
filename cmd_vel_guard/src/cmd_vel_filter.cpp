#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using geometry_msgs::msg::Twist;

class CmdVelFilter : public rclcpp::Node
{
public:
  CmdVelFilter() : Node("cmd_vel_filter")
  {
    // 参数：declare_parameter(名字, 默认值)，可在 yaml 中覆盖
    max_lin_      = declare_parameter("max_linear_speed", 1.0);   // m/s
    max_ang_      = declare_parameter("max_angular_speed", 1.5);  // rad/s
    max_lin_acc_  = declare_parameter("max_linear_accel", 1.0);   // m/s^2
    max_ang_acc_  = declare_parameter("max_angular_accel", 3.0);  // rad/s^2
    timeout_      = declare_parameter("timeout", 0.5);            // s
    allow_lateral_ = declare_parameter("allow_lateral", true);    // 全向底盘允许横移
    const double rate = declare_parameter("publish_rate", 50.0);  // Hz
    dt_ = 1.0 / rate;

    sub_ = create_subscription<Twist>(
      "cmd_vel", 10,
      [this](const Twist::SharedPtr msg) { onCmd(*msg); });
    pub_       = create_publisher<Twist>("cmd_vel_safe", 10);
    event_pub_ = create_publisher<std_msgs::msg::String>("cmd_vel_events", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), [this]() { onTimer(); });

    last_rx_ = now();
    RCLCPP_INFO(get_logger(), "cmd_vel_filter started");
  }

private:
  // 收到原始指令：校验 + 限速，只更新目标值
  void onCmd(const Twist & msg)
  {
    const double v[6] = {msg.linear.x, msg.linear.y, msg.linear.z,
                         msg.angular.x, msg.angular.y, msg.angular.z};
    for (double x : v) {
      if (!std::isfinite(x)) {
        report("INVALID");
        return;  // 丢弃，且不刷新 last_rx_：若持续是坏数据，会触发超时刹停
      }
    }

    last_rx_ = now();
    if (timed_out_) {
      timed_out_ = false;
      report("RECOVERED");
    }

    bool clamped = false;
    target_ = Twist();  // 地面机器人：z、roll、pitch 恒为 0
    target_.linear.x  = clampAbs(msg.linear.x, max_lin_, clamped);
    target_.linear.y  = allow_lateral_ ? clampAbs(msg.linear.y, max_lin_, clamped) : 0.0;
    target_.angular.z = clampAbs(msg.angular.z, max_ang_, clamped);
    if (clamped) {
      report("CLAMPED");
    }
  }

  // 定时器：超时检测 + 加速度限幅 + 发布
  void onTimer()
  {
    if (!timed_out_ && (now() - last_rx_).seconds() > timeout_) {
      timed_out_ = true;
      target_ = Twist();
      report("TIMEOUT");
    }

    out_.linear.x  = ramp(out_.linear.x,  target_.linear.x,  max_lin_acc_ * dt_);
    out_.linear.y  = ramp(out_.linear.y,  target_.linear.y,  max_lin_acc_ * dt_);
    out_.angular.z = ramp(out_.angular.z, target_.angular.z, max_ang_acc_ * dt_);
    pub_->publish(out_);
  }

  static double clampAbs(double v, double lim, bool & flag)
  {
    if (v > lim)  { flag = true; return lim; }
    if (v < -lim) { flag = true; return -lim; }
    return v;
  }

  // 每个周期最多变化 step
  static double ramp(double cur, double target, double step)
  {
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
  }

  void report(const std::string & event)
  {
    std_msgs::msg::String m;
    m.data = event;
    event_pub_->publish(m);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "event: %s", event.c_str());
  }

  rclcpp::Subscription<Twist>::SharedPtr sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Twist target_, out_;
  rclcpp::Time last_rx_;
  bool timed_out_ = true;  // 启动时还没收到数据，视为超时，输出 0
  double max_lin_, max_ang_, max_lin_acc_, max_ang_acc_, timeout_, dt_;
  bool allow_lateral_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelFilter>());
  rclcpp::shutdown();
  return 0;
}
