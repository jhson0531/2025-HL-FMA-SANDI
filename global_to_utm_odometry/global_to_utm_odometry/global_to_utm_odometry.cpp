#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

class GlobalToUtmOdometry : public rclcpp::Node
{
public:
  GlobalToUtmOdometry()
  : Node("global_to_utm_odometry"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    in_topic_  = declare_parameter<std::string>("in_topic", "/odometry/global");
    out_topic_ = declare_parameter<std::string>("out_topic", "/odometry/global_utm");
    target_frame_ = declare_parameter<std::string>("target_frame", "utm"); // or "local_enu"
    rotate_covariance_ = declare_parameter<bool>("rotate_covariance", true);
    timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    use_latest_on_timeout_ = declare_parameter<bool>("use_latest_on_timeout", true);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GlobalToUtmOdometry::odomCb, this, std::placeholders::_1));

    pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic_, rclcpp::QoS(50));

    RCLCPP_INFO(get_logger(),
      "Reproject '%s' -> '%s' (target=%s, rotate_cov=%s)",
      in_topic_.c_str(), out_topic_.c_str(), target_frame_.c_str(),
      rotate_covariance_ ? "true" : "false");
  }

private:
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &src_frame = msg->header.frame_id;
    if (src_frame.empty()) return;

    geometry_msgs::msg::TransformStamped tf_map_to_target;
    const auto timeout = rclcpp::Duration::from_seconds(timeout_sec_);

    // 1) TF lookup (map -> utm)
    try {
      tf_map_to_target = tf_buffer_.lookupTransform(
        target_frame_, src_frame, msg->header.stamp, timeout);
    } catch (const std::exception &e) {
      if (use_latest_on_timeout_) {
        try {
          tf_map_to_target = tf_buffer_.lookupTransform(
            target_frame_, src_frame, tf2::TimePointZero);
        } catch (const std::exception &e2) {
          RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e2.what());
          return;
        }
      } else {
        RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
        return;
      }
    }

    // 2) Pose 변환 (위치+자세)
    geometry_msgs::msg::PoseStamped ps_in, ps_out;
    ps_in.header = msg->header;
    ps_in.pose   = msg->pose.pose;
    tf2::doTransform(ps_in, ps_out, tf_map_to_target);

    // 3) 공분산 회전 (T C Tᵀ, T=diag(R,R))
    std::array<double,36> cov_out = msg->pose.covariance;
    if (rotate_covariance_) {
      const auto &q = tf_map_to_target.transform.rotation;
      tf2::Matrix3x3 R(tf2::Quaternion(q.x,q.y,q.z,q.w));

      Eigen::Matrix<double,6,6> T = Eigen::Matrix<double,6,6>::Zero();
      for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
        T(r,c) = R[r][c];
        T(r+3,c+3) = R[r][c];
      }
      Eigen::Matrix<double,6,6> Cin = Eigen::Matrix<double,6,6>::Zero();
      for (int r=0;r<6;++r) for (int c=0;c<6;++c)
        Cin(r,c) = msg->pose.covariance[r*6+c];
      Eigen::Matrix<double,6,6> Cout = T * Cin * T.transpose();
      for (int r=0;r<6;++r) for (int c=0;c<6;++c)
        cov_out[r*6+c] = Cout(r,c);
    }

    // 4) 결과 퍼블리시
    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;
    out.child_frame_id  = msg->child_frame_id;     // 보통 "base_link"
    out.pose.pose       = ps_out.pose;
    out.pose.covariance = cov_out;
    out.twist           = msg->twist;              // base_link 기준 값 그대로
    pub_->publish(out);
  }

  // params
  std::string in_topic_, out_topic_, target_frame_;
  bool rotate_covariance_;
  double timeout_sec_;
  bool use_latest_on_timeout_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // IO
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalToUtmOdometry>());
  rclcpp::shutdown();
  return 0;
}
