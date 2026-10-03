/// @file fake_holonomic_robot.cpp
/// 実機の代わりに cmd_vel で動く模擬の全方位移動ロボット。手動テスト用。
///
/// - cmd_vel (機体座標系) に1次遅れ (plant_tau) で追従する
/// - 一定の外乱速度 (disturbance_*, フィールド座標系) が乗る
/// - 自己位置は pose_rate で、撮った時刻 (stamp) のまま pose_delay 遅れて
///   TF (field_frame -> base_frame) に流す。sotoba_node の TF と同じ形
/// - 真値は遅れ無しで ~/truth (PoseStamped) に出す

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "holonomic_tracker/types.hpp"

namespace {
	using holonomic_tracker::Pose2;
	using holonomic_tracker::Twist2;

	class FakeHolonomicRobot final : public rclcpp::Node {
	public:
		FakeHolonomicRobot() : rclcpp::Node{"fake_holonomic_robot"} {
			this->field_frame_ = this->declare_parameter<std::string>("field_frame", "field");
			this->base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
			this->tau_ = this->declare_parameter<double>("plant_tau", 0.08);
			this->pose_period_ = 1.0 / this->declare_parameter<double>("pose_rate", 20.0);
			this->pose_delay_ = this->declare_parameter<double>("pose_delay", 0.05);
			this->sigma_pos_ = this->declare_parameter<double>("sigma_position", 0.003);
			this->sigma_yaw_ = this->declare_parameter<double>("sigma_yaw", 0.003);
			this->disturbance_ = Twist2{
				this->declare_parameter<double>("disturbance_vx", 0.0),
				this->declare_parameter<double>("disturbance_vy", 0.0),
				0.0,
			};
			this->pose_ = Pose2{
				this->declare_parameter<double>("initial_x", 1.1),
				this->declare_parameter<double>("initial_y", -0.05),
				this->declare_parameter<double>("initial_yaw", 0.1),
			};

			this->cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
				"cmd_vel",
				rclcpp::QoS{10},
				[this](const geometry_msgs::msg::Twist::ConstSharedPtr m) {
					this->cmd_ = Twist2{m->linear.x, m->linear.y, m->angular.z};
				}
			);
			this->truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/truth", rclcpp::QoS{50});
			this->tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

			this->timer_ = rclcpp::create_timer(
				this,
				this->get_clock(),
				rclcpp::Duration::from_seconds(sim_dt),
				[this] { this->step(); }
			);
		}

	private:
		static constexpr double sim_dt = 0.002;

		auto step() -> void {
			const rclcpp::Time now = this->now();
			const double t = now.seconds();
			if (!this->last_) {
				this->last_ = t;
				this->next_pose_ = t;
			}
			const double dt = std::clamp(t - *this->last_, 0.0, 0.05);
			this->last_ = t;

			// 機体座標系で1次遅れ
			const double a = 1.0 - std::exp(-dt / this->tau_);
			this->vel_.vx += a * (this->cmd_.vx - this->vel_.vx);
			this->vel_.vy += a * (this->cmd_.vy - this->vel_.vy);
			this->vel_.omega += a * (this->cmd_.omega - this->vel_.omega);
			const auto vf = holonomic_tracker::body_to_field(this->vel_, this->pose_.yaw + 0.5 * this->vel_.omega * dt);
			this->pose_.x += (vf.vx + this->disturbance_.vx) * dt;
			this->pose_.y += (vf.vy + this->disturbance_.vy) * dt;
			this->pose_.yaw = holonomic_tracker::wrap_angle(this->pose_.yaw + vf.omega * dt);

			geometry_msgs::msg::PoseStamped truth{};
			truth.header.stamp = now;
			truth.header.frame_id = this->field_frame_;
			truth.pose.position.x = this->pose_.x;
			truth.pose.position.y = this->pose_.y;
			truth.pose.orientation.z = std::sin(0.5 * this->pose_.yaw);
			truth.pose.orientation.w = std::cos(0.5 * this->pose_.yaw);
			if (++this->truth_count_ % 10 == 0) { this->truth_pub_->publish(truth); }

			// 自己位置: 撮った時刻のまま、遅れて出す
			if (t >= this->next_pose_) {
				this->next_pose_ += this->pose_period_;
				geometry_msgs::msg::TransformStamped tf{};
				tf.header.stamp = now;
				tf.header.frame_id = this->field_frame_;
				tf.child_frame_id = this->base_frame_;
				tf.transform.translation.x = this->pose_.x + this->sigma_pos_ * this->n01_(this->rng_);
				tf.transform.translation.y = this->pose_.y + this->sigma_pos_ * this->n01_(this->rng_);
				const double yaw = this->pose_.yaw + this->sigma_yaw_ * this->n01_(this->rng_);
				tf.transform.rotation.z = std::sin(0.5 * yaw);
				tf.transform.rotation.w = std::cos(0.5 * yaw);
				this->pending_.emplace_back(t + this->pose_delay_, tf);
			}
			while (!this->pending_.empty() && this->pending_.front().first <= t) {
				this->tf_->sendTransform(this->pending_.front().second);
				this->pending_.pop_front();
			}
		}

		std::string field_frame_;
		std::string base_frame_;
		double tau_{};
		double pose_period_{};
		double pose_delay_{};
		double sigma_pos_{};
		double sigma_yaw_{};
		Twist2 disturbance_{};

		Pose2 pose_{};
		Twist2 vel_{};
		Twist2 cmd_{};
		std::optional<double> last_{};
		double next_pose_{};
		std::size_t truth_count_{0};
		std::deque<std::pair<double, geometry_msgs::msg::TransformStamped>> pending_{};
		std::mt19937 rng_{1};
		std::normal_distribution<double> n01_{0.0, 1.0};

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_{};
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_{};
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_{};
		rclcpp::TimerBase::SharedPtr timer_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FakeHolonomicRobot>());
	rclcpp::shutdown();
	return 0;
}
