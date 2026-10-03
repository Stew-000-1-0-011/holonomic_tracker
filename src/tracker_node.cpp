/// @file tracker_node.cpp
/// 全方位移動ロボットの軌道追従ノード。
///
/// - 目標 (位置・向き・速度FF) を `~/reference` で受け取る
/// - 自己位置 (姿勢だけ) を TF か PoseStamped で受け取る
/// - 機体速度は VelocityObserver が自己位置と自分の出した cmd_vel から推定する
/// - TrackingController でフィードバックをかけ、`cmd_vel` (機体座標系) を出す
///
/// 推定・制御そのものは ROS 非依存 (observer.cpp, controller.cpp)。
/// ここは ROS の入出力とパラメータの面倒だけを見る。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "holonomic_tracker/controller.hpp"
#include "holonomic_tracker/msg/tracking_reference.hpp"
#include "holonomic_tracker/msg/tracking_status.hpp"
#include "holonomic_tracker/observer.hpp"
#include "holonomic_tracker/types.hpp"

namespace {
	using holonomic_tracker::ControllerParams;
	using holonomic_tracker::ObserverParams;
	using holonomic_tracker::Pose2;
	using holonomic_tracker::Reference;
	using holonomic_tracker::TrackingController;
	using holonomic_tracker::Twist2;
	using holonomic_tracker::UpdateResult;
	using holonomic_tracker::VelocityObserver;
	using holonomic_tracker::msg::TrackingReference;
	using holonomic_tracker::msg::TrackingStatus;

	auto yaw_of(const double qx, const double qy, const double qz, const double qw) -> double {
		return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
	}

	auto to_msg(const Twist2& t) -> geometry_msgs::msg::Twist {
		geometry_msgs::msg::Twist m{};
		m.linear.x = t.vx;
		m.linear.y = t.vy;
		m.angular.z = t.omega;
		return m;
	}

	class TrackerNode final : public rclcpp::Node {
	public:
		TrackerNode() : rclcpp::Node{"tracker_node"} {
			// --- 起動時にしか読まないもの ---
			this->field_frame_ = this->declare_parameter<std::string>("field_frame", "field");
			this->base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
			const auto pose_source = this->declare_parameter<std::string>("pose_source", "tf");
			const auto pose_topic = this->declare_parameter<std::string>("pose_topic", "pose");
			const auto cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
			const bool cmd_vel_stamped = this->declare_parameter<bool>("cmd_vel_stamped", false);
			const double rate = this->declare_parameter<double>("control_rate", 50.0);

			// --- 実行中に変えられるもの (ros2 param set で効く) ---
			this->declare_parameter<double>("reference_timeout", 0.2);
			this->declare_parameter<double>("pose_timeout", 0.3);
			this->declare_parameter<double>("max_reference_extrapolation", 0.2);
			this->declare_parameter<double>("cmd_delay", 0.0);

			this->declare_parameter<double>("gains.linear.kp", 3.0);
			this->declare_parameter<double>("gains.linear.ki", 0.0);
			this->declare_parameter<double>("gains.linear.kd", 0.3);
			this->declare_parameter<double>("gains.linear.integral_limit", 0.1);
			this->declare_parameter<double>("gains.angular.kp", 4.0);
			this->declare_parameter<double>("gains.angular.ki", 0.0);
			this->declare_parameter<double>("gains.angular.kd", 0.3);
			this->declare_parameter<double>("gains.angular.integral_limit", 0.1);
			this->declare_parameter<double>("heading_lookahead", 0.5 / rate);

			this->declare_parameter<double>("limits.max_velocity_linear", 2.0);
			this->declare_parameter<double>("limits.max_velocity_angular", 6.0);
			this->declare_parameter<double>("limits.max_accel_linear", 5.0);
			this->declare_parameter<double>("limits.max_accel_angular", 20.0);

			this->declare_parameter<double>("observer.tau_linear", 0.1);
			this->declare_parameter<double>("observer.tau_angular", 0.1);
			this->declare_parameter<double>("observer.accel_noise_linear", 1.0);
			this->declare_parameter<double>("observer.accel_noise_angular", 2.0);
			this->declare_parameter<double>("observer.disturbance_noise_linear", 0.05);
			this->declare_parameter<double>("observer.disturbance_noise_angular", 0.1);
			this->declare_parameter<double>("observer.sigma_position", 0.01);
			this->declare_parameter<double>("observer.sigma_yaw", 0.01);
			this->declare_parameter<double>("observer.initial_sigma_velocity_linear", 0.5);
			this->declare_parameter<double>("observer.initial_sigma_velocity_angular", 1.0);
			this->declare_parameter<double>("observer.initial_sigma_disturbance_linear", 0.1);
			this->declare_parameter<double>("observer.initial_sigma_disturbance_angular", 0.2);
			this->declare_parameter<double>("observer.gate_sigma", 0.0);
			this->declare_parameter<int>("observer.reset_after_rejects", 5);
			this->declare_parameter<double>("observer.max_gap", 1.0);

			this->load_params();
			// 変更を受け付けてから読み直す。値の検証は load_params 側で丸める
			this->param_cb_ = this->add_post_set_parameters_callback(
				[this](const std::vector<rclcpp::Parameter>&) { this->load_params(); }
			);

			// --- 入力 ---
			if (pose_source == "tf") {
				this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
				this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
			} else if (pose_source == "topic") {
				this->pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
					pose_topic,
					rclcpp::SensorDataQoS{},
					[this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { this->on_pose(*msg); }
				);
			} else {
				throw std::invalid_argument{"pose_source must be \"tf\" or \"topic\": " + pose_source};
			}

			this->reference_sub_ = this->create_subscription<TrackingReference>(
				"~/reference",
				rclcpp::QoS{10},
				[this](const TrackingReference::ConstSharedPtr msg) { this->on_reference(*msg); }
			);

			this->enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
				"~/enable",
				[this](
					const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
					const std_srvs::srv::SetBool::Response::SharedPtr res
				) {
					this->enabled_ = req->data;
					res->success = true;
					res->message = this->enabled_ ? "enabled" : "disabled";
				}
			);

			// --- 出力 ---
			if (cmd_vel_stamped) {
				this->cmd_stamped_pub_ =
					this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, rclcpp::QoS{10});
			} else {
				this->cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS{10});
			}
			this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS{10});
			this->status_pub_ = this->create_publisher<TrackingStatus>("~/status", rclcpp::QoS{10});

			this->period_ = 1.0 / rate;
			// use_sim_time に従うよう、ノードの時計で回す
			this->timer_ = rclcpp::create_timer(
				this,
				this->get_clock(),
				rclcpp::Duration::from_seconds(this->period_),
				[this] { this->on_timer(); }
			);

			RCLCPP_INFO(
				this->get_logger(),
				"tracking %s in %s at %.1f Hz (pose from %s)",
				this->base_frame_.c_str(),
				this->field_frame_.c_str(),
				rate,
				pose_source.c_str()
			);
		}

	private:
		auto load_params() -> void {
			const auto d = [this](const char* name) { return this->get_parameter(name).as_double(); };

			this->reference_timeout_ = d("reference_timeout");
			this->pose_timeout_ = d("pose_timeout");
			this->max_ref_extrapolation_ = std::max(0.0, d("max_reference_extrapolation"));
			this->cmd_delay_ = std::max(0.0, d("cmd_delay"));

			ControllerParams cp{};
			cp.linear = {d("gains.linear.kp"), d("gains.linear.ki"), d("gains.linear.kd"), d("gains.linear.integral_limit")};
			cp.angular = {
				d("gains.angular.kp"),
				d("gains.angular.ki"),
				d("gains.angular.kd"),
				d("gains.angular.integral_limit"),
			};
			cp.heading_lookahead = d("heading_lookahead");
			cp.max_velocity_linear = d("limits.max_velocity_linear");
			cp.max_velocity_angular = d("limits.max_velocity_angular");
			cp.max_accel_linear = d("limits.max_accel_linear");
			cp.max_accel_angular = d("limits.max_accel_angular");
			this->controller_.set_params(cp);

			ObserverParams op{};
			op.tau_linear = d("observer.tau_linear");
			op.tau_angular = d("observer.tau_angular");
			op.accel_noise_linear = d("observer.accel_noise_linear");
			op.accel_noise_angular = d("observer.accel_noise_angular");
			op.disturbance_noise_linear = d("observer.disturbance_noise_linear");
			op.disturbance_noise_angular = d("observer.disturbance_noise_angular");
			op.sigma_position = std::max(1e-6, d("observer.sigma_position"));
			op.sigma_yaw = std::max(1e-6, d("observer.sigma_yaw"));
			op.initial_sigma_velocity_linear = d("observer.initial_sigma_velocity_linear");
			op.initial_sigma_velocity_angular = d("observer.initial_sigma_velocity_angular");
			op.initial_sigma_disturbance_linear = d("observer.initial_sigma_disturbance_linear");
			op.initial_sigma_disturbance_angular = d("observer.initial_sigma_disturbance_angular");
			op.gate_sigma = d("observer.gate_sigma");
			op.reset_after_rejects =
				static_cast<std::size_t>(std::max<std::int64_t>(0, this->get_parameter("observer.reset_after_rejects").as_int()));
			op.max_gap = d("observer.max_gap");
			this->observer_.set_params(op);
		}

		auto now_sec() -> double { return this->now().seconds(); }

		auto on_reference(const TrackingReference& msg) -> void {
			if (!msg.header.frame_id.empty() && msg.header.frame_id != this->field_frame_) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(),
					*this->get_clock(),
					2000,
					"reference frame_id \"%s\" != field_frame \"%s\", ignored",
					msg.header.frame_id.c_str(),
					this->field_frame_.c_str()
				);
				return;
			}
			const double received = this->now_sec();
			const double stamp = rclcpp::Time{msg.header.stamp}.seconds();
			this->reference_ = Reference{Pose2{msg.x, msg.y, msg.yaw}, Twist2{msg.vx, msg.vy, msg.omega}};
			this->reference_stamp_ = stamp > 0.0 ? stamp : received;
			this->reference_received_ = received;
		}

		auto on_pose(const geometry_msgs::msg::PoseStamped& msg) -> void {
			if (!msg.header.frame_id.empty() && msg.header.frame_id != this->field_frame_) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(),
					*this->get_clock(),
					2000,
					"pose frame_id \"%s\" != field_frame \"%s\", ignored",
					msg.header.frame_id.c_str(),
					this->field_frame_.c_str()
				);
				return;
			}
			const auto& q = msg.pose.orientation;
			this->measure(
				rclcpp::Time{msg.header.stamp}.seconds(),
				Pose2{msg.pose.position.x, msg.pose.position.y, yaw_of(q.x, q.y, q.z, q.w)}
			);
		}

		/// TF から最新の自己位置を引く。新しいものだけ観測として入れる。
		auto poll_tf() -> void {
			if (!this->tf_buffer_) { return; }
			geometry_msgs::msg::TransformStamped tf{};
			try {
				tf = this->tf_buffer_->lookupTransform(this->field_frame_, this->base_frame_, tf2::TimePointZero);
			} catch (const tf2::TransformException& e) {
				RCLCPP_DEBUG(this->get_logger(), "lookupTransform: %s", e.what());
				return;
			}
			const double stamp = rclcpp::Time{tf.header.stamp}.seconds();
			if (this->last_tf_stamp_ && stamp <= *this->last_tf_stamp_) { return; }
			this->last_tf_stamp_ = stamp;
			const auto& t = tf.transform.translation;
			const auto& q = tf.transform.rotation;
			this->measure(stamp, Pose2{t.x, t.y, yaw_of(q.x, q.y, q.z, q.w)});
		}

		auto measure(const double stamp, const Pose2& pose) -> void {
			switch (this->observer_.update(stamp, pose)) {
				case UpdateResult::initialized:
					RCLCPP_INFO(this->get_logger(), "observer initialized at (%.3f, %.3f, %.3f)", pose.x, pose.y, pose.yaw);
					break;
				case UpdateResult::gated:
					RCLCPP_WARN_THROTTLE(
						this->get_logger(),
						*this->get_clock(),
						1000,
						"pose measurement rejected by gate (jump?)"
					);
					break;
				case UpdateResult::out_of_order:
					RCLCPP_DEBUG(this->get_logger(), "out-of-order pose dropped");
					break;
				case UpdateResult::accepted:
					break;
			}
		}

		auto publish_cmd(const Twist2& cmd, const rclcpp::Time& stamp) -> void {
			const double now = stamp.seconds();
			if (this->cmd_pub_) {
				this->cmd_pub_->publish(to_msg(cmd));
			} else {
				geometry_msgs::msg::TwistStamped m{};
				m.header.stamp = stamp;
				m.header.frame_id = this->base_frame_;
				m.twist = to_msg(cmd);
				this->cmd_stamped_pub_->publish(m);
			}
			// オブザーバには「機体に効き始める時刻」で入れる
			this->observer_.add_input(now + this->cmd_delay_, cmd);
		}

		auto on_timer() -> void {
			const rclcpp::Time stamp = this->now();
			const double now = stamp.seconds();

			// シミュレーション時刻の巻き戻し (bag のループなど) は全部捨てる
			if (this->last_now_ && now < *this->last_now_ - 1e-3) {
				RCLCPP_WARN(this->get_logger(), "time jumped backwards, resetting");
				this->observer_.reset();
				this->reference_.reset();
				this->last_tf_stamp_.reset();
				this->active_ = false;
				if (this->tf_buffer_) { this->tf_buffer_->clear(); }
			}
			const double dt = this->last_now_ ? std::clamp(now - *this->last_now_, 0.0, 5.0 * this->period_) : 0.0;
			this->last_now_ = now;

			this->poll_tf();

			const auto est = this->observer_.estimate(now);
			const auto last_meas = this->observer_.last_measurement_stamp();

			std::uint8_t state = TrackingStatus::STATE_ACTIVE;
			if (!this->enabled_) {
				state = TrackingStatus::STATE_DISABLED;
			} else if (!est) {
				state = TrackingStatus::STATE_WAITING_POSE;
			} else if (now - *last_meas > this->pose_timeout_) {
				state = TrackingStatus::STATE_POSE_TIMEOUT;
			} else if (!this->reference_ || now - this->reference_received_ > this->reference_timeout_) {
				state = TrackingStatus::STATE_NO_REFERENCE;
			}

			TrackingStatus status{};
			status.header.stamp = stamp;
			status.header.frame_id = this->field_frame_;
			status.state = state;
			if (last_meas) { status.pose_age = now - *last_meas; }

			if (state == TrackingStatus::STATE_ACTIVE) {
				if (!this->active_) {
					// 今の動きから始める (加速度制限の起点)
					this->controller_.reset(holonomic_tracker::field_to_body(est->velocity, est->pose.yaw));
					RCLCPP_INFO(this->get_logger(), "tracking started");
				}
				this->active_ = true;

				// 目標をその時刻から今まで速度FFで外挿する
				Reference ref = *this->reference_;
				const double lag = std::clamp(
					now - this->reference_stamp_,
					-this->max_ref_extrapolation_,
					this->max_ref_extrapolation_
				);
				ref.pose.x += ref.velocity.vx * lag;
				ref.pose.y += ref.velocity.vy * lag;
				ref.pose.yaw = holonomic_tracker::wrap_angle(ref.pose.yaw + ref.velocity.omega * lag);

				const auto out = this->controller_.step(ref, est->pose, est->velocity, dt);
				this->publish_cmd(out.command, stamp);

				status.saturated = out.saturated;
				status.error_x = out.error.x;
				status.error_y = out.error.y;
				status.error_yaw = out.error.yaw;
				// 進行方向基準の誤差。止まっている目標なら機体の向きを基準にする
				const double heading = std::hypot(ref.velocity.vx, ref.velocity.vy) > 1e-3
					? std::atan2(ref.velocity.vy, ref.velocity.vx)
					: est->pose.yaw;
				status.error_along = std::cos(heading) * out.error.x + std::sin(heading) * out.error.y;
				status.error_cross = -std::sin(heading) * out.error.x + std::cos(heading) * out.error.y;
				status.feedforward = to_msg(out.feedforward);
				status.feedback = to_msg(out.feedback);
				status.command = to_msg(out.command);
			} else {
				if (this->active_) {
					// 止める。以降は (別のノードが動かすかもしれないので) 出し続けない
					this->publish_cmd(Twist2{}, stamp);
					static constexpr const char* reasons[] = {
						"", "waiting for pose", "pose timeout", "no reference", "disabled"};
					RCLCPP_WARN(this->get_logger(), "tracking stopped: %s", reasons[state]);
				}
				this->active_ = false;
			}

			if (est) {
				status.disturbance = to_msg(est->disturbance);
				this->publish_odom(*est, stamp);
			}
			this->status_pub_->publish(status);
		}

		auto publish_odom(const holonomic_tracker::Estimate& est, const rclcpp::Time& stamp) -> void {
			nav_msgs::msg::Odometry m{};
			m.header.stamp = stamp;
			m.header.frame_id = this->field_frame_;
			m.child_frame_id = this->base_frame_;
			m.pose.pose.position.x = est.pose.x;
			m.pose.pose.position.y = est.pose.y;
			m.pose.pose.orientation.z = std::sin(0.5 * est.pose.yaw);
			m.pose.pose.orientation.w = std::cos(0.5 * est.pose.yaw);
			m.pose.covariance[0] = est.sigma_pose.x * est.sigma_pose.x;
			m.pose.covariance[7] = est.sigma_pose.y * est.sigma_pose.y;
			m.pose.covariance[35] = est.sigma_pose.yaw * est.sigma_pose.yaw;
			// Odometry の twist は child_frame_id (機体座標系) で表す決まり
			m.twist.twist = to_msg(holonomic_tracker::field_to_body(est.velocity, est.pose.yaw));
			m.twist.covariance[0] = est.sigma_velocity.vx * est.sigma_velocity.vx;
			m.twist.covariance[7] = est.sigma_velocity.vy * est.sigma_velocity.vy;
			m.twist.covariance[35] = est.sigma_velocity.omega * est.sigma_velocity.omega;
			this->odom_pub_->publish(m);
		}

		std::string field_frame_;
		std::string base_frame_;
		double period_{0.02};
		double reference_timeout_{0.2};
		double pose_timeout_{0.3};
		double max_ref_extrapolation_{0.2};
		double cmd_delay_{0.0};

		VelocityObserver observer_{};
		TrackingController controller_{};

		std::optional<Reference> reference_{};
		double reference_stamp_{};
		double reference_received_{};
		std::optional<double> last_tf_stamp_{};
		std::optional<double> last_now_{};
		bool active_{false};
		bool enabled_{true};

		std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_{};
		rclcpp::Subscription<TrackingReference>::SharedPtr reference_sub_{};
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_{};
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_{};
		rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_{};
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_{};
		rclcpp::Publisher<TrackingStatus>::SharedPtr status_pub_{};
		rclcpp::TimerBase::SharedPtr timer_{};
		rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrackerNode>());
	rclcpp::shutdown();
	return 0;
}
