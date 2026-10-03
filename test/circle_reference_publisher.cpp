/// @file circle_reference_publisher.cpp
/// 「円を描きながら機体も回る」目標を流す。手動テスト用で、軌道生成の例でもある。

#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "omni3_tracker/msg/tracking_reference.hpp"
#include "omni3_tracker/types.hpp"

namespace {
	using omni3_tracker::msg::TrackingReference;

	class CircleReferencePublisher final : public rclcpp::Node {
	public:
		CircleReferencePublisher() : rclcpp::Node{"circle_reference_publisher"} {
			this->frame_ = this->declare_parameter<std::string>("field_frame", "field");
			this->cx_ = this->declare_parameter<double>("center_x", 0.0);
			this->cy_ = this->declare_parameter<double>("center_y", 0.0);
			this->radius_ = this->declare_parameter<double>("radius", 1.0);
			this->period_ = this->declare_parameter<double>("period", 6.0);
			this->spin_ = this->declare_parameter<double>("spin", 1.0);
			const double rate = this->declare_parameter<double>("rate", 50.0);

			this->pub_ = this->create_publisher<TrackingReference>("reference", rclcpp::QoS{10});
			this->timer_ = rclcpp::create_timer(
				this,
				this->get_clock(),
				rclcpp::Duration::from_seconds(1.0 / rate),
				[this] { this->publish(); }
			);
		}

	private:
		auto publish() -> void {
			const rclcpp::Time now = this->now();
			if (!this->t0_) { this->t0_ = now.seconds(); }
			const double t = now.seconds() - *this->t0_;
			const double w = 2.0 * std::numbers::pi / this->period_;

			TrackingReference m{};
			m.header.stamp = now;
			m.header.frame_id = this->frame_;
			m.x = this->cx_ + this->radius_ * std::cos(w * t);
			m.y = this->cy_ + this->radius_ * std::sin(w * t);
			m.yaw = omni3_tracker::wrap_angle(this->spin_ * t);
			m.vx = -this->radius_ * w * std::sin(w * t);
			m.vy = this->radius_ * w * std::cos(w * t);
			m.omega = this->spin_;
			this->pub_->publish(m);
		}

		std::string frame_;
		double cx_{};
		double cy_{};
		double radius_{};
		double period_{};
		double spin_{};
		std::optional<double> t0_{};
		rclcpp::Publisher<TrackingReference>::SharedPtr pub_{};
		rclcpp::TimerBase::SharedPtr timer_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CircleReferencePublisher>());
	rclcpp::shutdown();
	return 0;
}
