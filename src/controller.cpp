/// @file controller.cpp
/// TrackingController の実装。

#include "holonomic_tracker/controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace holonomic_tracker {
	namespace {
		auto clamp_abs(const double v, const double limit) -> double {
			if (limit <= 0.0) { return v; }
			return std::clamp(v, -limit, limit);
		}

		/// 並進を大きさ `limit` 以下に方向を保って縮める。縮めたら true。
		auto limit_norm(double& x, double& y, const double limit) -> bool {
			if (limit <= 0.0) { return false; }
			const double n = std::hypot(x, y);
			if (n <= limit) { return false; }
			x *= limit / n;
			y *= limit / n;
			return true;
		}
	} // namespace

	TrackingController::TrackingController(const ControllerParams& params) : params_{params} {}

	auto TrackingController::set_params(const ControllerParams& params) -> void { this->params_ = params; }

	auto TrackingController::reset(const Twist2& current_body) -> void {
		this->integral_ = Pose2{};
		this->previous_ = current_body;
	}

	auto TrackingController::step(const Reference& ref, const Pose2& pose, const Twist2& velocity, const double dt)
		-> ControlOutput {
		const auto& prm = this->params_;
		ControlOutput out{};

		out.error = Pose2{ref.pose.x - pose.x, ref.pose.y - pose.y, wrap_angle(ref.pose.yaw - pose.yaw)};
		out.feedforward = ref.velocity;

		// 積分の候補。制限が効いたら採用しない
		Pose2 integral = this->integral_;
		if (dt > 0.0) {
			integral.x = clamp_abs(integral.x + out.error.x * dt, prm.linear.integral_limit);
			integral.y = clamp_abs(integral.y + out.error.y * dt, prm.linear.integral_limit);
			integral.yaw = clamp_abs(integral.yaw + out.error.yaw * dt, prm.angular.integral_limit);
		}

		const auto fb = [](const AxisGains& g, const double e, const double i, const double dv) {
			return g.kp * e + g.ki * i + g.kd * dv;
		};
		out.feedback = Twist2{
			fb(prm.linear, out.error.x, integral.x, ref.velocity.vx - velocity.vx),
			fb(prm.linear, out.error.y, integral.y, ref.velocity.vy - velocity.vy),
			fb(prm.angular, out.error.yaw, integral.yaw, ref.velocity.omega - velocity.omega),
		};

		const Twist2 field{
			out.feedforward.vx + out.feedback.vx,
			out.feedforward.vy + out.feedback.vy,
			out.feedforward.omega + out.feedback.omega,
		};
		Twist2 cmd = field_to_body(field, pose.yaw + field.omega * prm.heading_lookahead);

		bool saturated = false;

		// 1. 速度
		saturated |= limit_norm(cmd.vx, cmd.vy, prm.max_velocity_linear);
		if (prm.max_velocity_angular > 0.0 && std::abs(cmd.omega) > prm.max_velocity_angular) {
			cmd.omega = std::copysign(prm.max_velocity_angular, cmd.omega);
			saturated = true;
		}

		// 2. 加速度
		if (dt > 0.0) {
			double dx = cmd.vx - this->previous_.vx;
			double dy = cmd.vy - this->previous_.vy;
			if (limit_norm(dx, dy, prm.max_accel_linear * dt)) {
				cmd.vx = this->previous_.vx + dx;
				cmd.vy = this->previous_.vy + dy;
				saturated = true;
			}
			const double dw = cmd.omega - this->previous_.omega;
			const double lim = prm.max_accel_angular * dt;
			if (prm.max_accel_angular > 0.0 && std::abs(dw) > lim) {
				cmd.omega = this->previous_.omega + std::copysign(lim, dw);
				saturated = true;
			}
		}


		if (!saturated) { this->integral_ = integral; }
		this->previous_ = cmd;

		out.command = cmd;
		out.saturated = saturated;
		return out;
	}
} // namespace holonomic_tracker
