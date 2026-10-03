#pragma once

/// @file types.hpp
/// 平面の姿勢・速度の素朴な型。ROS非依存。

#include <cmath>
#include <numbers>

namespace omni3_tracker {
	/// 平面上の姿勢。yaw は [rad]。
	struct Pose2 {
		double x{};
		double y{};
		double yaw{};
	};

	/// 平面上の速度。どの座標系で表したものかは使う側で明示する。
	struct Twist2 {
		double vx{};
		double vy{};
		double omega{};
	};

	/// [-pi, pi) へ折り返す。
	inline auto wrap_angle(const double a) -> double {
		return a - 2.0 * std::numbers::pi * std::floor((a + std::numbers::pi) / (2.0 * std::numbers::pi));
	}

	/// 機体座標系の速度をフィールド座標系へ回す (並進だけ回る)。
	inline auto body_to_field(const Twist2& body, const double yaw) -> Twist2 {
		const double c = std::cos(yaw);
		const double s = std::sin(yaw);
		return Twist2{c * body.vx - s * body.vy, s * body.vx + c * body.vy, body.omega};
	}

	/// フィールド座標系の速度を機体座標系へ回す。
	inline auto field_to_body(const Twist2& field, const double yaw) -> Twist2 {
		const double c = std::cos(yaw);
		const double s = std::sin(yaw);
		return Twist2{c * field.vx + s * field.vy, -s * field.vx + c * field.vy, field.omega};
	}
} // namespace omni3_tracker
