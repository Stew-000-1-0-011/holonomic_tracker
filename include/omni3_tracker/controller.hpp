#pragma once

/// @file controller.hpp
/// 軌道追従のフィードバック制御則。ROS非依存。
///
/// フィールド座標系で軸ごとに
///
///     u = v_ff + Kp e + Ki ∫e + Kd (v_ff - v_hat)
///     e = p_ref - p_hat   (yaw は折り返す)
///
/// を計算し、機体座標系へ回して cmd_vel とする。p_hat, v_hat はオブザーバの推定
/// (自己位置の遅れを指令で補償した現在値)。Kd の項は速度追従の遅れを補う。
///
/// 制限は次の順にかける。どれかが効いたら積分を止める (アンチワインドアップ)。
/// 1. 速度の上限 (並進はベクトルの大きさで、方向を保って縮める)
/// 2. 加速度の上限 (前回の指令からの変化量)
/// 3. オムニ3輪の車輪速の上限 (全体を一律に縮めるので、進む方向と回転の比は保たれる)

#include <array>
#include <numbers>

#include "omni3_tracker/types.hpp"

namespace omni3_tracker {
	/// 1軸ぶんのゲイン。
	struct AxisGains {
		double kp{};
		double ki{};
		double kd{};
		/// 積分値の上限 (絶対値) [m*s] / [rad*s]。<= 0 で制限なし
		double integral_limit{};
	};

	struct ControllerParams {
		AxisGains linear{3.0, 0.0, 0.3, 0.1};
		AxisGains angular{4.0, 0.0, 0.3, 0.1};

		/// 速度の上限 [m/s], [rad/s]。<= 0 で制限なし
		double max_velocity_linear{2.0};
		double max_velocity_angular{6.0};
		/// 加速度の上限 [m/s^2], [rad/s^2]。<= 0 で制限なし
		double max_accel_linear{5.0};
		double max_accel_angular{20.0};

		/// 車輪の取付角 [rad] (機体中心から見た車輪の方向。x前方から反時計回り)
		std::array<double, 3> wheel_angles{
			0.5 * std::numbers::pi,
			0.5 * std::numbers::pi + 2.0 * std::numbers::pi / 3.0,
			0.5 * std::numbers::pi + 4.0 * std::numbers::pi / 3.0,
		};
		/// 機体中心から車輪までの距離 [m]
		double wheel_distance{0.2};
		/// 車輪の周速の上限 [m/s]。<= 0 で制限なし
		double max_wheel_speed{0.0};

		/// 機体座標系へ回すときの yaw の先読み [s]。
		/// 指令が効いている間に機体が回るぶんを補う (制御周期の半分くらい)
		double heading_lookahead{0.01};
	};

	struct Reference {
		Pose2 pose{};
		/// 速度FF (フィールド座標系)
		Twist2 velocity{};
	};

	struct ControlOutput {
		/// cmd_vel (機体座標系)
		Twist2 command{};
		/// 以下はデバッグ用 (フィールド座標系)
		Pose2 error{};
		Twist2 feedforward{};
		Twist2 feedback{};
		bool saturated{false};
	};

	class TrackingController final {
	public:
		explicit TrackingController(const ControllerParams& params = ControllerParams{});

		auto set_params(const ControllerParams& params) -> void;
		[[nodiscard]] auto params() const -> const ControllerParams& { return this->params_; }

		/// 積分を捨て、加速度制限の起点を `current_body` にする。
		/// 追従を始めるとき (または止まった後に再開するとき) に呼ぶ。
		auto reset(const Twist2& current_body = Twist2{}) -> void;

		/// 1周期ぶん計算する。
		/// @param pose     推定した現在の姿勢
		/// @param velocity 推定した現在の速度 (フィールド座標系)
		/// @param dt       前回からの経過時間 [s]
		auto step(const Reference& ref, const Pose2& pose, const Twist2& velocity, double dt) -> ControlOutput;

		/// 車輪の周速 (オムニ3輪の逆運動学)。
		[[nodiscard]] auto wheel_speeds(const Twist2& body) const -> std::array<double, 3>;

	private:
		ControllerParams params_;
		Pose2 integral_{};
		Twist2 previous_{};
	};
} // namespace omni3_tracker
