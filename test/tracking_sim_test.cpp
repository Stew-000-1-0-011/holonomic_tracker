/// @file tracking_sim_test.cpp
/// オブザーバと制御則を、模擬した全方位移動ロボットで閉ループに回すテスト。ROS不要。
///
/// 模擬する実機:
/// - 下位の速度制御は指令に1次遅れ (時定数 plant_tau) で追従し、指令は cmd_delay 遅れて効く
/// - 自己位置は pose_period ごとに、pose_delay 遅れて、ノイズ付きで届く (スキャン時刻つき)
/// - 一定の外乱速度 (滑りなど) が乗る
///
/// 目標は「円を描きながら機体も回る」軌道。追従誤差と速度推定の誤差を見る。

#include <cmath>
#include <cstdio>
#include <deque>
#include <numbers>
#include <random>
#include <string>

#include "holonomic_tracker/controller.hpp"
#include "holonomic_tracker/observer.hpp"

namespace {
	using namespace holonomic_tracker;

	struct Scenario {
		std::string name;
		double plant_tau{0.08};
		double cmd_delay{0.01};
		double pose_period{0.05};
		double pose_delay{0.05};
		double sigma_pos{0.003};
		double sigma_yaw{0.003};
		Twist2 disturbance{}; // フィールド座標系
		/// false なら比較用に、オブザーバを使わず最新の観測をそのまま現在の姿勢とする
		bool use_observer{true};
		double ki{0.0};
	};

	struct Result {
		double rms_pos{};
		double max_pos{};
		double rms_yaw{};
		double rms_vel{}; // 速度推定の誤差
	};

	auto reference_at(const double t) -> Reference {
		constexpr double radius = 1.0;
		constexpr double w = 2.0 * std::numbers::pi / 6.0; // 周期 6 s
		constexpr double spin = 1.0; // 機体の回転 [rad/s]
		return Reference{
			Pose2{radius * std::cos(w * t), radius * std::sin(w * t), wrap_angle(spin * t)},
			Twist2{-radius * w * std::sin(w * t), radius * w * std::cos(w * t), spin},
		};
	}

	auto run(const Scenario& sc) -> Result {
		std::mt19937 rng{42};
		std::normal_distribution<double> n01{0.0, 1.0};

		constexpr double sim_dt = 0.001;
		constexpr double ctrl_dt = 0.02;
		constexpr double duration = 12.0;
		constexpr double settle = 3.0;

		ObserverParams op{};
		op.tau_linear = 0.08;
		op.tau_angular = 0.08;
		op.sigma_position = sc.sigma_pos;
		op.sigma_yaw = sc.sigma_yaw;
		op.gate_sigma = 6.0;
		VelocityObserver observer{op};

		ControllerParams cp{};
		cp.linear.ki = sc.ki;
		cp.angular.ki = sc.ki;
		TrackingController controller{cp};

		// 真値。初期姿勢は目標から少しずらす
		Pose2 pose{1.1, -0.05, 0.1};
		Twist2 vel_body{}; // 機体座標系の実速度

		std::deque<std::pair<double, Twist2>> cmd_queue{}; // (効く時刻, 指令)
		Twist2 applied{};
		std::deque<std::pair<double, Pose2>> meas_queue{}; // (届く時刻, (スキャン時刻, 姿勢))
		std::deque<double> meas_stamp{};
		Pose2 last_meas{};

		double next_ctrl = 0.0;
		double next_meas = 0.0;
		bool started = false;

		double sum_p2 = 0.0;
		double sum_y2 = 0.0;
		double sum_v2 = 0.0;
		double max_p = 0.0;
		int n = 0;

		for (double t = 0.0; t < duration; t += sim_dt) {
			// 自己位置の観測 (スキャン時刻 t で撮って pose_delay 後に届く)
			if (t >= next_meas) {
				next_meas += sc.pose_period;
				meas_queue.emplace_back(
					t + sc.pose_delay,
					Pose2{pose.x + sc.sigma_pos * n01(rng), pose.y + sc.sigma_pos * n01(rng),
						  wrap_angle(pose.yaw + sc.sigma_yaw * n01(rng))}
				);
				meas_stamp.push_back(t);
			}
			while (!meas_queue.empty() && meas_queue.front().first <= t) {
				observer.update(meas_stamp.front(), meas_queue.front().second);
				last_meas = meas_queue.front().second;
				meas_queue.pop_front();
				meas_stamp.pop_front();
			}

			// 制御
			if (t >= next_ctrl) {
				next_ctrl += ctrl_dt;
				if (const auto est = observer.estimate(t)) {
					if (!started) {
						controller.reset(field_to_body(est->velocity, est->pose.yaw));
						started = true;
					}
					Pose2 p_used = est->pose;
					Twist2 v_used = est->velocity;
					if (!sc.use_observer) {
						p_used = last_meas;
						v_used = reference_at(t).velocity; // Kd の項を消す
					}
					const auto out = controller.step(reference_at(t), p_used, v_used, ctrl_dt);
					cmd_queue.emplace_back(t + sc.cmd_delay, out.command);
					observer.add_input(t + sc.cmd_delay, out.command);

					if (t >= settle) {
						const auto ref = reference_at(t);
						const double ep = std::hypot(ref.pose.x - pose.x, ref.pose.y - pose.y);
						const double ey = wrap_angle(ref.pose.yaw - pose.yaw);
						const Twist2 vf = body_to_field(vel_body, pose.yaw);
						const Twist2 vtrue{vf.vx + sc.disturbance.vx, vf.vy + sc.disturbance.vy, vf.omega};
						const double ev = std::hypot(est->velocity.vx - vtrue.vx, est->velocity.vy - vtrue.vy);
						sum_p2 += ep * ep;
						sum_y2 += ey * ey;
						sum_v2 += ev * ev;
						max_p = std::max(max_p, ep);
						++n;
					}
				}
			}

			// 実機
			while (!cmd_queue.empty() && cmd_queue.front().first <= t) {
				applied = cmd_queue.front().second;
				cmd_queue.pop_front();
			}
			const double a = sim_dt / sc.plant_tau;
			vel_body.vx += a * (applied.vx - vel_body.vx);
			vel_body.vy += a * (applied.vy - vel_body.vy);
			vel_body.omega += a * (applied.omega - vel_body.omega);
			const Twist2 vf = body_to_field(vel_body, pose.yaw + 0.5 * vel_body.omega * sim_dt);
			pose.x += (vf.vx + sc.disturbance.vx) * sim_dt;
			pose.y += (vf.vy + sc.disturbance.vy) * sim_dt;
			pose.yaw = wrap_angle(pose.yaw + vf.omega * sim_dt);
		}

		return Result{std::sqrt(sum_p2 / n), max_p, std::sqrt(sum_y2 / n), std::sqrt(sum_v2 / n)};
	}

	int failures = 0;

	auto expect_le(const char* what, const double value, const double limit) -> void {
		const bool ok = value <= limit;
		std::printf("    %-22s %.4f (<= %.4f) %s\n", what, value, limit, ok ? "ok" : "FAIL");
		if (!ok) { ++failures; }
	}

	auto test_observer_alone() -> void {
		// 等速で動く対象を、遅れのある観測だけ (指令なし) で追えるか
		std::puts("observer: constant velocity, no command input");
		ObserverParams op{};
		op.tau_linear = 0.0;
		op.tau_angular = 0.0;
		op.sigma_position = 0.002;
		op.sigma_yaw = 0.002;
		VelocityObserver obs{op};
		const Twist2 v{0.8, -0.3, 1.5};
		for (int k = 0; k <= 60; ++k) {
			const double ts = 0.05 * k;
			obs.update(ts, Pose2{v.vx * ts, v.vy * ts, wrap_angle(v.omega * ts)});
		}
		// 最後の観測から 0.05 s 先 (= 観測遅れぶんの外挿)
		const auto est = obs.estimate(3.05);
		expect_le("vel error [m/s]", std::hypot(est->velocity.vx - v.vx, est->velocity.vy - v.vy), 0.01);
		expect_le("omega error [rad/s]", std::abs(est->velocity.omega - v.omega), 0.01);
		expect_le("pose error [m]", std::hypot(est->pose.x - v.vx * 3.05, est->pose.y - v.vy * 3.05), 0.002);

		// 古い観測は捨てる
		const bool dropped = obs.update(1.0, Pose2{}) == UpdateResult::out_of_order;
		std::printf("    %-22s %s\n", "out-of-order dropped", dropped ? "ok" : "FAIL");
		if (!dropped) { ++failures; }
	}

	auto report(const Scenario& sc, const Result& r) -> void {
		std::printf(
			"closed loop: %s\n    rms pos %.4f m, max pos %.4f m, rms yaw %.4f rad, rms vel est %.4f m/s\n",
			sc.name.c_str(),
			r.rms_pos,
			r.max_pos,
			r.rms_yaw,
			r.rms_vel
		);
	}
} // namespace

auto main() -> int {
	test_observer_alone();

	{
		const Scenario sc{.name = "nominal"};
		const auto r = run(sc);
		report(sc, r);
		expect_le("rms pos [m]", r.rms_pos, 0.02);
		expect_le("rms yaw [rad]", r.rms_yaw, 0.02);
		expect_le("rms vel est [m/s]", r.rms_vel, 0.07);
	}
	{
		// 観測が遅く疎い (10 Hz, 0.1 s 遅れ)
		const Scenario sc{.name = "slow localization", .pose_period = 0.1, .pose_delay = 0.1};
		const auto r = run(sc);
		report(sc, r);
		expect_le("rms pos [m]", r.rms_pos, 0.03);
		expect_le("rms yaw [rad]", r.rms_yaw, 0.03);
	}
	{
		// 一定外乱 (滑り) を積分で消す
		// オブザーバは外乱速度も推定するので、速度推定は外乱があってもずれない
		const Scenario sc{.name = "disturbance + Ki", .disturbance = Twist2{0.15, -0.1, 0.0}, .ki = 2.0};
		const auto r = run(sc);
		report(sc, r);
		expect_le("rms pos [m]", r.rms_pos, 0.02);
		expect_le("rms vel est [m/s]", r.rms_vel, 0.07);
	}

	{
		// 比較用 (判定しない): オブザーバ無しで、最新の自己位置をそのまま使う
		const Scenario sc{.name = "reference: without observer", .use_observer = false};
		report(sc, run(sc));
		const Scenario sc2{
			.name = "reference: without observer, slow localization",
			.pose_period = 0.1,
			.pose_delay = 0.1,
			.use_observer = false,
		};
		report(sc2, run(sc2));
	}

	std::printf("%s (%d failures)\n", failures == 0 ? "PASS" : "FAIL", failures);
	return failures == 0 ? 0 : 1;
}
