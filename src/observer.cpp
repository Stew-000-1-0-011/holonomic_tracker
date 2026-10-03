/// @file observer.cpp
/// VelocityObserver の実装。

#include "holonomic_tracker/observer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace holonomic_tracker {
	namespace {
		/// 回転の扱い (区間中央の yaw で回す) の誤差を抑えるための最大刻み [s]
		constexpr double max_step = 0.02;
		/// 指令の履歴の上限。観測が長く来ないときに無制限に溜めないため
		constexpr std::size_t max_inputs = 4096;
	} // namespace

	VelocityObserver::VelocityObserver(const ObserverParams& params) : params_{params} {}

	auto VelocityObserver::set_params(const ObserverParams& params) -> void { this->params_ = params; }

	auto VelocityObserver::reset() -> void {
		this->anchor_.reset();
		this->inputs_.clear();
		this->before_ = Twist2{};
		this->rejects_ = 0;
	}

	auto VelocityObserver::add_input(const double stamp, const Twist2& body_command) -> void {
		const auto it = std::upper_bound(
			this->inputs_.begin(),
			this->inputs_.end(),
			stamp,
			[](const double t, const Input& in) { return t < in.stamp; }
		);
		this->inputs_.insert(it, Input{stamp, body_command});

		if (!this->anchor_) {
			// 未初期化なら最新の1つだけ覚えておけばよい
			while (this->inputs_.size() > 1) {
				this->before_ = this->inputs_.front().body;
				this->inputs_.pop_front();
			}
			return;
		}
		this->prune_inputs();
		while (this->inputs_.size() > max_inputs) {
			this->before_ = this->inputs_.front().body;
			this->inputs_.pop_front();
		}
	}

	auto VelocityObserver::predict_axis(
		Axis& axis,
		const double dt,
		const double u,
		const double tau,
		const double q,
		const double qd
	) -> void {
		if (dt <= 0.0) { return; }

		// 状態遷移 F と入力の効き B。tau <= 0 なら等速モデルで、外乱は使わない
		using Mat = std::array<std::array<double, 3>, 3>;
		Mat F{{{1.0, dt, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
		std::array<double, 3> B{};
		if (tau > 0.0) {
			const double a = std::exp(-dt / tau);
			F[0][1] = tau * (1.0 - a);
			F[0][2] = dt;
			F[1][1] = a;
			B = {dt - tau * (1.0 - a), 1.0 - a, 0.0};
		}

		std::array<double, 3> x{};
		for (std::size_t i = 0; i < 3; ++i) {
			x[i] = B[i] * u;
			for (std::size_t j = 0; j < 3; ++j) { x[i] += F[i][j] * axis.x[j]; }
		}

		// P' = F P F^T + Q
		Mat FP{};
		for (std::size_t i = 0; i < 3; ++i) {
			for (std::size_t j = 0; j < 3; ++j) {
				for (std::size_t k = 0; k < 3; ++k) { FP[i][j] += F[i][k] * axis.P[k][j]; }
			}
		}
		Mat P{};
		for (std::size_t i = 0; i < 3; ++i) {
			for (std::size_t j = 0; j < 3; ++j) {
				for (std::size_t k = 0; k < 3; ++k) { P[i][j] += FP[i][k] * F[j][k]; }
			}
		}

		// Q。白色加速度ノイズ (p, v) と外乱のランダムウォーク (p, d) を
		// それぞれ等速モデルで離散化したもの
		const double dt2 = dt * dt;
		const double dt3 = dt2 * dt;
		const double q2 = q * q;
		const double qd2 = tau > 0.0 ? qd * qd : 0.0;
		P[0][0] += (q2 + qd2) * dt3 / 3.0;
		P[0][1] += q2 * dt2 / 2.0;
		P[1][0] += q2 * dt2 / 2.0;
		P[1][1] += q2 * dt;
		P[0][2] += qd2 * dt2 / 2.0;
		P[2][0] += qd2 * dt2 / 2.0;
		P[2][2] += qd2 * dt;

		axis = Axis{x, P};
	}

	auto VelocityObserver::update_axis(Axis& axis, const double innovation, const double r2) -> void {
		// H = [1, 0, 0]
		const double s = axis.P[0][0] + r2;
		std::array<double, 3> k{};
		for (std::size_t i = 0; i < 3; ++i) { k[i] = axis.P[i][0] / s; }
		for (std::size_t i = 0; i < 3; ++i) { axis.x[i] += k[i] * innovation; }
		const auto row0 = axis.P[0];
		for (std::size_t i = 0; i < 3; ++i) {
			for (std::size_t j = 0; j < 3; ++j) { axis.P[i][j] -= k[i] * row0[j]; }
		}
	}

	auto VelocityObserver::propagate(const State& from, const double to) const -> State {
		State s = from;
		double t = from.stamp;

		const auto step = [this, &s](double dt, const Twist2& u) {
			const auto& prm = this->params_;
			while (dt > 0.0) {
				const double h = std::min(dt, max_step);
				const double yaw0 = s.yaw.x[0];
				predict_axis(
					s.yaw, h, u.omega, prm.tau_angular, prm.accel_noise_angular, prm.disturbance_noise_angular
				);
				// 区間中の回転を区間中央の yaw で近似する
				const Twist2 uf = body_to_field(u, 0.5 * (yaw0 + s.yaw.x[0]));
				const double vx0 = s.x.x[1];
				const double vy0 = s.y.x[1];
				predict_axis(s.x, h, uf.vx, prm.tau_linear, prm.accel_noise_linear, prm.disturbance_noise_linear);
				predict_axis(s.y, h, uf.vy, prm.tau_linear, prm.accel_noise_linear, prm.disturbance_noise_linear);
				if (prm.tau_linear > 0.0) {
					// 1次遅れは機体座標系で起きるので、遅れて残っている速度 (a * v0) は
					// 機体と一緒に回る。各軸独立の予測ではそれが抜けるので平均だけ足す
					// (共分散は x, y がほぼ等方なので回さない)。
					// これが無いと、回りながら走るときに tau * omega * |v| の速度誤差が出る。
					const double a = std::exp(-h / prm.tau_linear);
					const double dyaw = s.yaw.x[0] - yaw0;
					const double c = std::cos(dyaw) - 1.0;
					const double sn = std::sin(dyaw);
					s.x.x[1] += a * (c * vx0 - sn * vy0);
					s.y.x[1] += a * (sn * vx0 + c * vy0);
				}
				dt -= h;
			}
		};

		// t の時点で効いている指令
		Twist2 u = this->before_;
		auto it = this->inputs_.begin();
		for (; it != this->inputs_.end() && it->stamp <= t; ++it) { u = it->body; }
		for (; it != this->inputs_.end() && it->stamp < to; ++it) {
			step(it->stamp - t, u);
			t = it->stamp;
			u = it->body;
		}
		step(to - t, u);
		s.stamp = std::max(to, from.stamp);
		return s;
	}

	auto VelocityObserver::initialize(const double stamp, const Pose2& measured) -> void {
		const auto& prm = this->params_;
		const auto make = [](const double p, const double sp, const double sv, const double sd) {
			Axis a{};
			a.x = {p, 0.0, 0.0};
			a.P[0][0] = sp * sp;
			a.P[1][1] = sv * sv;
			a.P[2][2] = sd * sd;
			return a;
		};
		const double sdl = prm.tau_linear > 0.0 ? prm.initial_sigma_disturbance_linear : 0.0;
		const double sda = prm.tau_angular > 0.0 ? prm.initial_sigma_disturbance_angular : 0.0;
		this->anchor_ = State{
			stamp,
			make(measured.x, prm.sigma_position, prm.initial_sigma_velocity_linear, sdl),
			make(measured.y, prm.sigma_position, prm.initial_sigma_velocity_linear, sdl),
			make(measured.yaw, prm.sigma_yaw, prm.initial_sigma_velocity_angular, sda),
		};
		this->rejects_ = 0;
		this->prune_inputs();
	}

	auto VelocityObserver::prune_inputs() -> void {
		if (!this->anchor_) { return; }
		while (!this->inputs_.empty() && this->inputs_.front().stamp <= this->anchor_->stamp) {
			this->before_ = this->inputs_.front().body;
			this->inputs_.pop_front();
		}
	}

	auto VelocityObserver::update(const double stamp, const Pose2& measured) -> UpdateResult {
		if (!this->anchor_ || stamp - this->anchor_->stamp > this->params_.max_gap) {
			this->initialize(stamp, measured);
			return UpdateResult::initialized;
		}
		if (stamp < this->anchor_->stamp) { return UpdateResult::out_of_order; }

		const auto& prm = this->params_;
		State s = this->propagate(*this->anchor_, stamp);

		const double rp = prm.sigma_position * prm.sigma_position;
		const double ry = prm.sigma_yaw * prm.sigma_yaw;
		const double ix = measured.x - s.x.x[0];
		const double iy = measured.y - s.y.x[0];
		const double iyaw = wrap_angle(measured.yaw - s.yaw.x[0]);

		if (prm.gate_sigma > 0.0) {
			const double g2 = prm.gate_sigma * prm.gate_sigma;
			const bool reject = ix * ix > g2 * (s.x.P[0][0] + rp) || iy * iy > g2 * (s.y.P[0][0] + rp)
				|| iyaw * iyaw > g2 * (s.yaw.P[0][0] + ry);
			if (reject) {
				++this->rejects_;
				if (prm.reset_after_rejects > 0 && this->rejects_ > prm.reset_after_rejects) {
					this->initialize(stamp, measured);
					return UpdateResult::initialized;
				}
				return UpdateResult::gated;
			}
		}
		this->rejects_ = 0;

		update_axis(s.x, ix, rp);
		update_axis(s.y, iy, rp);
		update_axis(s.yaw, iyaw, ry);
		// yaw は連続値で持つが、際限なく伸びないよう折り返しておく
		s.yaw.x[0] = measured.yaw + wrap_angle(s.yaw.x[0] - measured.yaw);

		this->anchor_ = s;
		this->prune_inputs();
		return UpdateResult::accepted;
	}

	auto VelocityObserver::estimate(const double stamp) const -> std::optional<Estimate> {
		if (!this->anchor_) { return std::nullopt; }
		const State s = this->propagate(*this->anchor_, stamp);
		// 機体の速度は v + d
		const auto vel = [](const Axis& a) { return a.x[1] + a.x[2]; };
		const auto vel_sigma = [](const Axis& a) {
			return std::sqrt(std::max(0.0, a.P[1][1] + a.P[2][2] + 2.0 * a.P[1][2]));
		};
		return Estimate{
			s.stamp,
			Pose2{s.x.x[0], s.y.x[0], wrap_angle(s.yaw.x[0])},
			Twist2{vel(s.x), vel(s.y), vel(s.yaw)},
			Twist2{s.x.x[2], s.y.x[2], s.yaw.x[2]},
			Pose2{std::sqrt(s.x.P[0][0]), std::sqrt(s.y.P[0][0]), std::sqrt(s.yaw.P[0][0])},
			Twist2{vel_sigma(s.x), vel_sigma(s.y), vel_sigma(s.yaw)},
		};
	}

	auto VelocityObserver::last_measurement_stamp() const -> std::optional<double> {
		if (!this->anchor_) { return std::nullopt; }
		return this->anchor_->stamp;
	}
} // namespace holonomic_tracker
