#pragma once

/// @file observer.hpp
/// 自己位置 (姿勢だけ) と自分が出した cmd_vel から、機体速度を推定するオブザーバ。ROS非依存。
///
/// x, y, yaw の各軸を [位置 p, 速度 v, 外乱速度 d] の3状態カルマンフィルタで持つ。モデルは
///
///     dv/dt = (u - v) / tau + w        (u: 指令速度, w: 白色加速度ノイズ)
///     dd/dt = w_d                       (ランダムウォーク)
///     dp/dt = v + d
///
/// すなわち「下位の速度制御は指令に1次遅れで追従し、それに滑りなどの
/// ゆっくり変わる外乱速度 d が乗る」。機体の速度の推定は v + d。
/// v の1次遅れは機体座標系で起きるとして扱う (回りながら走っても速度がずれない)。
/// d はフィールド座標系の量。
///
/// tau <= 0 なら指令を使わない等速モデル (dv/dt = w) になり、d は使わない
/// (v と区別できないため)。
///
/// 指令は機体座標系で受け取り、推定した yaw で各区間ごとにフィールド座標系へ回す。
/// そのため x, y は yaw を介してだけつながり、各軸は 3x3 の計算で済む。
///
/// 自己位置には遅れがある (スキャン時刻 + ICP の処理時間)。
/// 観測は「最後に観測を入れた時刻の状態 (アンカー)」に対して時刻どおりに入れ、
/// 現在の推定はそこから指令の履歴で予測し直す。したがって
/// - 観測の遅れはその時刻のまま正しく扱われる
/// - 現在までの区間は指令で外挿されるので、遅れ分の位置ずれが補償される
/// アンカーより古い観測 (順序の入れ替わり) は捨てる。

#include <array>
#include <cstddef>
#include <deque>
#include <optional>

#include "holonomic_tracker/types.hpp"

namespace holonomic_tracker {
	struct ObserverParams {
		/// 下位速度制御の時定数 [s]。<= 0 で等速モデル (指令を使わない)
		double tau_linear{0.1};
		double tau_angular{0.1};
		/// 白色加速度ノイズの強さ (連続時間のPSDの平方根) [m/s^2/sqrt(Hz)], [rad/s^2/sqrt(Hz)]
		double accel_noise_linear{1.0};
		double accel_noise_angular{2.0};
		/// 自己位置の観測ノイズ (標準偏差) [m], [rad]
		double sigma_position{0.01};
		double sigma_yaw{0.01};
		/// 外乱速度のランダムウォークの強さ [m/s/sqrt(s)], [rad/s/sqrt(s)]。0 で外乱を推定しない
		double disturbance_noise_linear{0.05};
		double disturbance_noise_angular{0.1};
		/// 初期化時の速度の標準偏差 [m/s], [rad/s]
		double initial_sigma_velocity_linear{0.5};
		double initial_sigma_velocity_angular{1.0};
		/// 初期化時の外乱速度の標準偏差 [m/s], [rad/s]
		double initial_sigma_disturbance_linear{0.1};
		double initial_sigma_disturbance_angular{0.2};
		/// イノベーションのゲート [sigma]。<= 0 で無効
		double gate_sigma{0.0};
		/// ゲートで連続してこの回数を超えて棄却したら、観測で初期化し直す。0 で初期化しない
		std::size_t reset_after_rejects{5};
		/// アンカーと観測の間隔がこれを超えたら、予測せず観測で初期化し直す [s]
		double max_gap{1.0};
	};

	/// 推定値。速度はフィールド座標系。
	struct Estimate {
		double stamp{};
		Pose2 pose{};
		/// 機体の速度 (指令への追従ぶん + 外乱)
		Twist2 velocity{};
		/// そのうち外乱のぶん
		Twist2 disturbance{};
		/// 各成分の標準偏差
		Pose2 sigma_pose{};
		Twist2 sigma_velocity{};
	};

	enum class UpdateResult {
		initialized, ///< 初回または初期化し直した
		accepted,
		out_of_order, ///< アンカーより古いので捨てた
		gated, ///< ゲートで棄却した
	};

	class VelocityObserver final {
	public:
		explicit VelocityObserver(const ObserverParams& params = ObserverParams{});

		auto set_params(const ObserverParams& params) -> void;
		[[nodiscard]] auto params() const -> const ObserverParams& { return this->params_; }

		/// 指令を記録する。`stamp` はその指令が機体に効き始める時刻
		/// (送信時刻 + 下位までの遅れ)。次の指令までの間、一定とみなす。
		auto add_input(double stamp, const Twist2& body_command) -> void;

		/// 自己位置の観測を入れる。
		auto update(double stamp, const Pose2& measured) -> UpdateResult;

		/// 時刻 `stamp` の推定。アンカーより前を聞かれたらアンカーを返す。
		/// 未初期化なら nullopt。
		[[nodiscard]] auto estimate(double stamp) const -> std::optional<Estimate>;

		/// 最後に受け入れた観測の時刻。未初期化なら nullopt。
		[[nodiscard]] auto last_measurement_stamp() const -> std::optional<double>;

		auto reset() -> void;

	private:
		/// 1軸ぶんの [位置, 速度, 外乱速度] と共分散。
		struct Axis {
			std::array<double, 3> x{};
			std::array<std::array<double, 3>, 3> P{};
		};

		struct State {
			double stamp{};
			Axis x{};
			Axis y{};
			Axis yaw{};
		};

		struct Input {
			double stamp{};
			Twist2 body{};
		};

		static auto predict_axis(Axis& axis, double dt, double u, double tau, double q, double qd) -> void;
		static auto update_axis(Axis& axis, double innovation, double r2) -> void;

		/// アンカー `from` を指令の履歴で `to` まで進める。
		[[nodiscard]] auto propagate(const State& from, double to) const -> State;
		auto initialize(double stamp, const Pose2& measured) -> void;
		/// アンカーより前の指令を捨てる (アンカー時点で効いている1つは残す)。
		auto prune_inputs() -> void;

		ObserverParams params_;
		std::optional<State> anchor_{};
		/// 時刻順。inputs_.front() より前は before_ が効いている。
		std::deque<Input> inputs_{};
		Twist2 before_{};
		std::size_t rejects_{0};
	};
} // namespace holonomic_tracker
