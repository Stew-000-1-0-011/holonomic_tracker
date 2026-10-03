# holonomic_tracker

全方位移動ロボット (オムニ、メカナムなど) の軌道追従を行う ROS 2 パッケージ (パッケージ名 `holonomic_tracker`)。

扱うのは機体速度 (vx, vy, omega) の `cmd_vel` だけで、駆動機構には依存しない。
車輪ごとの制限や逆運動学は、`cmd_vel` を受ける下位のドライバ側の仕事とする。

目標の位置・向きと速度FFを受け取り、自己位置にフィードバックをかけて `cmd_vel` を出す。
自己位置は姿勢しか来ないので、**機体速度はオブザーバで推定する**。オブザーバは
自分が出した `cmd_vel` も使い、自己位置の遅れ (スキャン時刻 + 処理時間) を補償した
現在の姿勢と速度を返す。

自己位置は [sotoba_ros](https://github.com/Stew-000-1-0-011/sotoba_ros) の TF
(`field -> base_link`) をそのまま使える。PoseStamped のトピックでもよい。

対応環境: ROS 2 Jazzy (Ubuntu 24.04) でビルド・テスト・動作を確認済み。
C++20 で書いているので、sotoba_ros と同じ Lyrical Luth でもそのまま通るはず (未確認)。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/holonomic_tracker/types.hpp` | 平面の姿勢・速度の型、角度の折り返し、座標変換。ROS非依存 |
| `include/holonomic_tracker/observer.hpp`, `src/observer.cpp` | 機体速度のオブザーバ (カルマンフィルタ)。ROS非依存 |
| `include/holonomic_tracker/controller.hpp`, `src/controller.cpp` | フィードバック制御則と制限。ROS非依存 |
| `src/tracker_node.cpp` | ノード本体。ROSの入出力とパラメータだけを見る |
| `msg/TrackingReference.msg` | 目標 (位置・向き・速度FF) |
| `msg/TrackingStatus.msg` | 内部状態 (誤差・FF・FB・外乱など。調整用) |
| `config/tracker_node.yaml` | パラメータ |
| `launch/tracker_node.launch.py` | 起動。`sim:=true` で模擬ロボットと円の目標も立てる |
| `test/tracking_sim_test.cpp` | 閉ループのシミュレーションテスト (ROS不要) |
| `test/fake_holonomic_robot.cpp` | `cmd_vel` で動く模擬ロボット。遅れ・ノイズ付きで TF を出す |
| `test/circle_reference_publisher.cpp` | 円を描きながら回る目標を出す (軌道生成の例) |
| `test/check_tracking.py` | 模擬ロボットの真値と目標を突き合わせる手動テスト |

## ビルド

```bash
cd ~/ros2_ws/src
git clone <このリポジトリ>
cd ~/ros2_ws
colcon build --packages-select holonomic_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 使い方

```bash
ros2 launch holonomic_tracker tracker_node.launch.py
```

sotoba_ros と組み合わせるなら、sotoba_node 側で `publish_tf: true`、
`tf_child_frame: base_link` にしておく。tracker_node は既定で
`field -> base_link` を TF から引く。

### トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| sub | `~/reference` | `holonomic_tracker/msg/TrackingReference` |
| sub | TF `field_frame -> base_frame` (`pose_source: tf`、既定) | |
| sub | `pose_topic` (`pose_source: topic` のとき) | `geometry_msgs/msg/PoseStamped` |
| pub | `cmd_vel_topic` (既定 `cmd_vel`) | `geometry_msgs/msg/Twist` (機体座標系)。`cmd_vel_stamped: true` で `TwistStamped` |
| pub | `~/odom` | `nav_msgs/msg/Odometry` (オブザーバの推定。twist は機体座標系) |
| pub | `~/status` | `holonomic_tracker/msg/TrackingStatus` |
| srv | `~/enable` | `std_srvs/srv/SetBool` (false で止める) |

### 目標の渡し方

`TrackingReference` は「時刻 `header.stamp` に、姿勢 (x, y, yaw) にいて、速度 (vx, vy, omega) で動いている」
という1点。座標系はすべてフィールド座標系 (`field_frame`)。速度FFもフィールド座標系で渡す。

tracker_node は受け取った点を、その時刻から今まで速度FFで外挿して使う
(`max_reference_extrapolation` まで)。なので軌道生成側は制御周期に合わせなくてよく、
多少遅れて届いても、少し先の時刻の点を送っても正しく扱われる。
`header.stamp` が 0 なら受信時刻とみなす。

`reference_timeout` (既定 0.2 s) 以上目標が来なければ、ゼロを1度出して止まる。
自己位置が `pose_timeout` (既定 0.3 s) より古くなったときも同じ。止まった後は
`cmd_vel` を出し続けないので、手動操縦など別のノードと切り替えて使える。

## 制御則

フィールド座標系で軸 (x, y, yaw) ごとに

    u = v_ff + kp * e + ki * ∫e + kd * (v_ff - v_hat)
    e = p_ref - p_hat   (yaw は [-pi, pi) に折り返す)

を計算し、機体座標系へ回して `cmd_vel` にする。`p_hat`, `v_hat` はオブザーバの推定。

- `kp` の項: 位置の誤差を戻す。主役
- `kd` の項: 速度FFに対して実際の速度が遅れているぶんを足す。下位の速度制御の遅れを補う
- `ki` の項: 滑りなどの一定外乱による定常偏差を消す。既定は 0

機体座標系へ回すときは、`heading_lookahead` 秒先の yaw を使う。
回りながら並進するとき、指令が効いている間に機体が回るぶんを補うため。

制限は次の順にかけ、どれかが効いたら積分を止める (アンチワインドアップ)。

1. 速度の上限。並進はベクトルの大きさで縮めるので、進む方向は変わらない
2. 加速度の上限。前回の指令からの変化量で見る

## オブザーバ

x, y, yaw の各軸を [位置, 速度, 外乱速度] の3状態カルマンフィルタで持つ。モデルは

    dv/dt = (u - v) / tau + (白色加速度ノイズ)
    dd/dt = (ランダムウォーク)
    dp/dt = v + d

で、「下位の速度制御は `cmd_vel` に時定数 `tau` の1次遅れで追従し、そこに
滑りなどのゆっくり変わる外乱速度 `d` が乗る」というもの。機体速度の推定は `v + d`。

- 1次遅れは**機体座標系で**起きるとして扱う。フィールド座標系で遅らせると、
  回りながら走るときに `tau * omega * |v|` の速度誤差が出る
  (下のテストで 0.077 → 0.023 m/s に改善)
- 外乱速度 `d` を持つので、滑りがあっても速度推定がずれない
  (下のテストで 0.13 → 0.04 m/s に改善)
- `tau <= 0` にすると指令を使わない等速モデルになる (外乱は使わない)

**自己位置の遅れの扱い。** 観測はそのスキャン時刻のまま、「最後に観測を入れた時刻の状態」に
時刻どおりに入れる。現在の推定は、そこから自分が出した `cmd_vel` の履歴で予測し直したもの。
なので遅れて届く自己位置でも、制御には「今」の姿勢が使われる。
古い観測 (順序の入れ替わり) は捨てる。

`gate_sigma` を設定すると、予測から大きく外れた自己位置 (ICPの誤対応など) を捨てる。
`reset_after_rejects` 回を超えて連続で捨てたら、自己位置の方を信じて初期化し直す。

## 調整の手順

1. **`observer.tau_linear` / `tau_angular`**: 下位の速度制御の時定数。
   ステップ状の `cmd_vel` を入れて、`~/odom` の速度が 63% に達するまでの時間を見る。
   ここが合っていないと、遅れの補償 (現在の姿勢の予測) がずれる
2. **`cmd_delay`**: `cmd_vel` を出してから車輪が動き出すまでの無駄時間。
   通信やマイコンの周期ぶん。分からなければ 0 から
3. **`observer.sigma_position` / `sigma_yaw`**: 自己位置のばらつき。止めた状態で
   自己位置を記録して標準偏差を見る
4. **`gains.*.kp`**: 小さめから上げていく。`~/status` の `error_along` (進行方向の遅れ) と
   `error_cross` (横ずれ) を見る。振動し始めたら下げる
5. **`gains.*.kd`**: 加減速のところで `error_along` が出るなら上げる
6. **`gains.*.ki`**: 定常的なずれが残るときだけ入れる

ゲイン・制限・オブザーバのパラメータは `ros2 param set /tracker_node gains.linear.kp 2.5`
のように実行中に変えられる。フレーム名・トピック名・制御周期は起動時のみ。

## テスト

### シミュレーションテスト (ROS不要)

`test/tracking_sim_test.cpp`。ノードと同じオブザーバ・制御則を、模擬した全方位移動ロボットで閉ループに回す。
模擬ロボットは、速度制御の1次遅れ (時定数 0.08 s)、指令の遅れ 0.01 s、
遅れてノイズの乗った自己位置 (20 Hz・遅れ 0.05 s・σ 3 mm) を持つ。
目標は半径 1 m・周期 6 s の円を、機体を 1 rad/s で回しながら走るもの (約 1 m/s)。

```bash
colcon test --packages-select holonomic_tracker
colcon test-result --verbose
```

実測 (立ち上がりの 3 s を除く 9 s):

| 条件 | 位置誤差 RMS | 位置誤差 最大 | yaw 誤差 RMS | 速度推定誤差 RMS |
| --- | --- | --- | --- | --- |
| 標準 | 4.6 mm | 7.1 mm | 0.0011 rad | 0.043 m/s |
| 自己位置が 10 Hz・遅れ 0.1 s | 3.9 mm | 6.9 mm | 0.0015 rad | 0.017 m/s |
| 外乱速度 (0.15, -0.1) m/s + ki=2 | 5.5 mm | 15.5 mm | 0.0012 rad | 0.043 m/s |
| (比較) オブザーバ無し・最新の自己位置をそのまま使う | 78 mm | 85 mm | 0.076 rad | - |
| (比較) オブザーバ無し・10 Hz・遅れ 0.1 s | 167 mm | 190 mm | 0.153 rad | - |

オブザーバ無しで大きくずれるのは、自己位置の遅れ分 (速度 × 遅れ) だけ常に後ろを見て制御するため。

### ノードごとの手動テスト

模擬ロボット (`fake_holonomic_robot`) と円の目標 (`circle_reference_publisher`) を一緒に立てる。
模擬ロボットは sotoba_node と同じ形 (スキャン時刻つき・遅れて届く TF) で自己位置を出す。

```bash
ros2 launch holonomic_tracker tracker_node.launch.py sim:=true
ros2 run holonomic_tracker check_tracking.py   # 別端末で。10 s 測って表示する
```

ROS 2 Jazzy で、位置誤差 RMS 8.2 mm・最大 10.1 mm、yaw 誤差 RMS 0.0011 rad を確認済み。
目標を止めると `tracking stopped: no reference` を出して止まること、
`ros2 param set` でゲインが変わることも確認済み。

`fake_holonomic_robot` はパラメータで時定数 (`plant_tau`)、自己位置の周期・遅れ・ノイズ、
外乱速度 (`disturbance_vx`, `disturbance_vy`) を変えられる。
