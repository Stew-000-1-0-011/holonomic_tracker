# tracker_node の内部状態 (デバッグ・調整用)。

uint8 STATE_ACTIVE = 0
uint8 STATE_WAITING_POSE = 1      # 自己位置がまだ来ていない
uint8 STATE_POSE_TIMEOUT = 2      # 自己位置が pose_timeout より古い
uint8 STATE_NO_REFERENCE = 3      # 目標が来ていない / reference_timeout より古い
uint8 STATE_DISABLED = 4          # ~/enable で止められている

std_msgs/Header header
uint8 state
# いずれかの制限 (速度・加速度・車輪速) が効いた。このとき積分は止まる
bool saturated

# 目標 - 推定 (フィールド座標系) [m], [rad]
float64 error_x
float64 error_y
float64 error_yaw
# 進行方向 (速度FFの向き) とその左向きに分けた位置誤差 [m]。速度FFが 0 なら機体の向き基準
float64 error_along
float64 error_cross

# 速度FF と フィードバック (フィールド座標系)
geometry_msgs/Twist feedforward
geometry_msgs/Twist feedback
# 出した cmd_vel (機体座標系)
geometry_msgs/Twist command

# オブザーバが推定した外乱速度 (フィールド座標系)
geometry_msgs/Twist disturbance
# 最後に受け入れた自己位置の古さ [s]
float64 pose_age
