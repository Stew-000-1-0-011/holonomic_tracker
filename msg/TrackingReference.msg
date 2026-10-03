# 軌道追従の目標 (1点ぶん)。
#
# header.frame_id : フィールド座標系。tracker_node の field_frame と一致させる (空なら一致とみなす)
# header.stamp    : この目標が指す時刻。0 なら受信時刻とみなす。
#                   tracker_node は (既定で) この時刻から現在まで速度FFで外挿して使うので、
#                   少し先の時刻の点を送ってもよい。

std_msgs/Header header

# 目標の位置 [m] と向き [rad] (フィールド座標系)
float64 x
float64 y
float64 yaw

# 速度FF [m/s], [rad/s] (フィールド座標系)
float64 vx
float64 vy
float64 omega
