cmake_minimum_required(VERSION 3.22)
# rosidl は C のターゲットも作るので C も有効にする
project(holonomic_tracker C CXX)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 26)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- メッセージ ---
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/TrackingReference.msg"
	"msg/TrackingStatus.msg"
	DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(tracker_msgs_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- オブザーバと制御則 (ROS非依存) ---
add_library(holonomic_tracker_core STATIC
	src/observer.cpp
	src/controller.cpp
)
target_include_directories(holonomic_tracker_core
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
set_target_properties(holonomic_tracker_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード ---
add_executable(tracker_node
	src/tracker_node.cpp
)
target_link_libraries(tracker_node
	PRIVATE
		holonomic_tracker_core
		${tracker_msgs_target}
		rclcpp::rclcpp
		${geometry_msgs_TARGETS}
		${nav_msgs_TARGETS}
		${std_srvs_TARGETS}
		tf2::tf2
		tf2_ros::tf2_ros
)

# --- 手動テスト用: 模擬ロボットと円の目標 ---
add_executable(fake_holonomic_robot
	test/fake_holonomic_robot.cpp
)
target_link_libraries(fake_holonomic_robot
	PRIVATE
		holonomic_tracker_core
		rclcpp::rclcpp
		${geometry_msgs_TARGETS}
		tf2_ros::tf2_ros
)

add_executable(circle_reference_publisher
	test/circle_reference_publisher.cpp
)
target_link_libraries(circle_reference_publisher
	PRIVATE
		holonomic_tracker_core
		${tracker_msgs_target}
		rclcpp::rclcpp
)

install(TARGETS tracker_node fake_holonomic_robot circle_reference_publisher
	DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS test/check_tracking.py
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
	DESTINATION include
)
install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	# ROSに依存しない閉ループのシミュレーション。
	# 模擬した全方位移動ロボット (1次遅れ・遅れた自己位置・外乱) に対して追従誤差と速度推定を見る。
	add_executable(holonomic_tracker_sim_test test/tracking_sim_test.cpp)
	target_link_libraries(holonomic_tracker_sim_test PRIVATE holonomic_tracker_core)
	# ament_add_test を使う (素の add_test だと結果ファイルが出ず colcon test-result から見えない)
	ament_add_test(holonomic_tracker_sim
		COMMAND "$<TARGET_FILE:holonomic_tracker_sim_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
