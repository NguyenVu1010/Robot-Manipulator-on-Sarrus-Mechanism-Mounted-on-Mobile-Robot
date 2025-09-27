cmake_minimum_required(VERSION 3.0.2)
project(my_robot_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs      
  control_msgs     
  trajectory_msgs  
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

# ===================================================================
# === BƯỚC 1: CẤU HÌNH PYTHON SETUP (QUAN TRỌNG NHẤT)             ===
# ===================================================================
# Bỏ bình luận dòng này. Nó nói với catkin hãy tìm và chạy setup.py,
# điều này làm cho thư viện 'robot_lib' có thể được import.
catkin_python_setup()

# ===================================================================
# === BƯỚC 2: TẠO MESSAGE TÙY CHỈNH                             ===
# ===================================================================

# Khai báo các file .msg cần được xử lý
add_message_files(
  FILES
  DesiredJointState.msg
)

# Tạo các message, khai báo các package mà message của chúng ta phụ thuộc vào
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ===================================================================
# === BƯỚC 3: KHAI BÁO PACKAGE                                   ===
# ===================================================================

catkin_package(
  # Các package mà một package khác sử dụng package NÀY cũng sẽ cần.
  CATKIN_DEPENDS
    rospy
    std_msgs
    message_runtime # QUAN TRỌNG: Cần cho các message đã được tạo
)

# ===================================================================
# === BƯỚC 4: CÀI ĐẶT (TÙY CHỌN NHƯNG NÊN CÓ)                     ===
# ===================================================================
# Đánh dấu các node Python để chúng được cài đặt
catkin_install_python(PROGRAMS
  src/trajectory_node.py
  src/control_node.py
  src/arm_gui_tkinter.py
  src/realtime_plotter.py
  # Thêm các node khác nếu có
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)