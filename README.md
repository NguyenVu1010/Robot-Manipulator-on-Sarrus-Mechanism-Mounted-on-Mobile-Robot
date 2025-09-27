# Robot Manipulator on Sarrus Mechanism Mounted on Mobile Robot

## 📖 Giới thiệu
Dự án này tích hợp **cánh tay robot** với cơ cấu **Sarrus** và gắn toàn bộ lên một **robot tự hành (mobile robot)**.  
Mục tiêu là mở rộng **vùng làm việc (workspace)** của cánh tay robot, cho phép robot thực hiện các nhiệm vụ phức tạp hơn trong không gian rộng lớn, chẳng hạn như:
- Gắp và đặt vật ở nhiều độ cao khác nhau.  
- Thực hiện thao tác trong môi trường hẹp.  
- Vận chuyển và thao tác trong kho, quầy hoặc dây chuyền sản xuất.  

Hệ thống này kết hợp giữa:  
- **Cơ cấu Sarrus**: tăng phạm vi chuyển động theo phương thẳng đứng.  
- **Cánh tay robot**: xử lý thao tác chính xác (pick & place, assembly).  
- **Robot tự hành**: di chuyển trong không gian, kết hợp SLAM/Navigation.  

---

## 🏗️ Kiến trúc hệ thống

+------------------+
| Mobile Robot | <-- Nền tảng di chuyển (ROS Navigation, LIDAR, camera)
+------------------+
│
▼
+------------------+
| Sarrus Lift | <-- Cơ cấu song song cho phép nâng/hạ theo phương Z
+------------------+
│
▼
+------------------+
| Robot Arm (6DOF)| <-- Cánh tay robot điều khiển bằng MoveIt!
+------------------+

- **Mobile Robot Layer**: ROS1/ROS2, Gazebo, Navigation Stack.  
- **Manipulator Layer**: MoveIt, URDF/Xacro định nghĩa cánh tay robot.  
- **Sarrus Mechanism Layer**: thêm bậc tự do nâng hạ.  
- **Control Layer**: ROS controller, action server, reinforcement learning (tùy chọn).  

---

## ⚙️ Yêu cầu hệ thống

### Phần mềm
- Ubuntu 20.04 / 22.04  
- ROS Noetic (ROS1) hoặc ROS2 Humble  
- Gazebo ≥ 11  
- MoveIt  
- RViz  
- Python 3.x  
- Các package ROS:  
  - `robot_state_publisher`  
  - `joint_state_publisher`  
  - `ros_control`  
  - `gazebo_ros`  
  - `navigation`  

### Phần cứng (nếu chạy thực tế)
- Robot tự hành (ví dụ MiR100, TurtleBot, hoặc custom base).  
- Bộ cơ cấu Sarrus cơ khí.  
- Cánh tay robot (ví dụ UR5, Franka, hoặc custom 6DOF).  
- LIDAR, camera RGB-D.  

---
## 🎥 Video minh họa

[Demo Video](./videos/demo.mp4)

## 🚀 Cài đặt

Clone repo và build workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/username/repo_name.git
cd ..
catkin_make
source devel/setup.bash
