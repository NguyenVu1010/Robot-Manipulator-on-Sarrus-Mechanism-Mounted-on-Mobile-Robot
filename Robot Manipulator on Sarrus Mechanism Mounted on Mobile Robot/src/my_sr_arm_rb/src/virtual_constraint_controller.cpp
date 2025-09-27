#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <numeric>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

// Struct để lưu cấu hình cho mỗi cánh tay 2-DOF
struct ArmConfig
{
    std::vector<std::string> joint_names;
    std::string base_frame; // Gốc của mỗi cánh tay, ví dụ "link_sr_11"
};

class VirtualConstraintController
{
public:
    VirtualConstraintController(ros::NodeHandle& nh)
        : nh_(nh), tf_listener_(tf_buffer_)
    {
        // ... (phần code này không thay đổi)
        controller_topic_ = "/sr_joints_position_controller/command";
        l1_ = 0.16;
        l2_ = 0.16;
        base_frame_ = "base_link";
        z_source_frame_ = "link_1";
        z_source_offset_ = 0.0641527277482495;
        control_rate_ = 100.0;
        q_offset = 0.745;
        scale = 0.97;
        target_x_extension_ = 0.0;
        arm_configs_ = {
            {{"joint_sr_11", "joint_sr_21"}, "link_sr_11"},
            {{"joint_sr_12", "joint_sr_22"}, "link_sr_12"},
            {{"joint_sr_13", "joint_sr_23"}, "link_sr_13"},
            {{"joint_sr_14", "joint_sr_24"}, "link_sr_14"}
        };
        command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(controller_topic_, 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &VirtualConstraintController::controlLoop, this);
        ROS_INFO("Virtual Constraint Controller (for Gazebo) started.");
        ROS_INFO("Publishing commands to: %s", controller_topic_.c_str());
        ROS_INFO("Controlling SR arms to follow the height of '%s'", z_source_frame_.c_str());
    }

private:
    /**
     * @brief Giải bài toán động học ngược cho một cánh tay 2-DOF trong mặt phẳng.
     *        Hàm này tính cả hai nghiệm (elbow up/down) và chọn nghiệm có góc q1 nhỏ hơn.
     */
    bool solveIk2d(double x, double y, double& q1, double& q2)
    {
        double d_sq = x * x + y * y;
        double d = sqrt(d_sq);

        // Kiểm tra xem mục tiêu có nằm trong tầm với không
        if (d > l1_ + l2_ - 1e-6 || d < std::abs(l1_ - l2_) + 1e-6) {
            ROS_WARN_THROTTLE(1.0, "IK target (x=%.2f, y=%.2f, d=%.2f) is out of reach for arm.", x, y, d);
            return false;
        }

        double cos_q2 = (d_sq - l1_ * l1_ - l2_ * l2_) / (2 * l1_ * l2_);
        cos_q2 = std::max(-1.0, std::min(1.0, cos_q2)); // Kẹp giá trị

        // 1. Tính cả hai nghiệm cho q2 (elbow up và elbow down)
        double q2_sol1 = acos(cos_q2);  // Nghiệm 1
        double q2_sol2 = -acos(cos_q2); // Nghiệm 2

        // 2. Tính q1 tương ứng cho mỗi nghiệm
        double alpha = atan2(y, x);

        // Tính cho nghiệm 1
        double beta1 = atan2(l2_ * sin(q2_sol1), l1_ + l2_ * cos(q2_sol1));
        double q1_sol1 = alpha - beta1;

        // Tính cho nghiệm 2
        double beta2 = atan2(l2_ * sin(q2_sol2), l1_ + l2_ * cos(q2_sol2));
        double q1_sol2 = alpha - beta2;

        // 3. So sánh và chọn nghiệm có q1 nhỏ hơn
        if (q1_sol1 <= q1_sol2) {
            q1 = q1_sol1;
            q2 = q2_sol1;
        } else {
            q1 = q1_sol2;
            q2 = q2_sol2;
        }
        
        return true;
    }

    // ... (các hàm controlLoop và publishCommand không thay đổi)
    void controlLoop(const ros::TimerEvent& event)
    {
        try
        {
            geometry_msgs::TransformStamped ts_base_to_z_source = tf_buffer_.lookupTransform(
                base_frame_, z_source_frame_, ros::Time(0));
            double target_z_global = ts_base_to_z_source.transform.translation.z + z_source_offset_*scale;

            std::vector<double> all_solutions;
            all_solutions.reserve(arm_configs_.size() * 2);
            bool all_ik_succeeded = true;

            for (const auto& arm : arm_configs_) {
                geometry_msgs::TransformStamped ts_base_to_arm_base = tf_buffer_.lookupTransform(
                    base_frame_, arm.base_frame, ros::Time(0));
                double arm_base_z_global = ts_base_to_arm_base.transform.translation.z;

                double x_target_local = target_x_extension_;
                double y_target_local = target_z_global - arm_base_z_global-0.003;
                double q1, q2;
                // std::cout<< "Target in local frame of " << arm.base_frame << ": ("
                // << x_target_local << ", " << y_target_local << ","<<ts_base_to_z_source.transform.translation.z<<","<<target_z_global<<","<<arm_base_z_global<<")" << std::endl;
                if (solveIk2d(x_target_local, y_target_local, q1, q2)) {
                    all_solutions.push_back((q1-q_offset)*scale);
                    all_solutions.push_back((q2-3.14159+2*q_offset)*scale);
                } else {
                    ROS_WARN_THROTTLE(1.0, "IK failed for arm '%s'. Aborting control cycle.", arm.base_frame.c_str());
                    all_ik_succeeded = false;
                    break;
                }
            }
            
            if (all_ik_succeeded) {
                publishCommand(all_solutions);
            }
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF Exception: %s. Waiting for complete TF tree...", ex.what());
        }
    }
    
    void publishCommand(const std::vector<double>& joint_positions)
    {
        std_msgs::Float64MultiArray msg;
        msg.data = joint_positions;
        command_pub_.publish(msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher command_pub_;
    ros::Timer timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string controller_topic_;
    double l1_, l2_;
    std::string base_frame_;
    std::string z_source_frame_;
    double z_source_offset_;
    double control_rate_;
    double target_x_extension_;
    double q_offset;
    double scale;
    std::vector<ArmConfig> arm_configs_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_constraint_controller_node");
    ros::NodeHandle nh; 
    VirtualConstraintController controller(nh);
    ros::spin();
    return 0;
}