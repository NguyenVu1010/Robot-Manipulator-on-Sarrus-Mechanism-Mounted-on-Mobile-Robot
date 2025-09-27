import numpy as np
from scipy.spatial import ConvexHull
import rospy
class RobotArm:
    """
    A class to represent a 5-DOF robotic arm and solve its inverse kinematics.
    """
    
    def __init__(self, L=(0.095,0.077,0.2,0.15,0.05), joint_limits=None):
        """
        Initialize the robotic arm with link lengths and joint limits.
        
        Parameters:
        L : list or array-like
            Link lengths [l1, l2, l3, l4, l5].
        joint_limits : dict, optional
            Dictionary of joint limits {'joint_1': (min, max), ...}.
            If None, defaults to provided limits.
        """
        if len(L) != 5:
            raise ValueError("L must contain exactly 5 link lengths [l1, l2, l3, l4, l5]")
        self.L = np.array(L, dtype=float)  # Store link lengths as numpy array
        
        # Default joint limits if none provided
        self.joint_limits = joint_limits if joint_limits else {
            'joint_1': (0, 0.25),   # q1 (prismatic)
            'joint_2': (-3.14, 3.14),  # q2 (revolute)
            'joint_3': (0, 3.14),   # q3 (revolute)
            'joint_4': (-1.57, 1.57),  # q4 (revolute)
            'joint_5': (-1.57, 1.57)   # q5 (revolute, assumed)
        }
        
        if len(self.joint_limits) != 5:
            raise ValueError("joint_limits must contain exactly 5 joints: joint_1 to joint_5")
    
    def forward1(self, q):
        """
        Compute the forward kinematics for the 5-DOF robotic arm.
        
        Parameters:
        q : array-like
            Joint angles [q1, q2, q3, q4, q5] in radians.
        
        Returns:
        xe : ndarray
            End-effector pose [x, y, z, theta].
        """
        l1, l2, l3, l4, l5 = self.L
        q1, q2, q3, q4, q5 = q
        
        # Precompute trigonometric combinations
        c2 = np.cos(q2)
        s2 = np.sin(q2)
        c34 = np.cos(q3 + q4)
        s34 = np.sin(q3 + q4)
        c345 = np.cos(q3 + q4 + q5)
        s345 = np.sin(q3 + q4 + q5)
        c3 = np.cos(q3)
        s3 = np.sin(q3)
        
        # Compute position components
        x = c2 * (l4 * c34 + l3 * c3 + l5 * c345)
        y = s2 * (l4 * c34 + l3 * c3 + l5 * c345)
        z = l1 + l2 + q1 + l4 * s34 + l3 * s3 + l5 * s345
        
        # Compute orientation (theta = q3 + q4 + q5)
        theta = q3 + q4 + q5
        
        return np.array([x, y, z, theta])
    
    def jacobian1(self, q):
        """
        Compute the Jacobian matrix for the 5-DOF robotic arm.
        
        Parameters:
        q : array-like
            Joint angles [q1, q2, q3, q4, q5] in radians.
        
        Returns:
        j2 : ndarray
            4x5 Jacobian matrix.
        """
        l1, l2, l3, l4, l5 = self.L
        q1, q2, q3, q4, q5 = q
        
        # Precompute trigonometric combinations
        s2 = np.sin(q2)
        c2 = np.cos(q2)
        s34 = np.sin(q3 + q4)
        c34 = np.cos(q3 + q4)
        s345 = np.sin(q3 + q4 + q5)
        c345 = np.cos(q3 + q4 + q5)
        s3 = np.sin(q3)
        c3 = np.cos(q3)
        
        # Terms for position components
        l34 = l4 * c34 + l3 * c3 + l5 * c345
        l34_45 = l4 * c34 + l5 * c345
        l45 = l5 * c345
        
        # Construct Jacobian matrix
        j2 = np.array([
            [0, -s2 * l34, -c2 * (l4 * s34 + l3 * s3 + l5 * s345), -c2 * (l4 * s34 + l5 * s345), -l5 * s345 * c2],
            [0, c2 * l34, -s2 * (l4 * s34 + l3 * s3 + l5 * s345), -s2 * (l4 * s34 + l5 * s345), -l5 * s345 * s2],
            [1, 0, l34, l34_45, l45],
            [0, 0, 1, 1, 1]
        ])
        
        return j2
    
    def newton_raphson(self, q_target,q):
        """
        Solve inverse kinematics using the Newton-Raphson method.
        
        Parameters:
        x : array-like
            Target end-effector pose [x, y, z, theta].
        
        Returns:
        q0 : ndarray
            Computed joint angles [q1, q2, q3, q4, q5].
        """
        # Get joint limits
        q1_min, q1_max = self.joint_limits['joint_1']
        q2_min, q2_max = self.joint_limits['joint_2']
        q3_min, q3_max = self.joint_limits['joint_3']
        q4_min, q4_max = self.joint_limits['joint_4']
        q5_min, q5_max = self.joint_limits['joint_5']
        
        # Convergence settings
        tolerance = 1e-4
        max_iterations = 200
        num_iterations = 0
        
        # Initialize output
        q0 = np.zeros_like(q)
        
        while num_iterations < max_iterations:
            # Compute position error
            fx = self.forward1(q) - q_target
            
            # Compute Jacobian and its pseudo-inverse
            j = self.jacobian1(q)
            jx = np.linalg.pinv(j)  # Pseudo-inverse
            
            # Update joint angles
            delta_q = jx @ (-fx)
            q = q + delta_q
            
            # Enforce joint limits
            q[0] = max(q1_min, min(q[0], q1_max))
            q[1] = max(q2_min, min(q[1], q2_max))
            q[2] = max(q3_min, min(q[2], q3_max))
            q[3] = max(q4_min, min(q[3], q4_max))
            q[4] = max(q5_min, min(q[4], q5_max))
            
            # Check convergence
            if np.linalg.norm(delta_q) < tolerance and np.linalg.norm(fx) < tolerance:
                q0 = q
                break
            
            num_iterations += 1
        
        # Warn if not converged
        if num_iterations >= max_iterations:
            print("Warning: Inverse kinematics did not converge within the maximum iterations.")
        
        # Compute and display final error
        q0 = q
        deltax = self.forward1(q0) - q_target
        print("deltax:")
        print(deltax)
        
        return q0
class WorkspaceChecker:
    """
    A class to check if a target pose is within the reachable workspace of the robotic arm using manual calculations.
    """
    
    def __init__(self, robot_arm, tolerance=1e-3):
        """
        Initialize the workspace checker with a RobotArm instance.
        
        Parameters:
        robot_arm : RobotArm
            The robotic arm instance to check the workspace for.
        tolerance : float, optional
            Tolerance for considering a point reachable.
        """
        self.robot_arm = robot_arm
        self.tolerance = tolerance
        self.x_range = None
        self.y_range = None
        self.z_range = None
        self.theta_range = None
        self._calculate_workspace_boundaries()
        self._log_workspace_info()

    def _calculate_workspace_boundaries(self):
        """
        Calculate the workspace boundaries manually based on link lengths and joint limits.
        """
        l1, l2, l3, l4, l5 = self.robot_arm.L
        joint_limits = self.robot_arm.joint_limits
        
        # Calculate maximum reach in xy-plane (based on l3, l4, l5 and q2, q3, q4, q5)
        max_reach = l3 + l4 + l5  # Maximum when links are fully extended (cos terms = 1)
        min_reach = 0  # Minimum when links are folded (cos terms = -1 or 0)
        self.x_range = [-max_reach, max_reach]
        self.y_range = [-max_reach, max_reach]
        
        # Calculate z-range
        q1_min, q1_max = joint_limits['joint_1']
        q3_min, q3_max = joint_limits['joint_3']
        q4_min, q4_max = joint_limits['joint_4']
        q5_min, q5_max = joint_limits['joint_5']
        # Z = l1 + l2 + q1 + l4 * sin(q3 + q4) + l3 * sin(q3) + l5 * sin(q3 + q4 + q5)
        max_sin_sum = np.sin(q3_max + q4_max) + np.sin(q3_max) + np.sin(q3_max + q4_max + q5_max)
        min_sin_sum = np.sin(q3_min + q4_min) + np.sin(q3_min) + np.sin(q3_min + q4_min + q5_min)
        z_max = l1 + l2 + q1_max + l4 * max_sin_sum + l3 * max_sin_sum + l5 * max_sin_sum
        z_min = l1 + l2 + q1_min + l4 * min_sin_sum + l3 * min_sin_sum + l5 * min_sin_sum
        self.z_range = [z_min, z_max]
        
        # Calculate theta range (theta = q3 + q4 + q5)
        theta_max = q3_max + q4_max + q5_max
        theta_min = q3_min + q4_min + q5_min
        self.theta_range = [theta_min, theta_max]

    def _log_workspace_info(self):
        """
        Log the workspace boundaries for debugging.
        """
        rospy.loginfo("Workspace boundaries:")
        rospy.loginfo("X range: [%.3f, %.3f]", self.x_range[0], self.x_range[1])
        rospy.loginfo("Y range: [%.3f, %.3f]", self.y_range[0], self.y_range[1])
        rospy.loginfo("Z range: [%.3f, %.3f]", self.z_range[0], self.z_range[1])
        rospy.loginfo("Theta range: [%.3f, %.3f]", self.theta_range[0], self.theta_range[1])

    def is_reachable(self, target_pose):
        """
        Check if the target pose [x, y, z, theta] is within the reachable workspace.
        
        Parameters:
        target_pose : array-like
            Target end-effector pose [x, y, z, theta].
        
        Returns:
        bool
            True if the pose is reachable within tolerance, False otherwise.
        """
        x, y, z, theta = target_pose
        
        # Check position in xy-plane (distance from origin)
        xy_distance = np.sqrt(x**2 + y**2)
        if xy_distance < self.x_range[0] - self.tolerance or xy_distance > self.x_range[1] + self.tolerance:
            return False
        
        # Check z-coordinate
        if z < self.z_range[0] - self.tolerance or z > self.z_range[1] + self.tolerance:
            return False
        
        # Check theta
        if theta < self.theta_range[0] - self.tolerance or theta > self.theta_range[1] + self.tolerance:
            return False
        
        return True
if __name__ == "__main__":
    c = WorkspaceChecker(RobotArm())
    print(c.is_reachable([0.5,0,0.5,1.57]))
