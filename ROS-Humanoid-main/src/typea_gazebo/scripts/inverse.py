#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf.transformations import euler_from_matrix, quaternion_matrix
from typea_gazebo.typea import TypeA

class HumanoidIKSolver:
    def __init__(self):
        rospy.init_node("humanoid_ik_solver")
        self.typea = TypeA()

        # Current joint states
        self.all_angles = self.typea.get_angles()
        self.joint_states = {
            "l_shoulder_lateral_joint": self.all_angles["l_shoulder_lateral_joint"],
            "l_shoulder_swing_joint": self.all_angles["l_shoulder_swing_joint"],
            "l_elbow_joint": self.all_angles["l_elbow_joint"],
        }

        # Desired end-effector position
        self.target_position = np.array([0.0, 0.0, 0.0])  # Example target
        self.target_orientation = np.array([0.0, 0.0, 0.0])  # Roll, Pitch, Yaw

        self.joint_limits = {
            "l_shoulder_swing_joint": (-np.pi/2, np.pi/2),
            "l_shoulder_lateral_joint": (-1.5, 1.5),
            "l_elbow_joint": (-1.77, 1.77)
        }

    def compute_fk(self, joint_angles):
        """
        Computes the forward kinematics for the humanoid arm.
        Returns the end-effector position and orientation.
        """
        # Example FK for 3-DOF arm
        L1 = 0.0675  # Shoulder-to-elbow length
        L2 = 0.058  # Elbow-to-end-effector length

        
        x = L1 * np.cos(joint_angles[0]) * np.cos(joint_angles[1]) + L2 * np.cos(joint_angles[0]) * np.cos(joint_angles[1] + joint_angles[2])
        y = L1 * np.sin(joint_angles[0]) * np.cos(joint_angles[1]) + L2 * np.sin(joint_angles[0]) * np.cos(joint_angles[1] + joint_angles[2])
        z = L1 * np.sin(joint_angles[1]) + L2 * np.sin(joint_angles[1] + joint_angles[2])
        return np.array([x, y, z]), [0.0, 0.0, 0.0]  # [Position, Orientation (Euler)]

    def compute_jacobian(self, joint_angles):
        """
        Computes the Jacobian matrix for the humanoid arm.
        """
        J = np.zeros((3, 3))

        L1 = 0.0675
        L2 = 0.058
        
        # Partial derivatives w.r.t. r_shoulder_lateral_joint (joint 0)
        J[0, 0] = -L1 * np.sin(joint_angles[0]) * np.cos(joint_angles[1]) - L2 * np.sin(joint_angles[0]) * np.cos(joint_angles[1] + joint_angles[2])
        J[1, 0] =  L1 * np.cos(joint_angles[0]) * np.cos(joint_angles[1]) + L2 * np.cos(joint_angles[0]) * np.cos(joint_angles[1] + joint_angles[2])
        J[2, 0] =  0.0  # No dependence of z on the lateral joint (joint 0)

        # Partial derivatives w.r.t. r_shoulder_swing_joint (joint 1)
        J[0, 1] = -L1 * np.cos(joint_angles[0]) * np.sin(joint_angles[1]) - L2 * np.cos(joint_angles[0]) * np.sin(joint_angles[1] + joint_angles[2])
        J[1, 1] = -L1 * np.sin(joint_angles[0]) * np.sin(joint_angles[1]) - L2 * np.sin(joint_angles[0]) * np.sin(joint_angles[1] + joint_angles[2])
        J[2, 1] =  L1 * np.cos(joint_angles[1]) + L2 * np.cos(joint_angles[1] + joint_angles[2])

        # Partial derivatives w.r.t. r_elbow_joint (joint 2)
        J[0, 2] = -L2 * np.cos(joint_angles[0]) * np.sin(joint_angles[1] + joint_angles[2])
        J[1, 2] = -L2 * np.sin(joint_angles[0]) * np.sin(joint_angles[1] + joint_angles[2])
        J[2, 2] =  L2 * np.cos(joint_angles[1] + joint_angles[2])

        return J

    def inverse_kinematics(self):
        """
        Solves IK using numerical methods.
        """
        # Initial guess for joint angles
        joint_names = list(self.joint_states.keys())
        joint_angles = np.array([self.joint_states[name] for name in joint_names])

        max_iterations = 100
        tolerance = 1e-3

        for iteration in range(max_iterations):
            # Compute current FK
            current_position, _ = self.compute_fk(joint_angles)
            delta_position = self.target_position - current_position

            # Check convergence
            if np.linalg.norm(delta_position) < tolerance:
                rospy.loginfo("IK converged in {} iterations.".format(iteration))
                break

            # Compute Jacobian and pseudo-inverse   
            J = self.compute_jacobian(joint_angles)
            J_pseudo_inverse = np.linalg.pinv(J)

            # Update joint angles
            delta_angles = np.dot(J_pseudo_inverse, delta_position)
            joint_angles += delta_angles

            # Apply joint limits
            for i, name in enumerate(joint_names):
                joint_angles[i] = np.clip(joint_angles[i], self.joint_limits[name][0], self.joint_limits[name][1])
        



        end = {}

        # Publish the result to the joints

        rospy.loginfo(self.target_position)
        for i, name in enumerate(joint_names):
            end[name] = joint_angles[i]

        self.typea.set_angles_slow(end)
        rospy.loginfo("Finished...")

        rospy.loginfo("Calculated : " + str(end))
        rospy.loginfo("After published and  moved : " + str(self.typea.get_angles()))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.inverse_kinematics()
            rate.sleep()

if __name__ == "__main__":
    try:
        solver = HumanoidIKSolver()
        solver.run()
    except rospy.ROSInterruptException:
        pass
