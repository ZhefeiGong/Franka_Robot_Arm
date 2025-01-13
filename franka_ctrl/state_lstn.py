#@time: Jan.13th 2025
#@func: the controller for franka robot arm
#@copyright: 
# - https://github.com/rail-berkeley/serl 
# - Zhefei(Jeffrey) Gong

import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from serl_franka_controllers.msg import ZeroJacobian
from dynamic_reconfigure.client import Client as ReconfClient


class FrankaStateListener():
    def __init__(self):
        super().__init__()

        # rospy.init_node('franka_state_listener')

        self.jacobian_sub = rospy.Subscriber(
            "/cartesian_impedance_controller/franka_jacobian", ZeroJacobian, self._set_jacobian,
        )
        self.state_sub = rospy.Subscriber(
            "franka_state_controller/franka_states", FrankaState, self._set_cur_joints
        )
        self.gripper_sub = rospy.Subscriber(
            "/franka_gripper/joint_states", JointState, self._set_cur_gripper
        )

        self.pose_grp = None    # current end-effector gripper pose
        self.pose = None        # current end-effector pose in robot base frame (xyz+rpy)
        self.vel = None         # current end-effector velocity in robot base frame
        self.q = None           # current joint position
        self.dq = None          # current joint velocity
        self.force = None       # estimated force on end-effector
        self.torque = None      # estimated torque on end-effector
        self.jacobian = None    # current zero-jacobian

    def _set_jacobian(self, msg):
        """update jacobian for velocity computation"""
        self.jacobian = np.array(list(msg.zero_jacobian)).reshape((6, 7), order="F")

    def _set_cur_joints(self, self):
        """update the current pose of the robot arm"""
        tmatrix = np.array(list(msg.O_T_EE)).reshape(4, 4).T
        r = R.from_matrix(tmatrix[:3, :3])
        pose = np.concatenate([tmatrix[:3, -1], r.as_quat()])
        self.pos = pose
        self.dq = np.array(list(msg.dq)).reshape((7,))
        self.q = np.array(list(msg.q)).reshape((7,))
        self.force = np.array(list(msg.K_F_ext_hat_K)[:3])
        self.torque = np.array(list(msg.K_F_ext_hat_K)[3:])
        try:
            self.vel = self.jacobian @ self.dq
        except:
            self.vel = np.zeros(6)
            rospy.logwarn(
                "Jacobian not set, end-effector velocity temporarily not available"
            )

    def _set_cur_gripper(self, msg):
        """internal callback to get the latest gripper position"""
        self.pose_grp = np.sum(msg.position)



