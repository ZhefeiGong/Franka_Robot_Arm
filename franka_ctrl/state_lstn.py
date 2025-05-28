#@time: Jan.13th 2025
#@func: the controller for franka robot arm
#@copyright: 
# - https://github.com/rail-berkeley/serl 
# - Zhefei(Jeffrey) Gong

import rospy
import time
import numpy as np
from franka_msgs.msg import FrankaState
from serl_franka_controllers.msg import ZeroJacobian
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState


class FrankaStateListener():
    def __init__(self):
        super().__init__()

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

    def _set_cur_joints(self, msg):
        """update the current pose of the robot arm"""
        tmatrix = np.array(list(msg.O_T_EE)).reshape(4, 4).T
        r = R.from_matrix(tmatrix[:3, :3])
        self.pose = np.concatenate([tmatrix[:3, -1], r.as_quat()])
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

    def get_state(self):
        """get the current state"""
        state = {
                "pose_grp": self.pose_grp,
                "pose": np.array(self.pose),
                "vel": np.array(self.vel),
                "q": np.array(self.q),
                "dq": np.array(self.dq),
                "force": np.array(self.force),
                "torque": np.array(self.torque),
                "jacobian": np.array(self.jacobian),
                }
        return state
    
    def get_cartesian_pose(self):
        """get the current end-effector pose"""
        return np.array(self.pose)
    
    def get_joint_pose(self):
        """get the current the pose of the current joints"""
        return np.array(self.q)


def test():
    """Test the franka state listener"""
    rospy.init_node('franka_state_listener')
    state_listener = FrankaStateListener()
    
    time.sleep(2)
    print('------ cartesian pose ------')
    print(state_listener.get_cartesian_pose())
    time.sleep(2)
    print('------ joint pose ------')
    print(state_listener.get_joint_pose())
    time.sleep(2)
    print('------ franka state ------')
    print(state_listener.get_state())
    
    
if __name__ == "__main__":
    test()


