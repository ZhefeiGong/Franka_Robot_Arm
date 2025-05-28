#@time: Jan.13th 2025
#@func: the controller for franka robot arm
#@copyright: 
# - Zhefei(Jeffrey) Gong

import time
import rospy
import subprocess
import numpy as np
import geometry_msgs.msg as geom_msg
from dynamic_reconfigure.client import Client 
from scipy.spatial.transform import Rotation as R


class FrankaJointsController():

    def __init__(self):
        super().__init__()
        self.ee_pub = rospy.Publisher(
            "/cartesian_impedance_controller/equilibrium_pose", geom_msg.PoseStamped, queue_size=10,
        )
        self.reconf_client = Client("/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node")
    
    def move(self, 
             pose: list, 
             is_euler: bool=False):
        """Move to a pose: [x,y,z,qx,qy,qz,qw] or [x,y,z,rx,ry,rz]"""
        # check
        assert len(pose) == 7 or (len(pose) == 6 and is_euler), "mismatch pose length"
        # position
        msg = geom_msg.PoseStamped()
        msg.header.frame_id = "0"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = geom_msg.Point(pose[0], pose[1], pose[2])
        # orientation | [qx, qy, qz, qw]
        if len(pose)==6 and is_euler:
            quat = R.from_euler('xyz', [pose[3], pose[4], pose[5]]).as_quat()
            msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
        else:
            msg.pose.orientation = geom_msg.Quaternion(pose[3], pose[4], pose[5], pose[6])
        # publish
        self.ee_pub.publish(msg)

    def set_conf(self):
        """Update the speed of movement"""
        for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
            self.reconf_client.update_configuration({"translational_clip_" + direction: 0.005})
            self.reconf_client.update_configuration({"rotational_clip_" + direction: 0.04})


def test():
    """Test the joints controller through """
    rospy.init_node('franka_joints_controller')
    joints_controller = FrankaJointsController()
    is_euler = True
    
    print('move to initial place')
    time.sleep(1)
    pose_start = [0.5, 0, 0.2, np.pi, 0, np.pi/2]
    joints_controller.move(pose_start, is_euler)
    
    # print('set the configuration')
    # time.sleep(1)
    # joints_controller.set_conf()
    
    print('move to target place')
    time.sleep(1)
    for i in range(10):
        pose = [0.5, 0, 0.2+i*0.02, np.pi, 0, np.pi/2]
        joints_controller.move(pose, is_euler)
        time.sleep(0.2)
    
    print('back to initial place')
    time.sleep(1)
    for i in range(10):
        pose = [0.5, 0, 0.4-i*0.02, np.pi, 0, np.pi/2]
        joints_controller.move(pose, is_euler)
        time.sleep(0.2)
    

if __name__ == "__main__":
    test()


