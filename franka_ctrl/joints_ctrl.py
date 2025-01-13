#@time: Jan.13th 2025
#@func: the controller for franka robot arm
#@copyright: 
# - Zhefei(Jeffrey) Gong

import time
import rospy
import numpy as np
import geometry_msgs.msg as geom_msg
from dynamic_reconfigure.client import Client 
from scipy.spatial.transform import Rotation as R


class FrankaJointsController():

    def __init__(self):
        super().__init__()

        # rospy.init_node('franka_joints_controller')
        
        self.ee_pub = rospy.Publisher(
            "/cartesian_impedance_controller/equilibrium_pose", geom_msg.PoseStamped, queue_size=10,
        )

        self.reconf_client = Client("/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node")
    
    def move(self, pose: list):
        """Move to a pose: [x, y, z, qx, qy, qz, qw]"""
        assert len(pose) == 7
        msg = geom_msg.PoseStamped()
        msg.header.frame_id = "0"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = geom_msg.Point(pose[0], pose[1], pose[2])
        # quat = R.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat()
        msg.pose.orientation = geom_msg.Quaternion(pose[3], pose[4], pose[5], pose[6])
        self.ee_pub.publish(msg)

    def set_conf(self):
        """Update the speed of movement"""
        for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
            self.reconf_client.update_configuration({"translational_clip_" + direction: 0.005})
            self.reconf_client.update_configuration({"rotational_clip_" + direction: 0.04})

    def reset_joint(self):
        """Reset joints (through Joint Position Controller)"""
        # First Stop impedance
        try:
            self.stop_impedance()
            self.clear()
        except:
            print("impedance Not Running")
        time.sleep(3)
        self.clear()

        # Launch joint controller reset
        # set rosparm with rospkg
        # rosparam set /target_joint_positions '[q1, q2, q3, q4, q5, q6, q7]'
        rospy.set_param("/target_joint_positions", self.reset_joint_target)

        self.joint_controller = subprocess.Popen(
            [
                "roslaunch",
                self.ros_pkg_name,
                "joint.launch",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Franka' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(1)
        print("RUNNING JOINT RESET")
        self.clear()

        # Wait until target joint angles are reached
        count = 0
        time.sleep(1)
        while not np.allclose(
            np.array(self.reset_joint_target) - np.array(self.q),
            0,
            atol=1e-2,
            rtol=1e-2,
        ):
            time.sleep(1)
            count += 1
            if count > 30:
                print("joint reset TIMEOUT")
                break

        # Stop joint controller
        print("RESET DONE")
        self.joint_controller.terminate()
        time.sleep(1)
        self.clear()
        print("KILLED JOINT RESET", self.pos)

        # Restart impedece controller
        self.start_impedance()
        print("impedance STARTED")



