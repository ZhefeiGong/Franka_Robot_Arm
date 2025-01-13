import sys
import rospy
import numpy as np
import geometry_msgs.msg as geom_msg
import time
import subprocess
from dynamic_reconfigure.client import Client
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String

class FrankaController:
    """
    用于管理 Franka Panda 机械臂的移动控制
    """
    def __init__(self, robot_ip, load_gripper):
        self.robot_ip = robot_ip
        self.load_gripper = load_gripper
        self.eepub = None
        self.gripperpub = None
        self.client = None
        self.impedance_controller = None

    def init_node(self):
        """
        启动 impedance 控制器
        """
        rospy.loginfo("启动 Impedance Controller...")
        try:
            try:
                roscore = subprocess.Popen('roscore')
                time.sleep(1)
            except:
                pass
            self.impedance_controller = subprocess.Popen(
                ['roslaunch', 'serl_franka_controllers', 'impedance.launch',
                 f'robot_ip:={self.robot_ip}', f'load_gripper:={self.load_gripper}'],
                stdout=subprocess.PIPE
            )
            time.sleep(3)  # 等待控制器启动
            if self.impedance_controller.poll() is not None:
                raise RuntimeError("Failed to start impedance controller.")
            rospy.loginfo("Impedance Controller 启动成功。")
            
            ########################
            rospy.loginfo("初始化 ROS 节点...")
            self.eepub = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', 
                                     geom_msg.PoseStamped, queue_size=10)
            self.gripperpub = rospy.Publisher('/gripper_control', String, queue_size=10)
            rospy.init_node('franka_control_api')
            self.client = Client("/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node")
            rospy.loginfo("ROS 节点初始化完成。")

            ######################位置初始化
            initial_position = [0.5, 0, 0.2]
            initial_orientation = [np.pi, 0, np.pi / 2]
            self.move_to(initial_position, initial_orientation)
            time.sleep(1)

            rospy.loginfo("设置参考限制值...")
            for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
                self.client.update_configuration({"translational_clip_" + direction: 0.005})
                self.client.update_configuration({"rotational_clip_" + direction: 0.04})
            rospy.loginfo("参考限制值设置完成。")  
            time.sleep(1)     
        except Exception as e:
            rospy.logerr(f"初始化失败: {e}")
            sys.exit(1)

    def move_to(self, position, euler):
        """
        移动机械臂到指定位置和姿态
        example: position = [0.5, 0, 0.4]
        euler: [np.pi, 0, np.pi/2]
        """
        msg = geom_msg.PoseStamped()
        msg.header.frame_id = "0"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = geom_msg.Point(*position)
        quat = R.from_euler('xyz', [*euler]).as_quat()
        msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
        self.eepub.publish(msg)
        time.sleep(0.2)
        rospy.loginfo(f"机械臂移动到位置: {position}，姿态: {euler}")

    def gripper_action(self, action):
        """
        控制抓夹开合
        action: "open" 或 "close"
        """
        if action in ["open", "close"]:
            self.gripperpub.publish(action)
            rospy.loginfo(f"抓夹执行动作: {action}")
        else:
            rospy.logwarn("无效的抓夹动作: 仅支持 'open' 或 'close'")

    def stop_controller(self):
        """
        停止控制器
        """
        if self.impedance_controller:
            rospy.loginfo("停止 Impedance Controller...")
            self.impedance_controller.terminate()
            rospy.loginfo("Impedance Controller 已停止。")
