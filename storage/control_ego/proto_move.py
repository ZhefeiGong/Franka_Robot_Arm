import grpc
import image_action_pb2
import image_action_pb2_grpc
import cv2
import numpy as np
import pyrealsense2 as rs
import time
import sys
import rospy
from absl import app, flags
from move_control import FrankaController  # 确保此模块在PYTHONPATH中

FLAGS = flags.FLAGS
flags.DEFINE_string("robot_ip", "192.168.1.10", "IP address of the robot.")
flags.DEFINE_string("load_gripper", 'false', "Whether or not to load the gripper.")

def generate_images(pipeline):
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            
            # 编码图像为JPEG
            ret, buffer = cv2.imencode('.jpg', color_image)
            if not ret:
                continue
            image_bytes = buffer.tobytes()
            
            image_msg = image_action_pb2.ImageMessage(image_data=image_bytes)
            yield image_msg
    except GeneratorExit:
        print("Image generator closed.")

def run_client():
    # 初始化RealSense
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    # 连接到gRPC服务器
    channel = grpc.insecure_channel('10.13.117.92:50051')  # 如果服务器在其他机器上，替换为服务器IP
    stub = image_action_pb2_grpc.ImageActionServiceStub(channel)

    # 读取FLAGS参数
    robot_ip = FLAGS.robot_ip
    load_gripper = FLAGS.load_gripper

    # 创建FrankController实例
    franka = FrankaController(robot_ip, load_gripper)
    franka.init_node()

    # 生成图片并发送，同时接收动作
    responses = stub.ImageActionStream(generate_images(pipeline))

    try:
        for action_msg in responses:
            print(f"Received action: {action_msg.action}")
            if action_msg.action == "PERFORM_ACTION":
                # 执行动作
                perform_robot_action(franka, action_msg)
    except grpc.RpcError as e:
        print(f"gRPC error: {e}")
    finally:
        franka.stop_controller()
        pipeline.stop()
        sys.exit()

def perform_robot_action(franka, action_msg):
    try:
        # 示例动作执行，基于你提供的ROS代码
        # 这里可以直接集成ROS执行动作的逻辑
        # 为简单起见，这里假设执行一个简单的开抓夹动作
        print('###', action_msg)
        position = [0.5, 0, 0.2]
        initial_orientation = [np.pi, 0, np.pi / 2]
        #franka.move_to(position, initial_orientation)
        #franka.gripper_action("open")
    except Exception as e:
        rospy.logerr(f"执行动作时出错: {e}")

def main(argv):
    run_client()
    

if __name__ == '__main__':
    app.run(main)