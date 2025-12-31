import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import scipy.special
import sys
import os

UFLD_PATH = '/root/ros2_ws/src/Ultra-Fast-Lane-Detection'
sys.path.append(UFLD_PATH)

from model.model import parsingNet

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Image,
            '/carla/hero/camera_front/image_color',
            self.image_callback,
            qos_profile)
        
        self.publisher = self.create_publisher(Image, '/lane/result', 10)
        self.bridge = CvBridge()

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net = parsingNet(pretrained=False, backbone='18', cls_dim=(100+1, 56, 4), use_aux=False).to(self.device)
        
        pth_path = os.path.join(UFLD_PATH, 'tusimple_18.pth')
        if os.path.exists(pth_path):
            state_dict = torch.load(pth_path, map_location=self.device)['model']
            self.net.load_state_dict(state_dict, strict=False)
            self.net.eval()
            self.get_logger().info('Model Loaded!')
        
        self.img_w = 1280
        self.img_h = 720
        self.row_anchor = self.get_tusimple_row_anchor()

    def get_tusimple_row_anchor(self):
        return [64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288]

    def image_callback(self, msg):
        try:
            if msg.encoding == 'bgra8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            img = cv2.resize(cv_image, (800, 288))
            img_tensor = torch.from_numpy(img).permute(2, 0, 1).to(self.device)
            img_tensor = img_tensor.float().div(255.0).unsqueeze(0)

            with torch.no_grad():
                out = self.net(img_tensor)

            col_sample = np.linspace(0, 800 - 1, 100 + 1)
            col_sample_w = col_sample[1] - col_sample[0]

            out_j = out[0].data.cpu().numpy()
            out_j = out_j[:, ::-1, :]
            prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
            idx = np.arange(100).reshape(-1, 1, 1)
            loc = np.sum(prob * idx, axis=0)
            out_j = np.argmax(out_j, axis=0)
            loc[out_j == 100] = 0
            out_j = loc

            # 시각화 및 좌표 추출
            vis_img = cv2.resize(cv_image, (1280, 720))
            lane_points = {} # { lane_idx : [(x, y), ...] }

            for i in range(out_j.shape[1]):
                if np.sum(out_j[:, i] != 0) > 2:
                    points = []
                    for k in range(out_j.shape[0]):
                        if out_j[k, i] > 0:
                            # 100 + 1이 아니라 55 - k로 수정된 부분 (TuSimple 기준)
                            x = int(out_j[k, i] * col_sample_w * 1280 / 800) - 1
                            y = int(self.img_h * (self.row_anchor[55 - k] / 288)) - 1
                            points.append((x, y))
                            cv2.circle(vis_img, (x, y), 5, (0, 255, 0), -1)
                    lane_points[i] = points

            # [핵심] 조향각 계산
            steering_angle = self.calc_steering_angle(lane_points)
            
            # 화면에 텍스트로 표시
            cv2.putText(vis_img, f"Steering: {steering_angle:.2f} deg", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            out_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def calc_steering_angle(self, lane_points):
        """
        차선의 중앙값과 이미지 중앙값의 차이를 이용해 조향각을 계산합니다.
        """
        img_center_x = 1280 // 2  # 이미지의 가로 중앙
        look_ahead_idx = 15 

        left_lane_x = None
        right_lane_x = None

        for lane_idx, points in lane_points.items():
            if len(points) > look_ahead_idx:
                px, py = points[look_ahead_idx] 
                
                if px < img_center_x:
                    if left_lane_x is None or px > left_lane_x:
                        left_lane_x = px
                
                if px > img_center_x:
                    if right_lane_x is None or px < right_lane_x:
                        right_lane_x = px

        # 목표 지점 계산
        target_x = img_center_x 
        
        if left_lane_x is not None and right_lane_x is not None:
            target_x = (left_lane_x + right_lane_x) / 2
        elif left_lane_x is not None:
            target_x = left_lane_x + 350
        elif right_lane_x is not None:
            target_x = right_lane_x - 350
        
        # 오차 계산 및 P-Control
        error_x = target_x - img_center_x
        Kp = 0.1 
        angle_deg = error_x * Kp
        
        # 로그 출력
        self.get_logger().info(f"Left: {left_lane_x}, Right: {right_lane_x}, Err: {error_x:.1f}, Angle: {angle_deg:.2f}")
        return angle_deg

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
