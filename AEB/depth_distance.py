import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/carla/hero/camera_depth/image_depth',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 전체 이미지를 미터 단위 거리로 변환
            b = cv_image[:, :, 0]
            g = cv_image[:, :, 1]
            r = cv_image[:, :, 2]
            
            # CARLA Depth 공식
            normalized = (r.astype(float) + g.astype(float) * 256.0 + b.astype(float) * 256.0 * 256.0) / (256.0**3 - 1)
            depth_map = 1000.0 * normalized
            
            height, width = depth_map.shape
            
            # ================= [수정된 부분] =================
            # 중앙보다 얼마나 더 아래를 볼지 설정 (픽셀 단위)
            # 이 값을 조절해서 앞차가 잘 잡히는 위치를 찾으세요.
            offset_y = 0 
            
            # 새로운 중심점 (기존 중앙 + offset)
            center_x = width // 2
            center_y = (height // 2) + offset_y
            
            # 영역 크기 (중앙에서 +- 50픽셀 -> 총 100x100 박스)
            roi_size = 50 
            
            # 박스 좌표 계산 (화면 밖으로 나가지 않도록 max, min 처리)
            x_start = max(0, center_x - roi_size)
            x_end = min(width, center_x + roi_size)
            y_start = max(0, center_y - roi_size)
            y_end = min(height, center_y + roi_size)
            # ===============================================
            
            # 설정한 영역(ROI)만 잘라냅니다.
            center_roi = depth_map[y_start:y_end, x_start:x_end]
            
            # 영역이 유효한지 확인 (가끔 계산 오류로 빈 배열이 될 수 있음)
            if center_roi.size > 0:
                # 그 영역 안에서 '가장 가까운 값(최소값)'을 찾습니다.
                min_distance = np.min(center_roi)
                print(f'전방 하단({offset_y}px) 거리: {min_distance:.2f} meters')
            else:
                print("영역 설정 오류: 범위를 확인하세요.")
            
        except Exception as e:
            print(f'에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
