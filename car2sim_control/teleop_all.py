import carla
import can
import struct
import time
import threading
import pygame

# -----------------------------------------------------------
# [설정] 아이오닉 5 환경 보정
# -----------------------------------------------------------
CAN_CHANNEL = 'can0'
CAN_BAUDRATE = 500000

# 1. 핸들 설정
REAL_STEER_MAX_DEG = 450.0  # 핸들 최대 각도
RES_STEER = 1/60.0          # 해상도

# 2. 페달 설정
RES_PEDAL = 0.001526        # 해상도

# -----------------------------------------------------------
# [전역 변수] 스레드 공유 데이터
# -----------------------------------------------------------
class VehicleState:
    def __init__(self):
        self.steer = 0.0      # -1.0 ~ 1.0
        self.throttle = 0.0   # 0.0 ~ 1.0
        self.brake = 0.0      # 0.0 ~ 1.0
        self.reverse = False  # True=후진, False=전진(D)
        self.running = True

shared_state = VehicleState()

# -----------------------------------------------------------
# [Thread 1] CAN Bus Reader (실차 데이터 수신)
# -----------------------------------------------------------
def can_loop():
    print(f"[CAN] {CAN_CHANNEL} 연결 시도 중...")
    try:
        bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan', bitrate=CAN_BAUDRATE)
    except OSError:
        print(f"[CAN Error] {CAN_CHANNEL} 장치를 찾을 수 없습니다.")
        shared_state.running = False
        return

    print("[CAN] 연결 성공. 데이터 수신 중...")

    while shared_state.running:
        try:
            msg = bus.recv(timeout=0.5)
            if msg is None: continue

            # --- [핸들] ID: 0x104 ---
            if msg.arbitration_id == 0x104 and msg.dlc == 8:
                try:
                    raw_data = struct.unpack('<ii', msg.data)
                    raw_steer_val = raw_data[0] 
                    
                    real_deg = raw_steer_val * RES_STEER
                    
                    # [수정됨] 핸들 방향 반전 (-1 곱하기)
                    # 기존: real_deg / 450.0
                    # 변경: -(real_deg / 450.0)
                    norm_steer = -1 * (real_deg / REAL_STEER_MAX_DEG)
                    
                    if norm_steer > 1.0: norm_steer = 1.0
                    if norm_steer < -1.0: norm_steer = -1.0
                    
                    shared_state.steer = norm_steer
                except: pass

            # --- [페달] ID: 0x314 ---
            elif msg.arbitration_id == 0x314 and msg.dlc == 8:
                try:
                    raw_data = struct.unpack('<HHHH', msg.data)
                    raw_brake = raw_data[0]
                    raw_accel = raw_data[2]

                    # 액셀
                    real_accel_pct = raw_accel * RES_PEDAL
                    norm_accel = real_accel_pct / 100.0
                    if norm_accel > 1.0: norm_accel = 1.0
                    if norm_accel < 0.0: norm_accel = 0.0
                    shared_state.throttle = norm_accel

                    # 브레이크 (반전 로직 유지)
                    real_brake_pct = raw_brake * RES_PEDAL
                    inverted_brake = 100.0 - real_brake_pct
                    norm_brake = inverted_brake / 100.0
                    
                    if norm_brake < 0.05: norm_brake = 0.0
                    if norm_brake > 1.0: norm_brake = 1.0
                    shared_state.brake = norm_brake
                except: pass

        except Exception:
            pass

    bus.shutdown()

# -----------------------------------------------------------
# [Thread 2] Main Loop (PyGame + CARLA Control)
# -----------------------------------------------------------
def main():
    # 1. CARLA 연결
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    try:
        world = client.get_world()
        vehicles = world.get_actors().filter('vehicle.*')
        if len(vehicles) > 0:
            vehicle = vehicles[0]
            print(f"[CARLA] 차량 제어: {vehicle.type_id}")
        else:
            print("[Error] CARLA에 차량이 없습니다. 차량을 먼저 소환해주세요.")
            return
    except:
        print("[Error] CARLA 연결 실패.")
        return

    # 2. PyGame 초기화 (키보드 입력을 위해 필요)
    pygame.init()
    display = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Real-Car Controller (Press D / R)")
    font = pygame.font.SysFont("monospace", 20)
    clock = pygame.time.Clock()

    # 3. CAN 스레드 시작
    t_can = threading.Thread(target=can_loop)
    t_can.start()

    print("\n=== 시작됨: PyGame 창을 클릭하고 R/D 키를 누르세요 ===")

    try:
        while True:
            # --- 이벤트 처리 (키보드) ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    shared_state.running = False
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        shared_state.reverse = True
                        print("\n[Gear] REVERSE (후진)")
                    elif event.key == pygame.K_d:
                        shared_state.reverse = False
                        print("\n[Gear] DRIVE (전진)")

            # --- CARLA 제어 적용 ---
            control = carla.VehicleControl()
            control.steer = shared_state.steer
            control.throttle = shared_state.throttle
            control.brake = shared_state.brake
            control.reverse = shared_state.reverse
            control.manual_gear_shift = False
            
            vehicle.apply_control(control)

            # --- 화면 그리기 ---
            display.fill((0, 0, 0)) # 검은 배경
            
            gear_str = "REVERSE (R)" if shared_state.reverse else "DRIVE (D)"
            color = (255, 100, 100) if shared_state.reverse else (100, 255, 100)

            lines = [
                f"CARLA Real-Car Link",
                f"-------------------",
                f"Gear    : {gear_str}",
                f"Steer   : {control.steer:5.2f}",
                f"Throttle: {control.throttle:5.2f}",
                f"Brake   : {control.brake:5.2f}",
                f"-------------------",
                f"Press 'R' for Reverse",
                f"Press 'D' for Drive"
            ]

            for i, line in enumerate(lines):
                text = font.render(line, True, color if "Gear" in line else (255, 255, 255))
                display.blit(text, (20, 20 + i * 30))

            pygame.display.flip()
            clock.tick(30) # 30 FPS

    except KeyboardInterrupt:
        pass
    finally:
        shared_state.running = False
        t_can.join()
        pygame.quit()
        print("종료되었습니다.")

if __name__ == '__main__':
    main()
