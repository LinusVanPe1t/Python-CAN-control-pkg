import can
import time
import threading
import struct

# ==========================================
# 사용자 설정 (속도 조절은 여기서!)
# ==========================================
CHANNEL = 'can0'
BITRATE = 500000
CYCLE_TIME = 0.01  # 10ms 주기 (변경 금지)

# ⭐ 속도 설정 (도/초)
# 예: 30이면 1초에 30도 돌아가는 속도
# 숫자가 작을수록 느리게, 클수록 빠르게 움직입니다.
TARGET_SPEED = 100.0 

# ==========================================
# 전역 변수
# ==========================================
current_sending_angle = 0.0  # 현재 CAN으로 보내고 있는 각도
user_target_angle = 0.0      # 사용자가 입력한 최종 목표 각도
running = True

def degrees_to_can_data(degree):
    """
    각도(float)를 입력받아 CAN 메시지용 4바이트(Little Endian)로 변환
    """
    try:
        pulse = int(float(degree) * 60)
        pulse = max(min(pulse, 32400), -32400) # 안전 범위 제한
        return pulse.to_bytes(4, byteorder='little', signed=True)
    except Exception as e:
        print(f"변환 오류: {e}")
        return bytearray([0x00, 0x00, 0x00, 0x00])

def send_cyclic_message():
    """
    백그라운드에서 10ms마다 계산된 각도를 쏘는 함수 (램핑 로직 포함)
    """
    global running, current_sending_angle, user_target_angle
    
    # CAN 버스 연결
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface='socketcan')
        print(f"✅ {CHANNEL} 연결 성공! 부드러운 제어 모드 시작...")
    except OSError:
        print(f"❌ 오류: {CHANNEL} 인터페이스를 찾을 수 없습니다.")
        return

    msg = can.Message(arbitration_id=0x101, is_extended_id=False, dlc=4)
    
    # 10ms(0.01초) 동안 이동할 수 있는 최대 각도 계산
    step_per_tick = TARGET_SPEED * CYCLE_TIME 

    while running:
        # --- [핵심: 속도 제어 로직] ---
        diff = user_target_angle - current_sending_angle
        
        # 목표와의 차이가 스텝보다 작으면 바로 목표값으로 고정 (도착)
        if abs(diff) <= step_per_tick:
            current_sending_angle = user_target_angle
        # 목표가 더 크면 스텝만큼 증가
        elif diff > 0:
            current_sending_angle += step_per_tick
        # 목표가 더 작으면 스텝만큼 감소
        else:
            current_sending_angle -= step_per_tick
        # ---------------------------

        # 계산된 현재 각도를 CAN 데이터로 변환하여 전송
        msg.data = degrees_to_can_data(current_sending_angle)
        
        try:
            bus.send(msg)
        except can.CanError:
            print("CAN 전송 에러!")
        
        time.sleep(CYCLE_TIME)

# ==========================================
# 메인 실행부
# ==========================================
if __name__ == "__main__":
    t = threading.Thread(target=send_cyclic_message)
    t.daemon = True
    t.start()

    print("\n" + "="*40)
    print("🚗 부드러운 스티어링 제어 프로그램")
    print(f"   - 설정 속도: {TARGET_SPEED} deg/sec")
    print("   - 초기 위치: 0도")
    print("   - 종료 하려면 'q' 입력")
    print("="*40 + "\n")

    try:
        while True:
            user_input = input("👉 목표 각도를 입력하세요 (도): ")
            
            if user_input.lower() == 'q':
                break
            
            try:
                # 사용자는 목표만 던져주고, 이동은 쓰레드가 알아서 함
                target = float(user_input)
                user_target_angle = target
                print(f"   --> 목표 설정 완료: {target}도로 이동 시작...")
            except ValueError:
                print("⚠️ 숫자를 입력해주세요.")

    except KeyboardInterrupt:
        print("\n프로그램 종료 중...")
    finally:
        # 안전 종료 절차: 0도로 천천히 복귀 후 종료하려면 아래 주석 해제
        # user_target_angle = 0
        # time.sleep(2) 
        
        running = False
        t.join()
        print("안전하게 종료되었습니다.")
