import can
import time
import threading
import struct

# ==========================================
# 사용자 설정
# ==========================================
CHANNEL = 'can0'
BITRATE = 500000
CYCLE_TIME = 0.01  # 10ms 주기 [cite: 81]

# 전역 변수 (쓰레드와 메인 루프가 공유)
# 0x200 (Enable) 초기값: 서보 OFF (0x00, 0x00, 0x00)
current_cmd_data = bytearray([0x00, 0x00, 0x00]) 
# 0x201 (Target) 초기값: 0mm
current_pos_data = bytearray([0x00, 0x00, 0x00, 0x00]) 

running = True
is_servo_on = False # 현재 서보 상태 추적용

def mm_to_can_data(mm):
    """
    거리(mm)를 입력받아 CAN 메시지용 4바이트(Little Endian)로 변환
    공식: Value = mm * 100 
    """
    try:
        val = float(mm)
        
        # 1. 분해능 적용 (1mm = 100)
        pulse = int(val * 100)
        
        # 2. 범위 제한 (0mm ~ 60mm) 
        # ADA-B 기본 Max Limit은 6000(60mm)입니다.
        pulse = max(min(pulse, 6000), 0)
        
        # 3. 4바이트 Little Endian, Signed Integer 변환
        return pulse.to_bytes(4, byteorder='little', signed=True)
    except Exception as e:
        print(f"변환 오류: {e}")
        return None

def set_servo_state(enable):
    """
    [cite_start]ON : 0xE7 01 FF (SON=1, PC=1, TL=1, EMG=1, LSP=1, LSN=1, LOP=1, TLA=Max) [cite: 89]
    OFF: 0xE6 01 FF (SON=0, 나머지는 ON 상태와 똑같이 유지해야 함)
    """
    if enable:
        # ON: 1110 0111 (E7)
        return bytearray([0xE7, 0x01, 0xFF])
    else:
        # OFF: 1110 0110 (E6) 
        # SON만 끄고, PC/TL/안전센서/토크리미트(FF)는 그대로 살려둡니다.
        return bytearray([0xE6, 0x01, 0xFF])

def send_cyclic_message():
    """
    백그라운드에서 10ms마다 계속 메시지를 쏘는 함수
    브레이크는 0x200(상태)과 0x201(위치) 두 개를 계속 보내야 함 
    """
    global running, current_cmd_data, current_pos_data
    
    # CAN 버스 연결
    try:
        bus = can.interface.Bus(channel=CHANNEL, bustype='socketcan', bitrate=BITRATE)
        print(f"✅ {CHANNEL} 연결 성공! 주기적 전송 시작...")
    except OSError:
        print(f"❌ 오류: {CHANNEL} 인터페이스를 찾을 수 없습니다.")
        return

    # 메시지 객체 생성 (ID 미리 지정)
    msg_cmd = can.Message(arbitration_id=0x200, is_extended_id=False, dlc=3)
    msg_pos = can.Message(arbitration_id=0x201, is_extended_id=False, dlc=4)

    while running:
        # 현재 설정된 데이터를 메시지에 담기
        msg_cmd.data = current_cmd_data
        msg_pos.data = current_pos_data
        
        try:
            # 두 메시지 모두 전송 (순서는 크게 상관없으나 둘 다 10ms 안에 나가야 함)
            bus.send(msg_cmd)
            bus.send(msg_pos)
        except can.CanError:
            print("CAN 전송 에러!")
        
        time.sleep(CYCLE_TIME) # 10ms 대기

# ==========================================
# 메인 실행부
# ==========================================
if __name__ == "__main__":
    # 1. 백그라운드 전송 쓰레드 시작
    t = threading.Thread(target=send_cyclic_message)
    t.daemon = True
    t.start()

    print("\n" + "="*40)
    print("🛑 ADA-B 브레이크 제어 프로그램 시작")
    print("   - 초기 상태: 서보 OFF / 위치 0mm")
    print("   - 명령어:")
    print("     'on'  : 서보 켜기 (필수)")
    print("     'off' : 서보 끄기")
    print("     숫자  : 해당 깊이(mm)로 제어 (예: 15.5)")
    print("     'q'   : 종료")
    print("="*40 + "\n")

    # 2. 사용자 입력 루프
    try:
        while True:
            user_input = input("👉 명령 입력 (on/off/mm): ").strip().lower()
            
            if user_input == 'q':
                break
            
            elif user_input == 'on':
                current_cmd_data = set_servo_state(True)
                is_servo_on = True
                print("   --> 서보 ON 명령 설정 (0xE701FF)")
                
            elif user_input == 'off':
                current_cmd_data = set_servo_state(False)
                is_servo_on = False
                print("   --> 서보 OFF 명령 설정 (0x000000)")
                
            else:
                # 숫자인 경우 위치 제어로 판단
                if not is_servo_on:
                    print("   !! 경고: 서보가 꺼져 있습니다. 'on'을 먼저 입력하세요.")
                
                new_data = mm_to_can_data(user_input)
                
                if new_data:
                    current_pos_data = new_data
                    print(f"   --> 목표 위치 변경: {user_input}mm (Hex: {new_data.hex()})")

    except KeyboardInterrupt:
        print("\n프로그램 종료 중...")
    finally:
        # 종료 전 안전하게 0으로 복귀 및 서보 끄기
        current_pos_data = mm_to_can_data(0)
        time.sleep(0.5)
        current_cmd_data = set_servo_state(False)
        time.sleep(0.1)
        
        running = False
        t.join()
        print("안전하게 종료되었습니다.")
