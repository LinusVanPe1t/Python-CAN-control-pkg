import can
import time
import threading

# ==========================================
# 사용자 설정
# ==========================================
CHANNEL = 'can0'
CYCLE_TIME = 0.01  # 10ms (100Hz) [cite: 12]

# 전역 변수 (쓰레드와 메인 루프가 공유)
# 초기 상태: 제어 ON(0x04), 목표값 0%
# ID 0x311(ADE_A_CMD)은 8바이트 데이터입니다.
current_target_data = bytearray([0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]) 
running = True

def percent_to_can_data(percent):
    """
    퍼센트(%)를 입력받아 ADE-A 제어용 8바이트 메시지로 변환
    공식: Value = Percent / 0.001526
    """
    try:
        # 1. 입력값 범위 제한 (0 ~ 100%)
        val_float = float(percent)
        val_float = max(0.0, min(100.0, val_float))
        
        # 2. Scale Factor 적용 (DBC 기준: 0.001526) 
        raw_value = int(val_float / 0.001526)
        
        # 3. 2바이트 Little Endian 변환
        val_low = raw_value & 0xFF
        val_high = (raw_value >> 8) & 0xFF
        
        # 4. 전체 8바이트 패킷 구성
        # Byte 0: 0x04 (Percent Control Enable) 
        # Byte 2~3: Target Value
        data = [0x04, 0x00, val_low, val_high, 0x00, 0x00, 0x00, 0x00]
        
        return bytearray(data)
        
    except Exception as e:
        print(f"변환 오류: {e}")
        return None

def send_cyclic_message():
    """
    백그라운드에서 10ms마다 계속 메시지를 쏘는 함수
    """
    global running, current_target_data
    
    # CAN 버스 연결
    try:
        bus = can.interface.Bus(channel=CHANNEL, bustype='socketcan')
        print(f"✅ {CHANNEL} 연결 성공! 주기적 전송 시작...")
    except OSError:
        print(f"❌ 오류: {CHANNEL} 인터페이스를 찾을 수 없습니다.")
        return

    # ADE-A Command ID: 0x311 (785) 
    msg = can.Message(arbitration_id=0x311, is_extended_id=False, dlc=8)

    while running:
        msg.data = current_target_data
        try:
            bus.send(msg)
        except can.CanError:
            print("CAN 전송 에러!")
        
        time.sleep(CYCLE_TIME) # 10ms 대기 [cite: 12]

# ==========================================
# 메인 실행부
# ==========================================
if __name__ == "__main__":
    # 1. 백그라운드 전송 쓰레드 시작
    t = threading.Thread(target=send_cyclic_message)
    t.daemon = True
    t.start()

    print("\n" + "="*40)
    print("🚀 ADE-A 가속 페달 제어 프로그램 시작")
    print("   - 초기 상태: 0% (제어 ON)")
    print("   - 종료 하려면 'q' 또는 Ctrl+C 입력")
    print("="*40 + "\n")

    # 2. 사용자 입력 루프
    try:
        while True:
            user_input = input("👉 원하는 가속도를 입력하세요 (%): ")
            
            if user_input.lower() == 'q':
                break
            
            # 데이터 변환 및 업데이트
            new_data = percent_to_can_data(user_input)
            
            if new_data:
                current_target_data = new_data
                # 사람이 보기 좋게 입력값 확인
                print(f"   --> 명령 변경 완료: {user_input}% (Hex: {new_data.hex()})")

    except KeyboardInterrupt:
        print("\n프로그램 종료 중...")
        
    finally:
        running = False
        t.join()
        
        # 안전 종료: 제어 해제 메시지 전송 (Bypass 모드)
        try:
            bus = can.interface.Bus(channel=CHANNEL, bustype='socketcan')
            # Byte 0을 0x00으로 설정하여 제어권 해제 [cite: 21]
            off_msg = can.Message(arbitration_id=0x311, data=[0]*8, is_extended_id=False)
            bus.send(off_msg)
            print("✅ 제어권 해제 신호 전송 완료 (Bypass Mode)")
        except:
            pass
            
        print("안전하게 종료되었습니다.")
