import can
import time
import threading
import struct

# ==========================================
# 사용자 설정 (수정됨)
# ==========================================
CHANNEL = 'vcan0'   # <-- [변경 1] vcan0로 변경
BITRATE = 500000    # 가상 포트라 무시되지만 유지
CYCLE_TIME = 0.01

current_target_data = bytearray([0x00, 0x00, 0x00, 0x00])
running = True

def degrees_to_can_data(degree):
    """
    각도(도)를 입력받아 CAN 메시지용 4바이트(Little Endian)로 변환
    공식: Pulse = Angle * 100 (ADA-S 기준)
    """
    try:
        # [변경 2] 1도당 100 펄스로 변경 (ADA-S 사양에 맞춤)
        # 만약 시뮬레이터가 60 기준이면 60으로 유지하세요.
        pulse = int(float(degree) * 100)
        
        # 안전 범위 제한
        pulse = max(min(pulse, 50000), -50000)
        
        return pulse.to_bytes(4, byteorder='little', signed=True)
    except Exception as e:
        print(f"변환 오류: {e}")
        return None

def send_cyclic_message():
    # vcan0 연결
    try:
        bus = can.interface.Bus(channel=CHANNEL, bustype='socketcan')
        print(f"✅ {CHANNEL} (가상 차량) 연결 성공! 전송 시작...")
    except OSError:
        print(f"❌ 오류: {CHANNEL} 포트가 없습니다. (sudo ip link add... 했나요?)")
        return

    msg = can.Message(arbitration_id=0x101, is_extended_id=False, dlc=4)

    while running:
        msg.data = current_target_data
        try:
            bus.send(msg)
        except can.CanError:
            print("CAN 전송 에러!")
        
        time.sleep(CYCLE_TIME)

if __name__ == "__main__":
    t = threading.Thread(target=send_cyclic_message)
    t.daemon = True
    t.start()

    print("\n" + "="*40)
    print("🚗 [가상] 스티어링 제어 프로그램")
    print("   - 주의: 먼저 다른 터미널에서 시동(0x100)을 걸어야 합니다!")
    print("   - 명령어: cansend vcan0 100#E701FF")
    print("="*40 + "\n")

    try:
        while True:
            user_input = input("👉 목표 각도 입력 (종료: q): ")
            if user_input.lower() == 'q':
                break
            
            data = degrees_to_can_data(user_input)
            if data:
                current_target_data = data
                print(f"   --> 전송 데이터(Hex): {data.hex().upper()}")
                
    except KeyboardInterrupt:
        pass
    
    running = False
    print("\n종료합니다.")
