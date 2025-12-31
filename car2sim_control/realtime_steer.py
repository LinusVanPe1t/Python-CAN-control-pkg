import can
import struct
import time

def parse_steer_data():
    # 1. SocketCAN 인터페이스 설정 (일반적으로 'can0' 또는 'vcan0')
    # 실제 장비와 연결된 경우 'can0'을 사용하세요.
    channel = 'can0'
    
    try:
        bus = can.interface.Bus(channel=channel, bustype='socketcan')
        print(f"Checking CAN bus on {channel}...")
        print("Waiting for ID 0x104 (Steering Feedback)...")
    except OSError:
        print(f"Error: Could not find {channel}. 인터페이스가 활성화되었는지 확인해주세요.")
        return

    # 2. 해상도 설정 (문서 기준: v1=0.015, v2=1/60)
    # 문서에 v2가 1/60로 명시되어 있으므로 이를 기본으로 합니다.
    RESOLUTION = 1/60.0 

    try:
        for msg in bus:
            # 3. ID 필터링: 0x104 메시지만 처리
            if msg.arbitration_id == 0x104:
                
                # 데이터 길이 확인 (8바이트여야 함)
                if msg.dlc != 8:
                    continue

                # 4. 데이터 파싱 (Binary -> Integer)
                # 문서 내용:
                # Byte 0~3: encoder_pos (현재 제어 포지션)
                # Byte 4~7: servo_abs_pos (절대 위치 값)
                # 포맷: '<ii' (Little Endian, 4byte signed int 2개)
                
                try:
                    raw_data = struct.unpack('<ii', msg.data)
                    raw_encoder_pos = raw_data[0] # 앞쪽 4바이트
                    raw_abs_pos = raw_data[1]     # 뒤쪽 4바이트

                    # 5. 물리 값으로 변환 (값 * 해상도)
                    real_deg = raw_encoder_pos * RESOLUTION
                    abs_deg = raw_abs_pos * RESOLUTION

                    # 6. 실시간 출력
                    # \r을 사용하여 같은 줄에 계속 업데이트
                    print(f"\r[ID: 0x104] Current Steer: {real_deg:.2f}° | Absolute: {abs_deg:.2f}°", end="", flush=True)
                
                except struct.error:
                    print(f"\nData Unpack Error: {msg.data.hex()}")

    except KeyboardInterrupt:
        print("\n\n프로그램을 종료합니다.")
        bus.shutdown()

if __name__ == "__main__":
    parse_steer_data()
