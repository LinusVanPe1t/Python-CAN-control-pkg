import can
import struct
import time

def parse_brake_data():
    # 1. 가상 CAN 포트 설정
    channel = 'vcan0'
    
    try:
        # bustype='socketcan' 대신 interface='socketcan' 권장 (최신 python-can)
        bus = can.interface.Bus(channel=channel, interface='socketcan')
        print(f"{channel} 연결 성공!")
        print("   -> ID 0x204 (Brake Feedback) 데이터를 모니터링 중...")
    except OSError:
        print(f"오류: {channel} 포트를 찾을 수 없습니다.")
        return

    # 2. 해상도 설정 (브레이크는 1/100)
    # 문서 ADA-B.xlsx 기준
    RESOLUTION = 1/100.0 

    try:
        for msg in bus:
            # 3. ID 필터링: 0x204 (Brake Feedback) 메시지만 처리
            if msg.arbitration_id == 0x204:
                
                # --- [포인트 1] 데이터 길이 체크 ---
                # 브레이크 시뮬레이터와 정의서는 8바이트를 사용합니다.
                if msg.dlc != 8:
                    continue

                # --- [포인트 2] 데이터 파싱 ---
                try:
                    # 정의서상 Byte 0~3이 encoder_pos (Little Endian int)
                    # msg.data[:4]로 앞쪽 4바이트만 잘라서 해석합니다.
                    (raw_encoder_pos,) = struct.unpack('<i', msg.data[:4])

                    # 5. 물리 값 변환 (Pulse -> mm)
                    real_mm = raw_encoder_pos * RESOLUTION

                    # 6. 실시간 출력 (단위: mm)
                    print(f"\rCurrent Brake: {real_mm:6.1f} mm", end="")
                    
                except Exception as e:
                    print(f"\nParsing Error: {e}")

    except KeyboardInterrupt:
        print("\n\n프로그램을 종료합니다.")

if __name__ == "__main__":
    parse_brake_data()
