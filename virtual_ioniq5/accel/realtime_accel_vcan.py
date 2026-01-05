import can
import struct
import time

def parse_accel_data():
    # 1. 가상 CAN 포트 설정
    channel = 'vcan0'
    
    try:
        bus = can.interface.Bus(channel=channel, interface='socketcan')
        print(f"{channel} 연결 성공!")
        print("   -> ID 0x315 (Accel Feedback) 데이터를 모니터링 중...")
    except OSError:
        print(f"오류: {channel} 포트를 찾을 수 없습니다.")
        return

    # 2. 해상도 설정 (Raw값 -> 퍼센트 변환)
    # 0 ~ 65535(Max) 값을 0 ~ 100(%)으로 변환
    # 계산: 100 / 65535 = 0.0015259
    RESOLUTION_PCT = 0.0015259

    try:
        for msg in bus:
            # 3. ID 필터링: 0x315 (액셀 피드백)
            if msg.arbitration_id == 0x315:
                
                # 시뮬레이터는 8바이트를 보냅니다
                if msg.dlc != 8:
                    continue

                try:
                    # --- [수정 포인트] ---
                    # 1. 2바이트(H) 양수만 읽음 (액셀 데이터 구조)
                    # 2. msg.data[:2] 사용
                    (raw_val,) = struct.unpack('<H', msg.data[:2])

                    # 4. 물리 값 변환 (% 단위)
                    real_pct = raw_val * RESOLUTION_PCT
                    
                    # 5. 실시간 출력
                    print(f"\rCurrent Accel: {real_pct:6.1f} %", end="")
                    
                except Exception as e:
                    print(f"\nParsing Error: {e}")

    except KeyboardInterrupt:
        print("\n\n프로그램을 종료합니다.")

if __name__ == "__main__":
    parse_accel_data()
