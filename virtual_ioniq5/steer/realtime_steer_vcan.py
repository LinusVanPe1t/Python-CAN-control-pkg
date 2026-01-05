import can
import struct
import time

def parse_steer_data():
    # 1. 가상 CAN 포트 설정
    channel = 'vcan0'
    
    try:
        bus = can.interface.Bus(channel=channel, interface='socketcan')
        print(f"{channel} 연결 성공!")
        print("   -> ID 0x104 (Steering Feedback) 데이터를 모니터링 중...")
    except OSError:
        print(f"오류: {channel} 포트를 찾을 수 없습니다.")
        return

    # 2. 해상도 설정 (시뮬레이터 기준 1/100)
    RESOLUTION = 1/100.0 

    try:
        for msg in bus:
            # 3. ID 필터링: 0x104 메시지만 처리
            if msg.arbitration_id == 0x104:
                
                # --- [수정 포인트 1] 데이터 길이 체크 ---
                # 시뮬레이터는 4바이트를 보내므로 4가 아니면 무시하도록 변경
                if msg.dlc != 4:
                    continue

                # --- [수정 포인트 2] 데이터 파싱 ---
                # 기존 '<ii' (정수 2개, 8바이트) -> '<i' (정수 1개, 4바이트)로 변경
                try:
                    # unpack 결과는 튜플이므로 [0]으로 첫 번째 값을 꺼냄
                    (raw_encoder_pos,) = struct.unpack('<i', msg.data)

                    # 5. 물리 값 변환
                    real_deg = raw_encoder_pos * RESOLUTION

                    # 6. 실시간 출력
                    print(f"\rCurrent Steer: {real_deg:6.1f} deg", end="")
                    
                except Exception as e:
                    print(f"\nParsing Error: {e}")

    except KeyboardInterrupt:
        print("\n\n프로그램을 종료합니다.")

if __name__ == "__main__":
    parse_steer_data()
