import can
import struct

def parse_human_brake_inverted():
    # 1. 설정
    channel = 'can0'
    
    try:
        bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=500000)
        print(f"Checking CAN bus on {channel}...")
        print("Waiting for Human Brake Signal (Inverted Mode)...")
    except OSError:
        print(f"Error: {channel} 인터페이스를 확인해주세요.")
        return

    # DBC 파일 기준 변환 계수
    SCALE_PCT = 0.001526

    # 최대값 기준 (안 밟았을 때 나오는 값)
    # 99.xx가 나온다면 안전하게 100을 기준으로 잡습니다.
    MAX_PCT_REF = 100.0 

    try:
        for msg in bus:
            # ID 0x314: ADE-A (액셀/브레이크 센서)
            if msg.arbitration_id == 0x314 and msg.dlc == 8:
                try:
                    raw_data = struct.unpack('<HHHH', msg.data)
                    
                    # Byte 0-1: BRK_S (Raw 값)
                    raw_brake = raw_data[0]
                    
                    # 1. 원래 퍼센트로 변환 (99% -> 0%로 줄어드는 값)
                    original_pct = raw_brake * SCALE_PCT

                    # 2. 값 반전 (100 - 원래값)
                    # 만약 99.5가 나오면 -> 0.5% (떼고 있음)
                    # 만약 10.0이 나오면 -> 90.0% (밟고 있음)
                    inverted_pct = MAX_PCT_REF - original_pct
                    
                    # (선택) 0보다 작거나 100보다 큰 노이즈 값 보정
                    if inverted_pct < 0: inverted_pct = 0.0
                    if inverted_pct > 100: inverted_pct = 100.0

                    # 출력
                    print(f"\r[ID: 0x314] Brake Pedal: {inverted_pct:6.2f} % (Raw converted: {original_pct:.1f})", end="", flush=True)

                except struct.error:
                    pass

    except KeyboardInterrupt:
        print("\n종료합니다.")
        bus.shutdown()

if __name__ == "__main__":
    parse_human_brake_inverted()
