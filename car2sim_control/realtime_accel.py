import can
import struct
import time

def parse_accel_data():
    # 1. 설정
    channel = 'can0'  # 윈도우의 경우 'Vector' 등 환경에 맞게 변경
    
    try:
        # ADE-A 통신 속도: 500kbps
        bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=500000)
        print(f"Checking CAN bus on {channel}...")
        print("Waiting for ID 0x314 (Accel Data)...")
    except OSError:
        print(f"Error: {channel} 인터페이스를 찾을 수 없습니다. (sudo ip link set up can0)")
        return

    # 2. DBC 파일 기반 정확한 변환 계수
    # 엑셀 문서에는 0.000076으로 적혀있을 수 있으나, DBC 파일 기준 0.001526이 정확합니다.
    SCALE_PCT = 0.001526  # Raw 65535 = 100%
    SCALE_VOLT = 0.000076 # Raw 65535 = 5V (참고용)

    try:
        for msg in bus:
            # 3. ID 필터링: 0x314 (Main State)
            if msg.arbitration_id == 0x314 and msg.dlc == 8:
                
                # 데이터 파싱 (Little Endian)
                # Byte 0-1: BRK_S (브레이크 센서 값도 일부 포함됨)
                # Byte 2-3: Flags (오버라이드 여부 등)
                # Byte 4-5: APS_IN (액셀 입력)
                # Byte 6-7: APS_OUT (액셀 출력)
                
                try:
                    # 포맷: '<H' (ushort, 2byte), 'x' (pad byte), 'H', 'H'
                    # 구조: [BRK(2)] [Flags(2)] [APS_IN(2)] [APS_OUT(2)]
                    # 중간에 Flag 2바이트를 건너뛰거나(xx) 별도로 읽을 수 있음
                    
                    # 전체를 16비트 단위로 쪼개기
                    raw_data = struct.unpack('<HHHH', msg.data)
                    
                    raw_aps_in = raw_data[2]  # 3번째 워드 (Byte 4-5)
                    raw_aps_out = raw_data[3] # 4번째 워드 (Byte 6-7)
                    
                    # 물리 값 변환
                    real_aps_in = raw_aps_in * SCALE_PCT
                    real_aps_out = raw_aps_out * SCALE_PCT

                    # 출력 (밟을 때 값이 올라가는지 확인)
                    print(f"\r[ID: 0x314] Accel Input: {real_aps_in:6.2f} % | Output: {real_aps_out:6.2f} %", end="", flush=True)

                except struct.error:
                    pass

            # (옵션) 전압까지 보고 싶다면 0x315도 체크 가능
            # elif msg.arbitration_id == 0x315: ...

    except KeyboardInterrupt:
        print("\n\n종료합니다.")
        bus.shutdown()

if __name__ == "__main__":
    parse_accel_data()
