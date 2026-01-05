import tkinter as tk
import math
import can
import threading
import time
import struct

# --- [설정] ADA-S 통신 규격 ---
CAN_CHANNEL = 'vcan0'

# 수신 ID (Students -> GUI)
ID_CONTROL_CMD = 0x100  # 제어 플래그 (SON: Servo On)
ID_TARGET_POS  = 0x101  # 목표 각도 명령

# 송신 ID (GUI -> Students)
ID_CURRENT_POS = 0x104  # 현재 핸들 각도
ID_DEVICE_STATE= 0x106  # 장치 상태 (3: Ready, 5: Servo On)

# 물리 계수 설정
DEG_TO_PULSE = 100.0 

# [추가됨] 최대 조향 각도 제한 (단위: 도)
# 아이오닉5 실제 조향각과 비슷하게 +/- 540도(1.5바퀴)로 설정
MAX_ANGLE_LIMIT = 450.0 

class VirtualADAS(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Virtual ADA-S Simulator (IONIQ 5) - Limited")
        self.geometry("500x550")
        
        # --- 내부 변수 ---
        self.servo_on = False       # SON (Servo On) 상태
        self.current_angle = 0.0    # 현재 각도 (Degree)
        self.target_angle = 0.0     # 목표 각도 (Degree)
        self.fsm_state = 3          # 3: Ready(대기), 5: Control(제어 중)
        
        # --- UI 구성 ---
        self.setup_ui()
        
        # --- CAN 통신 설정 ---
        try:
            self.bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
            print(f"✅ {CAN_CHANNEL} Connected.")
        except Exception as e:
            self.lbl_status.config(text=f"CAN Error: {e}", fg="red")
            print("❌ CAN Open Fail. (Did you run 'sudo ip link add...'?)")

        # --- 쓰레드 시작 ---
        self.running = True
        
        # 1. 수신 쓰레드 (학생들의 명령을 받음)
        threading.Thread(target=self.rx_thread, daemon=True).start()
        
        # 2. 송신 쓰레드 (차량 상태를 학생들에게 쏘아줌 - 10ms 주기)
        threading.Thread(target=self.tx_thread, daemon=True).start()
        
        # 3. GUI 애니메이션 업데이트
        self.update_gui_loop()

    def setup_ui(self):
        # 헤더
        tk.Label(self, text="ADA-S Steering Simulator", font=("Arial", 16, "bold")).pack(pady=10)
        
        # 상태 표시창 (Ready / Run)
        self.lbl_servo = tk.Label(self, text="SERVO: OFF (State: 3)", font=("Arial", 14), fg="gray", bg="#eee", width=30, height=2)
        self.lbl_servo.pack(pady=5)

        # 핸들 캔버스
        self.canvas = tk.Canvas(self, width=300, height=300, bg="white", highlightthickness=0)
        self.canvas.pack(pady=10)
        self.draw_handle(0)

        # 데이터 모니터링
        self.lbl_data = tk.Label(self, text="Target: 0.0 | Current: 0.0", font=("Consolas", 12))
        self.lbl_data.pack(pady=10)
        
        self.lbl_status = tk.Label(self, text="Waiting for Command...", fg="blue")
        self.lbl_status.pack()

    def draw_handle(self, angle):
        self.canvas.delete("all")
        cx, cy, r = 150, 150, 100
        
        # 외곽원
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, width=8, outline="#333")
        
        # 회전 적용
        rad = math.radians(angle - 90) # -90은 12시 방향 보정
        x_end = cx + r * math.cos(rad)
        y_end = cy + r * math.sin(rad)
        
        # 핸들 살 (빨간색 = 현재 방향)
        self.canvas.create_line(cx, cy, x_end, y_end, width=8, fill="red" if self.servo_on else "gray")
        
        # 센터 로고
        self.canvas.create_oval(cx-20, cy-20, cx+20, cy+20, fill="#333")
        self.canvas.create_text(cx, cy, text="H", fill="white", font=("Arial", 12, "bold"))

    def rx_thread(self):
        """ 학생들의 CAN 메시지를 수신 (CMD_0x100, CMD_0x101) """
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if not msg: continue
                
                # [로직 1] ID 0x100 (Control Flags) 처리
                if msg.arbitration_id == ID_CONTROL_CMD:
                    son_flag = msg.data[0] & 0x01 # 0번 비트 마스킹
                    
                    if son_flag == 1:
                        self.servo_on = True
                        self.fsm_state = 5 # Control Mode
                    else:
                        self.servo_on = False
                        self.fsm_state = 3 # Ready Mode
                        
                # [로직 2] ID 0x101 (Target Position) 처리
                elif msg.arbitration_id == ID_TARGET_POS:
                    if self.servo_on: 
                        raw_val = int.from_bytes(msg.data[:4], byteorder='little', signed=True)
                        requested_angle = raw_val / DEG_TO_PULSE

                        # --- [추가됨] 각도 제한 로직 ---
                        # 입력값이 MAX보다 크면 MAX로, -MAX보다 작으면 -MAX로 고정
                        if requested_angle > MAX_ANGLE_LIMIT:
                            requested_angle = MAX_ANGLE_LIMIT
                            print(f"⚠️ Warning: Target {raw_val} exceeds limit! Clamped to {MAX_ANGLE_LIMIT}")
                        elif requested_angle < -MAX_ANGLE_LIMIT:
                            requested_angle = -MAX_ANGLE_LIMIT
                            print(f"⚠️ Warning: Target {raw_val} exceeds limit! Clamped to {-MAX_ANGLE_LIMIT}")
                        
                        self.target_angle = requested_angle
                    else:
                        pass 

            except Exception as e:
                print(f"RX Error: {e}")

    def tx_thread(self):
        """ 차량 상태(0x106)와 현재 각도(0x104)를 학생들에게 전송 """
        while self.running:
            try:
                # 1. 현재 각도 전송 (ID 0x104)
                current_pulse = int(self.current_angle * DEG_TO_PULSE)
                data_pos = current_pulse.to_bytes(4, byteorder='little', signed=True)
                msg_pos = can.Message(arbitration_id=ID_CURRENT_POS, data=data_pos, is_extended_id=False)
                self.bus.send(msg_pos)
                
                # 2. 장치 상태 전송 (ID 0x106)
                data_state = bytes([self.fsm_state, 0, 0, 0, 0, 0, 0, 0])
                msg_state = can.Message(arbitration_id=ID_DEVICE_STATE, data=data_state, is_extended_id=False)
                self.bus.send(msg_state)
                
                time.sleep(0.02)
                
            except Exception as e:
                print(f"TX Error: {e}")
            
    def update_gui_loop(self):
        """ 물리 엔진 시뮬레이션 """
        diff = self.target_angle - self.current_angle
        
        if self.servo_on:
            step = 2.0
            if abs(diff) > step:
                self.current_angle += step if diff > 0 else -step
            else:
                self.current_angle = self.target_angle
        else:
            pass 

        self.draw_handle(self.current_angle)
        
        if self.servo_on:
            self.lbl_servo.config(text="SERVO: ON (State: 5)", fg="white", bg="green")
        else:
            self.lbl_servo.config(text="SERVO: OFF (State: 3)", fg="gray", bg="#eee")
            
        self.lbl_data.config(text=f"Target: {self.target_angle:.1f}° | Current: {self.current_angle:.1f}°")
        
        self.after(50, self.update_gui_loop)

if __name__ == "__main__":
    app = VirtualADAS()
    app.mainloop()