import tkinter as tk
import can
import threading
import time
import struct

# --- [설정] ADA-B (Brake) 통신 규격 ---
CAN_CHANNEL = 'vcan0'

# 수신 ID (제어기 -> 시뮬레이터)
ID_CONTROL_CMD = 0x200  # 제어 플래그 (SON: Servo On)
ID_TARGET_VAL  = 0x201  # 목표 스트로크 (mm)

# 송신 ID (시뮬레이터 -> 제어기)
ID_CURRENT_VAL = 0x204  # 현재 스트로크 (mm)
ID_DEVICE_STATE= 0x206  # 장치 상태 (3: Ready, 5: Control)

# 물리 계수 설정 (CSV 기준)
# Resolution: 1/100 (1mm = 100 Pulse)
MM_TO_PULSE = 100.0 

# 최대 브레이크 스트로크 제한 (단위: mm)
# 일반적인 BBW(Brake By Wire)의 유효 스트로크를 약 40~50mm로 가정
MAX_STROKE_LIMIT = 50.0 

class VirtualBrake(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Virtual ADA-B (Brake) Simulator")
        self.geometry("400x600")
        
        # --- 내부 변수 ---
        self.servo_on = False       # SON 상태
        self.current_mm = 0.0       # 현재 스트로크 (mm)
        self.target_mm = 0.0        # 목표 스트로크 (mm)
        self.fsm_state = 3          # 3: Ready(대기), 5: Control(제어)
        
        # --- UI 구성 ---
        self.setup_ui()
        
        # --- CAN 통신 설정 ---
        try:
            self.bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
            print(f"{CAN_CHANNEL} Connected (Brake System).")
        except Exception as e:
            self.lbl_status.config(text=f"CAN Error: {e}", fg="red")
            print("CAN Open Fail.")

        self.running = True
        
        # 쓰레드 시작
        threading.Thread(target=self.rx_thread, daemon=True).start()
        threading.Thread(target=self.tx_thread, daemon=True).start()
        self.update_gui_loop()

    def setup_ui(self):
        tk.Label(self, text="ADA-B Brake Simulator", font=("Arial", 16, "bold")).pack(pady=15)
        
        # 상태 표시창
        self.lbl_servo = tk.Label(self, text="SERVO: OFF (State: 3)", font=("Arial", 14), fg="gray", bg="#eee", width=30, height=2)
        self.lbl_servo.pack(pady=5)

        # 브레이크 게이지 (Canvas)
        self.canvas_height = 300
        self.canvas_width = 100
        self.canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white", highlightthickness=1, highlightbackground="#999")
        self.canvas.pack(pady=20)
        
        # 게이지 배경 (회색)
        self.bar_bg = self.canvas.create_rectangle(20, 20, 80, 280, outline="#333", width=2)
        # 게이지 바 (빨간색 - 초기값 0)
        self.bar_fill = self.canvas.create_rectangle(21, 279, 79, 279, fill="red", outline="")

        # 데이터 모니터링
        self.lbl_data = tk.Label(self, text="Target: 0.0 mm | Current: 0.0 mm", font=("Consolas", 12))
        self.lbl_data.pack(pady=10)
        
        self.lbl_status = tk.Label(self, text="Waiting for Command...", fg="blue")
        self.lbl_status.pack()

    def update_gauge(self, mm):
        # mm (0 ~ MAX)를 캔버스 높이 좌표로 변환
        # 높이: 20(Top) ~ 280(Bottom) -> 총 260 픽셀 사용
        ratio = mm / MAX_STROKE_LIMIT
        if ratio > 1.0: ratio = 1.0
        if ratio < 0.0: ratio = 0.0
        
        fill_height = int(ratio * 260)
        
        # Canvas 좌표: (x1, y1, x2, y2)
        # y1이 위쪽이므로, 280(Bottom)에서 fill_height만큼 뺀 값이 Top이 됨
        self.canvas.coords(self.bar_fill, 21, 280 - fill_height, 79, 279)

    def rx_thread(self):
        """ CAN 메시지 수신 (0x200, 0x201) """
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if not msg: continue
                
                # [ID 0x200] Control Command (SON 확인)
                if msg.arbitration_id == ID_CONTROL_CMD:
                    son_flag = msg.data[0] & 0x01
                    if son_flag == 1:
                        self.servo_on = True
                        self.fsm_state = 5
                    else:
                        self.servo_on = False
                        self.fsm_state = 3
                        
                # [ID 0x201] Target Stroke Command
                elif msg.arbitration_id == ID_TARGET_VAL:
                    if self.servo_on:
                        # 4바이트 Little Endian -> 정수 -> mm 변환
                        raw_val = int.from_bytes(msg.data[:4], byteorder='little', signed=True)
                        req_mm = raw_val / MM_TO_PULSE

                        # Limit Check
                        if req_mm > MAX_STROKE_LIMIT: req_mm = MAX_STROKE_LIMIT
                        if req_mm < 0: req_mm = 0.0
                        
                        self.target_mm = req_mm

            except Exception as e:
                print(f"RX Error: {e}")

    def tx_thread(self):
        """ 상태(0x206) 및 피드백(0x204) 전송 """
        while self.running:
            try:
                # 1. 현재 스트로크 (ID 0x204)
                curr_pulse = int(self.current_mm * MM_TO_PULSE)
                data_pos = curr_pulse.to_bytes(4, byteorder='little', signed=True)
                # 8바이트를 맞추기 위해 뒤에 4바이트 0 padding (servo_abs_pos 등)
                msg_pos = can.Message(arbitration_id=ID_CURRENT_VAL, data=data_pos + b'\x00\x00\x00\x00', is_extended_id=False)
                self.bus.send(msg_pos)
                
                # 2. 장치 상태 (ID 0x206)
                # CSV 정의서에 따라 fsm_state_id는 Byte 6에 위치
                data_state = bytes([0, 0, 0, 0, 0, 0, self.fsm_state, 0])
                msg_state = can.Message(arbitration_id=ID_DEVICE_STATE, data=data_state, is_extended_id=False)
                self.bus.send(msg_state)
                
                time.sleep(0.02) # 20ms 주기
                
            except Exception as e:
                print(f"TX Error: {e}")
            
    def update_gui_loop(self):
        # 물리 시뮬레이션 (부드러운 움직임)
        diff = self.target_mm - self.current_mm
        
        if self.servo_on:
            step = 0.5  # 브레이크는 반응이 빠름
            if abs(diff) > step:
                self.current_mm += step if diff > 0 else -step
            else:
                self.current_mm = self.target_mm
        else:
            # 서보 꺼지면 서서히 0으로 복귀 (리턴 스프링 효과)
            if self.current_mm > 0:
                self.current_mm -= 0.5
                if self.current_mm < 0: self.current_mm = 0

        self.update_gauge(self.current_mm)
        
        if self.servo_on:
            self.lbl_servo.config(text="SERVO: ON (State: 5)", fg="white", bg="red")
        else:
            self.lbl_servo.config(text="SERVO: OFF (State: 3)", fg="gray", bg="#eee")
            
        self.lbl_data.config(text=f"Target: {self.target_mm:.1f} mm | Current: {self.current_mm:.1f} mm")
        self.after(20, self.update_gui_loop)

if __name__ == "__main__":
    app = VirtualBrake()
    app.mainloop()
