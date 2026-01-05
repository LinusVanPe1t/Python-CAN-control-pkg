import tkinter as tk
import math
import can
import threading
import time
import struct

# --- [설정] ADA-S 통신 규격 ---
CAN_CHANNEL = 'vcan0'

# 수신 ID
ID_CONTROL_CMD = 0x100  # 제어 플래그
ID_TARGET_POS  = 0x101  # 목표 각도

# 송신 ID
ID_CURRENT_POS = 0x104  # 현재 핸들 각도
ID_DEVICE_STATE= 0x106  # 장치 상태

# 물리 계수
DEG_TO_PULSE = 100.0 
MAX_ANGLE_LIMIT = 450.0 

class VirtualADAS(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Virtual ADA-S Simulator (IONIQ 5) - Spec Fixed")
        self.geometry("500x550")
        
        self.servo_on = False       
        self.current_angle = 0.0    
        self.target_angle = 0.0     
        self.fsm_state = 3          # Default: 3 (Ready)
        
        self.setup_ui()
        
        try:
            self.bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
            print(f"{CAN_CHANNEL} Connected.")
        except Exception as e:
            self.lbl_status.config(text=f"CAN Error: {e}", fg="red")

        self.running = True
        threading.Thread(target=self.rx_thread, daemon=True).start()
        threading.Thread(target=self.tx_thread, daemon=True).start()
        self.update_gui_loop()

    def setup_ui(self):
        tk.Label(self, text="ADA-S Steering Simulator", font=("Arial", 16, "bold")).pack(pady=10)
        self.lbl_servo = tk.Label(self, text="SERVO: OFF (State: 3)", font=("Arial", 14), fg="gray", bg="#eee", width=30, height=2)
        self.lbl_servo.pack(pady=5)

        self.canvas = tk.Canvas(self, width=300, height=300, bg="white", highlightthickness=0)
        self.canvas.pack(pady=10)
        self.draw_handle(0)

        self.lbl_data = tk.Label(self, text="Target: 0.0 | Current: 0.0", font=("Consolas", 12))
        self.lbl_data.pack(pady=10)
        self.lbl_status = tk.Label(self, text="Waiting for Command...", fg="blue")
        self.lbl_status.pack()

    def draw_handle(self, angle):
        self.canvas.delete("all")
        cx, cy, r = 150, 150, 100
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, width=8, outline="#333")
        rad = math.radians(angle - 90) 
        x_end = cx + r * math.cos(rad)
        y_end = cy + r * math.sin(rad)
        self.canvas.create_line(cx, cy, x_end, y_end, width=8, fill="red" if self.servo_on else "gray")
        self.canvas.create_oval(cx-20, cy-20, cx+20, cy+20, fill="#333")
        self.canvas.create_text(cx, cy, text="H", fill="white", font=("Arial", 12, "bold"))

    def rx_thread(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if not msg: continue
                
                if msg.arbitration_id == ID_CONTROL_CMD:
                    son_flag = msg.data[0] & 0x01 
                    if son_flag == 1:
                        self.servo_on = True
                        self.fsm_state = 5 # Control Mode
                    else:
                        self.servo_on = False
                        self.fsm_state = 3 # Ready Mode
                        
                elif msg.arbitration_id == ID_TARGET_POS:
                    if self.servo_on: 
                        raw_val = int.from_bytes(msg.data[:4], byteorder='little', signed=True)
                        requested_angle = raw_val / DEG_TO_PULSE

                        if requested_angle > MAX_ANGLE_LIMIT:
                            requested_angle = MAX_ANGLE_LIMIT
                        elif requested_angle < -MAX_ANGLE_LIMIT:
                            requested_angle = -MAX_ANGLE_LIMIT
                        
                        self.target_angle = requested_angle

            except Exception as e:
                print(f"RX Error: {e}")

    def tx_thread(self):
        while self.running:
            try:
                # 1. 현재 각도 전송 (ID 0x104)
                current_pulse = int(self.current_angle * DEG_TO_PULSE)
                data_pos = current_pulse.to_bytes(4, byteorder='little', signed=True)
                msg_pos = can.Message(arbitration_id=ID_CURRENT_POS, data=data_pos, is_extended_id=False)
                self.bus.send(msg_pos)
                
                # 2. 장치 상태 전송 (ID 0x106)
                # [수정됨] 정의서에 따라 fsm_state_id를 Byte 6(인덱스 6)에 배치
                # 구조: [Error, ErrorCnt, CANErr, RecvCnt, Timeout, LoopCnt, FSM_STATE, Reserved]
                # 나머지는 0으로 채움
                data_state = bytes([0, 0, 0, 0, 0, 0, self.fsm_state, 0])
                
                msg_state = can.Message(arbitration_id=ID_DEVICE_STATE, data=data_state, is_extended_id=False)
                self.bus.send(msg_state)
                
                time.sleep(0.02)
                
            except Exception as e:
                print(f"TX Error: {e}")
            
    def update_gui_loop(self):
        diff = self.target_angle - self.current_angle
        if self.servo_on:
            step = 2.0
            if abs(diff) > step:
                self.current_angle += step if diff > 0 else -step
            else:
                self.current_angle = self.target_angle
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
