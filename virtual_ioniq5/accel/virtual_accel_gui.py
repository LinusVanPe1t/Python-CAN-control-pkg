import tkinter as tk
import can
import threading
import time
import struct

# --- [설정] ADE-A (Accel) 통신 규격 ---
CAN_CHANNEL = 'vcan0'

# ID 정의
ID_CMD = 0x311  # 제어 명령 (Host -> ADE-A)
ID_FB  = 0x315  # 상태 피드백 (ADE-A -> Host)

# 변환 계수 (CSV 정의 기반)
# Percent: 16bit (0~65535) -> 0~100%
# Factor: 0.00152588 (100 / 65535)
SCALE_PCT = 0.0015259 

# Voltage: 16bit (0~65535) -> 0~5V
# Factor: 0.00007629 (5 / 65535)
SCALE_VOLT = 0.0000763

class VirtualAccel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Virtual ADE-A (Accel) Simulator")
        self.geometry("400x600")
        
        # --- 내부 변수 ---
        self.is_active = False      # OVR_PERCENT 모드 활성화 여부
        self.current_pct = 0.0      # 현재 밟은 양 (%)
        self.target_pct = 0.0       # 목표 양 (%)
        
        # --- UI 구성 ---
        self.setup_ui()
        
        # --- CAN 통신 설정 ---
        try:
            self.bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
            print(f"✅ {CAN_CHANNEL} Connected (ADE-A System).")
        except Exception as e:
            self.lbl_status.config(text=f"CAN Error: {e}", fg="red")
            print("❌ CAN Open Fail.")

        self.running = True
        
        # 쓰레드 시작
        threading.Thread(target=self.rx_thread, daemon=True).start()
        threading.Thread(target=self.tx_thread, daemon=True).start()
        self.update_gui_loop()

    def setup_ui(self):
        tk.Label(self, text="ADE-A Accelerator Simulator", font=("Arial", 16, "bold")).pack(pady=15)
        
        # 상태 표시창
        self.lbl_mode = tk.Label(self, text="MODE: Manual (Default)", font=("Arial", 14), fg="gray", bg="#eee", width=30, height=2)
        self.lbl_mode.pack(pady=5)

        # 액셀 페달 게이지 (Canvas)
        self.canvas_height = 300
        self.canvas_width = 100
        self.canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white", highlightthickness=1, highlightbackground="#999")
        self.canvas.pack(pady=20)
        
        # 게이지 배경
        self.bar_bg = self.canvas.create_rectangle(20, 20, 80, 280, outline="#333", width=2)
        # 게이지 바 (초록색 - 초기값 0)
        self.bar_fill = self.canvas.create_rectangle(21, 279, 79, 279, fill="#00CC00", outline="")

        # 데이터 모니터링
        self.lbl_data = tk.Label(self, text="Target: 0.0 % | Output: 0.00 V", font=("Consolas", 12))
        self.lbl_data.pack(pady=10)
        
        self.lbl_status = tk.Label(self, text="Waiting for Command...", fg="blue")
        self.lbl_status.pack()

    def update_gauge(self, pct):
        # pct (0 ~ 100)를 좌표로 변환
        ratio = pct / 100.0
        if ratio > 1.0: ratio = 1.0
        if ratio < 0.0: ratio = 0.0
        
        fill_height = int(ratio * 260)
        self.canvas.coords(self.bar_fill, 21, 280 - fill_height, 79, 279)

    def rx_thread(self):
        """ CAN 메시지 수신 (0x311) """
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if not msg: continue
                
                # [ID 0x311] Command
                if msg.arbitration_id == ID_CMD:
                    # Byte 0 Parsing
                    # Bit 2: OVR_PERCENT (0x04)
                    ovr_percent = (msg.data[0] & 0x04) >> 2
                    
                    if ovr_percent == 1:
                        self.is_active = True
                        
                        # Byte 2~3: Target Percent (Little Endian)
                        raw_val = int.from_bytes(msg.data[2:4], byteorder='little')
                        self.target_pct = raw_val * SCALE_PCT
                    else:
                        self.is_active = False
                        # 비활성화 시 페달 0으로 복귀
                        self.target_pct = 0.0

            except Exception as e:
                print(f"RX Error: {e}")

    def tx_thread(self):
        """ 피드백 전송 (0x315) """
        while self.running:
            try:
                # 현재 %를 전압(V)으로 변환 (0% = 0V, 100% = 5V 가정)
                current_volt = (self.current_pct / 100.0) * 5.0
                
                # Voltage -> Raw Value (16bit)
                raw_volt = int(current_volt / SCALE_VOLT)
                if raw_volt > 65535: raw_volt = 65535
                
                # Data Packing (Little Endian)
                # Byte 0~1: APS1 Voltage
                # Byte 2~3: APS2 Voltage (동일하게 설정)
                data_volt = raw_volt.to_bytes(2, byteorder='little')
                
                # Payload: [APS1_L, APS1_H, APS2_L, APS2_H, 0, 0, 0, 0]
                payload = data_volt + data_volt + b'\x00\x00\x00\x00'
                
                msg = can.Message(arbitration_id=ID_FB, data=payload, is_extended_id=False)
                self.bus.send(msg)
                
                time.sleep(0.02) # 20ms 주기
                
            except Exception as e:
                print(f"TX Error: {e}")
            
    def update_gui_loop(self):
        # 물리 시뮬레이션 (부드러운 가속)
        diff = self.target_pct - self.current_pct
        
        if self.is_active:
            step = 2.0  # 반응 속도
            if abs(diff) > step:
                self.current_pct += step if diff > 0 else -step
            else:
                self.current_pct = self.target_pct
        else:
            # 모드 꺼지면 0으로 복귀
            if self.current_pct > 0:
                self.current_pct -= 2.0
                if self.current_pct < 0: self.current_pct = 0

        self.update_gauge(self.current_pct)
        
        # 전압 계산 (모니터링용)
        curr_volt = (self.current_pct / 100.0) * 5.0
        
        if self.is_active:
            self.lbl_mode.config(text="MODE: AUTO (Percent)", fg="white", bg="#00CC00")
        else:
            self.lbl_mode.config(text="MODE: Manual", fg="gray", bg="#eee")
            
        self.lbl_data.config(text=f"Target: {self.target_pct:.1f} % | Output: {curr_volt:.2f} V")
        self.after(20, self.update_gui_loop)

if __name__ == "__main__":
    app = VirtualAccel()
    app.mainloop()
