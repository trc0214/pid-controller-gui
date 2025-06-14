import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import time
import threading
import math
import random
import serial
import serial.tools.list_ports as list_ports
import struct

# 設定matplotlib中文字體
plt.rcParams["font.sans-serif"] = [
    "Microsoft JhengHei",
    "SimHei",
    "DejaVu Sans",
]  # 設定中文字體
plt.rcParams["axes.unicode_minus"] = False  # 解決負號顯示問題


class PIDControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PID控制系統")
        self.root.geometry("1200x900")
        self.root.configure(bg="white")

        # 全域變數
        self.current_mode = "position"
        self.current_target = None
        self.time_data = []
        self.position_data = []
        self.velocity_data = []
        self.target_position_data = []
        self.target_velocity_data = []
        self.time_counter = 0
        self.is_running = False
        self.serial: serial.Serial = None # serial.Serial("COM4", 921600)
        self.last_update_time = time.time()
        self.fps_cnt = 0
        self.last_print_fps = time.time()

        # PID參數
        self.position_pid = {"Kp": 1.0, "Ki": 0.1, "Kd": 0.01}
        self.velocity_pid = {"Kp": 0.8, "Ki": 0.05, "Kd": 0.005}

        self.setup_ui()
        self.setup_charts()
        self.update_display_mode()  # 移到這裡，確保所有UI元素都已創建
        # self.start_simulation()
        self.start_receiver()

    def setup_ui(self):
        # 主容器
        main_frame = tk.Frame(self.root, bg="white")
        main_frame.pack(fill="both", expand=True, padx=1, pady=1)

        # 左側控制面板
        self.control_panel = tk.Frame(
            main_frame, bg="#f5f5f5", width=350, relief="solid", bd=1
        )
        self.control_panel.pack(side="left", fill="y", padx=(0, 1))
        self.control_panel.pack_propagate(False)

        # 右側圖表區域
        self.chart_area = tk.Frame(main_frame, bg="white")
        self.chart_area.pack(side="right", fill="both", expand=True)

        self.setup_control_panel()

    def setup_control_panel(self):
        # 標題
        title_label = tk.Label(
            self.control_panel,
            text="PID 控制系統",
            font=("Microsoft JhengHei", 20, "bold"),
            bg="#f5f5f5",
            fg="#333333",
        )
        title_label.pack(pady=(20, 20))

        # 連線設定區塊
        connection_frame = tk.Frame(self.control_panel, bg="#f5f5f5", relief="solid", bd=1)
        connection_frame.pack(fill="x", padx=20, pady=(0, 20))

        connection_title = tk.Label(
            connection_frame,
            text="連線設定",
            font=("Microsoft JhengHei", 14, "bold"),
            bg="#f5f5f5",
            fg="#333333",
        )
        connection_title.pack(anchor="w", padx=15, pady=(15, 10))

        ports_raw = list_ports.comports()
        ports = ["Disconnect"]
        for port, desc, hwid in sorted(ports_raw):
            print(f"  {port}: {desc} [{hwid}]")
            ports.append(f"{port} {desc}")

        self.com_var = tk.StringVar(value=ports[0])
        com_combo = ttk.Combobox(
            connection_frame,
            textvariable=self.com_var,
            values=ports,
            state="readonly",
            font=("Microsoft JhengHei", 12),
        )
        # 設定顯示文字映射
        # com_combo.configure(values=["COM?", "COM?"])
        com_combo.pack(fill="x", padx=15, pady=(0, 15))
        com_combo.bind("<<ComboboxSelected>>", self.on_com_port_change)

        # 控制模式區塊
        mode_frame = tk.Frame(self.control_panel, bg="#f5f5f5", relief="solid", bd=1)
        mode_frame.pack(fill="x", padx=20, pady=(0, 20))

        mode_title = tk.Label(
            mode_frame,
            text="控制模式",
            font=("Microsoft JhengHei", 14, "bold"),
            bg="#f5f5f5",
            fg="#333333",
        )
        mode_title.pack(anchor="w", padx=15, pady=(15, 10))

        self.mode_var = tk.StringVar(value="position")
        mode_combo = ttk.Combobox(
            mode_frame,
            textvariable=self.mode_var,
            values=["position", "velocity"],
            state="readonly",
            font=("Microsoft JhengHei", 12),
        )
        # 設定顯示文字映射
        mode_combo.configure(values=["position", "velocity"])
        mode_combo.pack(fill="x", padx=15, pady=(0, 15))
        mode_combo.bind("<<ComboboxSelected>>", self.on_mode_change)

        # 目標設定區塊
        target_frame = tk.Frame(self.control_panel, bg="#f5f5f5", relief="solid", bd=1)
        target_frame.pack(fill="x", padx=20, pady=(0, 20))

        target_title = tk.Label(
            target_frame,
            text="目標設定",
            font=("Microsoft JhengHei", 14, "bold"),
            bg="#f5f5f5",
            fg="#333333",
        )
        target_title.pack(anchor="w", padx=15, pady=(15, 10))

        # 目標位置
        self.target_pos_frame = tk.Frame(target_frame, bg="#f5f5f5")
        self.target_pos_frame.pack(fill="x", padx=15, pady=(0, 0))

        pos_label = tk.Label(
            self.target_pos_frame,
            text="目標位置 (mm)",
            font=("Microsoft JhengHei", 12),
            bg="#f5f5f5",
            fg="#333333",
        )
        pos_label.pack(anchor="w", pady=(0, 5))

        self.target_position_var = tk.DoubleVar(value=50)
        pos_entry = tk.Entry(
            self.target_pos_frame,
            textvariable=self.target_position_var,
            font=("Microsoft JhengHei", 14),
            bg="white",
            fg="#333333",
        )
        pos_entry.pack(fill="x")

        target_btn = tk.Button(
            self.target_pos_frame,
            text="發送目標",
            bg="#007bff",
            fg="white",
            font=("Microsoft JhengHei", 12, "bold"),
            command=self.send_target,
        )
        target_btn.pack(fill="x", padx=15, pady=(15, 5))

        # 目標速度
        self.target_vel_frame = tk.Frame(target_frame, bg="#f5f5f5")
        self.target_vel_frame.pack(fill="x", padx=15, pady=(0, 0))

        vel_label = tk.Label(
            self.target_vel_frame,
            text="目標速度 (mm/s)",
            font=("Microsoft JhengHei", 12),
            bg="#f5f5f5",
            fg="#333333",
        )
        vel_label.pack(anchor="w", pady=(0, 5))

        self.target_velocity_var = tk.DoubleVar(value=20)
        vel_entry = tk.Entry(
            self.target_vel_frame,
            textvariable=self.target_velocity_var,
            font=("Microsoft JhengHei", 14),
            bg="white",
            fg="#333333",
        )
        vel_entry.pack(fill="x")

        target_btn = tk.Button(
            self.target_vel_frame,
            text="發送目標",
            bg="#007bff",
            fg="white",
            font=("Microsoft JhengHei", 12, "bold"),
            command=self.send_target,
        )
        target_btn.pack(fill="x", padx=15, pady=(15, 5))

        # 位置PID參數區塊
        self.position_pid_frame = tk.Frame(
            self.control_panel, bg="#f5f5f5", relief="solid", bd=1
        )
        self.position_pid_frame.pack(fill="x", padx=20, pady=(0, 20))

        pos_title = tk.Label(
            self.position_pid_frame,
            text="位置 PID 參數",
            font=("Microsoft JhengHei", 14, "bold"),
            bg="#f5f5f5",
            fg="#333333",
        )
        pos_title.pack(anchor="w", padx=15, pady=(15, 10))

        # 位置PID輸入框
        self.pos_kp_var = tk.DoubleVar(value=1.0)
        self.pos_ki_var = tk.DoubleVar(value=0.1)
        self.pos_kd_var = tk.DoubleVar(value=0.01)

        self.create_pid_inputs(
            self.position_pid_frame,
            [("Kp", self.pos_kp_var), ("Ki", self.pos_ki_var), ("Kd", self.pos_kd_var)],
        )

        pos_btn = tk.Button(
            self.position_pid_frame,
            text="確認位置參數",
            bg="#007bff",
            fg="white",
            font=("Microsoft JhengHei", 12, "bold"),
            command=lambda: self.update_pid("position"),
        )
        pos_btn.pack(fill="x", padx=15, pady=(0, 15))

        # 速度PID參數區塊
        self.velocity_pid_frame = tk.Frame(
            self.control_panel, bg="#f5f5f5", relief="solid", bd=1
        )
        self.velocity_pid_frame.pack(fill="x", padx=20, pady=(0, 20))

        vel_title = tk.Label(
            self.velocity_pid_frame,
            text="速度 PID 參數",
            font=("Microsoft JhengHei", 14, "bold"),
            bg="#f5f5f5",
            fg="#333333",
        )
        vel_title.pack(anchor="w", padx=15, pady=(15, 10))

        # 速度PID輸入框
        self.vel_kp_var = tk.DoubleVar(value=0.8)
        self.vel_ki_var = tk.DoubleVar(value=0.05)
        self.vel_kd_var = tk.DoubleVar(value=0.005)

        self.create_pid_inputs(
            self.velocity_pid_frame,
            [("Kp", self.vel_kp_var), ("Ki", self.vel_ki_var), ("Kd", self.vel_kd_var)],
        )

        vel_btn = tk.Button(
            self.velocity_pid_frame,
            text="確認速度參數",
            bg="#007bff",
            fg="white",
            font=("Microsoft JhengHei", 12, "bold"),
            command=lambda: self.update_pid("velocity"),
        )
        vel_btn.pack(fill="x", padx=15, pady=(0, 15))

        

        # 移除這裡的初始化調用，因為此時狀態欄還沒創建
        # self.update_display_mode()

    def create_pid_inputs(self, parent, params):
        for label, var in params:
            frame = tk.Frame(parent, bg="#f5f5f5")
            frame.pack(fill="x", padx=15, pady=(0, 10))

            lbl = tk.Label(
                frame,
                text=label,
                font=("Microsoft JhengHei", 12),
                bg="#f5f5f5",
                fg="#333333",
            )
            lbl.pack(anchor="w", pady=(0, 5))

            entry = tk.Entry(
                frame,
                textvariable=var,
                font=("Microsoft JhengHei", 14),
                bg="white",
                fg="#333333",
            )
            entry.pack(fill="x")

    def setup_charts(self):
        # 創建圖表容器
        chart_container = tk.Frame(self.chart_area, bg="white")
        chart_container.pack(fill="both", expand=True, padx=20, pady=20)

        # 創建matplotlib圖表
        self.fig = Figure(figsize=(12, 10), facecolor="white")

        # 位置圖表
        self.pos_ax = self.fig.add_subplot(2, 1, 1)
        self.pos_ax.set_title(
            "即時位置監控", fontsize=16, fontweight="600", color="#2c3e50"
        )
        self.pos_ax.set_ylabel("位置 (mm)", color="#34495e")
        self.pos_ax.grid(True, color="#bdc3c7", linewidth=0.8)

        # 速度圖表
        self.vel_ax = self.fig.add_subplot(2, 1, 2)
        self.vel_ax.set_title(
            "即時速度監控", fontsize=16, fontweight="600", color="#2c3e50"
        )
        self.vel_ax.set_xlabel("時間", color="#34495e")
        self.vel_ax.set_ylabel("速度 (mm/s)", color="#34495e")
        self.vel_ax.grid(True, color="#bdc3c7", linewidth=0.8)

        # 嵌入tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, chart_container)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # 狀態欄
        status_frame = tk.Frame(
            self.chart_area, bg="#f5f5f5", height=40, relief="solid", bd=1
        )
        status_frame.pack(fill="x", padx=20, pady=(0, 20))
        status_frame.pack_propagate(False)

        self.mode_label = tk.Label(
            status_frame,
            text="當前模式: 位置控制",
            font=("Microsoft JhengHei", 12, "bold"),
            bg="#f5f5f5",
            fg="#333333",
        )
        self.mode_label.pack(side="left", padx=20, pady=10)

        self.target_label = tk.Label(
            status_frame,
            text="目標: 未設定",
            font=("Microsoft JhengHei", 12, "bold"),
            bg="#f5f5f5",
            fg="#007bff",
        )
        self.target_label.pack(side="left", padx=20, pady=10)

    def on_mode_change(self, event=None):
        self.current_mode = (
            "position" if self.mode_var.get() == "position" else "velocity"
        )
        self.current_target = None
        self.update_display_mode()

    def on_com_port_change(self, event=None):
        self.is_running = False
        com = self.com_var.get()
        if com == "Disconnect":
            if self.serial is not None:
                self.serial.close()
                print(f"close port")
        ## serial not open
        elif(self.serial is None or not self.serial.is_open):
            com = com[:com.find(" ")]
            self.serial = serial.Serial(com, 921600)
            self.is_running = True
            print(f"open port: {com}: {self.serial.is_open}")
        
        # self.current_mode = (
        #     "position" if self.mode_var.get() == "position" else "velocity"
        # )
        # self.current_target = None
        # self.update_display_mode()

    def update_display_mode(self):
        if self.current_mode == "position":
            # 隱藏速度相關組件
            self.velocity_pid_frame.pack_forget()
            self.target_vel_frame.pack_forget()

            # 顯示位置相關組件
            self.position_pid_frame.pack(fill="x", padx=20, pady=(0, 20))
            self.target_pos_frame.pack(fill="x", padx=15, pady=(0, 10))

            # 檢查 mode_label 是否存在再更新
            if hasattr(self, "mode_label"):
                self.mode_label.config(text="當前模式: 位置控制")
        else:
            # 隱藏位置相關組件
            self.position_pid_frame.pack_forget()
            self.target_pos_frame.pack_forget()

            # 顯示速度相關組件
            self.velocity_pid_frame.pack(fill="x", padx=20, pady=(0, 20))
            self.target_vel_frame.pack(fill="x", padx=15, pady=(0, 10))

            # 檢查 mode_label 是否存在再更新
            if hasattr(self, "mode_label"):
                self.mode_label.config(text="當前模式: 速度控制")

        # 檢查 target_label 是否存在再更新
        if hasattr(self, "target_label"):
            self.target_label.config(text="目標: 未設定")

    def update_pid(self, pid_type):
        if pid_type == "position":
            self.position_pid["Kp"] = self.pos_kp_var.get()
            self.position_pid["Ki"] = self.pos_ki_var.get()
            self.position_pid["Kd"] = self.pos_kd_var.get()
            data = struct.pack('<4sfffi', b"\xAA\xBB\xCC\xFF", self.position_pid["Kp"], self.position_pid["Ki"], self.position_pid["Kd"], 1)
            self.serial.write(data)
            messagebox.showinfo(
                "參數更新",
                f"位置PID參數已更新:\nKp={self.position_pid['Kp']}\nKi={self.position_pid['Ki']}\nKd={self.position_pid['Kd']}",
            )
        else:
            self.velocity_pid["Kp"] = self.vel_kp_var.get()
            self.velocity_pid["Ki"] = self.vel_ki_var.get()
            self.velocity_pid["Kd"] = self.vel_kd_var.get()
            data = struct.pack('<4sfffi', b"\xAA\xBB\xCC\xFF", self.velocity_pid["Kp"], self.velocity_pid["Ki"], self.velocity_pid["Kd"], 0)
            self.serial.write(data)
            messagebox.showinfo(
                "參數更新",
                f"速度PID參數已更新:\nKp={self.velocity_pid['Kp']}\nKi={self.velocity_pid['Ki']}\nKd={self.velocity_pid['Kd']}",
            )

    def send_target(self):
        if self.current_mode == "position":
            self.current_target = self.target_position_var.get()
            self.target_label.config(text=f"目標: {self.current_target} mm")
            data = struct.pack('<4sffi', b"\xAA\xBB\xCC\xEE", 0, self.current_target, 1)
            self.serial.write(data)
        else:
            self.current_target = self.target_velocity_var.get()
            self.target_label.config(text=f"目標: {self.current_target} mm/s")
            data = struct.pack('<4sffi', b"\xAA\xBB\xCC\xEE", self.current_target, 0, 0)
            self.serial.write(data)

    def start_receiver(self):
        self.is_running = False
        self.receiver_thread = threading.Thread(
            target=self.receiver_loop, daemon=True
        )
        self.receiver_thread.start()

    def receiver_loop(self):
        while True:
            if not self.is_running:
                time.sleep(0.1)
                continue
            # print(self.serial.readline())
            # continue
            data = self.serial.read_until(b"\xAA\xBB\xCC\xDD", None)
            if(len(data) != 36):
                print(len(data))
                continue
            # print(f"==ST==\n", data, "\n==ED==", len(data), "\n")
            vel, pos, target_vel, target_pos, output, p, i, d, *_ = struct.unpack('<8f4B', data)
            
            self.time_counter += 0.01
            velocity = vel
            position = pos
            self.time_data.append(self.time_counter)
            self.position_data.append(position)
            self.velocity_data.append(velocity)
            self.target_position_data.append(target_pos)
            self.target_velocity_data.append(target_vel)
            # 限制數據點數量
            if len(self.time_data) > 1000:
                self.time_data.pop(0)
                self.position_data.pop(0)
                self.velocity_data.pop(0)
                self.target_position_data.pop(0)
                self.target_velocity_data.pop(0)
            # 更新圖表
            if(time.time() - self.last_update_time >= 0.1):
                self.last_update_time = time.time()
                self.root.after(0, self.update_charts)
            self.fps_cnt += 1
            if(time.time() - self.last_print_fps >= 1):
                self.last_print_fps = time.time()
                print(f"pps: {self.fps_cnt}")
                self.fps_cnt = 0
            # time.sleep(0.1)

    def start_simulation(self):
        self.is_running = True
        self.simulation_thread = threading.Thread(
            target=self.simulation_loop, daemon=True
        )
        self.simulation_thread.start()

    def simulation_loop(self):
        while self.is_running:
            self.time_counter += 1

            # 模擬位置和速度數據
            position = 50 + 30 * math.sin(self.time_counter * 0.1) + random.random() * 5
            velocity = 20 + 15 * math.cos(self.time_counter * 0.1) + random.random() * 3

            self.time_data.append(self.time_counter)
            self.position_data.append(position)
            self.velocity_data.append(velocity)

            # 目標數據
            if self.current_mode == "position" and self.current_target is not None:
                self.target_position_data.append(self.current_target)
            else:
                self.target_position_data.append(None)

            if self.current_mode == "velocity" and self.current_target is not None:
                self.target_velocity_data.append(self.current_target)
            else:
                self.target_velocity_data.append(None)

            # 限制數據點數量
            if len(self.time_data) > 50:
                self.time_data.pop(0)
                self.position_data.pop(0)
                self.velocity_data.pop(0)
                self.target_position_data.pop(0)
                self.target_velocity_data.pop(0)

            # 更新圖表
            self.root.after(0, self.update_charts)

            time.sleep(0.1)

    def update_charts(self):
        # 清除舊的繪圖
        self.pos_ax.clear()
        self.vel_ax.clear()

        # 重新設置圖表
        self.pos_ax.set_title(
            "即時位置監控", fontsize=16, fontweight="600", color="#2c3e50"
        )
        self.pos_ax.set_ylabel("位置 (mm)", color="#34495e")
        self.pos_ax.grid(True, color="#bdc3c7", linewidth=0.8)

        self.vel_ax.set_title(
            "即時速度監控", fontsize=16, fontweight="600", color="#2c3e50"
        )
        self.vel_ax.set_xlabel("時間", color="#34495e")
        self.vel_ax.set_ylabel("速度 (mm/s)", color="#34495e")
        self.vel_ax.grid(True, color="#bdc3c7", linewidth=0.8)

        if self.time_data:
            # 繪製位置數據
            self.pos_ax.plot(
                self.time_data,
                self.position_data,
                color="#2E5BBA",
                linewidth=2.5,
                label="實際位置 (mm)",
            )

            # 繪製位置目標線
            target_pos_clean = [
                t if t is not None else np.nan for t in self.target_position_data
            ]
            if any(t is not None for t in self.target_position_data):
                self.pos_ax.plot(
                    self.time_data,
                    target_pos_clean,
                    color="#D73027",
                    linewidth=2.5,
                    linestyle="--",
                    label="目標位置 (mm)",
                )

            # 繪製速度數據
            self.vel_ax.plot(
                self.time_data,
                self.velocity_data,
                color="#1A9641",
                linewidth=2.5,
                label="實際速度 (mm/s)",
            )

            # 繪製速度目標線
            target_vel_clean = [
                t if t is not None else np.nan for t in self.target_velocity_data
            ]
            if any(t is not None for t in self.target_velocity_data):
                self.vel_ax.plot(
                    self.time_data,
                    target_vel_clean,
                    color="#F57C00",
                    linewidth=2.5,
                    linestyle="--",
                    label="目標速度 (mm/s)",
                )

        # 添加圖例，固定在右上角
        self.pos_ax.legend(loc="upper right", frameon=True, fancybox=True, shadow=True)
        self.vel_ax.legend(loc="upper right", frameon=True, fancybox=True, shadow=True)

        # 刷新畫布
        self.canvas.draw()

    def on_closing(self):
        self.is_running = False
        self.root.destroy()


def main():
    root = tk.Tk()
    app = PIDControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
