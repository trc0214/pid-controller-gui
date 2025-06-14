import tkinter as tk
from tkinter import ttk, font
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque


# --- 主應用程式類別 ---
class PIDControllerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PID 控制系統")
        self.geometry("1200x750")
        self.configure(bg="#ffffff")

        # --- 初始化變數 ---
        self.current_mode = "position"
        self.target_value = None
        self.time_step = 0

        # 使用 deque 可以高效地在固定長度的數據序列中新增和移除元素
        self.max_data_points = 100
        self.time_data = deque(maxlen=self.max_data_points)
        self.pos_actual_data = deque(maxlen=self.max_data_points)
        self.pos_target_data = deque(maxlen=self.max_data_points)
        self.vel_actual_data = deque(maxlen=self.max_data_points)
        self.vel_target_data = deque(maxlen=self.max_data_points)

        # --- 設定樣式 ---
        self.setup_styles()

        # --- 建立主體框架 ---
        main_frame = tk.Frame(self, bg="#ffffff")
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.control_panel = self.create_control_panel(main_frame)
        self.control_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 1), pady=0)

        self.chart_area = self.create_chart_area(main_frame)
        self.chart_area.pack(
            side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(1, 0), pady=0
        )

        # 啟動數據更新循環
        self.update_charts()

    def setup_styles(self):
        """設定 ttk 元件的樣式，以符合原始 CSS 設計"""
        style = ttk.Style(self)
        style.theme_use("clam")  # 使用一個比較好客製化的主題

        # 通用設定
        default_font = font.Font(family="Times New Roman", size=12)
        label_font = font.Font(family="Times New Roman", size=12)
        bold_font = font.Font(family="Times New Roman", size=12, weight="bold")
        title_font = font.Font(family="Times New Roman", size=14, weight="bold")
        header_font = font.Font(family="Times New Roman", size=20, weight="bold")

        # --- 框架和標籤框架 ---
        style.configure("TFrame", background="#f5f5f5")
        style.configure("Chart.TFrame", background="#ffffff")
        style.configure(
            "Status.TFrame", background="#f5f5f5", borderwidth=1, relief="solid"
        )

        style.configure(
            "TLabel", background="#f5f5f5", foreground="#333333", font=label_font
        )
        style.configure(
            "Header.TLabel",
            background="#f5f5f5",
            foreground="#333333",
            font=header_font,
        )
        style.configure(
            "Status.TLabel", background="#f5f5f5", foreground="#333333", font=bold_font
        )
        style.configure(
            "TargetStatus.TLabel",
            background="#f5f5f5",
            foreground="#007bff",
            font=bold_font,
        )

        style.configure(
            "TLabelframe", background="#f5f5f5", bordercolor="#d1d1d1", relief="solid"
        )
        style.configure(
            "TLabelframe.Label",
            background="#f5f5f5",
            foreground="#333333",
            font=title_font,
        )

        # --- 輸入框和下拉選單 ---
        style.configure(
            "TEntry",
            fieldbackground="#ffffff",
            foreground="#333333",
            bordercolor="#d1d1d1",
            borderwidth=1,
            insertwidth=1,
        )
        style.map("TEntry", bordercolor=[("focus", "#007bff")])
        style.configure(
            "TCombobox",
            fieldbackground="#ffffff",
            foreground="#333333",
            bordercolor="#d1d1d1",
            borderwidth=1,
        )
        style.map("TCombobox", fieldbackground=[("readonly", "#ffffff")])

        # --- 按鈕 ---
        style.configure(
            "TButton",
            background="#007bff",
            foreground="#ffffff",
            font=bold_font,
            borderwidth=0,
            padding=(8, 8),
        )
        style.map(
            "TButton",
            background=[("active", "#0056b3"), ("pressed", "#0056b3")],
            relief=[("pressed", "sunken")],
        )

    def create_control_panel(self, parent):
        """建立左側的控制面板"""
        panel = ttk.Frame(parent, width=350, style="TFrame")
        panel.pack_propagate(False)  # 防止框架縮小以適應內容

        # --- 標頭 ---
        header = ttk.Label(
            panel, text="PID 控制系統", style="Header.TLabel", anchor="center"
        )
        header.pack(pady=20, fill="x")

        # --- 控制模式區 ---
        mode_frame = ttk.Labelframe(panel, text="控制模式", padding=(15, 10))
        mode_frame.pack(fill="x", padx=20, pady=(0, 20))
        self.mode_select = ttk.Combobox(
            mode_frame, values=["位置模式", "速度模式"], state="readonly"
        )
        self.mode_select.set("位置模式")
        self.mode_select.pack(fill="x", expand=True)
        self.mode_select.bind("<<ComboboxSelected>>", self.switch_mode)

        # --- 位置 PID 參數區 ---
        self.position_pid_frame = self.create_pid_section(
            panel, "位置 PID 參數", "position", [1.0, 0.1, 0.01]
        )
        self.position_pid_frame.pack(fill="x", padx=20, pady=(0, 20))

        # --- 速度 PID 參數區 ---
        self.velocity_pid_frame = self.create_pid_section(
            panel, "速度 PID 參數", "velocity", [0.8, 0.05, 0.005]
        )

        # --- 目標設定區 ---
        self.create_target_section(panel)

        return panel

    def create_pid_section(self, parent, title, mode_type, defaults):
        """建立一個 PID 參數輸入區塊的通用函式"""
        frame = ttk.Labelframe(parent, text=title, padding=(15, 10))

        labels = ["Kp", "Ki", "Kd"]
        self.entries = {}
        for i, (label, default_val) in enumerate(zip(labels, defaults)):
            ttk.Label(frame, text=label).grid(row=i, column=0, sticky="w", pady=2)
            entry = ttk.Entry(frame, width=15)
            entry.insert(0, str(default_val))
            entry.grid(row=i, column=1, sticky="ew", pady=2, padx=5)
            self.entries[f"{mode_type}_{label}"] = entry

        frame.grid_columnconfigure(1, weight=1)

        btn = ttk.Button(
            frame,
            text=f"確認{title.split(' ')[0]}參數",
            command=lambda: self.update_pid(mode_type),
        )
        btn.grid(row=3, column=0, columnspan=2, pady=(10, 0), sticky="ew")
        return frame

    def create_target_section(self, parent):
        """建立目標設定區塊"""
        target_frame = ttk.Labelframe(parent, text="目標設定", padding=(15, 10))
        target_frame.pack(fill="x", padx=20, pady=(0, 20))

        # 位置目標
        self.target_pos_group = ttk.Frame(target_frame, style="TFrame")
        ttk.Label(self.target_pos_group, text="目標位置 (mm)").pack(anchor="w")
        self.target_pos_entry = ttk.Entry(self.target_pos_group)
        self.target_pos_entry.insert(0, "50")
        self.target_pos_entry.pack(fill="x", pady=(5, 0))
        self.target_pos_group.pack(fill="x", pady=(0, 10))

        # 速度目標
        self.target_vel_group = ttk.Frame(target_frame, style="TFrame")
        ttk.Label(self.target_vel_group, text="目標速度 (mm/s)").pack(anchor="w")
        self.target_vel_entry = ttk.Entry(self.target_vel_group)
        self.target_vel_entry.insert(0, "20")
        self.target_vel_entry.pack(fill="x", pady=(5, 0))

        # 發送按鈕
        send_btn = ttk.Button(target_frame, text="發送目標", command=self.send_target)
        send_btn.pack(fill="x", pady=(10, 0))

    def create_chart_area(self, parent):
        """建立右側的圖表顯示區域"""
        area = ttk.Frame(parent, style="Chart.TFrame")

        # --- 圖表容器 ---
        chart_container = ttk.Frame(area, style="Chart.TFrame", padding=20)
        chart_container.pack(fill="both", expand=True)

        # Matplotlib Figure 和 Subplots
        self.fig, (self.pos_ax, self.vel_ax) = plt.subplots(
            2, 1, figsize=(8, 6), facecolor="#ffffff"
        )
        self.fig.tight_layout(pad=4.0)

        # 設定位置圖表
        self.pos_ax.set_title(
            "即時位置監控",
            color="#2c3e50",
            fontdict={"family": "Arial", "size": 16, "weight": "600"},
        )
        (self.pos_line_actual,) = self.pos_ax.plot(
            [], [], label="實際位置 (mm)", color="#2E5BBA", linewidth=2.5
        )
        (self.pos_line_target,) = self.pos_ax.plot(
            [],
            [],
            label="目標位置 (mm)",
            color="#D73027",
            linestyle="--",
            dashes=(8, 4),
            linewidth=2.5,
        )
        self.setup_axis_style(self.pos_ax, "位置 (mm)")

        # 設定速度圖表
        self.vel_ax.set_title(
            "即時速度監控",
            color="#2c3e50",
            fontdict={"family": "Arial", "size": 16, "weight": "600"},
        )
        (self.vel_line_actual,) = self.vel_ax.plot(
            [], [], label="實際速度 (mm/s)", color="#1A9641", linewidth=2.5
        )
        (self.vel_line_target,) = self.vel_ax.plot(
            [],
            [],
            label="目標速度 (mm/s)",
            color="#F57C00",
            linestyle="--",
            dashes=(8, 4),
            linewidth=2.5,
        )
        self.setup_axis_style(self.vel_ax, "速度 (mm/s)")

        self.vel_ax.set_xlabel("時間 (步)", color="#34495e")

        # 將 Matplotlib 圖表嵌入 Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=chart_container)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.draw()

        # --- 狀態列 ---
        status_bar = ttk.Frame(area, style="Status.TFrame", height=40)
        status_bar.pack(fill="x", padx=20, pady=(0, 20))
        status_bar.pack_propagate(False)

        self.mode_indicator = ttk.Label(
            status_bar, text="當前模式: 位置控制", style="Status.TLabel"
        )
        self.mode_indicator.pack(side=tk.LEFT, padx=20)

        self.target_status = ttk.Label(
            status_bar, text="目標: 未設定", style="TargetStatus.TLabel"
        )
        self.target_status.pack(side=tk.LEFT, padx=20)

        return area

    def setup_axis_style(self, ax, ylabel):
        """設定圖表座標軸的通用樣式"""
        ax.set_facecolor("rgba(236, 240, 241, 0.3)")
        ax.legend(
            loc="upper right", prop={"family": "Arial", "size": 10, "weight": "500"}
        )
        ax.set_ylabel(ylabel, color="#34495e")
        ax.grid(True, color="#bdc3c7", linestyle="-", linewidth=0.8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#7f8c8d")
            spine.set_linewidth(1.5)
        ax.tick_params(axis="both", colors="#34495e")

    def switch_mode(self, event=None):
        """切換控制模式時更新 UI"""
        selected_mode_text = self.mode_select.get()
        new_mode = "position" if selected_mode_text == "位置模式" else "velocity"

        if new_mode == self.current_mode:
            return

        self.current_mode = new_mode
        self.target_value = None  # 切換模式時清除目標

        if self.current_mode == "position":
            self.position_pid_frame.pack(fill="x", padx=20, pady=(0, 20))
            self.velocity_pid_frame.pack_forget()
            self.target_pos_group.pack(fill="x", pady=(0, 10))
            self.target_vel_group.pack_forget()
            self.mode_indicator.config(text="當前模式: 位置控制")
        else:  # velocity
            self.velocity_pid_frame.pack(fill="x", padx=20, pady=(0, 20))
            self.position_pid_frame.pack_forget()
            self.target_vel_group.pack(fill="x", pady=(0, 10))
            self.target_pos_group.pack_forget()
            self.mode_indicator.config(text="當前模式: 速度控制")

        self.target_status.config(text="目標: 未設定")
        print(f"模式已切換至: {self.current_mode}")

    def update_pid(self, mode_type):
        """從輸入框讀取並更新PID參數 (模擬)"""
        kp = self.entries[f"{mode_type}_Kp"].get()
        ki = self.entries[f"{mode_type}_Ki"].get()
        kd = self.entries[f"{mode_type}_Kd"].get()
        print(f"更新 {mode_type} PID 參數: Kp={kp}, Ki={ki}, Kd={kd}")

    def send_target(self):
        """發送目標值並更新狀態列"""
        try:
            if self.current_mode == "position":
                self.target_value = float(self.target_pos_entry.get())
                self.target_status.config(text=f"目標: {self.target_value:.2f} mm")
                print(f"發送位置目標: {self.target_value} mm")
            else:
                self.target_value = float(self.target_vel_entry.get())
                self.target_status.config(text=f"目標: {self.target_value:.2f} mm/s")
                print(f"發送速度目標: {self.target_value} mm/s")
        except ValueError:
            self.target_value = None
            self.target_status.config(text="目標: 無效輸入")
            print("無效的目標值輸入")

    def update_charts(self):
        """更新圖表數據並重繪"""
        # --- 模擬數據生成 ---
        self.time_step += 1
        pos_actual = (
            50 + 30 * np.sin(self.time_step * 0.1) + (np.random.rand() - 0.5) * 10
        )
        vel_actual = 3 * np.cos(self.time_step * 0.1) + (np.random.rand() - 0.5) * 6

        # --- 更新數據佇列 ---
        self.time_data.append(self.time_step)
        self.pos_actual_data.append(pos_actual)
        self.vel_actual_data.append(vel_actual)

        if self.current_mode == "position" and self.target_value is not None:
            self.pos_target_data.append(self.target_value)
        else:
            self.pos_target_data.append(np.nan)  # 使用 NaN 使線段中斷

        if self.current_mode == "velocity" and self.target_value is not None:
            self.vel_target_data.append(self.target_value)
        else:
            self.vel_target_data.append(np.nan)

        # --- 更新圖表線條數據 ---
        self.pos_line_actual.set_data(self.time_data, self.pos_actual_data)
        self.pos_line_target.set_data(self.time_data, self.pos_target_data)
        self.vel_line_actual.set_data(self.time_data, self.vel_actual_data)
        self.vel_line_target.set_data(self.time_data, self.vel_target_data)

        # --- 自動調整座標軸範圍 ---
        for ax in [self.pos_ax, self.vel_ax]:
            ax.relim()
            ax.autoscale_view()

        # --- 重繪 Canvas ---
        self.canvas.draw()

        # --- 設定下一次更新 ---
        self.after(100, self.update_charts)


if __name__ == "__main__":
    app = PIDControllerApp()
    app.mainloop()
