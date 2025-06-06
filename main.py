import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np

# 主窗口
root = tk.Tk()
root.title("Linear Rail Control")
root.geometry("800x600")

# 模擬數據
time_data = []
speed_data = []
position_data = []

# 繪圖設置
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5, 4))
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)


def update_plot(frame):
    time_data.append(frame)
    speed_data.append(np.random.uniform(0, 100))  # 模擬速度數據
    position_data.append(np.random.uniform(0, 500))  # 模擬位置數據
    ax1.clear()
    ax2.clear()
    ax1.plot(time_data[-50:], speed_data[-50:], "b-", label="Speed")
    ax2.plot(time_data[-50:], position_data[-50:], "r-", label="Position")
    ax1.set_title("Speed")
    ax2.set_title("Position")
    ax1.set_xlabel("Time")
    ax2.set_xlabel("Time")
    ax1.set_ylabel("Speed (m/s)")
    ax2.set_ylabel("Position (mm)")
    ax1.legend()
    ax2.legend()
    ax1.grid(True)
    ax2.grid(True)
    canvas.draw()


# 即時更新圖表
ani = FuncAnimation(fig, update_plot, interval=100)

# 模式選擇
tk.Label(root, text="Mode:").pack()
mode_combo = ttk.Combobox(
    root, values=["Speed Mode", "Position Mode"], state="readonly"
)
mode_combo.pack()
mode_combo.set("Speed Mode")

# PID輸入 - Group 1
pid_frame1 = tk.Frame(root)
pid_frame1.pack(pady=5)
tk.Label(pid_frame1, text="PID Group 1").pack()
tk.Label(pid_frame1, text="Kp").pack(side=tk.LEFT)
kp1_entry = tk.Entry(pid_frame1, width=10)
kp1_entry.insert(0, "1.0")
kp1_entry.pack(side=tk.LEFT)
tk.Label(pid_frame1, text="Ki").pack(side=tk.LEFT)
ki1_entry = tk.Entry(pid_frame1, width=10)
ki1_entry.insert(0, "0.1")
ki1_entry.pack(side=tk.LEFT)
tk.Label(pid_frame1, text="Kd").pack(side=tk.LEFT)
kd1_entry = tk.Entry(pid_frame1, width=10)
kd1_entry.insert(0, "0.05")
kd1_entry.pack(side=tk.LEFT)
tk.Button(
    pid_frame1,
    text="Confirm PID 1",
    command=lambda: print(
        f"PID 1: Kp={kp1_entry.get()}, Ki={ki1_entry.get()}, Kd={kd1_entry.get()}"
    ),
).pack(side=tk.LEFT, padx=5)

# PID輸入 - Group 2
pid_frame2 = tk.Frame(root)
pid_frame2.pack(pady=5)
tk.Label(pid_frame2, text="PID Group 2").pack()
tk.Label(pid_frame2, text="Kp").pack(side=tk.LEFT)
kp2_entry = tk.Entry(pid_frame2, width=10)
kp2_entry.insert(0, "1.0")
kp2_entry.pack(side=tk.LEFT)
tk.Label(pid_frame2, text="Ki").pack(side=tk.LEFT)
ki2_entry = tk.Entry(pid_frame2, width=10)
ki2_entry.insert(0, "0.1")
ki2_entry.pack(side=tk.LEFT)
tk.Label(pid_frame2, text="Kd").pack(side=tk.LEFT)
kd2_entry = tk.Entry(pid_frame2, width=10)
kd2_entry.insert(0, "0.05")
kd2_entry.pack(side=tk.LEFT)
tk.Button(
    pid_frame2,
    text="Confirm PID 2",
    command=lambda: print(
        f"PID 2: Kp={kp2_entry.get()}, Ki={ki2_entry.get()}, Kd={kd2_entry.get()}"
    ),
).pack(side=tk.LEFT, padx=5)

# 目標輸入
target_frame = tk.Frame(root)
target_frame.pack(pady=5)
tk.Label(target_frame, text="Target Speed (m/s):").pack(side=tk.LEFT)
speed_entry = tk.Entry(target_frame, width=10)
speed_entry.pack(side=tk.LEFT)
tk.Label(target_frame, text="Target Position (mm):").pack(side=tk.LEFT)
position_entry = tk.Entry(target_frame, width=10)
position_entry.pack(side=tk.LEFT)
tk.Button(
    target_frame,
    text="Send Target",
    command=lambda: print(
        f"Mode: {mode_combo.get()}, Speed: {speed_entry.get()}, Position: {position_entry.get()}"
    ),
).pack(side=tk.LEFT, padx=5)

root.mainloop()
