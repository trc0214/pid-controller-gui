import dearpygui.dearpygui as dpg
import time
import random

dpg.create_context()

def update_plot():
    # 模擬速度和位置數據
    speed_data.append(random.uniform(0, 100))
    position_data.append(random.uniform(0, 500))
    dpg.set_value("speed_series", [list(range(len(speed_data))), speed_data])
    dpg.set_value("position_series", [list(range(len(position_data))), position_data])

def confirm_pid(sender, app_data, user_data):
    group = user_data
    kp = dpg.get_value(f"kp_{group}")
    ki = dpg.get_value(f"ki_{group}")
    kd = dpg.get_value(f"kd_{group}")
    print(f"PID {group}: Kp={kp}, Ki={ki}, Kd={kd}")

def send_target():
    speed = dpg.get_value("target_speed")
    position = dpg.get_value("target_position")
    mode = dpg.get_value("mode_combo")
    print(f"Mode: {mode}, Target Speed: {speed}, Target Position: {position}")

# 數據存儲
speed_data = []
position_data = []

with dpg.window(label="Linear Rail Control", width=800, height=600):
    # 模式選擇
    dpg.add_combo(items=["Speed Mode", "Position Mode"], label="Mode", tag="mode_combo")
    
    # PID輸入
    dpg.add_text("PID Group 1")
    dpg.add_input_float(label="Kp", tag="kp_1", default_value=1.0)
    dpg.add_input_float(label="Ki", tag="ki_1", default_value=0.1)
    dpg.add_input_float(label="Kd", tag="kd_1", default_value=0.05)
    dpg.add_button(label="Confirm PID 1", callback=confirm_pid, user_data="Group 1")
    
    dpg.add_text("PID Group 2")
    dpg.add_input_float(label="Kp", tag="kp_2", default_value=1.0)
    dpg.add_input_float(label="Ki", tag="ki_2", default_value=0.1)
    dpg.add_input_float(label="Kd", tag="kd_2", default_value=0.05)
    dpg.add_button(label="Confirm PID 2", callback=confirm_pid, user_data="Group 2")
    
    # 目標輸入
    dpg.add_input_float(label="Target Speed", tag="target_speed")
    dpg.add_input_float(label="Target Position", tag="target_position")
    dpg.add_button(label="Send Target", callback=send_target)
    
    # 繪圖
    with dpg.plot(label="Speed", height=200, width=400):
        dpg.add_plot_axis(dpg.Axis.X, label="Time")
        dpg.add_plot_axis(dpg.Axis.Y, label="Speed")
        dpg.add_line_series([], [], label="Speed", tag="speed_series")
    
    with dpg.plot(label="Position", height=200, width=400):
        dpg.add_plot_axis(dpg.Axis.X, label="Time")
        dpg.add_plot_axis(dpg.Axis.Y, label="Position")
        dpg.add_line_series([], [], label="Position", tag="position_series")

# 更新數據的定時器
def update():
    while dpg.is_dearpygui_running():
        update_plot()
        dpg.render_dearpygui_frame()
        time.sleep(0.1)

dpg.create_viewport(title="Linear Rail GUI", width=800, height=600)
dpg.setup_dearpygui()
dpg.show_viewport()

# 啟動更新線程
from threading import Thread
Thread(target=update, daemon=True).start()

dpg.start_dearpygui()
dpg.destroy_context()