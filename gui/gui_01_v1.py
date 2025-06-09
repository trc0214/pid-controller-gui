import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget, QLabel, QComboBox, QGroupBox, QLineEdit, QPushButton
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import numpy as np

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID 控制系統")
        self.setGeometry(100, 100, 1200, 800)
        
        # 主佈局
        main_layout = QHBoxLayout()
        
        # 控制面板
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, 1)
        
        # 圖表區域
        chart_area = self.create_chart_area()
        main_layout.addWidget(chart_area, 3)
        
        # 設置中央 widget
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # 初始化數據
        self.time = 0
        self.position_data = []
        self.speed_data = []
        self.target_position_data = []
        self.target_speed_data = []
        self.current_mode = 'position'
        self.current_target = None
        
        # 設置定時器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)
    
    def create_control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout()
        
        # 標題
        title = QLabel("PID 控制系統")
        title.setStyleSheet("font-size: 20px; font-weight: bold; color: #333333;")
        layout.addWidget(title)
        
        # 控制模式
        mode_group = QGroupBox("控制模式")
        mode_layout = QVBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["位置模式", "速度模式"])
        self.mode_combo.currentTextChanged.connect(self.mode_changed)
        mode_layout.addWidget(self.mode_combo)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # 位置 PID
        self.position_pid_group = QGroupBox("位置 PID 參數")
        position_pid_layout = QVBoxLayout()
        self.position_kp = QLineEdit("1.0")
        self.position_ki = QLineEdit("0.1")
        self.position_kd = QLineEdit("0.01")
        position_pid_layout.addWidget(QLabel("Kp"))
        position_pid_layout.addWidget(self.position_kp)
        position_pid_layout.addWidget(QLabel("Ki"))
        position_pid_layout.addWidget(self.position_ki)
        position_pid_layout.addWidget(QLabel("Kd"))
        position_pid_layout.addWidget(self.position_kd)
        self.position_pid_button = QPushButton("確認位置參數")
        self.position_pid_button.clicked.connect(lambda: self.update_pid('position'))
        position_pid_layout.addWidget(self.position_pid_button)
        self.position_pid_group.setLayout(position_pid_layout)
        layout.addWidget(self.position_pid_group)
        
        # 速度 PID
        self.velocity_pid_group = QGroupBox("速度 PID 參數")
        velocity_pid_layout = QVBoxLayout()
        self.velocity_kp = QLineEdit("0.8")
        self.velocity_ki = QLineEdit("0.05")
        self.velocity_kd = QLineEdit("0.005")
        velocity_pid_layout.addWidget(QLabel("Kp"))
        velocity_pid_layout.addWidget(self.velocity_kp)
        velocity_pid_layout.addWidget(QLabel("Ki"))
        velocity_pid_layout.addWidget(self.velocity_ki)
        velocity_pid_layout.addWidget(QLabel("Kd"))
        velocity_pid_layout.addWidget(self.velocity_kd)
        self.velocity_pid_button = QPushButton("確認速度參數")
        self.velocity_pid_button.clicked.connect(lambda: self.update_pid('velocity'))
        velocity_pid_layout.addWidget(self.velocity_pid_button)
        self.velocity_pid_group.setLayout(velocity_pid_layout)
        self.velocity_pid_group.setVisible(False)
        layout.addWidget(self.velocity_pid_group)
        
        # 目標設定
        self.target_position_group = QGroupBox("目標設定")
        target_position_layout = QVBoxLayout()
        self.target_position = QLineEdit("50")
        target_position_layout.addWidget(QLabel("目標位置 (mm)"))
        target_position_layout.addWidget(self.target_position)
        self.target_position_group.setLayout(target_position_layout)
        layout.addWidget(self.target_position_group)
        
        self.target_velocity_group = QGroupBox("目標設定")
        target_velocity_layout = QVBoxLayout()
        self.target_velocity = QLineEdit("20")
        target_velocity_layout.addWidget(QLabel("目標速度 (mm/s)"))
        target_velocity_layout.addWidget(self.target_velocity)
        self.target_velocity_group.setLayout(target_velocity_layout)
        self.target_velocity_group.setVisible(False)
        layout.addWidget(self.target_velocity_group)
        
        self.send_target_button = QPushButton("發送目標")
        self.send_target_button.clicked.connect(self.send_target)
        layout.addWidget(self.send_target_button)
        
        panel.setLayout(layout)
        return panel
    
    def create_chart_area(self):
        area = QWidget()
        layout = QVBoxLayout()
        
        # 位置圖表
        self.position_plot = pg.PlotWidget(title="即時位置監控")
        self.position_plot.setLabel('left', '位置 (mm)')
        self.position_plot.setLabel('bottom', '時間')
        self.position_curve = self.position_plot.plot(pen=pg.mkPen(color='#2E5BBA', width=2.5))
        self.target_position_curve = self.position_plot.plot(pen=pg.mkPen(color='#D73027', width=2.5, dash=[8,4]))
        layout.addWidget(self.position_plot)
        
        # 速度圖表
        self.speed_plot = pg.PlotWidget(title="即時速度監控")
        self.speed_plot.setLabel('left', '速度 (mm/s)')
        self.speed_plot.setLabel('bottom', '時間')
        self.speed_curve = self.speed_plot.plot(pen=pg.mkPen(color='#1A9641', width=2.5))
        self.target_speed_curve = self.speed_plot.plot(pen=pg.mkPen(color='#F57C00', width=2.5, dash=[8,4]))
        layout.addWidget(self.speed_plot)
        
        # 狀態欄
        self.status_bar = QLabel("當前模式: 位置控制 | 目標: 未設定")
        self.status_bar.setStyleSheet("background: #f5f5f5; border: 1px solid #d1d1d1; padding: 5px;")
        layout.addWidget(self.status_bar)
        
        area.setLayout(layout)
        return area
    
    def mode_changed(self, text):
        if text == "位置模式":
            self.current_mode = 'position'
            self.position_pid_group.setVisible(True)
            self.velocity_pid_group.setVisible(False)
            self.target_position_group.setVisible(True)
            self.target_velocity_group.setVisible(False)
            self.status_bar.setText("當前模式: 位置控制 | 目標: 未設定")
        else:
            self.current_mode = 'velocity'
            self.position_pid_group.setVisible(False)
            self.velocity_pid_group.setVisible(True)
            self.target_position_group.setVisible(False)
            self.target_velocity_group.setVisible(True)
            self.status_bar.setText("當前模式: 速度控制 | 目標: 未設定")
        self.current_target = None
    
    def update_pid(self, type):
        print(f"更新 {type} PID 參數")
    
    def send_target(self):
        if self.current_mode == 'position':
            target = float(self.target_position.text())
            self.current_target = target
            self.status_bar.setText(f"當前模式: 位置控制 | 目標: {target} mm")
        else:
            target = float(self.target_velocity.text())
            self.current_target = target
            self.status_bar.setText(f"當前模式: 速度控制 | 目標: {target} mm/s")
    
    def update_data(self):
        self.time += 1
        position = 50 + 30 * np.sin(self.time * 0.1) + np.random.normal(0, 2.5)
        speed = 20 + 15 * np.cos(self.time * 0.1) + np.random.normal(0, 1.5)
        
        self.position_data.append(position)
        self.speed_data.append(speed)
        
        if self.current_mode == 'position' and self.current_target is not None:
            self.target_position_data.append(self.current_target)
        else:
            self.target_position_data.append(np.nan)
        
        if self.current_mode == 'velocity' and self.current_target is not None:
            self.target_speed_data.append(self.current_target)
        else:
            self.target_speed_data.append(np.nan)
        
        if len(self.position_data) > 50:
            self.position_data.pop(0)
            self.speed_data.pop(0)
            self.target_position_data.pop(0)
            self.target_speed_data.pop(0)
        
        self.position_curve.setData(self.position_data)
        self.target_position_curve.setData(self.target_position_data)
        self.speed_curve.setData(self.speed_data)
        self.target_speed_curve.setData(self.target_speed_data)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())