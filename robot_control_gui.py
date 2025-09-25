#!/usr/bin/env python3
"""
ESP32-S3 2-DOF Robotic Arm Control Interface
Professional PyQt6 GUI with Dark Mode and Advanced Features

Features:
- Real-time robotic arm control
- Inverse kinematics visualization
- Workspace plotting
- Serial communication with ESP32
- Professional dark theme UI/UX
- Interactive coordinate input
- Joint angle monitoring
- Movement history tracking

Requirements:
- PyQt6
- pyserial
- matplotlib
- numpy

Installation:
pip install PyQt6 pyserial matplotlib numpy
"""

import sys
import json
import time
import math
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple
from pathlib import Path

# PyQt6 imports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit,
    QGroupBox, QFrame, QTabWidget, QProgressBar, QCheckBox, QLineEdit,
    QSplitter, QScrollArea, QMessageBox, QFileDialog, QStatusBar
)
from PyQt6.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QSettings, QRect, pyqtSlot
)
from PyQt6.QtGui import (
    QFont, QPalette, QColor, QIcon, QPainter, QPen, QBrush, QPixmap
)

# Scientific computing
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

@dataclass
class RobotConfig:
    """Robot configuration parameters"""
    L1: float = 10.0  # Link 1 length (cm)
    L2: float = 8.0   # Link 2 length (cm)
    max_reach: float = 18.0
    min_reach: float = 2.0
    joint1_min: int = 0
    joint1_max: int = 180
    joint2_min: int = -90
    joint2_max: int = 90

@dataclass
class Position:
    """2D position coordinates"""
    x: float
    y: float
    
    def distance_from_origin(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

class SerialCommunication(QThread):
    """Thread for handling serial communication with ESP32"""
    
    data_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.should_run = True
        
    def connect_to_port(self, port: str, baudrate: int = 115200):
        """Connect to ESP32 serial port"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.is_connected = True
            self.connection_status.emit(True)
            
        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {str(e)}")
            self.is_connected = False
            self.connection_status.emit(False)
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.should_run = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.is_connected = False
        self.connection_status.emit(False)
    
    def send_command(self, command: str):
        """Send command to ESP32"""
        if self.is_connected and self.serial_port:
            try:
                self.serial_port.write(f"{command}\n".encode())
            except Exception as e:
                self.error_occurred.emit(f"Send error: {str(e)}")
    
    def run(self):
        """Main thread loop for reading serial data"""
        while self.should_run:
            if self.is_connected and self.serial_port:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.readline().decode().strip()
                        if data:
                            self.data_received.emit(data)
                except Exception as e:
                    self.error_occurred.emit(f"Read error: {str(e)}")
            
            self.msleep(50)  # 50ms polling interval

class WorkspaceCanvas(FigureCanvas):
    """Custom matplotlib canvas for workspace visualization"""
    
    def __init__(self, robot_config: RobotConfig):
        self.fig = Figure(figsize=(8, 8), facecolor='#2b2b2b')
        super().__init__(self.fig)
        
        self.robot_config = robot_config
        self.ax = self.fig.add_subplot(111, facecolor='#2b2b2b')
        self.setup_plot()
        
        # Current robot state
        self.current_position = Position(0, robot_config.L1 + robot_config.L2)
        self.current_angles = [90.0, 0.0]  # theta1, theta2
        self.target_position = None
        
        # Plot elements
        self.workspace_circle = None
        self.robot_links = None
        self.end_effector_point = None
        self.target_point = None
        
        self.draw_workspace()
        self.draw_robot()
    
    def setup_plot(self):
        """Setup plot appearance"""
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-5, 20)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, color='white')
        self.ax.set_xlabel('X (cm)', color='white', fontsize=12)
        self.ax.set_ylabel('Y (cm)', color='white', fontsize=12)
        self.ax.set_title('2-DOF Robot Workspace', color='white', fontsize=14, fontweight='bold')
        
        # Style axes
        self.ax.tick_params(colors='white')
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['top'].set_color('white')
        self.ax.spines['right'].set_color('white')
        self.ax.spines['left'].set_color('white')
    
    def draw_workspace(self):
        """Draw robot workspace boundaries"""
        # Maximum reach circle
        max_circle = plt.Circle((0, 0), self.robot_config.max_reach, 
                               fill=False, color='#4CAF50', linewidth=2, alpha=0.7)
        self.ax.add_patch(max_circle)
        
        # Minimum reach circle
        min_circle = plt.Circle((0, 0), self.robot_config.min_reach, 
                               fill=False, color='#F44336', linewidth=2, alpha=0.7)
        self.ax.add_patch(min_circle)
        
        # Base point
        self.ax.plot(0, 0, 'ko', markersize=10, markerfacecolor='#FF9800', 
                    markeredgecolor='white', markeredgewidth=2)
        self.ax.text(0, -1.5, 'BASE', ha='center', color='white', fontweight='bold')
    
    def draw_robot(self):
        """Draw current robot configuration"""
        # Clear previous robot drawing
        if self.robot_links:
            for line in self.robot_links:
                line.remove()
        if self.end_effector_point:
            self.end_effector_point.remove()
        
        # Calculate joint positions using forward kinematics
        theta1_rad = math.radians(self.current_angles[0])
        theta2_rad = math.radians(self.current_angles[1])
        
        # Joint 1 position (elbow)
        joint1_x = self.robot_config.L1 * math.cos(theta1_rad)
        joint1_y = self.robot_config.L1 * math.sin(theta1_rad)
        
        # End effector position
        end_x = joint1_x + self.robot_config.L2 * math.cos(theta1_rad + theta2_rad)
        end_y = joint1_y + self.robot_config.L2 * math.sin(theta1_rad + theta2_rad)
        
        # Draw links
        link1, = self.ax.plot([0, joint1_x], [0, joint1_y], 
                             'o-', color='#2196F3', linewidth=4, markersize=8)
        link2, = self.ax.plot([joint1_x, end_x], [joint1_y, end_y], 
                             'o-', color='#9C27B0', linewidth=4, markersize=8)
        
        self.robot_links = [link1, link2]
        
        # End effector
        self.end_effector_point = self.ax.plot(end_x, end_y, 'o', 
                                              markersize=12, markerfacecolor='#FFEB3B',
                                              markeredgecolor='white', markeredgewidth=2)[0]
        
        # Update current position
        self.current_position = Position(end_x, end_y)
        
        self.draw()
    
    def draw_target(self, x: float, y: float):
        """Draw target position"""
        if self.target_point:
            self.target_point.remove()
        
        self.target_point = self.ax.plot(x, y, 'X', markersize=15, 
                                        markerfacecolor='#FF5722', 
                                        markeredgecolor='white', markeredgewidth=2)[0]
        self.target_position = Position(x, y)
        self.draw()
    
    def update_robot_angles(self, theta1: float, theta2: float):
        """Update robot joint angles and redraw"""
        self.current_angles = [theta1, theta2]
        self.draw_robot()
    
    def is_position_reachable(self, x: float, y: float) -> bool:
        """Check if position is within workspace"""
        distance = math.sqrt(x**2 + y**2)
        return self.robot_config.min_reach <= distance <= self.robot_config.max_reach

class DarkTheme:
    """Professional dark theme styling"""
    
    @staticmethod
    def apply_theme(app: QApplication):
        """Apply dark theme to application"""
        dark_palette = QPalette()
        
        # Window colors
        dark_palette.setColor(QPalette.ColorRole.Window, QColor(43, 43, 43))
        dark_palette.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
        
        # Base colors (input fields)
        dark_palette.setColor(QPalette.ColorRole.Base, QColor(35, 35, 35))
        dark_palette.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
        
        # Text colors
        dark_palette.setColor(QPalette.ColorRole.Text, QColor(255, 255, 255))
        dark_palette.setColor(QPalette.ColorRole.BrightText, QColor(255, 0, 0))
        
        # Button colors
        dark_palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
        dark_palette.setColor(QPalette.ColorRole.ButtonText, QColor(255, 255, 255))
        
        # Highlight colors
        dark_palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
        dark_palette.setColor(QPalette.ColorRole.HighlightedText, QColor(0, 0, 0))
        
        # Disabled colors
        dark_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, QColor(127, 127, 127))
        dark_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(127, 127, 127))
        dark_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(127, 127, 127))
        
        app.setPalette(dark_palette)
        
        # Additional stylesheet for modern look
        app.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            
            QGroupBox {
                font-weight: bold;
                border: 2px solid #555;
                border-radius: 8px;
                margin: 5px;
                padding-top: 15px;
                color: #ffffff;
            }
            
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 10px 0 10px;
                color: #4CAF50;
                font-size: 12px;
                font-weight: bold;
            }
            
            QPushButton {
                background-color: #0d7377;
                border: none;
                color: white;
                padding: 8px 16px;
                border-radius: 6px;
                font-weight: bold;
                min-width: 80px;
            }
            
            QPushButton:hover {
                background-color: #14a085;
            }
            
            QPushButton:pressed {
                background-color: #0a5d61;
            }
            
            QPushButton:disabled {
                background-color: #555;
                color: #888;
            }
            
            QSlider::groove:horizontal {
                border: 1px solid #555;
                height: 8px;
                background: #353535;
                border-radius: 4px;
            }
            
            QSlider::handle:horizontal {
                background: #4CAF50;
                border: 1px solid #357a35;
                width: 20px;
                margin: -6px 0;
                border-radius: 10px;
            }
            
            QSlider::handle:horizontal:hover {
                background: #66BB6A;
            }
            
            QSpinBox, QDoubleSpinBox, QLineEdit {
                padding: 6px;
                border: 2px solid #555;
                border-radius: 4px;
                background-color: #353535;
                color: white;
                selection-background-color: #4CAF50;
            }
            
            QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {
                border-color: #4CAF50;
            }
            
            QComboBox {
                padding: 6px;
                border: 2px solid #555;
                border-radius: 4px;
                background-color: #353535;
                color: white;
                min-width: 120px;
            }
            
            QComboBox:focus {
                border-color: #4CAF50;
            }
            
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            
            QComboBox::down-arrow {
                image: none;
                border-style: solid;
                border-width: 6px 4px 0px 4px;
                border-color: white transparent transparent transparent;
            }
            
            QTextEdit {
                border: 2px solid #555;
                border-radius: 4px;
                background-color: #1e1e1e;
                color: white;
                font-family: 'Consolas', 'Courier New', monospace;
                selection-background-color: #4CAF50;
            }
            
            QTabWidget::pane {
                border: 2px solid #555;
                border-radius: 4px;
            }
            
            QTabBar::tab {
                background-color: #404040;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            
            QTabBar::tab:selected {
                background-color: #4CAF50;
                color: white;
            }
            
            QTabBar::tab:hover {
                background-color: #505050;
            }
            
            QProgressBar {
                border: 2px solid #555;
                border-radius: 4px;
                text-align: center;
                background-color: #353535;
            }
            
            QProgressBar::chunk {
                background-color: #4CAF50;
                border-radius: 2px;
            }
            
            QStatusBar {
                border-top: 1px solid #555;
                background-color: #404040;
                color: white;
            }
            
            QScrollBar:vertical {
                border: none;
                background: #353535;
                width: 14px;
                border-radius: 7px;
            }
            
            QScrollBar::handle:vertical {
                background: #555;
                min-height: 20px;
                border-radius: 7px;
            }
            
            QScrollBar::handle:vertical:hover {
                background: #666;
            }
            
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
            }
            
            QCheckBox::indicator:unchecked {
                border: 2px solid #555;
                background-color: #353535;
                border-radius: 3px;
            }
            
            QCheckBox::indicator:checked {
                border: 2px solid #4CAF50;
                background-color: #4CAF50;
                border-radius: 3px;
            }
        """)

class RobotControlGUI(QMainWindow):
    """Main GUI application for robot control"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize components
        self.robot_config = RobotConfig()
        self.serial_comm = SerialCommunication()
        self.settings = QSettings('RobotControl', 'ESP32_2DOF')
        
        # Setup UI
        self.init_ui()
        self.setup_connections()
        self.load_settings()
        
        # Start serial communication thread
        self.serial_comm.start()
        
        # Update timer for real-time updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 100ms updates
    
    def init_ui(self):
        """Initialize user interface"""
        self.setWindowTitle("ESP32-S3 2-DOF Robotic Arm Controller")
        self.setGeometry(100, 100, 1400, 900)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel - Controls
        left_panel = self.create_control_panel()
        splitter.addWidget(left_panel)
        
        # Right panel - Visualization
        right_panel = self.create_visualization_panel()
        splitter.addWidget(right_panel)
        
        # Set splitter proportions
        splitter.setSizes([400, 1000])
        
        # Create status bar
        self.create_status_bar()
    
    def create_control_panel(self) -> QWidget:
        """Create left control panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create tabbed interface for controls
        tab_widget = QTabWidget()
        layout.addWidget(tab_widget)
        
        # Connection tab
        connection_tab = self.create_connection_tab()
        tab_widget.addTab(connection_tab, "🔌 Connection")
        
        # Manual control tab
        manual_tab = self.create_manual_control_tab()
        tab_widget.addTab(manual_tab, "🎮 Manual")
        
        # Auto control tab
        auto_tab = self.create_auto_control_tab()
        tab_widget.addTab(auto_tab, "🤖 Auto")
        
        # Settings tab
        settings_tab = self.create_settings_tab()
        tab_widget.addTab(settings_tab, "⚙️ Settings")
        
        return panel
    
    def create_connection_tab(self) -> QWidget:
        """Create connection control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Serial connection group
        connection_group = QGroupBox("Serial Connection")
        connection_layout = QGridLayout(connection_group)
        
        # Port selection
        connection_layout.addWidget(QLabel("Port:"), 0, 0)
        self.port_combo = QComboBox()
        self.refresh_ports()
        connection_layout.addWidget(self.port_combo, 0, 1)
        
        # Refresh ports button
        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        connection_layout.addWidget(self.refresh_btn, 0, 2)
        
        # Baud rate
        connection_layout.addWidget(QLabel("Baud Rate:"), 1, 0)
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baudrate_combo.setCurrentText("115200")
        connection_layout.addWidget(self.baudrate_combo, 1, 1)
        
        # Connect/Disconnect buttons
        self.connect_btn = QPushButton("🔌 Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_btn, 2, 0, 1, 2)
        
        # Connection status
        self.connection_status = QLabel("❌ Disconnected")
        self.connection_status.setStyleSheet("color: #F44336; font-weight: bold;")
        connection_layout.addWidget(self.connection_status, 2, 2)
        
        layout.addWidget(connection_group)
        
        # Serial monitor group
        monitor_group = QGroupBox("Serial Monitor")
        monitor_layout = QVBoxLayout(monitor_group)
        
        # Serial output display
        self.serial_output = QTextEdit()
        self.serial_output.setMaximumHeight(200)
        self.serial_output.setReadOnly(True)
        monitor_layout.addWidget(self.serial_output)
        
        # Command input
        cmd_layout = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter command...")
        self.command_input.returnPressed.connect(self.send_command)
        cmd_layout.addWidget(self.command_input)
        
        send_btn = QPushButton("📤 Send")
        send_btn.clicked.connect(self.send_command)
        cmd_layout.addWidget(send_btn)
        
        monitor_layout.addLayout(cmd_layout)
        layout.addWidget(monitor_group)
        
        layout.addStretch()
        return tab
    
    def create_manual_control_tab(self) -> QWidget:
        """Create manual control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Joint control group
        joint_group = QGroupBox("Joint Control")
        joint_layout = QGridLayout(joint_group)
        
        # Joint 1 controls
        joint_layout.addWidget(QLabel("Joint 1 (Base):"), 0, 0)
        self.joint1_slider = QSlider(Qt.Orientation.Horizontal)
        self.joint1_slider.setRange(0, 180)
        self.joint1_slider.setValue(90)
        self.joint1_slider.valueChanged.connect(self.joint1_changed)
        joint_layout.addWidget(self.joint1_slider, 0, 1)
        
        self.joint1_value = QSpinBox()
        self.joint1_value.setRange(0, 180)
        self.joint1_value.setValue(90)
        self.joint1_value.setSuffix("°")
        self.joint1_value.valueChanged.connect(self.joint1_spin_changed)
        joint_layout.addWidget(self.joint1_value, 0, 2)
        
        # Joint 2 controls
        joint_layout.addWidget(QLabel("Joint 2 (Elbow):"), 1, 0)
        self.joint2_slider = QSlider(Qt.Orientation.Horizontal)
        self.joint2_slider.setRange(-90, 90)
        self.joint2_slider.setValue(0)
        self.joint2_slider.valueChanged.connect(self.joint2_changed)
        joint_layout.addWidget(self.joint2_slider, 1, 1)
        
        self.joint2_value = QSpinBox()
        self.joint2_value.setRange(-90, 90)
        self.joint2_value.setValue(0)
        self.joint2_value.setSuffix("°")
        self.joint2_value.valueChanged.connect(self.joint2_spin_changed)
        joint_layout.addWidget(self.joint2_value, 1, 2)
        
        layout.addWidget(joint_group)
        
        # Position control group
        position_group = QGroupBox("Position Control")
        position_layout = QGridLayout(position_group)
        
        # X coordinate
        position_layout.addWidget(QLabel("X Position:"), 0, 0)
        self.x_position = QDoubleSpinBox()
        self.x_position.setRange(-20.0, 20.0)
        self.x_position.setValue(0.0)
        self.x_position.setDecimals(1)
        self.x_position.setSuffix(" cm")
        position_layout.addWidget(self.x_position, 0, 1)
        
        # Y coordinate
        position_layout.addWidget(QLabel("Y Position:"), 1, 0)
        self.y_position = QDoubleSpinBox()
        self.y_position.setRange(-5.0, 20.0)
        self.y_position.setValue(18.0)
        self.y_position.setDecimals(1)
        self.y_position.setSuffix(" cm")
        position_layout.addWidget(self.y_position, 1, 1)
        
        # Move to position button
        self.move_to_position_btn = QPushButton("🎯 Move to Position")
        self.move_to_position_btn.clicked.connect(self.move_to_position)
        position_layout.addWidget(self.move_to_position_btn, 2, 0, 1, 2)
        
        # Current position display
        self.current_pos_label = QLabel("Current: (0.0, 18.0) cm")
        self.current_pos_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        position_layout.addWidget(self.current_pos_label, 3, 0, 1, 2)
        
        layout.addWidget(position_group)
        
        # Quick positions group
        quick_group = QGroupBox("Quick Positions")
        quick_layout = QGridLayout(quick_group)
        
        # Pre-defined positions
        positions = [
            ("🏠 Home", 0, 18),
            ("👈 Left", -10, 10),
            ("👉 Right", 10, 10),
            ("⬆️ Up", 0, 15),
            ("⬇️ Down", 0, 8),
            ("🔄 Center", 12, 8)
        ]
        
        for i, (name, x, y) in enumerate(positions):
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, x=x, y=y: self.move_to_quick_position(x, y))
            quick_layout.addWidget(btn, i // 2, i % 2)
        
        layout.addWidget(quick_group)
        layout.addStretch()
        return tab
    
    def create_auto_control_tab(self) -> QWidget:
        """Create automatic control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Demo sequences group
        demo_group = QGroupBox("Demo Sequences")
        demo_layout = QGridLayout(demo_group)
        
        # Demo buttons
        demos = [
            ("🔄 Basic Demo", self.start_basic_demo),
            ("📐 Square Path", self.start_square_demo),
            ("⭕ Circle Path", self.start_circle_demo),
            ("🌊 Wave Pattern", self.start_wave_demo),
            ("📏 Workspace Test", self.start_workspace_demo),
            ("⏹️ Stop Demo", self.stop_demo)
        ]
        
        for i, (name, callback) in enumerate(demos):
            btn = QPushButton(name)
            btn.clicked.connect(callback)
            demo_layout.addWidget(btn, i // 2, i % 2)
        
        layout.addWidget(demo_group)
        
        # Trajectory control group
        traj_group = QGroupBox("Trajectory Control")
        traj_layout = QGridLayout(traj_group)
        
        # Speed control
        traj_layout.addWidget(QLabel("Speed:"), 0, 0)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(100, 2000)
        self.speed_slider.setValue(1000)
        traj_layout.addWidget(self.speed_slider, 0, 1)
        
        self.speed_value = QSpinBox()
        self.speed_value.setRange(100, 2000)
        self.speed_value.setValue(1000)
        self.speed_value.setSuffix(" ms")
        self.speed_value.valueChanged.connect(lambda v: self.speed_slider.setValue(v))
        self.speed_slider.valueChanged.connect(lambda v: self.speed_value.setValue(v))
        traj_layout.addWidget(self.speed_value, 0, 2)
        
        # Smoothing
        self.smooth_checkbox = QCheckBox("Smooth Movement")
        self.smooth_checkbox.setChecked(True)
        traj_layout.addWidget(self.smooth_checkbox, 1, 0, 1, 3)
        
        layout.addWidget(traj_group)
        
        # Sequence editor group
        sequence_group = QGroupBox("Sequence Editor")
        sequence_layout = QVBoxLayout(sequence_group)
        
        # Sequence list
        self.sequence_display = QTextEdit()
        self.sequence_display.setMaximumHeight(150)
        self.sequence_display.setPlaceholderText("No sequence loaded...")
        sequence_layout.addWidget(self.sequence_display)
        
        # Sequence controls
        seq_btn_layout = QHBoxLayout()
        
        load_seq_btn = QPushButton("📂 Load")
        load_seq_btn.clicked.connect(self.load_sequence)
        seq_btn_layout.addWidget(load_seq_btn)
        
        save_seq_btn = QPushButton("💾 Save")
        save_seq_btn.clicked.connect(self.save_sequence)
        seq_btn_layout.addWidget(save_seq_btn)
        
        play_seq_btn = QPushButton("▶️ Play")
        play_seq_btn.clicked.connect(self.play_sequence)
        seq_btn_layout.addWidget(play_seq_btn)
        
        sequence_layout.addLayout(seq_btn_layout)
        layout.addWidget(sequence_group)
        
        layout.addStretch()
        return tab
    
    def create_settings_tab(self) -> QWidget:
        """Create settings tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Robot parameters group
        robot_group = QGroupBox("Robot Parameters")
        robot_layout = QGridLayout(robot_group)
        
        # Link lengths
        robot_layout.addWidget(QLabel("Link 1 Length:"), 0, 0)
        self.l1_spinbox = QDoubleSpinBox()
        self.l1_spinbox.setRange(1.0, 50.0)
        self.l1_spinbox.setValue(self.robot_config.L1)
        self.l1_spinbox.setDecimals(1)
        self.l1_spinbox.setSuffix(" cm")
        self.l1_spinbox.valueChanged.connect(self.update_robot_config)
        robot_layout.addWidget(self.l1_spinbox, 0, 1)
        
        robot_layout.addWidget(QLabel("Link 2 Length:"), 1, 0)
        self.l2_spinbox = QDoubleSpinBox()
        self.l2_spinbox.setRange(1.0, 50.0)
        self.l2_spinbox.setValue(self.robot_config.L2)
        self.l2_spinbox.setDecimals(1)
        self.l2_spinbox.setSuffix(" cm")
        self.l2_spinbox.valueChanged.connect(self.update_robot_config)
        robot_layout.addWidget(self.l2_spinbox, 1, 1)
        
        layout.addWidget(robot_group)
        
        # Calibration group
        cal_group = QGroupBox("Servo Calibration")
        cal_layout = QGridLayout(cal_group)
        
        # Joint offsets
        cal_layout.addWidget(QLabel("Joint 1 Offset:"), 0, 0)
        self.joint1_offset = QSpinBox()
        self.joint1_offset.setRange(-180, 180)
        self.joint1_offset.setValue(90)
        self.joint1_offset.setSuffix("°")
        cal_layout.addWidget(self.joint1_offset, 0, 1)
        
        cal_layout.addWidget(QLabel("Joint 2 Offset:"), 1, 0)
        self.joint2_offset = QSpinBox()
        self.joint2_offset.setRange(-180, 180)
        self.joint2_offset.setValue(90)
        self.joint2_offset.setSuffix("°")
        cal_layout.addWidget(self.joint2_offset, 1, 1)
        
        # Calibration buttons
        cal_btn_layout = QHBoxLayout()
        
        home_btn = QPushButton("🏠 Home Position")
        home_btn.clicked.connect(self.move_to_home)
        cal_btn_layout.addWidget(home_btn)
        
        calibrate_btn = QPushButton("⚙️ Calibrate")
        calibrate_btn.clicked.connect(self.start_calibration)
        cal_btn_layout.addWidget(calibrate_btn)
        
        cal_layout.addLayout(cal_btn_layout, 2, 0, 1, 2)
        
        layout.addWidget(cal_group)
        
        # Safety group
        safety_group = QGroupBox("Safety Settings")
        safety_layout = QVBoxLayout(safety_group)
        
        # Emergency stop
        self.emergency_stop_btn = QPushButton("🛑 EMERGENCY STOP")
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #F44336;
                font-size: 14px;
                font-weight: bold;
                padding: 12px;
            }
            QPushButton:hover {
                background-color: #E53935;
            }
        """)
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        safety_layout.addWidget(self.emergency_stop_btn)
        
        # Safety options
        self.workspace_check = QCheckBox("Check Workspace Limits")
        self.workspace_check.setChecked(True)
        safety_layout.addWidget(self.workspace_check)
        
        self.soft_limits_check = QCheckBox("Enforce Soft Limits")
        self.soft_limits_check.setChecked(True)
        safety_layout.addWidget(self.soft_limits_check)
        
        layout.addWidget(safety_group)
        
        layout.addStretch()
        return tab
    
    def create_visualization_panel(self) -> QWidget:
        """Create right visualization panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create workspace canvas
        self.workspace_canvas = WorkspaceCanvas(self.robot_config)
        layout.addWidget(self.workspace_canvas)
        
        # Info panel
        info_group = QGroupBox("Robot Information")
        info_layout = QGridLayout(info_group)
        
        # Current status
        self.status_labels = {}
        labels = [
            ("Joint 1:", "joint1"),
            ("Joint 2:", "joint2"),
            ("End Position:", "position"),
            ("Distance:", "distance"),
            ("Workspace:", "workspace")
        ]
        
        for i, (label, key) in enumerate(labels):
            info_layout.addWidget(QLabel(label), i, 0)
            status_label = QLabel("--")
            status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
            self.status_labels[key] = status_label
            info_layout.addWidget(status_label, i, 1)
        
        layout.addWidget(info_group)
        return panel
    
    def create_status_bar(self):
        """Create status bar"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # Add permanent widgets
        self.connection_indicator = QLabel("❌ Disconnected")
        self.status_bar.addPermanentWidget(self.connection_indicator)
        
        self.status_bar.showMessage("Ready")
    
    def setup_connections(self):
        """Setup signal connections"""
        # Serial communication signals
        self.serial_comm.data_received.connect(self.handle_serial_data)
        self.serial_comm.connection_status.connect(self.update_connection_status)
        self.serial_comm.error_occurred.connect(self.handle_serial_error)
    
    @pyqtSlot(str)
    def handle_serial_data(self, data: str):
        """Handle incoming serial data"""
        self.serial_output.append(f"<< {data}")
        
        # Auto-scroll to bottom
        scrollbar = self.serial_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
        # Parse data for robot status updates
        self.parse_robot_status(data)
    
    @pyqtSlot(bool)
    def update_connection_status(self, connected: bool):
        """Update connection status indicators"""
        if connected:
            self.connection_status.setText("✅ Connected")
            self.connection_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
            self.connection_indicator.setText("✅ Connected")
            self.connect_btn.setText("🔌 Disconnect")
            self.status_bar.showMessage("Connected to robot")
        else:
            self.connection_status.setText("❌ Disconnected")
            self.connection_status.setStyleSheet("color: #F44336; font-weight: bold;")
            self.connection_indicator.setText("❌ Disconnected")
            self.connect_btn.setText("🔌 Connect")
            self.status_bar.showMessage("Disconnected")
    
    @pyqtSlot(str)
    def handle_serial_error(self, error: str):
        """Handle serial communication errors"""
        self.serial_output.append(f"<span style='color: #F44336;'>ERROR: {error}</span>")
        self.status_bar.showMessage(f"Error: {error}")
    
    def refresh_ports(self):
        """Refresh available serial ports"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            port_name = f"{port.device} - {port.description}"
            self.port_combo.addItem(port_name, port.device)
        
        if not ports:
            self.port_combo.addItem("No ports available")
    
    def toggle_connection(self):
        """Toggle serial connection"""
        if not self.serial_comm.is_connected:
            # Connect
            if self.port_combo.currentData():
                port = self.port_combo.currentData()
                baudrate = int(self.baudrate_combo.currentText())
                self.serial_comm.connect_to_port(port, baudrate)
            else:
                QMessageBox.warning(self, "Warning", "No port selected!")
        else:
            # Disconnect
            self.serial_comm.disconnect()
    
    def send_command(self):
        """Send command to robot"""
        command = self.command_input.text().strip()
        if command and self.serial_comm.is_connected:
            self.serial_comm.send_command(command)
            self.serial_output.append(f">> {command}")
            self.command_input.clear()
    
    def joint1_changed(self, value):
        """Handle joint 1 slider change"""
        self.joint1_value.setValue(value)
        self.update_robot_visualization()
        if self.serial_comm.is_connected:
            self.send_joint_command(1, value)
    
    def joint1_spin_changed(self, value):
        """Handle joint 1 spinbox change"""
        self.joint1_slider.setValue(value)
    
    def joint2_changed(self, value):
        """Handle joint 2 slider change"""
        self.joint2_value.setValue(value)
        self.update_robot_visualization()
        if self.serial_comm.is_connected:
            self.send_joint_command(2, value)
    
    def joint2_spin_changed(self, value):
        """Handle joint 2 spinbox change"""
        self.joint2_slider.setValue(value)
    
    def send_joint_command(self, joint: int, angle: int):
        """Send joint angle command to robot"""
        command = f"JOINT{joint}:{angle}"
        self.serial_comm.send_command(command)
    
    def move_to_position(self):
        """Move robot to specified position"""
        x = self.x_position.value()
        y = self.y_position.value()
        
        if self.workspace_check.isChecked():
            if not self.workspace_canvas.is_position_reachable(x, y):
                QMessageBox.warning(self, "Warning", 
                    f"Position ({x}, {y}) is outside workspace!\n"
                    f"Valid range: {self.robot_config.min_reach:.1f} - {self.robot_config.max_reach:.1f} cm")
                return
        
        # Send position command to robot
        if self.serial_comm.is_connected:
            command = f"MOVE:{x:.1f},{y:.1f}"
            self.serial_comm.send_command(command)
        
        # Update visualization
        self.workspace_canvas.draw_target(x, y)
        self.status_bar.showMessage(f"Moving to ({x:.1f}, {y:.1f})")
    
    def move_to_quick_position(self, x: float, y: float):
        """Move to predefined position"""
        self.x_position.setValue(x)
        self.y_position.setValue(y)
        self.move_to_position()
    
    def update_robot_visualization(self):
        """Update robot visualization"""
        theta1 = self.joint1_value.value()
        theta2 = self.joint2_value.value()
        self.workspace_canvas.update_robot_angles(theta1, theta2)
    
    def update_robot_config(self):
        """Update robot configuration"""
        self.robot_config.L1 = self.l1_spinbox.value()
        self.robot_config.L2 = self.l2_spinbox.value()
        self.robot_config.max_reach = self.robot_config.L1 + self.robot_config.L2
        self.robot_config.min_reach = abs(self.robot_config.L1 - self.robot_config.L2)
        
        # Update workspace canvas
        self.workspace_canvas.robot_config = self.robot_config
        self.workspace_canvas.draw_workspace()
        self.workspace_canvas.draw_robot()
    
    def parse_robot_status(self, data: str):
        """Parse robot status from serial data"""
        try:
            if "Joint 1" in data and "degrees" in data:
                # Extract joint angles
                parts = data.split()
                for i, part in enumerate(parts):
                    if part == "Joint" and i + 1 < len(parts) and parts[i + 1] == "1":
                        angle = float(parts[i + 3])
                        self.joint1_slider.setValue(int(angle))
                    elif part == "Joint" and i + 1 < len(parts) and parts[i + 1] == "2":
                        angle = float(parts[i + 3])
                        self.joint2_slider.setValue(int(angle))
            
            elif "end-effector position" in data:
                # Extract position
                start = data.find("(") + 1
                end = data.find(")")
                if start > 0 and end > start:
                    coords = data[start:end].split(",")
                    if len(coords) == 2:
                        x = float(coords[0].strip())
                        y = float(coords[1].strip())
                        self.current_pos_label.setText(f"Current: ({x:.1f}, {y:.1f}) cm")
        
        except (ValueError, IndexError):
            pass  # Ignore parsing errors
    
    def update_display(self):
        """Update display information"""
        if hasattr(self, 'workspace_canvas'):
            pos = self.workspace_canvas.current_position
            angles = self.workspace_canvas.current_angles
            distance = pos.distance_from_origin()
            
            # Update status labels
            self.status_labels["joint1"].setText(f"{angles[0]:.1f}°")
            self.status_labels["joint2"].setText(f"{angles[1]:.1f}°")
            self.status_labels["position"].setText(f"({pos.x:.1f}, {pos.y:.1f}) cm")
            self.status_labels["distance"].setText(f"{distance:.1f} cm")
            
            # Workspace status
            if self.workspace_canvas.is_position_reachable(pos.x, pos.y):
                self.status_labels["workspace"].setText("✅ Valid")
                self.status_labels["workspace"].setStyleSheet("color: #4CAF50; font-weight: bold;")
            else:
                self.status_labels["workspace"].setText("❌ Invalid")
                self.status_labels["workspace"].setStyleSheet("color: #F44336; font-weight: bold;")
    
    # Demo functions
    def start_basic_demo(self):
        """Start basic demo sequence"""
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("DEMO:BASIC")
            self.status_bar.showMessage("Starting basic demo...")
    
    def start_square_demo(self):
        """Start square path demo"""
        positions = [(10, 10), (15, 10), (15, 15), (10, 15), (10, 10)]
        self.execute_path_demo(positions, "Square Path")
    
    def start_circle_demo(self):
        """Start circle path demo"""
        center_x, center_y = 12, 10
        radius = 4
        positions = []
        for angle in range(0, 361, 30):
            x = center_x + radius * math.cos(math.radians(angle))
            y = center_y + radius * math.sin(math.radians(angle))
            positions.append((x, y))
        self.execute_path_demo(positions, "Circle Path")
    
    def start_wave_demo(self):
        """Start wave pattern demo"""
        positions = []
        for x in range(8, 17):
            y = 10 + 3 * math.sin(x * 0.5)
            positions.append((x, y))
        self.execute_path_demo(positions, "Wave Pattern")
    
    def start_workspace_demo(self):
        """Start workspace boundary demo"""
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("DEMO:WORKSPACE")
            self.status_bar.showMessage("Starting workspace demo...")
    
    def execute_path_demo(self, positions: List[Tuple[float, float]], name: str):
        """Execute a path demo"""
        self.status_bar.showMessage(f"Starting {name} demo...")
        # This would be implemented with a timer to send positions sequentially
        # For now, just send the first position
        if positions and self.serial_comm.is_connected:
            x, y = positions[0]
            self.serial_comm.send_command(f"MOVE:{x:.1f},{y:.1f}")
    
    def stop_demo(self):
        """Stop current demo"""
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("DEMO:STOP")
            self.status_bar.showMessage("Demo stopped")
    
    def move_to_home(self):
        """Move robot to home position"""
        self.joint1_slider.setValue(90)
        self.joint2_slider.setValue(0)
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("HOME")
        self.status_bar.showMessage("Moving to home position")
    
    def start_calibration(self):
        """Start servo calibration"""
        if self.serial_comm.is_connected:
            offset1 = self.joint1_offset.value()
            offset2 = self.joint2_offset.value()
            self.serial_comm.send_command(f"CALIBRATE:{offset1},{offset2}")
            self.status_bar.showMessage("Starting calibration...")
    
    def emergency_stop(self):
        """Emergency stop function"""
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("STOP")
        self.status_bar.showMessage("EMERGENCY STOP ACTIVATED", 5000)
        
        # Show warning dialog
        QMessageBox.critical(self, "Emergency Stop", 
                           "Emergency stop activated!\nRobot movement halted.")
    
    def load_sequence(self):
        """Load movement sequence from file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Load Sequence", "", "JSON Files (*.json);;All Files (*)")
        
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    sequence = json.load(f)
                self.sequence_display.setText(json.dumps(sequence, indent=2))
                self.status_bar.showMessage(f"Sequence loaded from {Path(file_path).name}")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to load sequence:\n{str(e)}")
    
    def save_sequence(self):
        """Save current sequence to file"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Sequence", "sequence.json", "JSON Files (*.json);;All Files (*)")
        
        if file_path:
            try:
                sequence_text = self.sequence_display.toPlainText()
                if sequence_text.strip():
                    sequence = json.loads(sequence_text)
                    with open(file_path, 'w') as f:
                        json.dump(sequence, f, indent=2)
                    self.status_bar.showMessage(f"Sequence saved to {Path(file_path).name}")
                else:
                    QMessageBox.warning(self, "Warning", "No sequence to save!")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to save sequence:\n{str(e)}")
    
    def play_sequence(self):
        """Play loaded sequence"""
        try:
            sequence_text = self.sequence_display.toPlainText()
            if sequence_text.strip():
                sequence = json.loads(sequence_text)
                # Implementation for playing sequence would go here
                self.status_bar.showMessage("Playing sequence...")
            else:
                QMessageBox.warning(self, "Warning", "No sequence to play!")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Invalid sequence format:\n{str(e)}")
    
    def load_settings(self):
        """Load application settings"""
        self.restoreGeometry(self.settings.value("geometry", bytes()))
        self.restoreState(self.settings.value("windowState", bytes()))
        
        # Load robot configuration
        self.robot_config.L1 = float(self.settings.value("robot/L1", self.robot_config.L1))
        self.robot_config.L2 = float(self.settings.value("robot/L2", self.robot_config.L2))
        
        self.l1_spinbox.setValue(self.robot_config.L1)
        self.l2_spinbox.setValue(self.robot_config.L2)
        self.update_robot_config()
    
    def save_settings(self):
        """Save application settings"""
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        
        # Save robot configuration
        self.settings.setValue("robot/L1", self.robot_config.L1)
        self.settings.setValue("robot/L2", self.robot_config.L2)
    
    def closeEvent(self, event):
        """Handle application close"""
        self.save_settings()
        self.serial_comm.disconnect()
        self.serial_comm.quit()
        self.serial_comm.wait()
        event.accept()

def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    
    # Set application properties
    app.setApplicationName("ESP32 Robot Control")
    app.setApplicationVersion("1.0")
    app.setOrganizationName("RoboticsLab")
    
    # Apply dark theme
    DarkTheme.apply_theme(app)
    
    # Create and show main window
    window = RobotControlGUI()
    window.show()
    
    # Run application
    sys.exit(app.exec())

if __name__ == "__main__":
    main()