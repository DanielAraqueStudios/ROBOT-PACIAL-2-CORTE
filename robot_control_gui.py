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
    Qt, QTimer, QThread, pyqtSignal, QSettings, QRect, pyqtSlot, QPoint
)
from PyQt6.QtGui import (
    QFont, QPalette, QColor, QIcon, QPainter, QPen, QBrush, QPixmap, QPolygonF
)

# Scientific computing
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

@dataclass
class RobotConfig:
    """Robot configuration parameters"""
    L1: float = 10.0  # Link 1 length (cm)
    L2: float = 10.0  # Link 2 length (cm) - UPDATED to match simple_kinematics.ino
    max_reach: float = 20.0  # UPDATED: L1 + L2 = 20.0
    min_reach: float = 0.0   # UPDATED: abs(L1 - L2) = 0.0
    joint1_min: int = 0
    joint1_max: int = 180
    joint2_min: int = 0      # UPDATED: Physical servo limitation
    joint2_max: int = 180     # UPDATED: Servo 2 full range 0-180° (NOT limited)

@dataclass
class Position:
    """2D position coordinates"""
    x: float
    y: float
    
    def distance_from_origin(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

class KinematicsCalculator:
    """Forward and inverse kinematics calculations"""
    
    @staticmethod
    def forward_kinematics(theta1_deg: float, theta2_deg: float, L1: float, L2: float) -> Position:
        """
        Calculate end-effector position from joint angles
        theta1: Base joint angle in degrees
        theta2: Elbow joint angle in degrees
        Returns: Position(x, y) in cm
        """
        theta1_rad = math.radians(theta1_deg)
        theta2_rad = math.radians(theta2_deg)
        
        x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
        y = L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad)
        
        return Position(x, y)
    
    @staticmethod
    def inverse_kinematics(target_x: float, target_y: float, L1: float, L2: float) -> tuple:
        """
        Calculate joint angles from target position
        Returns: (theta1_deg, theta2_deg, is_valid)
        """
        # Check if position is reachable
        distance = math.sqrt(target_x**2 + target_y**2)
        max_reach = L1 + L2
        min_reach = abs(L1 - L2)
        
        if distance > max_reach or distance < min_reach or target_y < 0:
            return (0, 0, False)
        
        # Calculate angles using inverse kinematics
        try:
            cos_theta2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
            if cos_theta2 < -1 or cos_theta2 > 1:
                return (0, 0, False)
                
            theta2_rad = math.acos(cos_theta2)
            alpha = math.atan2(target_y, target_x)
            beta = math.atan2(L2 * math.sin(theta2_rad), L1 + L2 * math.cos(theta2_rad))
            theta1_rad = alpha - beta
            
            theta1_deg = math.degrees(theta1_rad)
            theta2_deg = math.degrees(theta2_rad)
            
            # Normalize angles to 0-360 range, then to 0-180 for servos
            while theta1_deg < 0:
                theta1_deg += 360
            while theta1_deg >= 360:
                theta1_deg -= 360
                
            if theta1_deg > 180:
                theta1_deg = 360 - theta1_deg
                
            # Validate servo ranges
            if 0 <= theta1_deg <= 180 and 0 <= theta2_deg <= 180:
                return (theta1_deg, theta2_deg, True)
            else:
                return (0, 0, False)
                
        except:
            return (0, 0, False)

class DialControl(QWidget):
    """Custom circular dial/potentiometer control widget"""
    
    valueChanged = pyqtSignal(int)
    
    def __init__(self, minimum=0, maximum=180, value=90, title="Servo"):
        super().__init__()
        self.minimum = minimum
        self.maximum = maximum
        self._value = value
        self.title = title
        self.setFixedSize(120, 140)
        self.setMouseTracking(True)
        
    def value(self):
        return self._value
        
    def setValue(self, value):
        value = max(self.minimum, min(self.maximum, value))
        if value != self._value:
            self._value = value
            self.update()
            self.valueChanged.emit(value)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.updateValueFromMouse(event.pos())
    
    def mouseMoveEvent(self, event):
        if event.buttons() == Qt.MouseButton.LeftButton:
            self.updateValueFromMouse(event.pos())
    
    def updateValueFromMouse(self, pos):
        center = QPoint(60, 70)  # Center of the dial
        angle = math.atan2(pos.y() - center.y(), pos.x() - center.x())
        angle_deg = math.degrees(angle) + 90  # Adjust for vertical starting position
        
        # Normalize to 0-360
        if angle_deg < 0:
            angle_deg += 360
        
        # Convert to value range (0-180 degrees maps to 0-max)
        if angle_deg <= 180:
            new_value = int((angle_deg / 180) * (self.maximum - self.minimum) + self.minimum)
        else:
            # For angles > 180, map to the remaining range
            remaining_angle = angle_deg - 180
            new_value = int(((180 - remaining_angle) / 180) * (self.maximum - self.minimum) + self.minimum)
        
        self.setValue(new_value)
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Draw background circle
        painter.setPen(QPen(QColor("#555"), 3))
        painter.setBrush(QBrush(QColor("#2b2b2b")))
        painter.drawEllipse(10, 10, 100, 100)
        
        # Draw value arc
        painter.setPen(QPen(QColor("#4CAF50"), 4))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        
        # Calculate sweep angle based on value
        sweep_angle = int(((self._value - self.minimum) / (self.maximum - self.minimum)) * 180 * 16)
        painter.drawArc(10, 10, 100, 100, 90 * 16, -sweep_angle)
        
        # Draw indicator line
        angle = math.radians(((self._value - self.minimum) / (self.maximum - self.minimum)) * 180)
        center_x, center_y = 60, 60
        line_length = 35
        end_x = center_x + line_length * math.cos(math.pi/2 - angle)
        end_y = center_y - line_length * math.sin(math.pi/2 - angle)
        
        painter.setPen(QPen(QColor("#FF9800"), 3, Qt.PenStyle.SolidLine))
        painter.drawLine(center_x, center_y, int(end_x), int(end_y))
        
        # Draw center dot
        painter.setPen(QPen(QColor("#FF9800"), 2))
        painter.setBrush(QBrush(QColor("#FF9800")))
        painter.drawEllipse(55, 55, 10, 10)
        
        # Draw title and value
        painter.setPen(QColor("white"))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        painter.drawText(0, 130, 120, 20, Qt.AlignmentFlag.AlignCenter, self.title)
        
        painter.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        painter.drawText(0, 115, 120, 20, Qt.AlignmentFlag.AlignCenter, f"{self._value}°")

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
        self.current_angles = [90.0, 90.0]  # theta1, theta2 - FIXED: Both joints at center
        self.target_position = None
        
        # Plot elements
        self.workspace_circle = None
        self.robot_links = None
        self.end_effector_point = None
        self.target_point = None
        
        self.draw_workspace()
        self.draw_robot()
    
    def update_robot_state(self, theta1_deg, theta2_deg, end_x, end_y):
        """Update robot state with new angles and position"""
        self.current_angles = [theta1_deg, theta2_deg]
        self.current_position = Position(end_x, end_y)
        self.draw_robot()
        self.draw()
    
    def set_target_position(self, x, y):
        """Set target position for visualization"""
        self.target_position = Position(x, y)
        self.draw_robot()
        self.draw()
    
    def setup_plot(self):
        """Setup plot appearance - UPDATED for simple_kinematics.ino (max_reach=20, Y>=0)"""
        self.ax.set_xlim(-22, 22)  # Increased for max_reach=20
        self.ax.set_ylim(-1, 22)   # Changed to show Y>=0 with small negative margin for axis
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, color='white')
        self.ax.set_xlabel('X (cm)', color='white', fontsize=12)
        self.ax.set_ylabel('Y (cm) - Only Y ≥ 0 supported', color='white', fontsize=12)  # Added constraint note
        self.ax.set_title('2-DOF Robot Workspace (L1=10cm, L2=10cm)', color='white', fontsize=14, fontweight='bold')
        
        # Style axes
        self.ax.tick_params(colors='white')
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['top'].set_color('white')
        self.ax.spines['right'].set_color('white')
        self.ax.spines['left'].set_color('white')
        
        # Add Y=0 line to show constraint
        self.ax.axhline(y=0, color='red', linestyle='--', alpha=0.7, label='Y=0 limit')
    
    def draw_workspace(self):
        """Draw robot workspace boundaries"""
        # Maximum reach circle
        max_circle = patches.Circle((0, 0), self.robot_config.max_reach, 
                                  fill=False, color='#4CAF50', linewidth=2, alpha=0.7)
        self.ax.add_patch(max_circle)
        
        # Minimum reach circle
        min_circle = patches.Circle((0, 0), self.robot_config.min_reach, 
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
        
        # Tab 1: Connection + Visualization
        connection_tab = self.create_connection_tab()
        tab_widget.addTab(connection_tab, "🔌 Connection")
        
        # Tab 2: Manual Servo Control + Visualization
        servo_tab = self.create_servo_control_tab()
        tab_widget.addTab(servo_tab, "🎛️ Servo Control")
        
        # Tab 3: Inverse Kinematics + Visualization
        kinematics_tab = self.create_kinematics_tab()
        tab_widget.addTab(kinematics_tab, "📐 Kinematics")
        
        # Tab 4: Auto control (keep as is)
        auto_tab = self.create_auto_control_tab()
        tab_widget.addTab(auto_tab, "🤖 Auto")
        
        # Tab 5: Settings (keep as is)
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
        
        # Add mini workspace canvas for connection tab
        mini_canvas_group = QGroupBox("Robot Status Visualization")
        canvas_layout = QVBoxLayout(mini_canvas_group)
        self.connection_mini_canvas = WorkspaceCanvas(self.robot_config)
        self.connection_mini_canvas.setFixedSize(300, 300)
        canvas_layout.addWidget(self.connection_mini_canvas)
        layout.addWidget(mini_canvas_group)
        
        layout.addStretch()
        return tab
    
    def create_servo_control_tab(self) -> QWidget:
        """Create servo control tab with dial potentiometers"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Servo controls group
        servo_group = QGroupBox("Servo Controls")
        servo_layout = QHBoxLayout(servo_group)
        
        # Joint 1 dial control
        joint1_layout = QVBoxLayout()
        self.joint1_dial = DialControl(0, 180, 90, "Joint 1 (Base)")
        self.joint1_dial.valueChanged.connect(self.joint1_dial_changed)
        joint1_layout.addWidget(self.joint1_dial)
        servo_layout.addLayout(joint1_layout)
        
        # Joint 2 dial control  
        joint2_layout = QVBoxLayout()
        self.joint2_dial = DialControl(0, 180, 90, "Joint 2 (Elbow)")
        self.joint2_dial.valueChanged.connect(self.joint2_dial_changed)
        joint2_layout.addWidget(self.joint2_dial)
        servo_layout.addLayout(joint2_layout)
        
        layout.addWidget(servo_group)
        
        # Calculated position group
        position_group = QGroupBox("Calculated Position (Forward Kinematics)")
        position_layout = QGridLayout(position_group)
        
        # Position display
        self.servo_x_label = QLabel("X: 0.0 cm")
        self.servo_x_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 14px;")
        position_layout.addWidget(QLabel("X Position:"), 0, 0)
        position_layout.addWidget(self.servo_x_label, 0, 1)
        
        self.servo_y_label = QLabel("Y: 20.0 cm")
        self.servo_y_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 14px;")
        position_layout.addWidget(QLabel("Y Position:"), 1, 0)
        position_layout.addWidget(self.servo_y_label, 1, 1)
        
        self.servo_distance_label = QLabel("Distance: 20.0 cm")
        self.servo_distance_label.setStyleSheet("color: #2196F3; font-weight: bold; font-size: 14px;")
        position_layout.addWidget(QLabel("Distance from origin:"), 2, 0)
        position_layout.addWidget(self.servo_distance_label, 2, 1)
        
        layout.addWidget(position_group)
        
        # Quick servo positions
        quick_group = QGroupBox("Quick Servo Positions")
        quick_layout = QGridLayout(quick_group)
        
        servo_positions = [
            ("🏠 Home (90°, 90°)", 90, 90),
            ("⬆️ Straight Up (90°, 0°)", 90, 0),
            ("👈 Left (45°, 90°)", 45, 90),
            ("👉 Right (135°, 90°)", 135, 90),
            ("⬇️ Fold Down (90°, 180°)", 90, 180),
            ("🔄 Center (90°, 90°)", 90, 90)
        ]
        
        for i, (name, angle1, angle2) in enumerate(servo_positions):
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, a1=angle1, a2=angle2: self.set_servo_angles(a1, a2))
            quick_layout.addWidget(btn, i // 2, i % 2)
        
        layout.addWidget(quick_group)
        
        # Add mini workspace canvas for this tab
        mini_canvas_group = QGroupBox("Robot Position Visualization")
        canvas_layout = QVBoxLayout(mini_canvas_group)
        self.servo_mini_canvas = WorkspaceCanvas(self.robot_config)
        self.servo_mini_canvas.setFixedSize(300, 300)
        canvas_layout.addWidget(self.servo_mini_canvas)
        layout.addWidget(mini_canvas_group)
        
        layout.addStretch()
        return tab
    
    def create_kinematics_tab(self) -> QWidget:
        """Create inverse kinematics tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Coordinate input group
        coord_group = QGroupBox("Target Coordinates (Inverse Kinematics)")
        coord_layout = QGridLayout(coord_group)
        
        # X coordinate input
        coord_layout.addWidget(QLabel("X Position:"), 0, 0)
        self.ik_x_input = QDoubleSpinBox()
        self.ik_x_input.setRange(-20.0, 20.0)
        self.ik_x_input.setValue(0.0)
        self.ik_x_input.setDecimals(1)
        self.ik_x_input.setSuffix(" cm")
        self.ik_x_input.valueChanged.connect(self.calculate_inverse_kinematics)
        coord_layout.addWidget(self.ik_x_input, 0, 1)
        
        # Y coordinate input (only Y >= 0)
        coord_layout.addWidget(QLabel("Y Position:"), 1, 0)
        self.ik_y_input = QDoubleSpinBox()
        self.ik_y_input.setRange(0.0, 20.0)
        self.ik_y_input.setValue(15.0)
        self.ik_y_input.setDecimals(1)
        self.ik_y_input.setSuffix(" cm")
        self.ik_y_input.valueChanged.connect(self.calculate_inverse_kinematics)
        coord_layout.addWidget(self.ik_y_input, 1, 1)
        
        # Move button
        self.ik_move_btn = QPushButton("🎯 Move to Position")
        self.ik_move_btn.clicked.connect(self.move_to_ik_position)
        coord_layout.addWidget(self.ik_move_btn, 2, 0, 1, 2)
        
        layout.addWidget(coord_group)
        
        # Calculated angles group
        angles_group = QGroupBox("Calculated Joint Angles")
        angles_layout = QGridLayout(angles_group)
        
        self.ik_theta1_label = QLabel("θ1: 90.0°")
        self.ik_theta1_label.setStyleSheet("color: #FF9800; font-weight: bold; font-size: 14px;")
        angles_layout.addWidget(QLabel("Joint 1 (Base):"), 0, 0)
        angles_layout.addWidget(self.ik_theta1_label, 0, 1)
        
        self.ik_theta2_label = QLabel("θ2: 45.0°")
        self.ik_theta2_label.setStyleSheet("color: #FF9800; font-weight: bold; font-size: 14px;")
        angles_layout.addWidget(QLabel("Joint 2 (Elbow):"), 1, 0)
        angles_layout.addWidget(self.ik_theta2_label, 1, 1)
        
        self.ik_valid_label = QLabel("✅ Position is reachable")
        self.ik_valid_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
        angles_layout.addWidget(self.ik_valid_label, 2, 0, 1, 2)
        
        layout.addWidget(angles_group)
        
        # Quick target positions
        targets_group = QGroupBox("Quick Target Positions")
        targets_layout = QGridLayout(targets_group)
        
        target_positions = [
            ("🏠 Home (0, 20)", 0.0, 20.0),
            ("👈 Left (-15, 10)", -15.0, 10.0),
            ("👉 Right (15, 10)", 15.0, 10.0),
            ("⬆️ High (0, 18)", 0.0, 18.0),
            ("⬇️ Low (0, 2)", 0.0, 2.0),
            ("🔄 Center (10, 10)", 10.0, 10.0)
        ]
        
        for i, (name, x, y) in enumerate(target_positions):
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, tx=x, ty=y: self.set_target_position(tx, ty))
            targets_layout.addWidget(btn, i // 2, i % 2)
        
        layout.addWidget(targets_group)
        
        # Add mini workspace canvas for this tab
        mini_canvas_group = QGroupBox("Target Position Visualization")
        canvas_layout = QVBoxLayout(mini_canvas_group)
        self.ik_mini_canvas = WorkspaceCanvas(self.robot_config)
        self.ik_mini_canvas.setFixedSize(300, 300)
        canvas_layout.addWidget(self.ik_mini_canvas)
        layout.addWidget(mini_canvas_group)
        
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
        self.joint2_slider.setRange(0, 180)    # FIXED: Full servo range 0-180°
        self.joint2_slider.setValue(90)        # FIXED: Center position for 0-180°
        self.joint2_slider.valueChanged.connect(self.joint2_changed)
        joint_layout.addWidget(self.joint2_slider, 1, 1)
        
        self.joint2_value = QSpinBox()
        self.joint2_value.setRange(0, 180)     # FIXED: Full servo range 0-180°
        self.joint2_value.setValue(90)         # FIXED: Center position for 0-180°
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
        
        # Y coordinate - UPDATED: Only Y >= 0 allowed (simple_kinematics.ino constraint)
        position_layout.addWidget(QLabel("Y Position:"), 1, 0)
        self.y_position = QDoubleSpinBox()
        self.y_position.setRange(0.0, 20.0)  # Changed from -5.0 to 0.0 (Y >= 0 only)
        self.y_position.setValue(10.0)       # Changed default to positive value
        self.y_position.setDecimals(1)
        self.y_position.setSuffix(" cm")
        position_layout.addWidget(self.y_position, 1, 1)
        
        # Move to position button
        self.move_to_position_btn = QPushButton("🎯 Move to Position")
        self.move_to_position_btn.clicked.connect(self.move_to_position)
        position_layout.addWidget(self.move_to_position_btn, 2, 0, 1, 2)
        
        # Current position display
        self.current_pos_label = QLabel("Current: (0.0, 20.0) cm")  # FIXED: Max reach is 20.0
        self.current_pos_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        position_layout.addWidget(self.current_pos_label, 3, 0, 1, 2)
        
        layout.addWidget(position_group)
        
        # Quick positions group
        quick_group = QGroupBox("Quick Positions")
        quick_layout = QGridLayout(quick_group)
        
        # Pre-defined positions - UPDATED for simple_kinematics.ino (L1=10, L2=10, Y>=0)
        positions = [
            ("🏠 Home", 0, 20),      # Maximum Y reach
            ("👈 Left", -15, 10),    # Left side, Y positive
            ("👉 Right", 15, 10),    # Right side, Y positive  
            ("⬆️ Up", 0, 18),        # High Y position
            ("⬇️ Down", 0, 5),       # Low Y but still positive
            ("🔄 Center", 10, 10)    # Balanced position
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
        if hasattr(self, 'joint1_value'):
            self.joint1_value.setValue(value)
        self.update_robot_visualization()
        if self.serial_comm.is_connected:
            self.send_joint_command(1, value)
    
    def joint1_spin_changed(self, value):
        """Handle joint 1 spinbox change"""
        if hasattr(self, 'joint1_slider'):
            self.joint1_slider.setValue(value)
    
    def joint2_changed(self, value):
        """Handle joint 2 slider change"""
        if hasattr(self, 'joint2_value'):
            self.joint2_value.setValue(value)
        self.update_robot_visualization()
        if self.serial_comm.is_connected:
            self.send_joint_command(2, value)
    
    def joint2_spin_changed(self, value):
        """Handle joint 2 spinbox change"""
        if hasattr(self, 'joint2_slider'):
            self.joint2_slider.setValue(value)
    
    # New methods for servo control tab
    def joint1_dial_changed(self, value):
        """Handle joint 1 dial change"""
        self.update_servo_position_display()
        self.update_robot_visualization()
        if self.serial_comm.is_connected:
            self.send_joint_command(1, value)
    
    def joint2_dial_changed(self, value):
        """Handle joint 2 dial change"""
        self.update_servo_position_display()
        self.update_robot_visualization()
        if self.serial_comm.is_connected:
            self.send_joint_command(2, value)
    
    def update_servo_position_display(self):
        """Update calculated position display in servo tab"""
        theta1 = self.joint1_dial.value()
        theta2 = self.joint2_dial.value()
        
        # Calculate forward kinematics
        position = KinematicsCalculator.forward_kinematics(
            theta1, theta2, self.robot_config.L1, self.robot_config.L2
        )
        
        distance = position.distance_from_origin()
        
        # Update labels
        self.servo_x_label.setText(f"X: {position.x:.1f} cm")
        self.servo_y_label.setText(f"Y: {position.y:.1f} cm")
        self.servo_distance_label.setText(f"Distance: {distance:.1f} cm")
        
        # Update mini canvas
        if hasattr(self, 'servo_mini_canvas'):
            self.servo_mini_canvas.update_robot_state(theta1, theta2, position.x, position.y)
    
    def set_servo_angles(self, angle1, angle2):
        """Set both servo angles"""
        self.joint1_dial.setValue(angle1)
        self.joint2_dial.setValue(angle2)
        self.update_servo_position_display()
    
    # New methods for inverse kinematics tab
    def calculate_inverse_kinematics(self):
        """Calculate and display inverse kinematics"""
        x = self.ik_x_input.value()
        y = self.ik_y_input.value()
        
        theta1, theta2, is_valid = KinematicsCalculator.inverse_kinematics(
            x, y, self.robot_config.L1, self.robot_config.L2
        )
        
        # Update angle displays
        self.ik_theta1_label.setText(f"θ1: {theta1:.1f}°")
        self.ik_theta2_label.setText(f"θ2: {theta2:.1f}°")
        
        # Update validity indicator
        if is_valid:
            self.ik_valid_label.setText("✅ Position is reachable")
            self.ik_valid_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
            self.ik_move_btn.setEnabled(True)
        else:
            self.ik_valid_label.setText("❌ Position not reachable")
            self.ik_valid_label.setStyleSheet("color: #F44336; font-weight: bold; font-size: 12px;")
            self.ik_move_btn.setEnabled(False)
        
        # Update mini canvas
        if hasattr(self, 'ik_mini_canvas'):
            if is_valid:
                self.ik_mini_canvas.update_robot_state(theta1, theta2, x, y)
            else:
                self.ik_mini_canvas.set_target_position(x, y)
    
    def move_to_ik_position(self):
        """Move robot to calculated IK position"""
        x = self.ik_x_input.value()
        y = self.ik_y_input.value()
        
        if self.serial_comm.is_connected:
            command = f"{x:.1f},{y:.1f}"
            self.serial_comm.send_command(command)
        
        # Update main visualization
        self.workspace_canvas.set_target_position(x, y)
    
    def set_target_position(self, x, y):
        """Set target position in IK tab"""
        self.ik_x_input.setValue(x)
        self.ik_y_input.setValue(y)
        self.calculate_inverse_kinematics()
    
    def send_joint_command(self, joint: int, angle: int):
        """Send joint angle command to robot - Updated for simple_kinematics.ino format"""
        command = f"S{joint},{angle}"  # Changed from JOINT{joint}:{angle} to S{joint},{angle}
        self.serial_comm.send_command(command)
    
    def move_to_position(self):
        """Move robot to specified position - Updated for simple_kinematics.ino format"""
        x = self.x_position.value()
        y = self.y_position.value()
        
        # UPDATED: Check for Y >= 0 constraint (simple_kinematics.ino only supports Y >= 0)
        if y < 0:
            QMessageBox.warning(self, "Warning", 
                "Negative Y coordinates not supported!\n"
                "Only positive Y coordinates are allowed (Y >= 0)")
            return
        
        if self.workspace_check.isChecked():
            if not self.workspace_canvas.is_position_reachable(x, y):
                QMessageBox.warning(self, "Warning", 
                    f"Position ({x}, {y}) is outside workspace!\n"
                    f"Valid range: {self.robot_config.min_reach:.1f} - {self.robot_config.max_reach:.1f} cm")
                return
        
        # Send position command to robot - UPDATED format: just "x,y" instead of "MOVE:x,y"
        if self.serial_comm.is_connected:
            command = f"{x:.1f},{y:.1f}"  # Changed from MOVE:x,y to x,y
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
        """Update robot visualization - works with both old and new controls"""
        # Try to get values from dial controls first (new interface)
        if hasattr(self, 'joint1_dial') and hasattr(self, 'joint2_dial'):
            theta1 = self.joint1_dial.value()
            theta2 = self.joint2_dial.value()
            
            # Calculate forward kinematics for position
            position = KinematicsCalculator.forward_kinematics(
                theta1, theta2, self.robot_config.L1, self.robot_config.L2
            )
            
            # Update main workspace canvas if it exists
            if hasattr(self, 'workspace_canvas'):
                self.workspace_canvas.update_robot_state(theta1, theta2, position.x, position.y)
        
        # Fallback for old controls
        elif hasattr(self, 'joint1_value') and hasattr(self, 'joint2_value'):
            theta1 = self.joint1_value.value()
            theta2 = self.joint2_value.value()
            
            if hasattr(self, 'workspace_canvas'):
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
    
    # Demo functions - UPDATED for simple_kinematics.ino compatibility
    def start_basic_demo(self):
        """Start basic demo sequence - Using simple coordinate commands"""
        if self.serial_comm.is_connected:
            # Instead of DEMO:BASIC, send simple movements
            self.serial_comm.send_command("10.0,10.0")  # Move to center position
            self.status_bar.showMessage("Starting basic demo - moving to center...")
    
    def start_square_demo(self):
        """Start square path demo - Updated positions for Y >= 0"""
        positions = [(10.0, 10.0), (15.0, 10.0), (15.0, 15.0), (10.0, 15.0), (10.0, 10.0)]
        self.execute_path_demo(positions, "Square Path")
    
    def start_circle_demo(self):
        """Start circle path demo - Adjusted for Y >= 0"""
        center_x, center_y = 10, 12  # Moved center up to keep Y positive
        radius = 4
        positions = []
        for angle in range(0, 361, 30):
            x = center_x + radius * math.cos(math.radians(angle))
            y = center_y + radius * math.sin(math.radians(angle))
            if y >= 0:  # Only include positions with Y >= 0
                positions.append((x, y))
        self.execute_path_demo(positions, "Circle Path")
    
    def start_wave_demo(self):
        """Start wave pattern demo - Adjusted for Y >= 0"""
        positions = []
        for x in range(8, 17):
            y = 12 + 3 * math.sin(x * 0.5)  # Raised baseline to keep Y positive
            if y >= 0:  # Ensure Y >= 0
                positions.append((x, y))
        self.execute_path_demo(positions, "Wave Pattern")
    
    def start_workspace_demo(self):
        """Start workspace boundary demo - Using coordinate commands"""
        if self.serial_comm.is_connected:
            # Send maximum reach position instead of DEMO:WORKSPACE
            self.serial_comm.send_command("0.0,20.0")  # Move to max Y position
            self.status_bar.showMessage("Starting workspace demo - moving to max reach...")
    
    def execute_path_demo(self, positions: List[Tuple[float, float]], name: str):
        """Execute a path demo - UPDATED for simple_kinematics.ino format"""
        self.status_bar.showMessage(f"Starting {name} demo...")
        # This would be implemented with a timer to send positions sequentially
        # For now, just send the first position
        if positions and self.serial_comm.is_connected:
            x, y = positions[0]
            self.serial_comm.send_command(f"{x:.1f},{y:.1f}")  # Changed from MOVE:x,y to x,y
    
    def stop_demo(self):
        """Stop current demo - Send HOME command instead of DEMO:STOP"""
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("HOME")  # Changed from DEMO:STOP to HOME
            self.status_bar.showMessage("Demo stopped - returned to HOME")
    
    def move_to_home(self):
        """Move robot to home position"""
        self.joint1_slider.setValue(90)
        self.joint2_slider.setValue(0)
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("HOME")
        self.status_bar.showMessage("Moving to home position")
    
    def start_calibration(self):
        """Start servo calibration - Using individual servo commands"""
        if self.serial_comm.is_connected:
            # Instead of CALIBRATE:, send individual servo positions
            self.serial_comm.send_command("S1,90")  # Center position for servo 1
            self.serial_comm.send_command("S2,90")  # Center position for servo 2
            self.status_bar.showMessage("Calibrating - moving servos to center position...")
    
    def emergency_stop(self):
        """Emergency stop function - Send HOME command"""
        if self.serial_comm.is_connected:
            self.serial_comm.send_command("HOME")  # Changed from "STOP" to "HOME"
        self.status_bar.showMessage("EMERGENCY STOP - Returned to HOME position", 5000)
        
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
        
        # Load robot configuration - FIXED: Force correct values to match Arduino code
        # Override any old saved settings that might have wrong L1/L2 values
        self.robot_config.L1 = 10.0  # FORCE: Must match simple_kinematics.ino
        self.robot_config.L2 = 10.0  # FORCE: Must match simple_kinematics.ino
        
        # Update GUI to show correct values
        self.l1_spinbox.setValue(self.robot_config.L1)
        self.l2_spinbox.setValue(self.robot_config.L2)
        self.update_robot_config()
    
    def save_settings(self):
        """Save application settings"""
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        
        # Save robot configuration - FORCE correct values
        self.settings.setValue("robot/L1", 10.0)  # FORCE: Always save correct values
        self.settings.setValue("robot/L2", 10.0)  # FORCE: Always save correct values
    
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