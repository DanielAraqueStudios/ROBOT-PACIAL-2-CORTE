#!/usr/bin/env python3
"""
Requirements installation script for ESP32-S3 Robot Control GUI
Run this script to install all required dependencies
"""

requirements = [
    "PyQt6>=6.4.0",
    "pyserial>=3.5",
    "matplotlib>=3.6.0",
    "numpy>=1.21.0",
]

def install_requirements():
    """Install all required packages"""
    import subprocess
    import sys
    
    print("🤖 ESP32-S3 Robot Control GUI - Dependency Installer")
    print("=" * 50)
    
    for package in requirements:
        print(f"📦 Installing {package}...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
            print(f"✅ {package} installed successfully")
        except subprocess.CalledProcessError as e:
            print(f"❌ Failed to install {package}: {e}")
            return False
    
    print("\n🎉 All dependencies installed successfully!")
    print("\n🚀 You can now run: python robot_control_gui.py")
    return True

if __name__ == "__main__":
    success = install_requirements()
    input("\nPress Enter to exit...")