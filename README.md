# ROV-Control-System-GUI
🌊 TEKNOFEST ROV - Ground Control Station (GUI)
This repository contains the primary Control Station Interface for an Unmanned Underwater Vehicle (ROV), specifically developed for TEKNOFEST missions. It integrates real-time telemetry, camera feeds, and autonomous control logic into a high-performance dashboard.

🚀 Key Functionalities:
Autonomous Depth Control: Real-time integration with PID Controller for vertical stabilization.

Advanced HUD (Heads-Up Display): Dynamic Gyroscope & IMU visualization with an artificial horizon for pitch/roll tracking.

Mission-Specific Modes: Quick-access toggles for specialized tasks like "Line Following" and "Navigation".

Telemetry Logging: Real-time event logging system for debugging sensor data and flight modes.

Multi-Vehicle Support: Seamless switching between the Main ROV and a Mini-ROV subsystem.

🛠️ Tech Stack:
Framework: PySide6 (Qt for Python)

Computer Vision: OpenCV

System: MAVLink-ready architecture with PID feedback loop.
📥 Installation & Setup
To run this Ground Control Station on your local machine, follow these steps:

Clone the repository:

### 📥 Installation & Setup

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/Nisa-q/ROV-Control-System.git](https://github.com/Nisa-q/ROV-Control-System.git)

2.Install all dependencies:
pip install -r requirements.txt

3.Launch the interface:
python main.py
