import sys
import os
import cv2
import datetime
import random
import time
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QFrame, QTextEdit, QSizePolicy, 
                             QProgressBar, QPushButton, QLineEdit)
from PySide6.QtCore import Qt, QTimer, QPoint
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont, QTextCursor

# Yol ayarı - yasin klasörüne erişim için
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

try:
    from yasin.pid import DepthPID
except ImportError:
    sys.path.append(os.path.abspath(os.path.dirname(__file__)))
    from yasin.pid import DepthPID

class ROVInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TEKNOFEST ROV COMMAND - GYRO EDITION")
        self.resize(1300, 900)
        
        # --- STİL TANIMLAMALARI ---
        self.setStyleSheet("""
            QMainWindow { background-color: #050a0f; }
            QLabel { color: #00f2ff; font-family: 'Consolas'; font-weight: bold; }
            QFrame#Panel { background-color: #101827; border: 1px solid #00f2ff; border-radius: 5px; }
            QPushButton { background-color: #1a222f; color: #00f2ff; border: 1px solid #00f2ff; padding: 10px; font-weight: bold; min-width: 100px; }
            QPushButton:hover { background-color: #00f2ff; color: #000; }
            QLineEdit { background-color: #000; color: #00ff66; border: 1px solid #333; padding: 5px; }
        """)

        # --- DEĞİŞKENLER ---
        self.pid = DepthPID()
        self.control_mode = "MANUAL"
        self.current_stage = "7.1 HAT TAKIBI"
        self.active_vehicle = "ANA ARAC"
        self.manual_pwm = 80.0
        self.target_depth = 0.5
        
        # GYRO Verileri (Varsayılan)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        
        self.initUI()
        self.capture = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_system)
        self.timer.start(33)

    def initUI(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # --- ÜST KOMUTA BARI ---
        command_bar = QHBoxLayout()
        self.btn_manual = QPushButton("MANUAL"); self.btn_manual.clicked.connect(lambda: self.set_mode("MANUAL"))
        self.btn_auto = QPushButton("AUTO"); self.btn_auto.clicked.connect(lambda: self.set_mode("AUTO"))
        self.lbl_mode_display = QLabel("MODE: MANUAL")
        self.lbl_mode_display.setStyleSheet("font-size: 22px; margin-left: 20px; color: #00f2ff;")
        command_bar.addWidget(self.btn_manual); command_bar.addWidget(self.btn_auto); command_bar.addWidget(self.lbl_mode_display); command_bar.addStretch()
        
        self.btn_stage1 = QPushButton("GÖREV 1"); self.btn_stage1.clicked.connect(lambda: self.set_stage("7.1 HAT TAKIBI"))
        self.btn_mini = QPushButton("MINI ROV"); self.btn_mini.clicked.connect(self.toggle_mini_rov)
        self.btn_stage2 = QPushButton("GÖREV 2"); self.btn_stage2.clicked.connect(lambda: self.set_stage("7.2 NAVIGASYON"))
        command_bar.addWidget(self.btn_stage1); command_bar.addWidget(self.btn_mini); command_bar.addWidget(self.btn_stage2)
        self.main_layout.addLayout(command_bar)

        content_layout = QHBoxLayout()
        
        # --- SOL PANEL (NAV & GYRO DATA) ---
        self.left_panel = QFrame(); self.left_panel.setObjectName("Panel"); self.left_panel.setFixedWidth(220)
        l_lay = QVBoxLayout(self.left_panel)
        l_lay.addWidget(QLabel("── NAV DATA ──"))
        self.lbl_depth = QLabel("DEPTH: 0.00m"); l_lay.addWidget(self.lbl_depth)
        self.lbl_err = QLabel("PID ERR: 0.000"); l_lay.addWidget(self.lbl_err)
        
        l_lay.addSpacing(15)
        l_lay.addWidget(QLabel("── GYRO/IMU ──"))
        self.lbl_yaw = QLabel("YAW  : 0.0°"); l_lay.addWidget(self.lbl_yaw)
        self.lbl_pitch = QLabel("PITCH: 0.0°"); l_lay.addWidget(self.lbl_pitch)
        self.lbl_roll = QLabel("ROLL : 0.0°"); l_lay.addWidget(self.lbl_roll)
        
        l_lay.addSpacing(15)
        l_lay.addWidget(QLabel("── TARGET COORDS ──"))
        self.in_lat = QLineEdit(); self.in_lat.setPlaceholderText("Target Lat"); l_lay.addWidget(self.in_lat)
        self.in_lon = QLineEdit(); self.in_lon.setPlaceholderText("Target Lon"); l_lay.addWidget(self.in_lon)
        l_lay.addStretch()

        # --- KAMERA ---
        self.camera_label = QLabel("CAM STANDBY"); self.camera_label.setAlignment(Qt.AlignCenter); self.camera_label.setMinimumSize(1,1)

        # --- SAĞ PANEL (MOTORS) ---
        self.right_panel = QFrame(); self.right_panel.setObjectName("Panel"); self.right_panel.setFixedWidth(220)
        r_lay = QVBoxLayout(self.right_panel)
        r_lay.addWidget(QLabel("── MOTORS ──"))
        self.thruster_data = []
        for i in range(1, 7):
            m_lay = QVBoxLayout(); h_lay = QHBoxLayout()
            h_lay.addWidget(QLabel(f"T{i}")); h_lay.addStretch()
            amp = QLabel("0.0A"); amp.setStyleSheet("color: #ffaa00; font-size: 10px;"); h_lay.addWidget(amp)
            bar = QProgressBar(); bar.setRange(0, 400); self.thruster_data.append({'bar': bar, 'amp': amp})
            m_lay.addLayout(h_lay); m_lay.addWidget(bar); r_lay.addLayout(m_lay)
        r_lay.addStretch()

        content_layout.addWidget(self.left_panel); content_layout.addWidget(self.camera_label, stretch=1); content_layout.addWidget(self.right_panel)
        self.main_layout.addLayout(content_layout)

        # --- LOG EKRANI ---
        self.log = QTextEdit(); self.log.setFixedHeight(120); self.log.setReadOnly(True)
        self.log.setStyleSheet("background-color: #000; color: #00ff66; padding-bottom: 10px; border: 1px solid #333;")
        self.main_layout.addWidget(self.log)

    def add_log(self, msg):
        t = datetime.datetime.now().strftime("%H:%M:%S")
        self.log.append(f"<span style='color:#00ff66;'>[{t}] > {msg}</span>")
        cursor = self.log.textCursor(); cursor.movePosition(QTextCursor.End); self.log.setTextCursor(cursor)
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def set_mode(self, mode):
        self.control_mode = mode; self.lbl_mode_display.setText(f"MODE: {mode}")
        color = "#00ff66" if mode == "AUTO" else "#00f2ff"
        self.lbl_mode_display.setStyleSheet(f"font-size: 22px; margin-left: 20px; color: {color};")
        self.add_log(f"Kontrol Modu: {mode}")

    def set_stage(self, stage):
        self.current_stage = stage; self.add_log(f"Görev: {stage} seçildi.")

    def toggle_mini_rov(self):
        self.active_vehicle = "MINI ROV" if self.active_vehicle == "ANA ARAC" else "ANA ARAC"
        self.add_log(f"Aktif Araç: {self.active_vehicle}")

    def keyPressEvent(self, event):
        if self.control_mode == "MANUAL":
            if event.key() == Qt.Key_W: self.manual_pwm = min(400, self.manual_pwm + 15)
            elif event.key() == Qt.Key_S: self.manual_pwm = max(0, self.manual_pwm - 15)

    def update_system(self):
        ret, frame = self.capture.read()
        if ret:
            # 1. SENSÖR OKUMA (Derinlik & Gyro Simülasyonu)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            avg_brightness = gray.mean()
            meas_depth = max(0.0, min(2.5, (160 - avg_brightness) / 100.0))
            
            # Gyro Değerlerini Güncelle (Simüle edilmiş - Gerçekte Serial/Mavlink ile gelir)
            self.yaw = (self.yaw + 0.5) % 360 # Sürekli dönen bir pusula gibi
            self.pitch = (avg_brightness / 10) - 8 # Işığa göre hafif eğilme
            self.roll = random.uniform(-2, 2) # Küçük sarsıntılar
            
            # 2. HESAPLAMA (PID)
            if self.control_mode == "AUTO":
                e, Up, I_val, D, u_final, meas_filt = self.pid.step(self.target_depth, meas_depth)
            else:
                u_final = self.manual_pwm
                e = self.target_depth - meas_depth
            
            # 3. ARAYÜZÜ GÜNCELLE
            self.lbl_depth.setText(f"DEPTH: {meas_depth:.2f}m")
            self.lbl_err.setText(f"PID ERR: {e:.3f}")
            self.lbl_yaw.setText(f"YAW  : {self.yaw:.1f}°")
            self.lbl_pitch.setText(f"PITCH: {self.pitch:.1f}°")
            self.lbl_roll.setText(f"ROLL : {self.roll:.1f}°")
            
            for item in self.thruster_data:
                val = abs(int(u_final))
                item['bar'].setValue(val)
                item['amp'].setText(f"{(val*0.04)+0.1:.1f}A")

            # 4. GÖRÜNTÜ VE HUD ÇİZİMİ
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pixmap = QPixmap.fromImage(QImage(frame_rgb.data, frame_rgb.shape[1], frame_rgb.shape[0], QImage.Format_RGB888))
            scaled = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            painter = QPainter(scaled)
            painter.setRenderHint(QPainter.Antialiasing)
            cx, cy = scaled.width()//2, scaled.height()//2
            
            # --- UFUK ÇİZGİSİ (Gyro HUD) ---
            painter.setPen(QPen(QColor(0, 255, 0, 150), 2))
            # Pitch ve Roll'a göre hareket eden bir çizgi
            offset = int(self.pitch * 5)
            painter.drawLine(cx-100, cy+offset, cx+100, cy+offset)
            
            # Nişangah
            painter.setPen(QPen(QColor(0, 242, 255, 120), 2))
            painter.drawEllipse(QPoint(cx, cy), 50, 50)
            
            # Görev 7.1 Hattı
            if self.current_stage == "7.1 HAT TAKIBI":
                painter.setPen(QPen(QColor(255, 255, 0, 180), 4, Qt.DashLine))
                painter.drawLine(cx, scaled.height(), cx, cy + 50)
            
            painter.end()
            self.camera_label.setPixmap(scaled)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROVInterface()
    window.show()
    sys.exit(app.exec())