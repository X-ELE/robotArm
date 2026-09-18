#!/usr/bin/env python3
"""
GUI de Control para Brazo Robot (MKS Gen v1.4 / RAMPS 1.4)
Desarrollado para PyQt6.
Control por teclas WASD, Flechas, panel lateral de posiciones y ángulos cinemáticos.
"""

import sys
import time
import math
import queue
import threading
from typing import Optional

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("Error: Requiere pyserial. Instálalo con: pip install pyserial")
    sys.exit(1)

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QTextEdit, QGroupBox, QRadioButton, QButtonGroup, QFrame,
    QMessageBox, QTabWidget
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QKeyEvent


# ==============================================================================
# HILO DE COMUNICACIÓN SERIE
# ==============================================================================
class SerialWorker(QObject):
    connected_signal = pyqtSignal(bool, str)
    log_signal = pyqtSignal(str, str)  # tipo ('tx', 'rx', 'info', 'err'), mensaje
    response_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ser: Optional[serial.Serial] = None
        self.cmd_queue = queue.Queue()
        self.running = False
        self.worker_thread: Optional[threading.Thread] = None

    def start_worker(self):
        self.running = True
        self.worker_thread = threading.Thread(target=self._run, daemon=True)
        self.worker_thread.start()

    def connect_port(self, port: str, baud: int = 9600):
        self.cmd_queue.put(('CONNECT', (port, baud)))

    def disconnect_port(self):
        self.cmd_queue.put(('DISCONNECT', None))

    def send_command(self, cmd: str):
        self.cmd_queue.put(('CMD', cmd))

    def _run(self):
        while self.running:
            try:
                task_type, data = self.cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if task_type == 'CONNECT':
                port, baud = data
                try:
                    self.log_signal.emit('info', f"Abriendo puerto {port} a {baud} baudios...")
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                    self.ser = serial.Serial(port, baud, timeout=1.0)
                    time.sleep(1.8)  # Esperar reinicio DTR del ATmega2560
                    welcome = ""
                    if self.ser.in_waiting:
                        welcome = self.ser.read(self.ser.in_waiting).decode("ascii", errors="replace").strip()
                    self.connected_signal.emit(True, f"Conectado a {port}")
                    self.log_signal.emit('rx', f"<< Placa conectada: {welcome or 'start'}")

                    # Habilitar motores automáticamente (M17) y sincronizar posición inicial
                    time.sleep(0.1)
                    self.ser.write(b"M17\r\n")
                    self.log_signal.emit('tx', ">> M17 (Activando corriente en motores)")
                    time.sleep(0.1)
                    if self.ser.in_waiting:
                        m17_resp = self.ser.readline().decode("ascii", errors="replace").strip()
                        self.log_signal.emit('rx', f"<< {m17_resp}")

                except Exception as e:
                    self.connected_signal.emit(False, str(e))
                    self.log_signal.emit('err', f"Error de conexión: {e}")

            elif task_type == 'DISCONNECT':
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.write(b"M18\r\n")  # Desactivar motores al desconectar
                        time.sleep(0.05)
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None
                self.connected_signal.emit(False, "Desconectado")
                self.log_signal.emit('info', "Puerto serie cerrado.")

            elif task_type == 'CMD':
                cmd_str = data.strip()
                if not cmd_str:
                    continue
                if not self.ser or not self.ser.is_open:
                    self.log_signal.emit('err', f"No se puede enviar '{cmd_str}': Puerto no conectado.")
                    continue

                try:
                    # El firmware de robotArm requiere retorno de carro CRLF (\r\n)
                    raw_bytes = (cmd_str + "\r\n").encode("ascii")
                    self.ser.write(raw_bytes)
                    self.log_signal.emit('tx', f">> {cmd_str}")

                    # Esperar respuesta 'ok'
                    start_t = time.time()
                    resp = ""
                    while time.time() - start_t < 2.0:
                        if self.ser.in_waiting:
                            line = self.ser.readline().decode("ascii", errors="replace")
                            resp += line
                            if "ok" in line or "!!" in line or "rs" in line:
                                break
                        time.sleep(0.01)

                    clean_resp = resp.strip()
                    if clean_resp:
                        self.log_signal.emit('rx', f"<< {clean_resp}")
                        self.response_signal.emit(clean_resp)
                except Exception as e:
                    self.log_signal.emit('err', f"Error en envío serie: {e}")


# ==============================================================================
# CINEMÁTICA INVERSA (Florin Tobler RobotArm)
# ==============================================================================
def calculate_joint_angles(x: float, y: float, z: float, shank_len: float = 120.0):
    try:
        rrot = math.sqrt((x * x) + (y * y))
        rside = math.sqrt((rrot * rrot) + (z * z))
        if rrot < 1e-4:
            rot_deg = 0.0
        else:
            val_rot = max(-1.0, min(1.0, x / rrot))
            rot_deg = math.degrees(math.asin(val_rot))

        val_high = (rside * 0.5) / shank_len
        val_high = max(-1.0, min(1.0, val_high))
        high_rad = -math.asin(val_high) * 2.0

        if rside > 1e-4:
            val_low = max(-1.0, min(1.0, rrot / rside))
            acos_val = math.acos(val_low)
        else:
            acos_val = 0.0

        if z > 0:
            low_rad = -acos_val + ((math.pi - high_rad) / 2.0) - (math.pi / 2.0)
        else:
            low_rad = acos_val + ((math.pi - high_rad) / 2.0) - (math.pi / 2.0)

        high_total_rad = high_rad + low_rad

        return rot_deg, math.degrees(low_rad), math.degrees(high_total_rad), rside
    except Exception:
        return 0.0, 0.0, 0.0, 0.0


def calculate_fk(rot_deg: float, low_deg: float, high_total_deg: float, shank_len: float = 120.0):
    """
    Cinemática directa exacta para la geometría de Florin Tobler.
    Convierte (rot, low, high_total) en grados a (X, Y, Z) en mm.
    """
    rot = math.radians(rot_deg)
    low = math.radians(low_deg)
    high_total = math.radians(high_total_deg)
    high_pure = high_total - low
    
    sin_val = math.sin(-high_pure / 2.0)
    rside = max(0.0, min(240.0, 2.0 * shank_len * sin_val))
    
    phi = - low - high_pure / 2.0
    rrot = rside * math.cos(phi)
    z = rside * math.sin(phi)
    x = rrot * math.sin(rot)
    y = rrot * math.cos(rot)
    return x, y, z


# ==============================================================================
# ESTILOS QSS
# ==============================================================================
DARK_STYLE = """
QMainWindow {
    background-color: #14171f;
}
QWidget {
    color: #e2e8f0;
    font-family: 'Segoe UI', 'Ubuntu', 'Helvetica', sans-serif;
    font-size: 12px;
}
QGroupBox {
    background-color: #1c212d;
    border: 1px solid #2d3748;
    border-radius: 6px;
    margin-top: 10px;
    font-weight: bold;
    padding-top: 10px;
    padding-left: 8px;
    padding-right: 8px;
    padding-bottom: 8px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 1px 6px;
    background-color: #2b3548;
    border-radius: 3px;
    color: #38bdf8;
    font-size: 11px;
}
QPushButton {
    background-color: #283042;
    border: 1px solid #3f4d6b;
    border-radius: 4px;
    padding: 2px 8px;
    font-weight: bold;
    font-size: 12px;
    min-height: 28px;
    max-height: 32px;
}
QPushButton:hover {
    background-color: #38445e;
    border-color: #0284c7;
}
QPushButton:pressed {
    background-color: #1e2533;
}
QLineEdit, QComboBox {
    background-color: #10131a;
    border: 1px solid #2d3748;
    border-radius: 4px;
    padding: 2px 6px;
    color: #ffffff;
    font-size: 12px;
    min-height: 26px;
    max-height: 30px;
}
QLineEdit:focus, QComboBox:focus {
    border: 1px solid #38bdf8;
}
QTextEdit {
    background-color: #10131a;
    border: 1px solid #2d3748;
    border-radius: 4px;
    padding: 4px 6px;
    color: #ffffff;
}
QRadioButton {
    spacing: 5px;
    font-size: 11px;
}
QRadioButton::indicator {
    width: 13px;
    height: 13px;
    border-radius: 6px;
    border: 1px solid #475569;
    background-color: #10131a;
}
QRadioButton::indicator:checked {
    background-color: #0284c7;
    border-color: #38bdf8;
}
QTabWidget::pane {
    border: 1px solid #2d3748;
    background-color: #161b24;
    border-radius: 5px;
    padding: 4px;
}
QTabBar::tab {
    background-color: #1e2533;
    color: #94a3b8;
    border: 1px solid #2d3748;
    border-bottom: none;
    padding: 3px 8px;
    font-size: 11px;
    font-weight: bold;
    min-height: 22px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    margin-right: 2px;
}
QTabBar::tab:selected {
    background-color: #161b24;
    color: #38bdf8;
    border-color: #38bdf8;
}
QTabBar::tab:hover {
    color: #ffffff;
    background-color: #2b3548;
}
"""


# ==============================================================================
# VENTANA PRINCIPAL
# ==============================================================================
class RobotArmGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control de Brazo Robotico - MKS Gen v1.4")
        self.resize(1180, 780)
        self.setMinimumSize(980, 680)
        self.setStyleSheet(DARK_STYLE)

        # Variables de estado
        self.x = 0.0
        self.y = 120.0
        self.z = 120.0
        self.e = 0.0
        # Cero de Trabajo (Relativo / Tara de Usuario)
        self.zero_x = 0.0
        self.zero_y = 120.0
        self.zero_z = 120.0

        self.gripper_closed = False
        self.steppers_enabled = True
        self.fan_enabled = False
        self.is_connected = False
        self.current_step = 5.0

        self.key_buttons = {}

        # Comunicación serie en segundo plano
        self.worker = SerialWorker()
        self.worker.connected_signal.connect(self.on_connection_changed)
        self.worker.log_signal.connect(self.on_log_message)
        self.worker.response_signal.connect(self.on_response_received)
        self.worker.start_worker()

        # Construir Interfaz
        self.init_ui()
        self.refresh_ports()
        self.update_telemetry()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(14, 10, 14, 10)
        main_layout.setSpacing(10)

        # 1. Barra Superior (Conexión)
        main_layout.addWidget(self.create_top_bar())

        # 2. Aviso de Alimentación 12V
        banner_power = QFrame()
        banner_power.setStyleSheet("background-color: #2a2415; border: 1px solid #856404; border-radius: 6px; padding: 4px;")
        b_layout = QHBoxLayout(banner_power)
        b_layout.setContentsMargins(10, 4, 10, 4)
        lbl_power_info = QLabel("⚡ <b>RECORDATORIO:</b> Requiere <b>fuente de 12V</b> encendida para mover motores. | <b>Mapeo Hardware:</b> Base = Driver Z | Hombro = Driver Y | Codo = Driver X")
        lbl_power_info.setStyleSheet("color: #ffe066; font-size: 12px;")
        b_layout.addWidget(lbl_power_info)
        main_layout.addWidget(banner_power)

        # 3. Área Central Dividida
        content_layout = QHBoxLayout()
        content_layout.setSpacing(14)
        content_layout.addWidget(self.create_left_panel(), stretch=6)
        content_layout.addWidget(self.create_right_panel(), stretch=4)
        main_layout.addLayout(content_layout, stretch=1)

        # 4. Terminal Serie
        main_layout.addWidget(self.create_console_panel(), stretch=0)

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # --------------------------------------------------------------------------
    # 1. BARRA SUPERIOR
    # --------------------------------------------------------------------------
    def create_top_bar(self) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("background-color: #1c212d; border-radius: 8px; border: 1px solid #2d3748;")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)

        lbl_port = QLabel("Puerto Serie:")
        lbl_port.setStyleSheet("font-weight: bold; color: #94a3b8;")
        layout.addWidget(lbl_port)

        self.cb_ports = QComboBox()
        self.cb_ports.setMinimumWidth(210)
        layout.addWidget(self.cb_ports)

        btn_refresh = QPushButton("↻")
        btn_refresh.setToolTip("Refrescar puertos disponibles")
        btn_refresh.setFixedWidth(32)
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)

        lbl_baud = QLabel("Baudios:")
        lbl_baud.setStyleSheet("font-weight: bold; color: #94a3b8;")
        layout.addWidget(lbl_baud)

        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["9600", "115200", "250000"])
        self.cb_baud.setCurrentText("9600")
        layout.addWidget(self.cb_baud)

        self.btn_connect = QPushButton("⚡ Conectar a Placa")
        self.btn_connect.setStyleSheet("background-color: #0284c7; color: white; min-width: 130px; font-weight: bold;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect)

        layout.addSpacing(10)

        self.lbl_status_badge = QLabel("🔴 DESCONECTADO")
        self.lbl_status_badge.setStyleSheet("""
            background-color: #3b181e; color: #f87171; font-weight: bold;
            font-size: 12px; padding: 6px 14px; border: 1px solid #7f1d1d; border-radius: 12px;
        """)
        layout.addWidget(self.lbl_status_badge)

        layout.addStretch()

        self.btn_toggle_steppers = QPushButton("🔋 Motores ON (M17)")
        self.btn_toggle_steppers.clicked.connect(self.toggle_steppers)
        layout.addWidget(self.btn_toggle_steppers)

        self.btn_toggle_fan = QPushButton("🌀 Fan OFF")
        self.btn_toggle_fan.clicked.connect(self.toggle_fan)
        layout.addWidget(self.btn_toggle_fan)

        self.btn_calibrate_home = QPushButton("📍 Calibrar Físico (M18/M17)")
        self.btn_calibrate_home.setStyleSheet("background-color: #1e293b; color: #38bdf8; border: 1px solid #0284c7; font-weight: bold;")
        self.btn_calibrate_home.setToolTip("Asistente para acomodar manualmente el brazo con motores libres y enclavarlo en HOME (0, 120, 120)")
        self.btn_calibrate_home.clicked.connect(self.calibrate_physical_home)
        layout.addWidget(self.btn_calibrate_home)

        self.btn_home = QPushButton("🏠 Home (0, 120, 120)")
        self.btn_home.setStyleSheet("background-color: #14532d; color: #4ade80; border: 1px solid #166534;")
        self.btn_home.clicked.connect(self.go_home)
        layout.addWidget(self.btn_home)

        return frame

    # --------------------------------------------------------------------------
    # 2. PANEL IZQUIERDO: CONTROLES DE TECLADO Y BOTONES
    # --------------------------------------------------------------------------
    def create_left_panel(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # 1. Selector de Incremento de Paso
        grp_step = QGroupBox("1. Tamaño de Paso por Pulsación")
        h_step = QHBoxLayout(grp_step)
        h_step.setContentsMargins(8, 4, 8, 4)
        self.step_group = QButtonGroup()

        steps = [("0.5 mm [1]", 0.5), ("1.0 mm [2]", 1.0), ("5.0 mm [3]", 5.0), ("10.0 mm [4]", 10.0), ("20.0 mm [5]", 20.0)]
        for i, (label, val) in enumerate(steps):
            rb = QRadioButton(label)
            if val == 5.0:
                rb.setChecked(True)
            rb.toggled.connect(lambda checked, v=val: self.on_step_selected(checked, v))
            self.step_group.addButton(rb, i)
            h_step.addWidget(rb)
        layout.addWidget(grp_step)

        # 2. Botones de Control (Pestañas de Modo con Botones 30x30 px)
        grp_keys = QGroupBox("2. Mando de Movimiento (Botones 30x30 / Teclado)")
        keys_v = QVBoxLayout(grp_keys)
        keys_v.setContentsMargins(6, 4, 6, 6)
        keys_v.setSpacing(4)

        self.tab_movement = QTabWidget()
        self.tab_movement.setFixedHeight(168)

        # ----------------------------------------------------------------------
        # PESTAÑA 1: Cilíndrico / Base (Recomendado para giro libre ±90°)
        # ----------------------------------------------------------------------
        tab_polar = QWidget()
        h_polar = QHBoxLayout(tab_polar)
        h_polar.setContentsMargins(6, 2, 6, 2)
        h_polar.setSpacing(14)

        # Pad WASD Polar
        w_box_p = QVBoxLayout()
        w_box_p.setSpacing(2)
        lbl_w_p = QLabel("Alcance / Base [WASD]")
        lbl_w_p.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_w_p.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px;")
        w_box_p.addWidget(lbl_w_p)

        g_wasd_p = QGridLayout()
        g_wasd_p.setSpacing(4)
        g_wasd_p.setAlignment(Qt.AlignmentFlag.AlignCenter)

        btn_w_p = self.make_key_button("W ▲", lambda: self.jog_reach_polar(self.current_step))
        btn_w_p.setToolTip("Extender radio hacia adelante (+R, Tecla W)")
        btn_s_p = self.make_key_button("S ▼", lambda: self.jog_reach_polar(-self.current_step))
        btn_s_p.setToolTip("Retraer radio hacia base (-R, Tecla S)")
        btn_a_p = self.make_key_button("A ↺", lambda: self.jog_base_polar(-self.get_angle_step()))
        btn_a_p.setToolTip("Giro Base en arco a la izquierda (↺, Tecla A) [Sin alterar radio]")
        btn_d_p = self.make_key_button("D ↻", lambda: self.jog_base_polar(self.get_angle_step()))
        btn_d_p.setToolTip("Giro Base en arco a la derecha (↻, Tecla D) [Sin alterar radio]")

        self.key_buttons['W'] = btn_w_p
        self.key_buttons['S'] = btn_s_p
        self.key_buttons['A'] = btn_a_p
        self.key_buttons['D'] = btn_d_p

        g_wasd_p.addWidget(btn_w_p, 0, 1)
        g_wasd_p.addWidget(btn_a_p, 1, 0)
        g_wasd_p.addWidget(btn_s_p, 1, 1)
        g_wasd_p.addWidget(btn_d_p, 1, 2)
        w_box_p.addLayout(g_wasd_p)
        h_polar.addLayout(w_box_p)

        # Separador vertical
        vsep_p = QFrame()
        vsep_p.setFrameShape(QFrame.Shape.VLine)
        vsep_p.setStyleSheet("color: #2d3748;")
        h_polar.addWidget(vsep_p)

        # Pad Flechas Polar
        arr_box_p = QVBoxLayout()
        arr_box_p.setSpacing(2)
        lbl_arr_p = QLabel("Altura y Base [Flechas]")
        lbl_arr_p.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_arr_p.setStyleSheet("color: #f59e0b; font-weight: bold; font-size: 11px;")
        arr_box_p.addWidget(lbl_arr_p)

        g_arr_p = QGridLayout()
        g_arr_p.setSpacing(4)
        g_arr_p.setAlignment(Qt.AlignmentFlag.AlignCenter)

        btn_up_p = self.make_key_button("↑ ▲", lambda: self.jog(z=self.current_step))
        btn_up_p.setToolTip("Codo subir vertical (+Z, Flecha Arriba)")
        btn_down_p = self.make_key_button("↓ ▼", lambda: self.jog(z=-self.current_step))
        btn_down_p.setToolTip("Codo bajar vertical (-Z, Flecha Abajo)")
        btn_left_p = self.make_key_button("← ↺", lambda: self.jog_base_polar(-self.get_angle_step()))
        btn_left_p.setToolTip("Giro Base en arco a la izquierda (↺, Flecha Izq)")
        btn_right_p = self.make_key_button("→ ↻", lambda: self.jog_base_polar(self.get_angle_step()))
        btn_right_p.setToolTip("Giro Base en arco a la derecha (↻, Flecha Der)")

        self.key_buttons['UP'] = btn_up_p
        self.key_buttons['DOWN'] = btn_down_p
        self.key_buttons['LEFT'] = btn_left_p
        self.key_buttons['RIGHT'] = btn_right_p

        g_arr_p.addWidget(btn_up_p, 0, 1)
        g_arr_p.addWidget(btn_left_p, 1, 0)
        g_arr_p.addWidget(btn_down_p, 1, 1)
        g_arr_p.addWidget(btn_right_p, 1, 2)
        arr_box_p.addLayout(g_arr_p)
        h_polar.addLayout(arr_box_p)

        self.tab_movement.addTab(tab_polar, "🌐 Cilíndrico / Base")

        # ----------------------------------------------------------------------
        # PESTAÑA 2: Articular (J1, J2, J3 en Grados por Cinemática Directa)
        # ----------------------------------------------------------------------
        tab_art = QWidget()
        v_art = QVBoxLayout(tab_art)
        v_art.setContentsMargins(12, 4, 12, 4)
        v_art.setSpacing(4)

        # Fila J1: Base (-90° a +90°)
        h_j1 = QHBoxLayout()
        h_j1.setSpacing(6)
        lbl_j1_n = QLabel("Base (J1):")
        lbl_j1_n.setFixedWidth(75)
        lbl_j1_n.setStyleSheet("font-weight: bold; color: #38bdf8;")
        btn_j1_m = self.make_key_button("◀ -", lambda: self.jog_joint(0, -self.get_angle_step()))
        btn_j1_m.setToolTip("Girar Base a la izquierda (- grados)")
        btn_j1_z = QPushButton("0° Frente")
        btn_j1_z.setFixedSize(70, 30)
        btn_j1_z.setStyleSheet("font-size: 11px; background-color: #1e293b;")
        btn_j1_z.clicked.connect(lambda: self.set_joint_abs(0, 0.0))
        btn_j1_p = self.make_key_button("+ ▶", lambda: self.jog_joint(0, self.get_angle_step()))
        btn_j1_p.setToolTip("Girar Base a la derecha (+ grados)")
        self.lbl_j1_val = QLabel("0.0°")
        self.lbl_j1_val.setFixedWidth(55)
        self.lbl_j1_val.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px;")
        h_j1.addWidget(lbl_j1_n)
        h_j1.addWidget(btn_j1_m)
        h_j1.addWidget(btn_j1_z)
        h_j1.addWidget(btn_j1_p)
        h_j1.addWidget(self.lbl_j1_val)
        h_j1.addStretch()
        v_art.addLayout(h_j1)

        # Fila J2: Hombro (-30° a +90°)
        h_j2 = QHBoxLayout()
        h_j2.setSpacing(6)
        lbl_j2_n = QLabel("Hombro (J2):")
        lbl_j2_n.setFixedWidth(75)
        lbl_j2_n.setStyleSheet("font-weight: bold; color: #a78bfa;")
        btn_j2_m = self.make_key_button("▼ -", lambda: self.jog_joint(1, -self.get_angle_step()))
        btn_j2_m.setToolTip("Hombro atrás (- grados)")
        btn_j2_z = QPushButton("0° Vert")
        btn_j2_z.setFixedSize(70, 30)
        btn_j2_z.setStyleSheet("font-size: 11px; background-color: #1e293b;")
        btn_j2_z.clicked.connect(lambda: self.set_joint_abs(1, 0.0))
        btn_j2_p = self.make_key_button("+ ▲", lambda: self.jog_joint(1, self.get_angle_step()))
        btn_j2_p.setToolTip("Hombro adelante (+ grados)")
        self.lbl_j2_val = QLabel("0.0°")
        self.lbl_j2_val.setFixedWidth(55)
        self.lbl_j2_val.setStyleSheet("color: #a78bfa; font-weight: bold; font-size: 11px;")
        h_j2.addWidget(lbl_j2_n)
        h_j2.addWidget(btn_j2_m)
        h_j2.addWidget(btn_j2_z)
        h_j2.addWidget(btn_j2_p)
        h_j2.addWidget(self.lbl_j2_val)
        h_j2.addStretch()
        v_art.addLayout(h_j2)

        # Fila J3: Codo (-120° a 0°)
        h_j3 = QHBoxLayout()
        h_j3.setSpacing(6)
        lbl_j3_n = QLabel("Codo (J3):")
        lbl_j3_n.setFixedWidth(75)
        lbl_j3_n.setStyleSheet("font-weight: bold; color: #f59e0b;")
        btn_j3_m = self.make_key_button("▼ -", lambda: self.jog_joint(2, -self.get_angle_step()))
        btn_j3_m.setToolTip("Codo bajar (- grados)")
        btn_j3_z = QPushButton("-90° Horiz")
        btn_j3_z.setFixedSize(70, 30)
        btn_j3_z.setStyleSheet("font-size: 11px; background-color: #1e293b;")
        btn_j3_z.clicked.connect(lambda: self.set_joint_abs(2, -90.0))
        btn_j3_p = self.make_key_button("+ ▲", lambda: self.jog_joint(2, self.get_angle_step()))
        btn_j3_p.setToolTip("Codo subir (+ grados)")
        self.lbl_j3_val = QLabel("-90.0°")
        self.lbl_j3_val.setFixedWidth(55)
        self.lbl_j3_val.setStyleSheet("color: #f59e0b; font-weight: bold; font-size: 11px;")
        h_j3.addWidget(lbl_j3_n)
        h_j3.addWidget(btn_j3_m)
        h_j3.addWidget(btn_j3_z)
        h_j3.addWidget(btn_j3_p)
        h_j3.addWidget(self.lbl_j3_val)
        h_j3.addStretch()
        v_art.addLayout(h_j3)

        self.tab_movement.addTab(tab_art, "🦾 Articular (J1,J2,J3)")

        # ----------------------------------------------------------------------
        # PESTAÑA 3: Cartesiano Lineal (X, Y, Z clásico)
        # ----------------------------------------------------------------------
        tab_cart = QWidget()
        h_cart = QHBoxLayout(tab_cart)
        h_cart.setContentsMargins(6, 2, 6, 2)
        h_cart.setSpacing(14)

        # Pad WASD Cartesiano
        w_box_c = QVBoxLayout()
        w_box_c.setSpacing(2)
        lbl_w_c = QLabel("Ejes X / Y [WASD]")
        lbl_w_c.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_w_c.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px;")
        w_box_c.addWidget(lbl_w_c)

        g_wasd_c = QGridLayout()
        g_wasd_c.setSpacing(4)
        g_wasd_c.setAlignment(Qt.AlignmentFlag.AlignCenter)

        btn_w_c = self.make_key_button("W ▲", lambda: self.jog(y=self.current_step))
        btn_w_c.setToolTip("Mover adelante lineal (+Y, Tecla W)")
        btn_s_c = self.make_key_button("S ▼", lambda: self.jog(y=-self.current_step))
        btn_s_c.setToolTip("Mover atrás lineal (-Y, Tecla S)")
        btn_a_c = self.make_key_button("A ◀", lambda: self.jog(x=-self.current_step))
        btn_a_c.setToolTip("Mover izquierda lineal (-X, Tecla A)")
        btn_d_c = self.make_key_button("D ▶", lambda: self.jog(x=self.current_step))
        btn_d_c.setToolTip("Mover derecha lineal (+X, Tecla D)")

        g_wasd_c.addWidget(btn_w_c, 0, 1)
        g_wasd_c.addWidget(btn_a_c, 1, 0)
        g_wasd_c.addWidget(btn_s_c, 1, 1)
        g_wasd_c.addWidget(btn_d_c, 1, 2)
        w_box_c.addLayout(g_wasd_c)
        h_cart.addLayout(w_box_c)

        # Separador vertical
        vsep_c = QFrame()
        vsep_c.setFrameShape(QFrame.Shape.VLine)
        vsep_c.setStyleSheet("color: #2d3748;")
        h_cart.addWidget(vsep_c)

        # Pad Flechas Cartesiano
        arr_box_c = QVBoxLayout()
        arr_box_c.setSpacing(2)
        lbl_arr_c = QLabel("Ejes Z / X [Flechas]")
        lbl_arr_c.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_arr_c.setStyleSheet("color: #f59e0b; font-weight: bold; font-size: 11px;")
        arr_box_c.addWidget(lbl_arr_c)

        g_arr_c = QGridLayout()
        g_arr_c.setSpacing(4)
        g_arr_c.setAlignment(Qt.AlignmentFlag.AlignCenter)

        btn_up_c = self.make_key_button("↑ ▲", lambda: self.jog(z=self.current_step))
        btn_up_c.setToolTip("Subir vertical (+Z, Flecha Arriba)")
        btn_down_c = self.make_key_button("↓ ▼", lambda: self.jog(z=-self.current_step))
        btn_down_c.setToolTip("Bajar vertical (-Z, Flecha Abajo)")
        btn_left_c = self.make_key_button("← ◀", lambda: self.jog(x=-self.current_step))
        btn_left_c.setToolTip("Mover izquierda lineal (-X, Flecha Izq)")
        btn_right_c = self.make_key_button("→ ▶", lambda: self.jog(x=self.current_step))
        btn_right_c.setToolTip("Mover derecha lineal (+X, Flecha Der)")

        g_arr_c.addWidget(btn_up_c, 0, 1)
        g_arr_c.addWidget(btn_left_c, 1, 0)
        g_arr_c.addWidget(btn_down_c, 1, 1)
        g_arr_c.addWidget(btn_right_c, 1, 2)
        arr_box_c.addLayout(g_arr_c)
        h_cart.addLayout(arr_box_c)

        self.tab_movement.addTab(tab_cart, "📐 Cartesiano (X,Y,Z)")

        keys_v.addWidget(self.tab_movement)
        layout.addWidget(grp_keys)

        # 3. Pinza Efectora
        grp_aux = QGroupBox("3. Pinza Efectora (Gripper)")
        h_aux = QHBoxLayout(grp_aux)
        h_aux.setContentsMargins(8, 4, 8, 4)

        self.btn_grip_toggle = QPushButton("✋ Alternar Pinza: Abrir / Cerrar [ESPACIO / G]")
        self.btn_grip_toggle.setFixedHeight(30)
        self.btn_grip_toggle.setStyleSheet("""
            QPushButton {
                background-color: #4c1d95;
                color: #e9d5ff;
                border: 1px solid #6d28d9;
                border-radius: 4px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #5b21b6;
                border-color: #a855f7;
            }
        """)
        self.btn_grip_toggle.clicked.connect(self.toggle_gripper)
        self.key_buttons['SPACE'] = self.btn_grip_toggle
        self.key_buttons['G'] = self.btn_grip_toggle
        h_aux.addWidget(self.btn_grip_toggle)
        layout.addWidget(grp_aux)

        # 4. Posicionamiento Directo (Campos 30px)
        grp_goto = QGroupBox("4. Posicionamiento Directo (Ir a Coordenadas)")
        h_goto = QHBoxLayout(grp_goto)
        h_goto.setSpacing(6)
        h_goto.setContentsMargins(8, 4, 8, 4)

        lbl_gx = QLabel("X:")
        self.in_goto_x = QLineEdit("0.0")
        self.in_goto_x.setFixedSize(50, 30)

        lbl_gy = QLabel("Y:")
        self.in_goto_y = QLineEdit("120.0")
        self.in_goto_y.setFixedSize(50, 30)

        lbl_gz = QLabel("Z:")
        self.in_goto_z = QLineEdit("120.0")
        self.in_goto_z.setFixedSize(50, 30)

        btn_do_goto = QPushButton("🚀 Mover (X,Y,Z)")
        btn_do_goto.setFixedHeight(30)
        btn_do_goto.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold;")
        btn_do_goto.clicked.connect(self.goto_coords)

        for w in (lbl_gx, self.in_goto_x, lbl_gy, self.in_goto_y, lbl_gz, self.in_goto_z, btn_do_goto):
            h_goto.addWidget(w)
        layout.addWidget(grp_goto)

        # 5. Envío Manual Directo G-Code
        grp_manual = QGroupBox("5. Envío Manual Directo (G-Code)")
        h_manual = QHBoxLayout(grp_manual)
        h_manual.setSpacing(6)
        h_manual.setContentsMargins(8, 4, 8, 4)

        self.in_gcode = QLineEdit()
        self.in_gcode.setFixedHeight(30)
        self.in_gcode.setPlaceholderText("Ej: G1 X10 Y130 Z100 o M17...")
        self.in_gcode.returnPressed.connect(self.send_custom_gcode)
        h_manual.addWidget(self.in_gcode)

        btn_send = QPushButton("Enviar")
        btn_send.setFixedHeight(30)
        btn_send.clicked.connect(self.send_custom_gcode)
        h_manual.addWidget(btn_send)

        layout.addWidget(grp_manual)
        layout.addStretch()
        return container

    def make_key_button(self, label_text: str, callback) -> QPushButton:
        btn = QPushButton(label_text)
        btn.setFixedSize(34, 30)
        btn.setStyleSheet("""
            QPushButton {
                background-color: #222938;
                border: 1px solid #3b465d;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
                padding: 0px;
                text-align: center;
            }
            QPushButton:hover {
                background-color: #2e384c;
                border-color: #38bdf8;
            }
        """)
        btn.clicked.connect(callback)
        return btn

    # --------------------------------------------------------------------------
    # 3. PANEL DERECHO: TELEMETRÍA Y POSICIONES
    # --------------------------------------------------------------------------
    def create_right_panel(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # 1. Matriz de Coordenadas y Cinemática (Campos 30px)
        grp_coords = QGroupBox("📊 Matriz de Posiciones y Cinemática (Campos 30px)")
        grid = QGridLayout(grp_coords)
        grid.setSpacing(6)
        grid.setContentsMargins(8, 8, 8, 8)

        # Encabezados de Columna
        h_col0 = QLabel("Eje / Articulación")
        h_col1 = QLabel("Absoluto (Base)")
        h_col2 = QLabel("Cero Relativo (Δ)")
        h_col3 = QLabel("Ángulo Motor")
        for i, h in enumerate((h_col0, h_col1, h_col2, h_col3)):
            h.setAlignment(Qt.AlignmentFlag.AlignCenter)
            h.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: bold;")
            grid.addWidget(h, 0, i)

        # Fila 1: X (Base)
        lbl_axis_x = QLabel("Base (X):")
        lbl_axis_x.setStyleSheet("color: #38bdf8; font-weight: bold;")
        self.val_x = self.create_display_box("", "0.00 mm", "#38bdf8", 90)
        self.val_dx = self.create_display_box("", "+0.00 mm", "#38bdf8", 90)
        self.val_rot = self.create_display_box("", "0.0°", "#38bdf8", 75)
        grid.addWidget(lbl_axis_x, 1, 0)
        grid.addWidget(self.val_x[1], 1, 1)
        grid.addWidget(self.val_dx[1], 1, 2)
        grid.addWidget(self.val_rot[1], 1, 3)

        # Fila 2: Y (Hombro)
        lbl_axis_y = QLabel("Hombro (Y):")
        lbl_axis_y.setStyleSheet("color: #4ade80; font-weight: bold;")
        self.val_y = self.create_display_box("", "120.00 mm", "#4ade80", 90)
        self.val_dy = self.create_display_box("", "+0.00 mm", "#4ade80", 90)
        self.val_low = self.create_display_box("", "0.0°", "#4ade80", 75)
        grid.addWidget(lbl_axis_y, 2, 0)
        grid.addWidget(self.val_y[1], 2, 1)
        grid.addWidget(self.val_dy[1], 2, 2)
        grid.addWidget(self.val_low[1], 2, 3)

        # Fila 3: Z (Codo)
        lbl_axis_z = QLabel("Codo (Z):")
        lbl_axis_z.setStyleSheet("color: #fbbf24; font-weight: bold;")
        self.val_z = self.create_display_box("", "120.00 mm", "#fbbf24", 90)
        self.val_dz = self.create_display_box("", "+0.00 mm", "#fbbf24", 90)
        self.val_high = self.create_display_box("", "-90.0°", "#fbbf24", 75)
        grid.addWidget(lbl_axis_z, 3, 0)
        grid.addWidget(self.val_z[1], 3, 1)
        grid.addWidget(self.val_dz[1], 3, 2)
        grid.addWidget(self.val_high[1], 3, 3)

        # Fila 4: Radio (R)
        lbl_axis_r = QLabel("Radio (R):")
        lbl_axis_r.setStyleSheet("color: #f43f5e; font-weight: bold;")
        self.val_reach = self.create_display_box("", "169.7 mm (72%)", "#f43f5e", 186)
        grid.addWidget(lbl_axis_r, 4, 0)
        grid.addWidget(self.val_reach[1], 4, 1, 1, 2)

        lbl_r_safe = QLabel("Seguro")
        lbl_r_safe.setFixedHeight(30)
        lbl_r_safe.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_r_safe.setStyleSheet("background-color: #052e16; color: #4ade80; font-weight: bold; border-radius: 4px; font-size: 11px;")
        grid.addWidget(lbl_r_safe, 4, 3)

        layout.addWidget(grp_coords)

        # 2. Botones de Cero y Referencia (30px)
        h_zero_btns = QHBoxLayout()
        h_zero_btns.setSpacing(6)

        btn_set_zero = QPushButton("🎯 Fijar Cero [Z]")
        btn_set_zero.setFixedHeight(30)
        btn_set_zero.setStyleSheet("background-color: #0369a1; color: white; font-weight: bold;")
        btn_set_zero.setToolTip("Establece la posición actual como Cero de Trabajo (Tecla rápida: Z)")
        btn_set_zero.clicked.connect(self.set_user_zero)
        h_zero_btns.addWidget(btn_set_zero)

        btn_goto_zero = QPushButton("↩ Volver al Cero")
        btn_goto_zero.setFixedHeight(30)
        btn_goto_zero.setToolTip("Regresa a la posición donde fijaste el Cero de Trabajo")
        btn_goto_zero.clicked.connect(self.goto_user_zero)
        h_zero_btns.addWidget(btn_goto_zero)

        btn_home = QPushButton("🏠 Home (0,120,120)")
        btn_home.setFixedHeight(30)
        btn_home.setStyleSheet("background-color: #14532d; color: #4ade80; border: 1px solid #166534;")
        btn_home.setToolTip("Regresa a la posición Home de fábrica (Tecla rápida: H)")
        btn_home.clicked.connect(self.go_home)
        h_zero_btns.addWidget(btn_home)

        layout.addLayout(h_zero_btns)

        # 3. Hardware y Estado (Líneas compactas)
        grp_hw = QGroupBox("⚙️ Estado de Hardware y Rango")
        v_hw = QVBoxLayout(grp_hw)
        v_hw.setSpacing(4)
        v_hw.setContentsMargins(8, 6, 8, 6)

        self.lbl_bounds = QLabel("• Extensión: 169.7 mm / 236 mm (72%) [Margen: 66.3 mm]")
        self.lbl_linear_limit = QLabel("• X Lineal Disp. (a Y=120, Z=120): ±164.0 mm")
        self.lbl_driver_state = QLabel("• Motores: ACTIVADOS (M17) | Fan: APAGADO | Pinza: ABIERTA")

        self.lbl_bounds.setStyleSheet("font-size: 11px;")
        self.lbl_linear_limit.setStyleSheet("font-size: 11px;")
        self.lbl_driver_state.setStyleSheet("font-size: 11px;")
        v_hw.addWidget(self.lbl_bounds)
        v_hw.addWidget(self.lbl_linear_limit)
        v_hw.addWidget(self.lbl_driver_state)

        layout.addWidget(grp_hw)
        layout.addStretch()
        return container

    def create_display_box(self, title: str, initial_val: str, color_hex: str, width: int = 0):
        lbl_val = QLabel(initial_val)
        lbl_val.setFixedHeight(30)
        if width > 0:
            lbl_val.setFixedWidth(width)
        lbl_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_val.setStyleSheet(f"""
            background-color: #0f1319;
            color: {color_hex};
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 12px;
            font-weight: bold;
            border: 1px solid #242c3d;
            border-radius: 4px;
            padding: 2px 4px;
        """)
        return None, lbl_val

    # --------------------------------------------------------------------------
    # 4. CONSOLA SERIE INFERIOR
    # --------------------------------------------------------------------------
    def create_console_panel(self) -> QGroupBox:
        grp = QGroupBox("Terminal Serie en Vivo (MKS Gen v1.4)")
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(8, 4, 8, 4)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(105)
        self.txt_log.setStyleSheet("""
            background-color: #0d1017;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 11px;
            color: #94a3b8;
            border: 1px solid #1e293b;
        """)
        layout.addWidget(self.txt_log)
        return grp

    # --------------------------------------------------------------------------
    # EVENTOS DE TECLADO Y MOVIMIENTO
    # --------------------------------------------------------------------------
    def get_angle_step(self) -> float:
        if self.current_step <= 1.0:
            return 1.0
        elif self.current_step == 5.0:
            return 5.0
        elif self.current_step == 10.0:
            return 10.0
        elif self.current_step >= 20.0:
            return 15.0
        return max(1.0, min(30.0, self.current_step))

    def on_step_selected(self, checked: bool, val: float):
        if checked:
            self.current_step = val
            self.on_log_message('info', f"Paso ajustado a: {self.current_step} mm (Ángulo: {self.get_angle_step():.0f}°)")

    def keyPressEvent(self, event: QKeyEvent):
        if self.in_gcode.hasFocus():
            if event.key() == Qt.Key.Key_Escape:
                self.setFocus()
            super().keyPressEvent(event)
            return

        key = event.key()
        matched_btn = None
        current_tab = self.tab_movement.currentIndex() if hasattr(self, 'tab_movement') else 0

        # Pestaña 0: Cilíndrico / Base (A/D giran base en arco sin alterar radio)
        if current_tab == 0:
            if key == Qt.Key.Key_W:
                self.jog_reach_polar(self.current_step)
                matched_btn = self.key_buttons.get('W')
            elif key == Qt.Key.Key_S:
                self.jog_reach_polar(-self.current_step)
                matched_btn = self.key_buttons.get('S')
            elif key in (Qt.Key.Key_A, Qt.Key.Key_Left):
                self.jog_base_polar(-self.get_angle_step())
                matched_btn = self.key_buttons.get('A') or self.key_buttons.get('LEFT')
            elif key in (Qt.Key.Key_D, Qt.Key.Key_Right):
                self.jog_base_polar(self.get_angle_step())
                matched_btn = self.key_buttons.get('D') or self.key_buttons.get('RIGHT')
            elif key == Qt.Key.Key_Up:
                self.jog(z=self.current_step)
                matched_btn = self.key_buttons.get('UP')
            elif key == Qt.Key.Key_Down:
                self.jog(z=-self.current_step)
                matched_btn = self.key_buttons.get('DOWN')

        # Pestaña 1: Articular (J1, J2, J3)
        elif current_tab == 1:
            if key in (Qt.Key.Key_A, Qt.Key.Key_Left):
                self.jog_joint(0, -self.get_angle_step())
            elif key in (Qt.Key.Key_D, Qt.Key.Key_Right):
                self.jog_joint(0, self.get_angle_step())
            elif key == Qt.Key.Key_W:
                self.jog_joint(1, self.get_angle_step())
            elif key == Qt.Key.Key_S:
                self.jog_joint(1, -self.get_angle_step())
            elif key == Qt.Key.Key_Up:
                self.jog_joint(2, self.get_angle_step())
            elif key == Qt.Key.Key_Down:
                self.jog_joint(2, -self.get_angle_step())

        # Pestaña 2: Cartesiano Lineal (X, Y, Z)
        else:
            if key in (Qt.Key.Key_A, Qt.Key.Key_Left):
                self.jog(x=-self.current_step)
                matched_btn = self.key_buttons.get('A') or self.key_buttons.get('LEFT')
            elif key in (Qt.Key.Key_D, Qt.Key.Key_Right):
                self.jog(x=self.current_step)
                matched_btn = self.key_buttons.get('D') or self.key_buttons.get('RIGHT')
            elif key == Qt.Key.Key_W:
                self.jog(y=self.current_step)
                matched_btn = self.key_buttons.get('W')
            elif key == Qt.Key.Key_S:
                self.jog(y=-self.current_step)
                matched_btn = self.key_buttons.get('S')
            elif key == Qt.Key.Key_Up:
                self.jog(z=self.current_step)
                matched_btn = self.key_buttons.get('UP')
            elif key == Qt.Key.Key_Down:
                self.jog(z=-self.current_step)
                matched_btn = self.key_buttons.get('DOWN')

        # Atajos globales comunes
        if key in (Qt.Key.Key_Space, Qt.Key.Key_G):
            self.toggle_gripper()
            matched_btn = self.key_buttons.get('SPACE')
        elif key == Qt.Key.Key_H:
            self.go_home()
        elif key == Qt.Key.Key_Z:
            self.set_user_zero()
        elif key == Qt.Key.Key_1:
            self.set_step_by_index(0)
        elif key == Qt.Key.Key_2:
            self.set_step_by_index(1)
        elif key == Qt.Key.Key_3:
            self.set_step_by_index(2)
        elif key == Qt.Key.Key_4:
            self.set_step_by_index(3)
        elif key == Qt.Key.Key_5:
            self.set_step_by_index(4)
        elif key == Qt.Key.Key_M:
            self.toggle_steppers()
        else:
            super().keyPressEvent(event)
            return

        if matched_btn:
            self.flash_button(matched_btn)

    def set_step_by_index(self, idx: int):
        btn = self.step_group.button(idx)
        if btn:
            btn.setChecked(True)

    def flash_button(self, btn: QPushButton):
        orig_style = btn.styleSheet()
        btn.setStyleSheet("background-color: #0284c7; color: white; border: 2px solid #38bdf8;")
        QTimer.singleShot(140, lambda: btn.setStyleSheet(orig_style))

    def jog(self, x=0.0, y=0.0, z=0.0, e=0.0):
        if not self.is_connected:
            self.on_log_message('err', "⚠️ No conectado a la placa. Pulsa '⚡ Conectar a Placa' primero.")
            return

        target_x = round(self.x + x, 2)
        target_y = round(self.y + y, 2)
        target_z = round(self.z + z, 2)
        target_e = round(self.e + e, 2)

        r_ground = math.sqrt(target_x * target_x + target_y * target_y)
        r_total = math.sqrt(r_ground * r_ground + target_z * target_z)

        # Límite esférico físico (2 segmentos de 120 mm = 240 mm max teórico)
        if r_total > 236.0:
            max_x = math.sqrt(max(0.0, 236.0**2 - target_y**2 - target_z**2))
            self.on_log_message('err', f"⚠️ Límite esférico: Radio {r_total:.1f} mm > 236.0 mm. Con Y={target_y:.0f} y Z={target_z:.0f}, X lineal está acotado a ±{max_x:.1f} mm. 💡 Consejo: Usa la pestaña 'Cilíndrico / Base' para girar la base ±90° sin aumentar el radio.")
            self.lbl_bounds.setText(f"• Rango de Trabajo: <font color='#ef4444'><b>¡LÍMITE MÁXIMO! ({r_total:.1f} mm / 236 mm)</b></font>")
            return
        if target_y < 15.0:
            self.on_log_message('err', "⚠️ Límite colisión: Y debe ser >= 15 mm para evitar chocar contra la base.")
            return
        if r_total < 35.0:
            self.on_log_message('err', "⚠️ Límite contracción: El brazo no puede plegarse sobre su propio centro (R < 35 mm).")
            return

        self.x = target_x
        self.y = target_y
        self.z = target_z
        self.e = target_e

        cmd = f"G1 X{self.x:.2f} Y{self.y:.2f} Z{self.z:.2f}"
        self.worker.send_command(cmd)
        self.update_telemetry()

    def jog_base_polar(self, delta_deg: float):
        """Gira la base en un arco circular sobre el plano horizontal sin alterar el radio esférico"""
        if not self.is_connected:
            self.on_log_message('err', "⚠️ No conectado a la placa. Pulsa '⚡ Conectar a Placa' primero.")
            return

        r_xy = math.sqrt(self.x * self.x + self.y * self.y)
        if r_xy < 1.0:
            r_xy = 120.0

        cur_rot, cur_low, cur_high, _ = calculate_joint_angles(self.x, self.y, self.z)
        target_rot = cur_rot + delta_deg

        if target_rot < -90.0 or target_rot > 90.0:
            self.on_log_message('err', f"⚠️ Límite de rotación de Base: {target_rot:+.1f}° (Rango permitido: -90° a +90°).")
            return

        rad = math.radians(target_rot)
        target_x = round(r_xy * math.sin(rad), 2)
        target_y = round(r_xy * math.cos(rad), 2)
        target_z = self.z

        if target_y < 10.0:
            target_y = 10.0

        r_total = math.sqrt(target_x * target_x + target_y * target_y + target_z * target_z)
        if r_total > 236.0:
            self.on_log_message('err', f"⚠️ Límite geométrico: Radio {r_total:.1f} mm > 236.0 mm.")
            return

        self.x = target_x
        self.y = target_y
        self.z = target_z

        cmd = f"G1 X{self.x:.2f} Y{self.y:.2f} Z{self.z:.2f}"
        self.worker.send_command(cmd)
        self.update_telemetry()

    def jog_reach_polar(self, delta_r: float):
        """Extiende o retrae el alcance radial hacia adelante/atrás a lo largo del ángulo actual"""
        if not self.is_connected:
            self.on_log_message('err', "⚠️ No conectado a la placa.")
            return

        cur_rot, cur_low, cur_high, _ = calculate_joint_angles(self.x, self.y, self.z)
        r_xy = math.sqrt(self.x * self.x + self.y * self.y)
        target_r = r_xy + delta_r

        if target_r < 35.0:
            self.on_log_message('err', "⚠️ Límite contracción: El brazo no puede plegarse a menos de 35 mm de su base.")
            return

        rad = math.radians(cur_rot)
        target_x = round(target_r * math.sin(rad), 2)
        target_y = round(target_r * math.cos(rad), 2)
        target_z = self.z

        if target_y < 15.0:
            self.on_log_message('err', "⚠️ Límite colisión: Y debe ser >= 15 mm para evitar chocar contra el poste.")
            return

        r_total = math.sqrt(target_x * target_x + target_y * target_y + target_z * target_z)
        if r_total > 236.0:
            self.on_log_message('err', f"⚠️ Límite geométrico esférico: Radio {r_total:.1f} mm > 236.0 mm. 💡 Baja la altura Z para extender más adelante.")
            return

        self.x = target_x
        self.y = target_y
        self.z = target_z

        cmd = f"G1 X{self.x:.2f} Y{self.y:.2f} Z{self.z:.2f}"
        self.worker.send_command(cmd)
        self.update_telemetry()

    def jog_joint(self, joint_idx: int, delta_deg: float):
        """Mueve individualmente cada motor en grados mediante Cinemática Directa (0=Base, 1=Hombro, 2=Codo)"""
        if not self.is_connected:
            self.on_log_message('err', "⚠️ No conectado a la placa.")
            return

        cur_rot, cur_low, cur_high, _ = calculate_joint_angles(self.x, self.y, self.z)
        new_rot = cur_rot
        new_low = cur_low
        new_high = cur_high

        if joint_idx == 0:
            new_rot += delta_deg
            if new_rot < -90.0 or new_rot > 90.0:
                self.on_log_message('err', f"⚠️ Límite articular Base: {new_rot:+.1f}° (Rango: -90° a +90°).")
                return
        elif joint_idx == 1:
            new_low += delta_deg
            if new_low < -35.0 or new_low > 95.0:
                self.on_log_message('err', f"⚠️ Límite articular Hombro: {new_low:+.1f}° (Rango: -35° a +95°).")
                return
        elif joint_idx == 2:
            new_high += delta_deg
            if new_high < -140.0 or new_high > 10.0:
                self.on_log_message('err', f"⚠️ Límite articular Codo: {new_high:+.1f}° (Rango: -140° a +10°).")
                return

        self.jog_joint_to_angles(new_rot, new_low, new_high)

    def set_joint_abs(self, joint_idx: int, target_angle: float):
        """Fija el ángulo absoluto de una articulación respetando las otras dos"""
        cur_rot, cur_low, cur_high, _ = calculate_joint_angles(self.x, self.y, self.z)
        new_rot = target_angle if joint_idx == 0 else cur_rot
        new_low = target_angle if joint_idx == 1 else cur_low
        new_high = target_angle if joint_idx == 2 else cur_high
        self.jog_joint_to_angles(new_rot, new_low, new_high)

    def jog_joint_to_angles(self, rot_deg: float, low_deg: float, high_deg: float):
        """Calcula coordenadas (X,Y,Z) mediante Cinemática Directa y envía G1"""
        tx, ty, tz = calculate_fk(rot_deg, low_deg, high_deg)
        r_ground = math.sqrt(tx * tx + ty * ty)
        r_total = math.sqrt(r_ground * r_ground + tz * tz)

        if r_total > 236.0:
            self.on_log_message('err', f"⚠️ Límite esférico alcanzado ({r_total:.1f} mm > 236 mm).")
            return
        if ty < 10.0:
            self.on_log_message('err', "⚠️ Límite colisión frontal: Y < 10 mm.")
            return

        self.x = round(tx, 2)
        self.y = round(ty, 2)
        self.z = round(tz, 2)

        cmd = f"G1 X{self.x:.2f} Y{self.y:.2f} Z{self.z:.2f}"
        self.worker.send_command(cmd)
        self.update_telemetry()

    def set_user_zero(self):
        """Fija la posición actual del brazo como el Cero de Trabajo (Tara / WCS)"""
        self.zero_x = self.x
        self.zero_y = self.y
        self.zero_z = self.z
        self.update_telemetry()
        self.on_log_message('info', f"🎯 Cero de Trabajo fijado en: Absoluto ({self.x:+.1f}, {self.y:.1f}, {self.z:.1f}) ➔ [ΔX=0, ΔY=0, ΔZ=0]")

    def goto_user_zero(self):
        """Regresa el brazo a la posición donde el usuario fijó el Cero de Trabajo"""
        if not self.is_connected:
            self.on_log_message('err', "⚠️ Conecta la placa primero.")
            return
        self.x = self.zero_x
        self.y = self.zero_y
        self.z = self.zero_z
        cmd = f"G1 X{self.x:.2f} Y{self.y:.2f} Z{self.z:.2f}"
        self.on_log_message('info', f"Regresando al Cero de Trabajo (Absoluto: {self.zero_x:.1f}, {self.zero_y:.1f}, {self.zero_z:.1f})...")
        self.worker.send_command(cmd)
        self.update_telemetry()

    def calibrate_physical_home(self):
        """Asistente para acomodar manualmente el brazo (M18) y calibrarlo en HOME (0, 120, 120)"""
        if not self.is_connected:
            QMessageBox.warning(self, "Aviso", "Conecta la placa primero.")
            return

        reply = QMessageBox.question(
            self,
            "Calibrar Posición Física HOME",
            "¿Deseas liberar los motores (M18) para acomodar el brazo a mano?\n\n"
            "Instrucciones de colocación manual:\n"
            "  1. BASE: Centrada apuntando al frente (0°).\n"
            "  2. HOMBRO: Totalmente vertical a 90° respecto a la base.\n"
            "  3. CODO: Horizontal a 90° formando una 'L' recta hacia adelante.\n\n"
            "Al pulsar 'Sí', los motores quedarán suaves y libres para mover a mano.\n"
            "Cuando lo acomodes, pulsa Aceptar en el siguiente mensaje para enclavar y calibrar.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.worker.send_command("M18")
            self.steppers_enabled = False
            self.lbl_driver_state.setText("• Motores: <font color='#94a3b8'><b>DESACTIVADOS (Libre para mover a mano)</b></font>")
            self.btn_toggle_steppers.setText("🔋 Motores OFF")
            self.on_log_message('info', "Motores liberados (M18). Acomoda el brazo físicamente en la 'L' recta de referencia...")

            QMessageBox.information(
                self,
                "Acomodar Brazo en Pose HOME",
                "Acomoda el brazo en posición 'L' recta:\n\n"
                "• Base: 0° al frente\n"
                "• Hombro: 90° vertical\n"
                "• Codo: 90° horizontal hacia adelante\n\n"
                "Una vez colocado en posición exacta, pulsa 'Aceptar' para enclavar motores (M17) y calibrar en (0, 120, 120)."
            )
            self.worker.send_command("M17")
            self.steppers_enabled = True
            self.lbl_driver_state.setText("• Motores: <font color='#4ade80'><b>ACTIVADOS (M17)</b></font>")
            self.btn_toggle_steppers.setText("🔋 Motores ON (M17)")

            # Sincronizar coordenadas absolutas y cero relativo
            self.x = 0.0
            self.y = 120.0
            self.z = 120.0
            self.zero_x = 0.0
            self.zero_y = 120.0
            self.zero_z = 120.0
            self.update_telemetry()
            self.on_log_message('info', "✅ Brazo calibrado con éxito en HOME: Absoluto (0, 120, 120) y Cero Relativo (0, 0, 0)")

    def go_home(self):
        if not self.is_connected:
            self.on_log_message('err', "⚠️ Conecta la placa primero.")
            return
        self.x = 0.0
        self.y = 120.0
        self.z = 120.0
        self.e = 0.0
        cmd = "G1 X0 Y120 Z120"
        self.on_log_message('info', "Regresando a posición HOME (0, 120, 120)...")
        self.worker.send_command(cmd)
        self.update_telemetry()

    def toggle_gripper(self):
        if not self.is_connected:
            self.on_log_message('err', "⚠️ Conecta la placa primero.")
            return
        self.gripper_closed = not self.gripper_closed
        steps = 600
        cmd = f"M3 T{steps}" if self.gripper_closed else f"M5 T{steps}"
        state_str = "CERRADA" if self.gripper_closed else "ABIERTA"
        self.on_log_message('info', f"Pinza {state_str}")
        self.lbl_gripper_state.setText(f"• Pinza: <b>{state_str}</b>")
        self.btn_grip_toggle.setText(f"✋ Pinza: {state_str}\n[ESPACIO / G]")
        self.worker.send_command(cmd)

    def toggle_steppers(self):
        if not self.is_connected:
            self.on_log_message('err', "⚠️ Conecta la placa primero.")
            return
        self.steppers_enabled = not self.steppers_enabled
        cmd = "M17" if self.steppers_enabled else "M18"
        state_str = "ACTIVADOS" if self.steppers_enabled else "DESACTIVADOS (Libre)"
        color = "#4ade80" if self.steppers_enabled else "#94a3b8"
        self.btn_toggle_steppers.setText(f"🔋 Motores {'ON' if self.steppers_enabled else 'OFF'}")
        self.lbl_driver_state.setText(f"• Motores: <font color='{color}'><b>{state_str}</b></font> ({cmd})")
        self.worker.send_command(cmd)

    def toggle_fan(self):
        if not self.is_connected:
            self.on_log_message('err', "⚠️ Conecta la placa primero.")
            return
        self.fan_enabled = not self.fan_enabled
        cmd = "M106" if self.fan_enabled else "M107"
        self.btn_toggle_fan.setText(f"🌀 Fan {'ON' if self.fan_enabled else 'OFF'}")
        self.lbl_fan_state.setText(f"• Ventilador: <b>{'ENCENDIDO' if self.fan_enabled else 'APAGADO'}</b> ({cmd})")
        self.worker.send_command(cmd)

    def send_custom_gcode(self):
        cmd = self.in_gcode.text().strip()
        if cmd:
            self.worker.send_command(cmd)
            self.in_gcode.clear()
        self.setFocus()

    def goto_coords(self):
        if not self.is_connected:
            self.on_log_message('err', "⚠️ Conecta la placa primero.")
            return
        try:
            tx = float(self.in_goto_x.text())
            ty = float(self.in_goto_y.text())
            tz = float(self.in_goto_z.text())
        except ValueError:
            self.on_log_message('err', "⚠️ Coordenadas inválidas. Ingresa números válidos en X, Y, Z.")
            return

        r_ground = math.sqrt(tx * tx + ty * ty)
        r_total = math.sqrt(r_ground * r_ground + tz * tz)

        if r_total > 236.0:
            self.on_log_message('err', f"⚠️ Límite geométrico: Radio {r_total:.1f} mm > 236.0 mm. 💡 Consejo: Para avanzar en Y hacia adelante, baja la altura Z.")
            return
        if ty < 15.0:
            self.on_log_message('err', "⚠️ Límite colisión: Y debe ser >= 15 mm para evitar chocar contra la base.")
            return
        if r_total < 35.0:
            self.on_log_message('err', "⚠️ Límite contracción: El brazo no puede plegarse sobre su propio centro (R < 35 mm).")
            return

        self.x = tx
        self.y = ty
        self.z = tz
        cmd = f"G1 X{self.x:.2f} Y{self.y:.2f} Z{self.z:.2f}"
        self.worker.send_command(cmd)
        self.update_telemetry()
        self.on_log_message('info', f"Moviendo a coordenadas: X={self.x:.1f}, Y={self.y:.1f}, Z={self.z:.1f}...")
        self.setFocus()

    def update_telemetry(self):
        # 1. Desplazamiento Relativo respecto al Cero de Trabajo
        dx = self.x - self.zero_x
        dy = self.y - self.zero_y
        dz = self.z - self.zero_z
        self.val_dx[1].setText(f"{dx:+.2f} mm")
        self.val_dy[1].setText(f"{dy:+.2f} mm")
        self.val_dz[1].setText(f"{dz:+.2f} mm")

        # 2. Coordenadas Absolutas
        self.val_x[1].setText(f"{self.x:+.2f} mm")
        self.val_y[1].setText(f"{self.y:.2f} mm")
        self.val_z[1].setText(f"{self.z:.2f} mm")

        # 3. Cinemática Inversa y Extensión
        rot_deg, low_deg, high_deg, r_total = calculate_joint_angles(self.x, self.y, self.z)
        self.val_rot[1].setText(f"{rot_deg:+.1f}°")
        self.val_low[1].setText(f"{low_deg:+.1f}°")
        self.val_high[1].setText(f"{high_deg:.1f}°")

        reach_pct = min(100, int((r_total / 236.0) * 100))
        color = "#4ade80" if reach_pct < 85 else ("#facc15" if reach_pct < 95 else "#ef4444")
        self.val_reach[1].setText(f"{r_total:.1f} mm ({reach_pct}%)")
        margin = max(0.0, 236.0 - r_total)
        self.lbl_bounds.setText(f"• Extensión del Brazo: <font color='{color}'><b>{r_total:.1f} mm / 236 mm ({reach_pct}%)</b> [Margen: {margin:.1f} mm]</font>")

        # 4. Actualizar etiquetas de ángulos en pestaña articular si existen
        if hasattr(self, 'lbl_j1_val'):
            self.lbl_j1_val.setText(f"{rot_deg:+.1f}°")
            self.lbl_j2_val.setText(f"{low_deg:+.1f}°")
            self.lbl_j3_val.setText(f"{high_deg:.1f}°")

        # 5. Indicador dinámico de recorrido lineal X disponible
        max_x = math.sqrt(max(0.0, 236.0**2 - self.y**2 - self.z**2))
        if hasattr(self, 'lbl_linear_limit'):
            self.lbl_linear_limit.setText(
                f"• X Lineal Máx (Y={self.y:.0f}, Z={self.z:.0f}): <font color='#38bdf8'><b>±{max_x:.1f} mm</b></font> "
                f"<i>(Usa Modo Base para rotar ±90°)</i>"
            )

    # --------------------------------------------------------------------------
    # CONEXIÓN Y LOG
    # --------------------------------------------------------------------------
    def refresh_ports(self):
        self.cb_ports.clear()
        ports = list_ports.comports()
        default_index = 0
        for i, p in enumerate(ports):
            desc = f"{p.device} ({p.description})"
            self.cb_ports.addItem(desc, p.device)
            if "USB" in p.device or "FT232" in p.description:
                default_index = i
        if ports:
            self.cb_ports.setCurrentIndex(default_index)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.cb_ports.currentData()
            if not port:
                QMessageBox.warning(self, "Aviso", "No se ha seleccionado ningún puerto serie.")
                return
            baud = int(self.cb_baud.currentText())
            self.lbl_status_badge.setText("🟡 CONECTANDO...")
            self.lbl_status_badge.setStyleSheet("""
                background-color: #422006; color: #facc15; font-weight: bold;
                font-size: 12px; padding: 6px 14px; border: 1px solid #a16207; border-radius: 12px;
            """)
            self.btn_connect.setEnabled(False)
            self.worker.connect_port(port, baud)
        else:
            self.worker.disconnect_port()

    def on_connection_changed(self, connected: bool, message: str):
        self.is_connected = connected
        self.btn_connect.setEnabled(True)
        if connected:
            self.btn_connect.setText("✖ Desconectar")
            self.btn_connect.setStyleSheet("background-color: #991b1b; color: white;")
            self.lbl_status_badge.setText(f"🟢 CONECTADO: {self.cb_ports.currentData()}")
            self.lbl_status_badge.setStyleSheet("""
                background-color: #052e16; color: #4ade80; font-weight: bold;
                font-size: 12px; padding: 6px 14px; border: 1px solid #15803d; border-radius: 12px;
            """)
            self.steppers_enabled = True
            self.lbl_driver_state.setText("• Motores: <font color='#4ade80'><b>ACTIVADOS (M17)</b></font>")
        else:
            self.btn_connect.setText("⚡ Conectar a Placa")
            self.btn_connect.setStyleSheet("background-color: #0284c7; color: white;")
            self.lbl_status_badge.setText("🔴 DESCONECTADO")
            self.lbl_status_badge.setStyleSheet("""
                background-color: #3b181e; color: #f87171; font-weight: bold;
                font-size: 12px; padding: 6px 14px; border: 1px solid #7f1d1d; border-radius: 12px;
            """)

    def on_log_message(self, msg_type: str, text: str):
        color = "#94a3b8"
        if msg_type == 'tx':
            color = "#38bdf8"
        elif msg_type == 'rx':
            color = "#4ade80"
        elif msg_type == 'err':
            color = "#f87171"
        elif msg_type == 'info':
            color = "#fbbf24"

        html = f"<span style='color: {color};'>{text}</span>"
        self.txt_log.append(html)

    def on_response_received(self, resp: str):
        pass

    def closeEvent(self, event):
        self.worker.running = False
        if self.worker.ser and self.worker.ser.is_open:
            try:
                self.worker.ser.close()
            except Exception:
                pass
        event.accept()


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    app = QApplication(sys.argv)
    window = RobotArmGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
