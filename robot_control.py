#!/usr/bin/env python3
"""
Controlador interactivo en Python para el Robot Arm (Florin Tobler / RAMPS 1.4 / MKS Gen v1.4).
Permite conexión serie, control manual de ejes (jogging), encendido/apagado de motores y secuencias G-code.
"""

import sys
import time
import argparse
try:
    import serial
except ImportError:
    print("Error: Requiere pyserial. Instalar con: pip install pyserial")
    sys.exit(1)


class RobotArm:
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.x = 0.0
        self.y = 120.0
        self.z = 120.0
        self.e = 0.0

    def connect(self):
        print(f"Conectando a {self.port} a {self.baudrate} baudios...")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2.0)  # Esperar reinicio tras abrir puerto DTR
        init_resp = self.ser.read(self.ser.in_waiting or 100).decode("ascii", errors="replace").strip()
        print(f"Respuesta de inicialización de la placa: {init_resp}")
        return True

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Puerto serie cerrado.")

    def send_command(self, cmd):
        """Envía comando G-code terminado en \\r\\n y espera respuesta ok"""
        cmd_clean = cmd.strip()
        if not cmd_clean:
            return ""
        if not self.ser or not self.ser.is_open:
            print("Error: No conectado.")
            return ""
        
        # El firmware de robotArm requiere retorno de carro \r o \r\n
        msg = (cmd_clean + "\r\n").encode("ascii")
        self.ser.write(msg)
        time.sleep(0.05)
        
        # Leer respuesta
        response = ""
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode("ascii", errors="replace")
                response += line
                if "ok" in line or "!!" in line or "rs" in line:
                    break
            time.sleep(0.02)
        return response.strip()

    def enable_steppers(self, enable=True):
        cmd = "M17" if enable else "M18"
        resp = self.send_command(cmd)
        state = "ACTIVADOS" if enable else "DESACTIVADOS"
        print(f"Motores {state}: {resp}")

    def enable_fan(self, enable=True):
        cmd = "M106" if enable else "M107"
        resp = self.send_command(cmd)
        state = "ENCENDIDO" if enable else "APAGADO"
        print(f"Ventilador {state}: {resp}")

    def goto(self, x=None, y=None, z=None, e=None, f=None):
        if x is not None: self.x = float(x)
        if y is not None: self.y = float(y)
        if z is not None: self.z = float(z)
        if e is not None: self.e = float(e)
        
        cmd = f"G1 X{self.x:.2f} Y{self.y:.2f} Z{self.z:.2f}"
        if e is not None:
            cmd += f" E{self.e:.2f}"
        if f is not None:
            cmd += f" F{f:.2f}"
        
        print(f"Moviendo a: Base/Z(X)={self.x:.2f}, Hombro/Y(Y)={self.y:.2f}, Codo/X(Z)={self.z:.2f}")
        resp = self.send_command(cmd)
        print(f"Respuesta: {resp}")

    def gripper(self, close=True, duration=50):
        cmd = f"M3 T{duration}" if close else f"M5 T{duration}"
        action = "Cerrando" if close else "Abriendo"
        print(f"{action} pinza ({duration} pasos)...")
        resp = self.send_command(cmd)
        print(f"Respuesta: {resp}")

    def home(self):
        """Mueve a la posición inicial por defecto (0, 120, 120)"""
        print("Moviendo a posición HOME (X=0, Y=120, Z=120)...")
        self.goto(x=0, y=120, z=120)


def interactive_mode(robot):
    step_coarse = 10.0
    step_fine = 2.0
    
    help_text = """
Comandos disponibles en el modo interactivo:
  [m17]  Activar motores paso a paso
  [m18]  Desactivar motores (para posicionar a mano)
  [home] Ir a posición Home (0, 120, 120)
  [fan1] Encender ventilador / [fan0] Apagar ventilador
  [gc]   Cerrar pinza / [go] Abrir pinza
  ----------------------------------------------------------------------
  Mapeo Hardware: Base = Driver Z | Hombro = Driver Y | Codo = Driver X
  ----------------------------------------------------------------------
  [x+] / [x-] Mover BASE giratoria (G1 X / Driver Z) (+/- coarse)
  [y+] / [y-] Mover HOMBRO / Alcance (G1 Y / Driver Y) (+/- coarse)
  [z+] / [z-] Mover CODO / Altura (G1 Z / Driver X) (+/- coarse)
  [fx+] / [fx-] / [fy+] / [fy-] / [fz+] / [fz-] Mover en pasos finos
  [goto X Y Z] Ir a coordenadas específicas (ej: goto 0 100 80)
  [raw <cmd>] Enviar comando G-code directo (ej: raw G1 X20 Y100 Z90)
  [pos]  Ver posición cartesiana y correspondencia con motores
  [help] Mostrar esta ayuda
  [quit] o [exit] Salir
"""
    print(help_text)
    
    while True:
        try:
            prompt = f"RobotArm (X:{robot.x:.1f} Y:{robot.y:.1f} Z:{robot.z:.1f}) > "
            user_in = input(prompt).strip()
            if not user_in:
                continue
            
            cmd = user_in.lower()
            if cmd in ("exit", "quit", "q"):
                break
            elif cmd == "help":
                print(help_text)
            elif cmd == "m17":
                robot.enable_steppers(True)
            elif cmd == "m18":
                robot.enable_steppers(False)
            elif cmd == "home":
                robot.home()
            elif cmd == "fan1":
                robot.enable_fan(True)
            elif cmd == "fan0":
                robot.enable_fan(False)
            elif cmd == "gc":
                robot.gripper(close=True, duration=50)
            elif cmd == "go":
                robot.gripper(close=False, duration=50)
            elif cmd == "x+":
                robot.goto(x=robot.x + step_coarse)
            elif cmd == "x-":
                robot.goto(x=robot.x - step_coarse)
            elif cmd == "y+":
                robot.goto(y=robot.y + step_coarse)
            elif cmd == "y-":
                robot.goto(y=robot.y - step_coarse)
            elif cmd == "z+":
                robot.goto(z=robot.z + step_coarse)
            elif cmd == "z-":
                robot.goto(z=robot.z - step_coarse)
            elif cmd == "fx+":
                robot.goto(x=robot.x + step_fine)
            elif cmd == "fx-":
                robot.goto(x=robot.x - step_fine)
            elif cmd == "fy+":
                robot.goto(y=robot.y + step_fine)
            elif cmd == "fy-":
                robot.goto(y=robot.y - step_fine)
            elif cmd == "fz+":
                robot.goto(z=robot.z + step_fine)
            elif cmd == "fz-":
                robot.goto(z=robot.z - step_fine)
            elif cmd == "pos":
                print(f"Posición actual:\n  - Base  (G1 X, Driver Z): {robot.x:.2f} mm\n  - Hombro (G1 Y, Driver Y): {robot.y:.2f} mm\n  - Codo   (G1 Z, Driver X): {robot.z:.2f} mm")
            elif cmd.startswith("goto "):
                parts = user_in.split()
                if len(parts) >= 4:
                    robot.goto(x=float(parts[1]), y=float(parts[2]), z=float(parts[3]))
                else:
                    print("Uso: goto <X> <Y> <Z>")
            elif cmd.startswith("raw "):
                raw_cmd = user_in[4:].strip()
                resp = robot.send_command(raw_cmd)
                print(f"Respuesta: {resp}")
            else:
                resp = robot.send_command(user_in)
                print(f"Respuesta: {resp}")
        except KeyboardInterrupt:
            print("\nInterrupción recibida.")
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(description="Control de Robot Arm MKS Gen v1.4")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Puerto serie (def: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="Baudrate (def: 9600)")
    parser.add_argument("-c", "--cmd", help="Comando individual a ejecutar")
    parser.add_argument("-f", "--file", help="Archivo de texto/G-code con secuencia de comandos a ejecutar")
    args = parser.parse_args()

    robot = RobotArm(port=args.port, baudrate=args.baud)
    try:
        robot.connect()
    except Exception as e:
        print(f"No se pudo conectar a {args.port}: {e}")
        sys.exit(1)

    try:
        if args.cmd:
            resp = robot.send_command(args.cmd)
            print(f"Respuesta: {resp}")
        elif args.file:
            print(f"Ejecutando secuencia desde {args.file}...")
            with open(args.file, "r") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("//") and not line.startswith("#"):
                        print(f">> {line}")
                        resp = robot.send_command(line)
                        print(f"<< {resp}")
                        time.sleep(0.1)
        else:
            interactive_mode(robot)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
