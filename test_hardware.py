#!/usr/bin/env python3
"""
Herramienta de Diagnostico y Prueba Individual de Hardware para Brazo Robot (MKS Gen v1.4)
Permite verificar de forma aislada:
 1. Ventilador de drivers (D9)
 2. Torque de retencion en todos los motores (M17 / M18)
 3. Movimiento suave eje por eje (Base, Hombro, Codo, Pinza)
"""

import sys
import time
try:
    import serial
except ImportError:
    print("Error: Requiere pyserial. Instalar con: pip install pyserial")
    sys.exit(1)

PORT = "/dev/ttyUSB0"
BAUD = 9600

def send_cmd(ser, cmd):
    print(f"  >> Enviando: {cmd}")
    ser.write((cmd + "\r\n").encode("ascii"))
    time.sleep(0.1)
    resp = ""
    start = time.time()
    while time.time() - start < 1.5:
        if ser.in_waiting:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                resp += line + " "
                if "ok" in line:
                    break
        time.sleep(0.02)
    print(f"  << Respuesta: {resp.strip() or 'Sin respuesta'}")
    return resp

def connect():
    print(f"\n[1/3] Conectando a {PORT} a {BAUD} baudios...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.5)
        time.sleep(2.0)
        init_resp = ser.read(ser.in_waiting or 100).decode("ascii", errors="replace").strip()
        print(f"  [OK] Placa inicializada: '{init_resp or 'start'}'")
        return ser
    except Exception as e:
        print(f"  [ERROR] No se pudo abrir {PORT}: {e}")
        sys.exit(1)

def menu():
    print("""
================================================================
    PANEL DE DIAGNÓSTICO INDIVIDUAL DE HARDWARE (MKS Gen v1.4)
================================================================
  [1] Probar Ventilador (D9) - Encender 3 seg y apagar (M106 / M107)
  [2] Activar Motores (M17) - Verificar torque de retención firme
  [3] Desactivar Motores (M18) - Comprobar que queden libres
  --------------------------------------------------------------
  [4] Probar BASE (Eje Z) - Pequeño giro izquierda y derecha
  [5] Probar HOMBRO (Eje Y) - Pequeño movimiento adelante y atrás
  [6] Probar CODO (Eje X) - Pequeño movimiento arriba y abajo
  [7] Probar PINZA (Gripper) - Cerrar y abrir suave (M3 T600 / M5 T600)
  [8] Test Diagnóstico LEDs ULN2003 - Secuencia lenta 1 a 1 (M3 T0)
  --------------------------------------------------------------
  [0] Salir y desactivar todo (M18 + M107)
================================================================
""")

def main():
    ser = connect()
    try:
        while True:
            menu()
            op = input("Selecciona una prueba [0-7]: ").strip()
            
            if op == "0":
                print("\nApagando todo de forma segura...")
                send_cmd(ser, "M18")
                send_cmd(ser, "M107")
                break
                
            elif op == "1":
                print("\n--- PRUEBA 1: Ventilador de Drivers (D9) ---")
                print("Encendiendo ventilador...")
                send_cmd(ser, "M106")
                print("Esperando 3 segundos (verifica si el ventilador gira)...")
                time.sleep(3)
                print("Apagando ventilador...")
                send_cmd(ser, "M107")
                
            elif op == "2":
                print("\n--- PRUEBA 2: Enclave de Motores (M17) ---")
                print("Activando corriente en los drivers...")
                send_cmd(ser, "M17")
                print("-> Comprobación: Intenta mover suavemente cada motor a mano.")
                print("   TODOS los motores deben estar duros/bloqueados con torque de retención.")
                input("Presiona Enter para continuar...")
                
            elif op == "3":
                print("\n--- PRUEBA 3: Liberación de Motores (M18) ---")
                send_cmd(ser, "M18")
                print("-> Comprobación: Intenta mover cada eje a mano. Deben estar suaves y libres.")
                input("Presiona Enter para continuar...")
                
            elif op == "4":
                print("\n--- PRUEBA 4: Eje Z (Rotación de Base) ---")
                send_cmd(ser, "M17")
                print("Moviendo base a la derecha (X=+15mm)...")
                send_cmd(ser, "G1 X15 Y120 Z120")
                time.sleep(1.5)
                print("Moviendo base a la izquierda (X=-15mm)...")
                send_cmd(ser, "G1 X-15 Y120 Z120")
                time.sleep(1.5)
                print("Regresando al centro (X=0)...")
                send_cmd(ser, "G1 X0 Y120 Z120")
                
            elif op == "5":
                print("\n--- PRUEBA 5: Eje Y (Hombro / Brazo Inferior) ---")
                send_cmd(ser, "M17")
                print("Extendiendo hombro adelante (Y=135mm)...")
                send_cmd(ser, "G1 X0 Y135 Z120")
                time.sleep(1.5)
                print("Retrocediendo hombro (Y=105mm)...")
                send_cmd(ser, "G1 X0 Y105 Z120")
                time.sleep(1.5)
                print("Regresando a Home (Y=120mm)...")
                send_cmd(ser, "G1 X0 Y120 Z120")
                
            elif op == "6":
                print("\n--- PRUEBA 6: Eje X (Codo / Brazo Superior) ---")
                send_cmd(ser, "M17")
                print("Elevando codo (Z=135mm)...")
                send_cmd(ser, "G1 X0 Y120 Z135")
                time.sleep(1.5)
                print("Bajando codo (Z=105mm)...")
                send_cmd(ser, "G1 X0 Y120 Z105")
                time.sleep(1.5)
                print("Regresando a Home (Z=120mm)...")
                send_cmd(ser, "G1 X0 Y120 Z120")
                
            elif op == "7":
                print("\n--- PRUEBA 7: Pinza (Gripper 28BYJ-48 + ULN2003) ---")
                print("Cerrando pinza suave (M3 T600)...")
                send_cmd(ser, "M3 T600")
                time.sleep(2.5)
                print("Abriendo pinza suave (M5 T600)...")
                send_cmd(ser, "M5 T600")
                time.sleep(2.5)

            elif op == "8":
                print("\n--- PRUEBA 8: Test Secuencial de LEDs del ULN2003 ---")
                print("Enviando M3 T0 a la placa...")
                ser.write(b"M3 T0\r\n")
                start_t = time.time()
                while time.time() - start_t < 8.0:
                    if ser.in_waiting:
                        line = ser.readline().decode("ascii", errors="replace").strip()
                        if line:
                            print(f"  {line}")
                            if "Secuencia finalizada" in line or "ok" in line and time.time() - start_t > 6.0:
                                break
                    time.sleep(0.05)
                print("\nSecuencia completada.")
                
            else:
                print("Opción no válida.")
    finally:
        send_cmd(ser, "M18")
        ser.close()
        print("\nPuerto serie cerrado. Prueba finalizada.")

if __name__ == "__main__":
    main()
