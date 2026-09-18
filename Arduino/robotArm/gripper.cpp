#include "gripper.h"

// Mapeo exacto segun conexion fisica AUX-2 -> ULN2003:
// IN1: D40 (AUX-2 pin 6)
// IN2: D63 (AUX-2 pin 4)
// IN3: D59 (AUX-2 pin 3)
// IN4: D64 (AUX-2 pin 5)
#define GRIPPER_IN1 40
#define GRIPPER_IN2 63
#define GRIPPER_IN3 59
#define GRIPPER_IN4 64

static const uint8_t GRIPPER_PINS[4] = {GRIPPER_IN1, GRIPPER_IN2, GRIPPER_IN3, GRIPPER_IN4};

// Secuencia Half-Step (8 medios pasos) para 28BYJ-48 + ULN2003
// Maximo torque, movimiento suave y sin vibracion
static const uint8_t HALF_STEP_SEQ[8][4] = {
  {HIGH, LOW,  LOW,  LOW},   // 1. IN1
  {HIGH, HIGH, LOW,  LOW},   // 2. IN1 + IN2
  {LOW,  HIGH, LOW,  LOW},   // 3. IN2
  {LOW,  HIGH, HIGH, LOW},   // 4. IN2 + IN3
  {LOW,  LOW,  HIGH, LOW},   // 5. IN3
  {LOW,  LOW,  HIGH, HIGH},  // 6. IN3 + IN4
  {LOW,  LOW,  LOW,  HIGH},  // 7. IN4
  {HIGH, LOW,  LOW,  HIGH}   // 8. IN4 + IN1
};

static int currentStep = 0;

void gripperInit() {
  for (int i = 0; i < 4; i++) {
    pinMode(GRIPPER_PINS[i], OUTPUT);
    digitalWrite(GRIPPER_PINS[i], LOW);
  }
}

void gripperOff() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(GRIPPER_PINS[i], LOW);
  }
}

// Prueba secuencial lenta: prende cada LED durante 1200 ms con mensaje serie
void gripperTestSequence() {
  gripperOff();
  const char* names[4] = {"LED A (IN1 - Pin D40)", "LED B (IN2 - Pin D63)", "LED C (IN3 - Pin D59)", "LED D (IN4 - Pin D64)"};
  for (int i = 0; i < 4; i++) {
    Serial.print("// [TEST] Encendiendo: ");
    Serial.println(names[i]);
    digitalWrite(GRIPPER_PINS[i], HIGH);
    delay(1200);
    digitalWrite(GRIPPER_PINS[i], LOW);
    delay(400);
  }
  gripperOff();
  Serial.println("// [TEST] Secuencia finalizada.");
}

void gripperStep(int steps, int stepDelayUs) {
  int dir = (steps >= 0) ? 1 : -1;
  int count = abs(steps);

  for (int s = 0; s < count; s++) {
    currentStep = (currentStep + dir + 8) % 8;
    for (int p = 0; p < 4; p++) {
      digitalWrite(GRIPPER_PINS[p], HALF_STEP_SEQ[currentStep][p]);
    }
    delayMicroseconds(stepDelayUs);
  }
  // Apagar todas las bobinas para no consumir corriente ni calentar el motor en reposo
  gripperOff();
}
