#ifndef GRIPPER_H_
#define GRIPPER_H_

#include <Arduino.h>

void gripperInit();
void gripperOff();
void gripperStep(int steps, int stepDelayUs = 1800);
void gripperTestSequence();

#endif // GRIPPER_H_
