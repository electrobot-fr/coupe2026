#include <AccelStepper.h>

AccelStepper stepper(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 12, 13);

void setup()
{
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  stepper.setMaxSpeed(2000);
  stepper.setSpeed(2000);
  stepper2.setMaxSpeed(2000);
  stepper2.setSpeed(2000);
  stepper3.setMaxSpeed(2000);
  stepper3.setSpeed(2000);
  stepper4.setMaxSpeed(2000);
  stepper4.setSpeed(2000);
}

void loop()
{
  stepper.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
}