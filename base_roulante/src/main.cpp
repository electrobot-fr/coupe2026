#include <FastAccelStepper.h>
#include <SerialTransfer.h>
#include "Wire.h"

#define LED_BUILTIN 2
#define MAX_SPEED 3500
#define DEAD_ZONE MAX_SPEED / 10
#define ACCELERATION 30000

SerialTransfer transfer;

// FastAccelStepper engine and steppers
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;

// helper to set continuous speed for a FastAccelStepper (signed steps/s)
static void setStepperTarget(FastAccelStepper *s, int speed)
{
  if (!s)
    return;
  if (speed == 0)
  {
    s->stopMove();
    return;
  }
  uint32_t sp = (uint32_t)abs(speed);
  s->setSpeedInHz(sp);
  if (speed > 0)
    s->runForward();
  else
    s->runBackward();
  s->applySpeedAcceleration();
}

struct __attribute__((packed)) STRUCT
{
  int16_t x;
  int16_t y;
  int16_t z;
  bool cmdGliss;         // Glissière : 0: retracter / 1: deployer
  bool cmdAimantInt;     // Pince aimant interieur 0: détacher / 1: attacher
  bool cmdAimantExt;     // Pince aimant exterieur
  bool cmdPompe;         // Commande Pompe : 0: Off / 1: On
  bool cmdVanne;         // Commande Electrovanne : 0: Off / 1: On
  bool cmdServoPlanche;  // Lever les planches
  bool cmdServoBanniere; // Lacher la banniere
  int16_t ascPlanche;    // Position de l'ascenceur des planches
  int16_t ascBoites;     // Position de l'ascenceur des boites
  uint8_t compteur;      // Compteur
} message;

void avancer(int speed)
{
  setStepperTarget(stepper, speed);
  setStepperTarget(stepper2, speed);
  setStepperTarget(stepper3, speed);
  setStepperTarget(stepper4, speed);
}

void translater(int speed)
{
  setStepperTarget(stepper, speed);
  setStepperTarget(stepper2, -speed);
  setStepperTarget(stepper3, speed);
  setStepperTarget(stepper4, -speed);
}

void pivoter(int speed)
{
  setStepperTarget(stepper, -speed);
  setStepperTarget(stepper2, -speed);
  setStepperTarget(stepper3, speed);
  setStepperTarget(stepper4, speed);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  int speed = 3000;
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // CHANGE ME

  // Initialize FastAccelStepper engine and allocate steppers
  engine.init();
  stepper = engine.stepperConnectToPin(14);
  stepper2 = engine.stepperConnectToPin(27);
  stepper3 = engine.stepperConnectToPin(25);
  stepper4 = engine.stepperConnectToPin(15);
  if (stepper)
  {
    stepper->setDirectionPin(12);
    stepper->setAutoEnable(true);
    stepper->setAcceleration(ACCELERATION);
    stepper->setSpeedInHz(0);
  }
  if (stepper2)
  {
    stepper2->setDirectionPin(26);
    stepper2->setAutoEnable(true);
    stepper2->setAcceleration(ACCELERATION);
    stepper2->setSpeedInHz(0);
  }
  if (stepper3)
  {
    stepper3->setDirectionPin(33);
    stepper3->setAutoEnable(true);
    stepper3->setAcceleration(ACCELERATION);
    stepper3->setSpeedInHz(0);
  }
  if (stepper4)
  {
    stepper4->setDirectionPin(32);
    stepper4->setAutoEnable(true);
    stepper4->setAcceleration(ACCELERATION);
    stepper4->setSpeedInHz(0);
  }

  avancer(0);

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  transfer.begin(Serial2);
  Serial.begin(115200);
}

void loop()
{
  if (transfer.available())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    uint16_t recSize = 0;
    recSize = transfer.rxObj(message, recSize);

    int x = map(message.x, 0, 1023, -MAX_SPEED, MAX_SPEED);
    int y = map(message.y, 0, 1023, -MAX_SPEED, MAX_SPEED);
    int z = map(message.z, 0, 1023, -MAX_SPEED, MAX_SPEED);

    if (abs(x) <= DEAD_ZONE && abs(y) <= DEAD_ZONE && abs(z) <= DEAD_ZONE)
    {
      avancer(0);
    }
    else
    {
      if (abs(x) > abs(y) && abs(x) > abs(z))
      {
        avancer(x);
      }
      else if (abs(y) > abs(x) && abs(y) > abs(z))
      {
        translater(y);
      }
      else if (abs(z) > abs(x) && abs(z) > abs(y))
      {
        pivoter(z);
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
}
