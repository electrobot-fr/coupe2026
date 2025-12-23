#include <AccelStepper.h>
#include <SerialTransfer.h>
#include "Wire.h"

#define LED_BUILTIN 2
#define MAX_SPEED 2500
#define DEAD_ZONE MAX_SPEED / 10

SerialTransfer transfer;

AccelStepper stepper(1, 14, 12);
AccelStepper stepper2(1, 27, 26);
AccelStepper stepper3(1, 25, 33);
AccelStepper stepper4(1, 15, 32);

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
  stepper.setSpeed(speed);
  stepper2.setSpeed(speed);
  stepper3.setSpeed(speed);
  stepper4.setSpeed(speed);
}

void translater(int speed)
{
  stepper.setSpeed(speed);
  stepper2.setSpeed(-speed);
  stepper3.setSpeed(speed);
  stepper4.setSpeed(-speed);
}

void pivoter(int speed)
{
  stepper.setSpeed(-speed);
  stepper2.setSpeed(-speed);
  stepper3.setSpeed(speed);
  stepper4.setSpeed(speed);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  int speed = 3000;
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // CHANGE ME

  stepper.setMaxSpeed(MAX_SPEED);
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper3.setMaxSpeed(MAX_SPEED);
  stepper4.setMaxSpeed(MAX_SPEED);

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
    digitalWrite(LED_BUILTIN, LOW);

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
  }
  stepper.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
}
