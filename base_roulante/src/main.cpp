
#include <AccelStepper.h>
#include "Arduino.h"
#include <SerialTransfer.h>
#include "Wire.h"

#define DEBUG 0
#define MAX_SPEED 2000
#define DEAD_ZONE MAX_SPEED/10

SerialTransfer transfer;

AccelStepper stepper(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 12, 13);

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
  // Initialisation du port série
  Serial.begin(115200);
  Serial2.begin(115200);
  transfer.begin(Serial2);

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  stepper.setMaxSpeed(MAX_SPEED);
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper3.setMaxSpeed(MAX_SPEED);
  stepper4.setMaxSpeed(MAX_SPEED);

  avancer(0);
}

int x_prev = 0;

void loop()
{
  if (transfer.available())
  {
    uint16_t recSize = 0;
    recSize = transfer.rxObj(message, recSize);

    int x = map(message.x, 0, 1023, -MAX_SPEED, MAX_SPEED);
    int y = map(message.y, 0, 1023, -MAX_SPEED, MAX_SPEED);
    int z = map(message.z, 0, 1023, -MAX_SPEED, MAX_SPEED);

#if DEBUG
    Serial.print("x=");
    Serial.print(x);
    Serial.print("\ty=");
    Serial.print(y);
    Serial.print("\tz=");
    Serial.print(z);
    Serial.print("\n");
#endif

    if (abs(x) <= DEAD_ZONE && abs(y) <= DEAD_ZONE && abs(z) <= DEAD_ZONE) {
      avancer(0);
    } 
    else{
      if (abs(x) > abs(y) && abs(x) > abs(z)) 
       { avancer(x);
        #if DEBUG
        Serial.println("avancer: " + String(x) + "\n");
        #endif
       }
       else if (abs(y) > abs(x) && abs(y) > abs(z))
       { translater(y);
         #if DEBUG
         Serial.println("translater: " + String(y) + "\n");
         #endif
       }
       else if (abs(z) > abs(x) && abs(z) > abs(y))
       { pivoter(z);
          #if DEBUG
           Serial.println("pivoter: " + String(z) + "\n");
          #endif
       }
  }
}
  stepper.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
 }

