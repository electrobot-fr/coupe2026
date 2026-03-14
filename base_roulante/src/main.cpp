#include <FastAccelStepper.h>
#include <SerialTransfer.h>

#define LED_BUILTIN 2
#define MAX_SPEED 4500
#define DEAD_ZONE (MAX_SPEED / 10)
#define ACCELERATION 30000

SerialTransfer transfer;
unsigned long lastPacketTime = 0;
#define WATCHDOG_TIMEOUT 200 // ms sans réception → arrêt moteurs

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
  uint16_t compteur[32]; 
} message;

void move(int A, int B, int R)
{
    // A = X (gauche / droite)
    // B = Y (avant / arrière)
    // R = rotation

    int v1 =  A - B - R;   // avant gauche
    int v2 =  A + B - R;   // arrière gauche
    int v3 =  A + B + R;   // avant droit
    int v4 =  A - B + R;   // arrière droit

    // Normalisation si nécessaire
    int max = abs(v1);
    if (abs(v2) > max) max = abs(v2);
    if (abs(v3) > max) max = abs(v3);
    if (abs(v4) > max) max = abs(v4);

    if (max > MAX_SPEED)
    {
        v1 = v1 * MAX_SPEED / max;
        v2 = v2 * MAX_SPEED / max;
        v3 = v3 * MAX_SPEED / max;
        v4 = v4 * MAX_SPEED / max;
    }

    setStepperTarget(stepper,  v1);
    setStepperTarget(stepper3, -v2);
    setStepperTarget(stepper2,  -v3);
    setStepperTarget(stepper4,  v4);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // Enable motors

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

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  transfer.begin(Serial2);
  Serial.begin(115200);
}

void loop()
{
  if (transfer.available())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    lastPacketTime = millis();
    uint16_t recSize = 0;
    recSize = transfer.rxObj(message, recSize);

    int x = map(message.x, 0, 1023, -MAX_SPEED, MAX_SPEED);
    int y = map(message.y, 0, 1023, -MAX_SPEED, MAX_SPEED);
    int z = map(message.z, 0, 1023, -MAX_SPEED, MAX_SPEED);

    if (abs(x) <= DEAD_ZONE) x = 0;
    if (abs(y) <= DEAD_ZONE) y = 0;
    if (abs(z) <= DEAD_ZONE) z = 0;

    move(-x, -y, -z);
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Watchdog : arrêt moteurs si perte de communication
  if (millis() - lastPacketTime > WATCHDOG_TIMEOUT)
  {
    move(0, 0, 0);
  }
}
