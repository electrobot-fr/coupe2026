#include <FastAccelStepper.h>
#include <SerialTransfer.h>

#define DEBUG_SERIAL // décommenter pour afficher le debug série

#define LED_BUILTIN 2
#define MAX_SPEED 3000
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

// BNO055 en UART sur le connecteur J1 de carte_moteurs (GPIO18 / GPIO21).
// TX/RX sont détectés au démarrage : si le BNO ne répond pas, on inverse.
#define BNO_PIN_A 18 // J1.3 (OLED_SCK)
#define BNO_PIN_B 21 // J1.4 (OLED_SDA)

#define REG_OPR_MODE 0x3D
#define REG_EULER    0x1A
#define MODE_IMU     0x08 // gyro + accéléro, sans magnéto : cap 0 = orientation au démarrage

// Asservissement en cap
#define CTRL_PERIOD  20     // ms (50 Hz)
#define KP           120.0f  // (steps/s) par degré d'erreur
#define KD           4.0f   // (steps/s) par degré/s
#define ANGLE_DEADBAND 1.0f // degrés : pas de correction en dessous
#define MAX_ROT      MAX_SPEED
#define ROT_SIGN     -1     // inverser (-1) si le robot s'éloigne de la consigne

unsigned long lastCtrlTime = 0;
bool bnoOk = false;
int cmdX = 0, cmdY = 0;
float lastError = 0;

bool readRegs(uint8_t reg, uint8_t len, uint8_t *out)
{
  while (Serial1.available()) Serial1.read();
  uint8_t cmd[] = {0xAA, 0x01, reg, len};
  Serial1.write(cmd, 4);

  uint32_t t0 = millis();
  while (Serial1.available() < 2)
  {
    if (millis() - t0 > 100) return false;
  }
  uint8_t hdr = Serial1.read();
  uint8_t n = Serial1.read();
  if (hdr != 0xBB) return false;

  t0 = millis();
  for (uint8_t i = 0; i < n; i++)
  {
    while (!Serial1.available())
    {
      if (millis() - t0 > 100) return false;
    }
    out[i] = Serial1.read();
  }
  return true;
}

bool writeReg(uint8_t reg, uint8_t val)
{
  while (Serial1.available()) Serial1.read();
  uint8_t cmd[] = {0xAA, 0x00, reg, 0x01, val};
  Serial1.write(cmd, 5);

  uint32_t t0 = millis();
  while (Serial1.available() < 2)
  {
    if (millis() - t0 > 100) return false;
  }
  uint8_t hdr = Serial1.read();
  uint8_t status = Serial1.read();
  return (hdr == 0xEE && status == 0x01);
}

struct __attribute__((packed)) STRUCT
{
  int16_t x;
  int16_t y;
  int16_t z; // consigne d'angle absolue (degrés), envoyée par la télécommande
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

  delay(700); // temps de démarrage du BNO055
#ifdef DEBUG_SERIAL
  Serial.println("\nbase_roulante : init");
#endif

  // Essai RX=18/TX=21, puis inversé si pas de réponse
  Serial1.begin(115200, SERIAL_8N1, BNO_PIN_A, BNO_PIN_B);
  bool ok = writeReg(REG_OPR_MODE, MODE_IMU);
  int rxPin = BNO_PIN_A, txPin = BNO_PIN_B;
  if (!ok)
  {
    Serial1.end();
    Serial1.begin(115200, SERIAL_8N1, BNO_PIN_B, BNO_PIN_A);
    ok = writeReg(REG_OPR_MODE, MODE_IMU);
    rxPin = BNO_PIN_B;
    txPin = BNO_PIN_A;
  }
  bnoOk = ok; // sinon pas d'asservissement en cap (LED éteinte)
#ifdef DEBUG_SERIAL
  if (ok)
    Serial.printf("BNO055 OK (mode IMU, RX=GPIO%d TX=GPIO%d)\n", rxPin, txPin);
  else
    Serial.println("BNO055 ne repond pas : pas d'asservissement en cap");
  Serial.println("En attente des paquets de la telecommande...");
#endif
  delay(30);
}

// Ramène un angle dans [-180, 180[
static float wrap180(float a)
{
  while (a >= 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Correction de rotation (steps/s) pour tenir le cap consigne (degrés)
static int headingCorrection(float consigne)
{
  uint8_t buf[2];
  if (!bnoOk || !readRegs(REG_EULER, 2, buf))
    return 0;

  float heading = (int16_t)(buf[0] | (buf[1] << 8)) / 16.0f;
  float error = wrap180(consigne - heading);
  float dError = wrap180(error - lastError) * 1000.0f / CTRL_PERIOD;
  lastError = error;

  float r = 0;
  if (fabsf(error) > ANGLE_DEADBAND)
    r = KP * error + KD * dError;
  r = constrain(r, -MAX_ROT, MAX_ROT);

#ifdef DEBUG_SERIAL
  Serial.printf(">t:%lu\n>consigne:%.2f\n>mesure:%.2f\n>reponse:%d\n", millis(), consigne, wrap180(heading), ROT_SIGN * (int)r);
#endif

  // LED basculée à chaque cycle d'asserv : clignote à CTRL_PERIOD * 2
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  return ROT_SIGN * (int)r;
}

void loop()
{
  if (transfer.available())
  {
    lastPacketTime = millis();
    uint16_t recSize = 0;
    recSize = transfer.rxObj(message, recSize);

    cmdX = map(message.x, 0, 1023, -MAX_SPEED, MAX_SPEED);
    cmdY = map(message.y, 0, 1023, -MAX_SPEED, MAX_SPEED);

    if (abs(cmdX) <= DEAD_ZONE) cmdX = 0;
    if (abs(cmdY) <= DEAD_ZONE) cmdY = 0;
  }

  // Watchdog : arrêt moteurs si perte de communication
  if (millis() - lastPacketTime > WATCHDOG_TIMEOUT)
  {
    move(0, 0, 0);
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if (millis() - lastCtrlTime >= CTRL_PERIOD)
  {
    lastCtrlTime = millis();
    move(-cmdX, -cmdY, headingCorrection(message.z));
  }
}
