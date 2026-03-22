#include <Wire.h>
#include <SerialTransfer.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50

#define LED_BUILTIN 2

struct __attribute__((packed)) STRUCT
{
  int16_t x;
  int16_t y;
  int16_t z;
  uint16_t compteur[32]; 
} message;

SerialTransfer transfer;

uint16_t lastCompteur[32] = {0};
unsigned long lastReceived = 0;
const unsigned long TIMEOUT_MS = 500;
bool ledState = false;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  transfer.begin(Serial2);

  // Serial.begin(115200);
  // transfer.begin(Serial);

  pwm1.begin();

  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);

  pwm2.begin();

  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);
}

void loop()
{
  if (transfer.available())
  {
    lastReceived = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

    uint16_t recSize = 0;
    recSize = transfer.rxObj(message, recSize);

    for (size_t i = 0; i < 16; i++)
    {
      uint16_t val1 = min(message.compteur[i], (uint16_t)4095);
      uint16_t val2 = min(message.compteur[16 + i], (uint16_t)4095);

      if (val1 != lastCompteur[i])
      {
        pwm1.setPWM(i, 0, val1);
        lastCompteur[i] = val1;
      }
      if (val2 != lastCompteur[16 + i])
      {
        pwm2.setPWM(i, 0, val2);
        lastCompteur[16 + i] = val2;
      }
    }
  }

  if (millis() - lastReceived > TIMEOUT_MS)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
