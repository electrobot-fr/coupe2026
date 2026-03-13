#include <Wire.h>
#include <SerialTransfer.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

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
    digitalWrite(LED_BUILTIN, HIGH);
    uint16_t recSize = 0;
    recSize = transfer.rxObj(message, recSize);

    for (size_t i = 0; i < 16; i++)
    {
      pwm1.setPWM(i, 0, message.compteur[i]);
      pwm2.setPWM(i, 0, message.compteur[16 + i]);
    }
  }
}
