#include "Arduino.h"
#include "Wire.h"
#include "TM1637Display.h"
TM1637Display display(3, 2); // CLK, DIO
#include "SerialTransfer.h"

SerialTransfer transfer;
uint8_t buttonState;

bool buttonSeqPrevUp;
bool buttonSeqPrevDown;

struct __attribute__((packed)) STRUCT
{
  int16_t x;
  int16_t y;
  int16_t z;
  uint16_t compteur[32]; 
} message = {};

int16_t compteur = 0;
int16_t afficheur = 0;
int16_t afficheurPrev = 1;

const uint8_t NUM_STATES = 2;

// #define DEBUG

void setup()
{
  Serial.begin(115200);
#ifndef DEBUG
  transfer.begin(Serial);
#endif

  buttonState = 0;
  buttonSeqPrevDown = false;
  buttonSeqPrevUp = false;
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);

  // Initialize buttons with INPUT_PULLUP mode
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);  // Button 1 to add 1
  pinMode(9, INPUT_PULLUP);  // Button 2 to subtract 1
  pinMode(10, INPUT_PULLUP); // Button 3 to add 5
  pinMode(11, INPUT_PULLUP); // Button 4 to subtract 5
  pinMode(12, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  // Set brightness of the display
  display.setBrightness(4);
}

void loop()
{
  message.x = analogRead(A3);
  message.y = map(analogRead(A2), 0, 1023, 1023, 0);
  message.z = analogRead(A0);


  bool btn8 = (digitalRead(8) == LOW);
  if (btn8 && !buttonSeqPrevUp)
  {
    buttonState = (buttonState + 1) % NUM_STATES;
  }
  buttonSeqPrevUp = btn8;

  bool btn9 = (digitalRead(9) == LOW);
  if (btn9 && !buttonSeqPrevDown)
  {
    buttonState = (buttonState - 1 + NUM_STATES) % NUM_STATES;
  }
  buttonSeqPrevDown = btn9;

  switch (buttonState)
  {
  case 0:
    message.compteur[0] = 200;
    break;
  case 1:
    message.compteur[0] = 400;
    break;
  }

  afficheur = buttonState;

  if (afficheur != afficheurPrev)
  {
    display.showNumberDec(afficheur);
    delay(50);
  }
  afficheurPrev = afficheur;

#ifndef DEBUG
  // Send the message using SerialTransfer
  uint16_t sendSize = 0;
  sendSize = transfer.txObj(message, sendSize);
  transfer.sendData(sendSize);
#endif


  delay(50);
}
