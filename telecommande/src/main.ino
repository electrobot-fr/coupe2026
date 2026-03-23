#include "Arduino.h"
#include "Wire.h"
#include "TM1637Display.h"
TM1637Display display(3, 2); // CLK, DIO
#include "SerialTransfer.h"

SerialTransfer transfer;
uint8_t buttonState;
uint8_t buttonState2;

bool buttonSeqPrevUp;
bool buttonSeqPrevDown;

struct __attribute__((packed)) STRUCT
{
  int16_t x;
  int16_t y;
  int16_t z;
  // Vannes: 0-3
  // Pompes: 4-7
  // Servo bras: 8-11
  // Servo tapis: 11-15
  uint16_t compteur[32]; 
} message = {};

int16_t compteur = 0;
int16_t afficheur = 0;
int16_t afficheurPrev = 1;

uint8_t retournerNoisettes = 0; // 4 bits: pin13=bit3, pin12=bit2, pin5=bit1, pin4=bit0

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50;

unsigned long vannesOffTime = 0;
bool vannesTimerActive = false;
const unsigned long VANNES_DELAY = 1000;

const uint8_t NUM_STATES = 3;
const uint8_t NUM_STATES_2 = 3;

// #define DEBUG

void vannesOn()
{
  message.compteur[0] = 4095;
  message.compteur[1] = 4095;
  message.compteur[2] = 4095;
  message.compteur[3] = 4095;
}

void vannesOff()
{
  message.compteur[0] = 0;
  message.compteur[1] = 0;
  message.compteur[2] = 0;
  message.compteur[3] = 0;
}

void pompesOn()
{
  message.compteur[4] = 4095;
  message.compteur[5] = 4095;
  message.compteur[6] = 4095;
  message.compteur[7] = 4095;
}

void pompesOff()
{
  message.compteur[4] = 0;
  message.compteur[5] = 0;
  message.compteur[6] = 0;
  message.compteur[7] = 0;
  // Ouvrir les vannes et demarrer le timer de 1s
  vannesOn();
  vannesTimerActive = true;
  vannesOffTime = millis();
}

void bras2cmAuDessus()
{
  setBras(450, 450, 460, 480);
}

void brasAccroche()
{
  setBras(495, 500, 500, 520);
}

void setBras(uint16_t b0, uint16_t b1, uint16_t b2, uint16_t b3)
{
  message.compteur[8] = b0;
  message.compteur[9] = b1;
  message.compteur[10] = b2;
  message.compteur[11] = b3;
}

void setup()
{
  Serial.begin(115200);
#ifndef DEBUG
  transfer.begin(Serial);
#endif

  buttonState = 0;
  buttonState2 = 0;
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
  pinMode(13, INPUT_PULLUP);
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
    buttonState2 = (buttonState2 + 1) % NUM_STATES_2;
  }
  buttonSeqPrevDown = btn9;

  // Sequence 1 (bouton 8)
  switch (buttonState)
  {
  case 0:
    bras2cmAuDessus();
    pompesOff();
    break;
  case 1:
    pompesOn();
    brasAccroche();
    break;
  case 2:
    bras2cmAuDessus();
    break;
  }

  // Sequence 2 (bouton 9)
  switch (buttonState2)
  {
  case 0:
    bras2cmAuDessus();
    pompesOff();
    break;
  case 1:
    // TODO: mouvement 1 sequence 2
    break;
  case 2:
    // TODO: mouvement 2 sequence 2
    break;
  }

  // Fermer les vannes 1s apres pompesOff()
  if (vannesTimerActive && (millis() - vannesOffTime >= VANNES_DELAY))
  {
    vannesOff();
    vannesTimerActive = false;
  }

  // Lecture des 4 switchs noisettes (LOW = actif car INPUT_PULLUP)
  retournerNoisettes = 0;
  if (digitalRead(13) == LOW) retournerNoisettes |= 0b1000;
  if (digitalRead(12) == LOW) retournerNoisettes |= 0b0100;
  if (digitalRead(5) == LOW)  retournerNoisettes |= 0b0010;
  if (digitalRead(4) == LOW)  retournerNoisettes |= 0b0001;

  afficheur = retournerNoisettes;

  if (afficheur != afficheurPrev)
  {
    display.showNumberDec(afficheur);
  }
  afficheurPrev = afficheur;

#ifndef DEBUG
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL)
  {
    lastSendTime = now;
    uint16_t sendSize = 0;
    sendSize = transfer.txObj(message, sendSize);
    transfer.sendData(sendSize);
  }
#endif
}
