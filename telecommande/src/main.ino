#include "Arduino.h"
#include "Wire.h"
#include "TM1637Display.h"
#include "SerialTransfer.h"
#include "message.h"
#include "actionneurs.h"

TM1637Display display(3, 2); // CLK, DIO
SerialTransfer transfer;
STRUCT message = {};

uint8_t buttonState;
uint8_t buttonState2;
bool buttonSeqPrevUp;
bool buttonSeqPrevDown;

int16_t afficheur = 0;
int16_t afficheurPrev = 1;

uint8_t retournerNoisettes = 0; // 4 bits: pin13=bit3, pin12=bit2, pin5=bit1, pin4=bit0
bool noisettePrev[4] = {false, false, false, false};

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50;

unsigned long seq1TimerStart = 0;
bool seq1TimerActive = false;
const unsigned long SEQ1_AUTO_DELAY = 1000;

const uint8_t NUM_STATES = 3;
const uint8_t NUM_STATES_2 = 3;

uint8_t prevButtonState = 255;
uint8_t prevButtonState2 = 255;

const unsigned long DEBOUNCE_DELAY = 50;
unsigned long lastDebounce8 = 0;
unsigned long lastDebounce9 = 0;

void setup()
{
  Serial.begin(115200);
  transfer.begin(Serial);

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
  if (millis() - lastDebounce8 >= DEBOUNCE_DELAY)
  {
    if (btn8 && !buttonSeqPrevUp)
    {
      lastDebounce8 = millis();
      buttonState = (buttonState + 1) % NUM_STATES;
      if (buttonState == 1)
      {
        seq1TimerActive = true;
        seq1TimerStart = millis();
      }
    }
    buttonSeqPrevUp = btn8;
  }

  // Auto-transition de state 1 vers 2 apres 1s
  if (seq1TimerActive && buttonState == 1 && (millis() - seq1TimerStart >= SEQ1_AUTO_DELAY))
  {
    buttonState = 2;
    seq1TimerActive = false;
  }

  bool btn9 = (digitalRead(9) == LOW);
  if (millis() - lastDebounce9 >= DEBOUNCE_DELAY)
  {
    if (btn9 && !buttonSeqPrevDown)
    {
      lastDebounce9 = millis();
      buttonState2 = (buttonState2 + 1) % NUM_STATES_2;
    }
    buttonSeqPrevDown = btn9;
  }

  // Sequence 1 (bouton 8) — uniquement sur transition
  if (buttonState != prevButtonState)
  {
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
    prevButtonState = buttonState;
  }

  // Sequence 2 (bouton 9) — uniquement sur transition
  if (buttonState2 != prevButtonState2)
  {
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
    prevButtonState2 = buttonState2;
  }

  // Fermer les vannes 1s apres pompesOff()
  updateVannesTimer();

  // Lecture des 4 boutons noisettes en toggle (poussoir → levier)
  const uint8_t noisettePins[4] = {13, 12, 5, 4};
  const uint8_t noisetteBits[4] = {0b1000, 0b0100, 0b0010, 0b0001};
  for (uint8_t i = 0; i < 4; i++)
  {
    bool pressed = (digitalRead(noisettePins[i]) == LOW);
    if (pressed && !noisettePrev[i])
    {
      retournerNoisettes ^= noisetteBits[i];
    }
    noisettePrev[i] = pressed;
  }

  afficheur = retournerNoisettes;
  // afficheur = buttonState;

  if (afficheur != afficheurPrev)
  {
    // Affiche les 4 bits sur les 4 digits du TM1637
    uint8_t digits[4];
    digits[0] = (afficheur >> 3) & 1;
    digits[1] = (afficheur >> 2) & 1;
    digits[2] = (afficheur >> 1) & 1;
    digits[3] = afficheur & 1;
    display.showNumberDecEx(digits[0] * 1000 + digits[1] * 100 + digits[2] * 10 + digits[3], 0, true);
    // display.showNumberDec(afficheur);
  }
  afficheurPrev = afficheur;

  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL)
  {
    lastSendTime = now;
    uint16_t sendSize = 0;
    sendSize = transfer.txObj(message, sendSize);
    transfer.sendData(sendSize);
  }
}
