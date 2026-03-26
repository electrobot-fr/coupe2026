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

uint8_t retournerNoisettes = 0b1111; // 4 bits: pin13=bit3, pin12=bit2, pin5=bit1, pin4=bit0
bool noisettePrev[4] = {false, false, false, false};
bool noisettePressed = false;
unsigned long noisetteDisplayTime = 0;
const unsigned long NOISETTE_DISPLAY_DURATION = 2000;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50;

unsigned long seq1TimerStart = 0;
bool seq1TimerActive = false;
const unsigned long SEQ1_AUTO_DELAY = 1000;

const uint8_t NUM_STATES = 4;

unsigned long seq1Timer2Start = 0;
bool seq1Timer2Active = false;
const unsigned long SEQ1_AUTO_DELAY_2 = 1000;
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
    }
    buttonSeqPrevUp = btn8;
  }

  // Auto-transition de state 1 vers 2 apres 1s
  if (seq1TimerActive && buttonState == 1 && (millis() - seq1TimerStart >= SEQ1_AUTO_DELAY))
  {
    buttonState = 2;
    seq1TimerActive = false;
  }

  // Auto-transition de state 2 vers 3 apres 1s
  if (seq1Timer2Active && buttonState == 2 && (millis() - seq1Timer2Start >= SEQ1_AUTO_DELAY_2))
  {
    buttonState = 3;
    seq1Timer2Active = false;
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
    case 0: // Les bras sont a 2cm, les pompes sont eteintes
      seq1TimerActive = false;
      seq1Timer2Active = false;
      retournerNoisettes = 0b1111;
      bras2cmAuDessus();
      setTapis(0, 0, 0, 0);
      pompesOff();
      break;
    case 1: // Les bras sont accroches, les pompes sont allumees
      seq1TimerActive = true;
      seq1TimerStart = millis();
      pompesOn();
      brasAccroche();
      break;
    case 2: // Les bras montent: retournerNoisettes=1 -> bras2cm, =0 -> brasRetourne + tapis
      seq1Timer2Active = true;
      seq1Timer2Start = millis();
      for (uint8_t i = 0; i < 4; i++)
      {
        if (retournerNoisettes & (1 << i))
        {
          setBrasIndex(i, BRAS_2CM[i]);
        }
        else
        {
          setBrasIndex(i, BRAS_RETOURNE[i]);
          setTapisIndex(i, 400);
        }
      }
      break;
    case 3: // Pompes off pour les noisettes retournees, vannes on 1s
      for (uint8_t i = 0; i < 4; i++)
      {
        if (!(retournerNoisettes & (1 << i)))
        {
          pompeOffAvecVanne(i);
        }
      }
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

  // Timers vannes
  updateVannesTimer();
  updateVannesIndivTimer();

  // Lecture des 4 boutons noisettes en toggle (poussoir -> levier)
  const uint8_t noisettePins[4] = {13, 12, 5, 4};
  const uint8_t noisetteBits[4] = {0b1000, 0b0100, 0b0010, 0b0001};
  for (uint8_t i = 0; i < 4; i++)
  {
    bool pressed = (digitalRead(noisettePins[i]) == LOW);
    if (pressed && !noisettePrev[i])
    {
      retournerNoisettes ^= noisetteBits[i];
      noisettePressed = true;
      noisetteDisplayTime = millis();
    }
    noisettePrev[i] = pressed;
  }

  // Affiche retournerNoisettes en binaire pendant 2s apres un appui noisette, sinon buttonState
  if (noisettePressed && (millis() - noisetteDisplayTime < NOISETTE_DISPLAY_DURATION))
  {
    afficheur = retournerNoisettes | 0x10; // offset pour distinguer du buttonState
    if (afficheur != afficheurPrev)
    {
      uint8_t digits[4];
      digits[0] = (retournerNoisettes >> 3) & 1;
      digits[1] = (retournerNoisettes >> 2) & 1;
      digits[2] = (retournerNoisettes >> 1) & 1;
      digits[3] = retournerNoisettes & 1;
      display.showNumberDecEx(digits[0] * 1000 + digits[1] * 100 + digits[2] * 10 + digits[3], 0, true);
    }
  }
  else
  {
    noisettePressed = false;
    afficheur = buttonState;
    if (afficheur != afficheurPrev)
    {
      display.showNumberDec(afficheur);
    }
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
