#include "Arduino.h"
#include "TM1637Display.h"
#include "SerialTransfer.h"
#include "message.h"
#include "actionneurs.h"

TM1637Display display(3, 2); // CLK, DIO
SerialTransfer transfer;
STRUCT message = {};

const uint8_t noisettePins[4] = {13, 12, 5, 4};

const unsigned long DEBOUNCE_DELAY = 100;
const unsigned long SEND_INTERVAL = 50;
const unsigned long ACCROCHE_DELAY = 1000;
const unsigned long BRAS_UP_DELAY = 1000;
const unsigned long RETOURNE_DELAY = 1000;
const unsigned long POMPE_OFF_DELAY = 200;
const unsigned long VANNE_DELAY = 2000;

// Bouton i (pin) controle la noisette hw = 3-i
enum NoisetteState : uint8_t {
  N_IDLE,
  N_ACCROCHE,
  N_BRAS_UP,
  N_HOLDING,     // attend bouton noisette pour retourner
  N_RETOURNE,
  N_POMPE_OFF,
  N_VANNE_OPEN
};

NoisetteState noisetteState[4] = {};
unsigned long noisetteTimer[4] = {};
bool sequenceActive = false;

bool btn8Prev = false;
bool noisettePrev[4] = {};
unsigned long lastDebounce8 = 0;
unsigned long lastDebounceNoisette[4] = {};
unsigned long lastSendTime = 0;

int16_t afficheurPrev = -1;

bool debounceRising(uint8_t pin, bool &prev, unsigned long &lastTime, unsigned long now) {
  bool state = (digitalRead(pin) == LOW);
  bool triggered = false;
  if (now - lastTime >= DEBOUNCE_DELAY) {
    if (state && !prev) {
      lastTime = now;
      triggered = true;
    }
  }
  prev = state;
  return triggered;
}

bool anyAutoTransition() {
  for (uint8_t i = 0; i < 4; i++) {
    if (noisetteState[i] == N_ACCROCHE || noisetteState[i] == N_BRAS_UP)
      return true;
  }
  return false;
}

void resetAll(unsigned long now) {
  sequenceActive = false;
  bras2cmAuDessus();
  tapisOff();
  for (uint8_t i = 0; i < 4; i++) {
    pompeOff(i);
    vanneOn(i);
    noisetteState[i] = N_VANNE_OPEN;
    noisetteTimer[i] = now;
  }
}

void startSequence(unsigned long now) {
  sequenceActive = true;
  brasAccroche();
  for (uint8_t i = 0; i < 4; i++) {
    pompeOn(i);
    noisetteState[i] = N_ACCROCHE;
    noisetteTimer[i] = now;
  }
}

void setup()
{
  Serial.begin(115200);
  transfer.begin(Serial);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  for (uint8_t pin = 4; pin <= 13; pin++) pinMode(pin, INPUT_PULLUP);

  display.setBrightness(4);
}

void loop()
{
  unsigned long now = millis();

  // Joysticks
  message.x = analogRead(A3);
  message.y = map(analogRead(A2), 0, 1023, 1023, 0);
  message.z = analogRead(A0);

  // Bouton 8: start / reset
  if (debounceRising(8, btn8Prev, lastDebounce8, now)) {
    if (!sequenceActive)
      startSequence(now);
    else if (!anyAutoTransition())
      resetAll(now);
  }

  // Noisettes: boutons + state machine
  for (uint8_t i = 0; i < 4; i++)
  {
    // Bouton noisette i → retourne hw = 3-i
    if (debounceRising(noisettePins[i], noisettePrev[i], lastDebounceNoisette[i], now))
    {
      uint8_t hw = 3 - i;
      if (noisetteState[hw] == N_HOLDING)
      {
        setBrasIndex(hw, BRAS_RETOURNE[hw]);
        setTapisIndex(hw, 400);
        noisetteState[hw] = N_RETOURNE;
        noisetteTimer[hw] = now;
      }
    }

    // State machine
    switch (noisetteState[i])
    {
    case N_ACCROCHE:
      if (now - noisetteTimer[i] >= ACCROCHE_DELAY) {
        setBrasIndex(i, BRAS_2CM[i]);
        noisetteState[i] = N_BRAS_UP;
        noisetteTimer[i] = now;
      }
      break;

    case N_BRAS_UP:
      if (now - noisetteTimer[i] >= BRAS_UP_DELAY) {
        noisetteState[i] = N_HOLDING;
      }
      break;

    case N_HOLDING:
      break;

    case N_RETOURNE:
      if (now - noisetteTimer[i] >= RETOURNE_DELAY) {
        pompeOff(i);
        noisetteState[i] = N_POMPE_OFF;
        noisetteTimer[i] = now;
      }
      break;

    case N_POMPE_OFF:
      if (now - noisetteTimer[i] >= POMPE_OFF_DELAY) {
        vanneOn(i);
        noisetteState[i] = N_VANNE_OPEN;
        noisetteTimer[i] = now;
      }
      break;

    case N_VANNE_OPEN:
      if (now - noisetteTimer[i] >= VANNE_DELAY) {
        vanneOff(i);
        noisetteState[i] = N_IDLE;
      }
      break;

    default: break;
    }
  }

  // Afficheur: 4 digits = 4 etats noisettes
  int16_t afficheur = noisetteState[0] * 1000 + noisetteState[1] * 100 + noisetteState[2] * 10 + noisetteState[3];
  if (afficheur != afficheurPrev) {
    display.showNumberDecEx(afficheur, 0, true);
    afficheurPrev = afficheur;
  }

  // Envoi SerialTransfer
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    uint16_t sendSize = 0;
    sendSize = transfer.txObj(message, sendSize);
    transfer.sendData(sendSize);
  }
}
