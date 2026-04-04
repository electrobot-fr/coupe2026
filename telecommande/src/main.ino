#include "Arduino.h"
#include "Wire.h"
#include "TM1637Display.h"
#include "SerialTransfer.h"
#include "message.h"
#include "actionneurs.h"

TM1637Display display(3, 2); // CLK, DIO
SerialTransfer transfer;
STRUCT message = {};

// --- Constantes ---

const uint8_t noisettePins[4] = {13, 12, 5, 4};
const uint8_t noisetteBits[4] = {0b1000, 0b0100, 0b0010, 0b0001};

const unsigned long DEBOUNCE_DELAY = 100;
const unsigned long SEND_INTERVAL = 50;
const unsigned long ACCROCHE_DELAY = 1000;
const unsigned long BRAS_UP_DELAY = 1000;
const unsigned long RETOURNE_DELAY = 1000;
const unsigned long POMPE_OFF_DELAY = 200;
const unsigned long VANNE_DELAY = 2000;

// --- State machine noisettes ---
//
// Bouton 8 (start):  toutes → N_ACCROCHE
// Bouton 8 (reset):  toutes → N_IDLE
//
// Cycle auto:  N_ACCROCHE -(1s)→ N_BRAS_UP -(1s)→ N_HOLDING ou N_POMPE_OFF
// Toggle:      N_HOLDING → N_RETOURNE -(1s)→ N_POMPE_OFF -(200ms)→ N_VANNE_OPEN -(2s)→ N_IDLE

enum NoisetteState : uint8_t {
  N_IDLE,        // 0 - repos
  N_ACCROCHE,    // 1 - bras accroche + pompe on, attend 1s
  N_BRAS_UP,     // 2 - bras monte (2cm ou retourne), attend 1s
  N_HOLDING,     // 3 - en attente, pompe on, peut etre toggle
  N_RETOURNE,    // 4 - toggle: bras retourne + tapis, attend 1s
  N_POMPE_OFF,   // 5 - pompe off, attend 200ms
  N_VANNE_OPEN   // 6 - vanne ouverte, attend 2s
};

NoisetteState noisetteState[4] = {};
unsigned long noisetteTimer[4] = {};
uint8_t retournerNoisettes = 0b1111;
bool sequenceActive = false;

// --- Variables ---

bool btn8Prev = false;
bool noisettePrev[4] = {};
unsigned long lastDebounce8 = 0;
unsigned long lastDebounceNoisette[4] = {};
unsigned long lastSendTime = 0;

int16_t afficheur = 0;
int16_t afficheurPrev = 1;

// #define EXPO

inline int16_t applyExpo(int16_t raw) {
#ifdef EXPO
  long centered = (long)raw - 512;
  if (centered == 0) return 512;
  long sign = (centered < 0) ? -1 : 1;
  long a = abs(centered);
  long num = a * (262144L + 2L * a * a);
  long y = num / 786432L;
  if (y > 512) y = 512;
  return (int16_t)(512 + sign * y);
#else
  return raw;
#endif
}

bool debounceRising(uint8_t pin, bool &prev, unsigned long &lastTime) {
  bool state = (digitalRead(pin) == LOW);
  bool triggered = false;
  if (millis() - lastTime >= DEBOUNCE_DELAY) {
    if (state && !prev) {
      lastTime = millis();
      triggered = true;
    }
  }
  prev = state;
  return triggered;
}

// Verifie si une noisette est dans une auto-transition (non interruptible)
bool anyAutoTransition() {
  for (uint8_t i = 0; i < 4; i++) {
    if (noisetteState[i] == N_ACCROCHE || noisetteState[i] == N_BRAS_UP)
      return true;
  }
  return false;
}

void resetAll() {
  sequenceActive = false;
  retournerNoisettes = 0b1111;
  bras2cmAuDessus();
  setTapis(0, 0, 0, 0);
  unsigned long now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    pompeOff(i);
    vanneOn(i);
    noisetteState[i] = N_VANNE_OPEN;
    noisetteTimer[i] = now;
  }
}

void startSequence() {
  sequenceActive = true;
  unsigned long now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    pompeOn(i);
    setBrasIndex(i, BRAS_ACCROCHE[i]);
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
  // --- Joysticks ---
  message.x = applyExpo(analogRead(A3));
  message.y = applyExpo(map(analogRead(A2), 0, 1023, 1023, 0));
  message.z = applyExpo(analogRead(A0));

  // --- Bouton 8: start / reset ---
  if (debounceRising(8, btn8Prev, lastDebounce8)) {
    if (!sequenceActive)
      startSequence();
    else if (!anyAutoTransition())
      resetAll();
  }

  // --- Boutons noisettes (toggle) ---
  for (uint8_t i = 0; i < 4; i++)
  {
    if (debounceRising(noisettePins[i], noisettePrev[i], lastDebounceNoisette[i]))
    {
      retournerNoisettes ^= noisetteBits[i];

      uint8_t hw = 3 - i;
      if (noisetteState[hw] == N_HOLDING && !(retournerNoisettes & noisetteBits[i]))
      {
        setBrasIndex(hw, BRAS_RETOURNE[hw]);
        setTapisIndex(hw, 400);
        noisetteState[hw] = N_RETOURNE;
        noisetteTimer[hw] = millis();
      }
    }
  }

  // --- State machine noisettes ---
  for (uint8_t i = 0; i < 4; i++)
  {
    switch (noisetteState[i])
    {
    case N_ACCROCHE:
      if (millis() - noisetteTimer[i] >= ACCROCHE_DELAY) {
        if (retournerNoisettes & (1 << i))
          setBrasIndex(i, BRAS_2CM[i]);
        else {
          setBrasIndex(i, BRAS_RETOURNE[i]);
          setTapisIndex(i, 400);
        }
        noisetteState[i] = N_BRAS_UP;
        noisetteTimer[i] = millis();
      }
      break;

    case N_BRAS_UP:
      if (millis() - noisetteTimer[i] >= BRAS_UP_DELAY) {
        if (retournerNoisettes & (1 << i))
          noisetteState[i] = N_HOLDING; // pas marquee, attend toggle
        else {
          pompeOff(i);
          noisetteState[i] = N_POMPE_OFF; // deja retournee, lacher
          noisetteTimer[i] = millis();
        }
      }
      break;

    case N_HOLDING:
      break; // attend toggle via bouton noisette

    case N_RETOURNE:
      if (millis() - noisetteTimer[i] >= RETOURNE_DELAY) {
        pompeOff(i);
        noisetteState[i] = N_POMPE_OFF;
        noisetteTimer[i] = millis();
      }
      break;

    case N_POMPE_OFF:
      if (millis() - noisetteTimer[i] >= POMPE_OFF_DELAY) {
        vanneOn(i);
        noisetteState[i] = N_VANNE_OPEN;
        noisetteTimer[i] = millis();
      }
      break;

    case N_VANNE_OPEN:
      if (millis() - noisetteTimer[i] >= VANNE_DELAY) {
        vanneOff(i);
        noisetteState[i] = N_IDLE;
      }
      break;

    default: break;
    }
  }

  // --- Afficheur: toujours les 4 etats (0-6) ---
  afficheur = noisetteState[0] * 1000 + noisetteState[1] * 100 + noisetteState[2] * 10 + noisetteState[3];
  if (afficheur != afficheurPrev)
    display.showNumberDecEx(afficheur, 0, true);
  afficheurPrev = afficheur;

  // --- Envoi SerialTransfer ---
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    uint16_t sendSize = 0;
    sendSize = transfer.txObj(message, sendSize);
    transfer.sendData(sendSize);
  }
}
