#include "actionneurs.h"
#include "message.h"

// Positions bras
const uint16_t BRAS_2CM[4] = {450, 450, 460, 480};
const uint16_t BRAS_ACCROCHE[4] = {495, 500, 500, 520};
const uint16_t BRAS_RETOURNE[4] = {140, 140, 140, 160};

// Timer vannes global (pompesOff → vannesOn 1s → vannesOff)
static unsigned long vannesOffTime = 0;
static bool vannesTimerActive = false;
static const unsigned long VANNES_DELAY = 2000;

// Timers vannes individuelles
static unsigned long vanneIndivTimer[4] = {0, 0, 0, 0};
static bool vanneIndivActive[4] = {false, false, false, false};
static const unsigned long VANNE_INDIV_DELAY = 2000;

// --- Vannes (indices 0-3) ---

void vanneOn(uint8_t i)
{
  if (i < 4) message.compteur[i] = 4095;
}

void vanneOff(uint8_t i)
{
  if (i < 4) message.compteur[i] = 0;
}

void vannesOn()
{
  for (uint8_t i = 0; i < 4; i++) vanneOn(i);
}

void vannesOff()
{
  for (uint8_t i = 0; i < 4; i++) vanneOff(i);
}

// --- Pompes (indices 4-7) ---

void pompeOn(uint8_t i)
{
  if (i < 4) message.compteur[4 + i] = 4095;
}

void pompeOff(uint8_t i)
{
  if (i < 4) message.compteur[4 + i] = 0;
}

void pompesOn()
{
  for (uint8_t i = 0; i < 4; i++) pompeOn(i);
}

void pompesOff()
{
  for (uint8_t i = 0; i < 4; i++) pompeOff(i);
  vannesOn();
  vannesTimerActive = true;
  vannesOffTime = millis();
}

// Eteint une pompe et ouvre sa vanne pendant 1s
void pompeOffAvecVanne(uint8_t i)
{
  if (i >= 4) return;
  pompeOff(i);
  vanneOn(i);
  vanneIndivActive[i] = true;
  vanneIndivTimer[i] = millis();
}

// --- Bras (indices 8-11) ---

void setBras(uint16_t b0, uint16_t b1, uint16_t b2, uint16_t b3)
{
  message.compteur[8] = b0;
  message.compteur[9] = b1;
  message.compteur[10] = b2;
  message.compteur[11] = b3;
}

void setBrasIndex(uint8_t i, uint16_t val)
{
  if (i < 4) message.compteur[8 + i] = val;
}

void bras2cmAuDessus()
{
  setBras(BRAS_2CM[0], BRAS_2CM[1], BRAS_2CM[2], BRAS_2CM[3]);
}

void brasAccroche()
{
  setBras(BRAS_ACCROCHE[0], BRAS_ACCROCHE[1], BRAS_ACCROCHE[2], BRAS_ACCROCHE[3]);
}

// --- Tapis (indices 12-15) ---

void setTapis(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t3)
{
  message.compteur[12] = t0;
  message.compteur[13] = t1;
  message.compteur[14] = t2;
  message.compteur[15] = t3;
}

void setTapisIndex(uint8_t i, uint16_t val)
{
  if (i < 4) message.compteur[12 + i] = val;
}

void tapisEnAvant()
{
  setTapis(200, 200, 200, 200);
}

void tapisEnArriere()
{
  setTapis(400, 400, 400, 400);
}

// --- Timers ---

void updateVannesTimer()
{
  if (vannesTimerActive && (millis() - vannesOffTime >= VANNES_DELAY))
  {
    vannesOff();
    vannesTimerActive = false;
  }
}

void updateVannesIndivTimer()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    if (vanneIndivActive[i] && (millis() - vanneIndivTimer[i] >= VANNE_INDIV_DELAY))
    {
      vanneOff(i);
      vanneIndivActive[i] = false;
    }
  }
}
