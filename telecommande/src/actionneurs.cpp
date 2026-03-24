#include "actionneurs.h"
#include "message.h"

static unsigned long vannesOffTime = 0;
static bool vannesTimerActive = false;
static const unsigned long VANNES_DELAY = 1000;

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
  vannesOn();
  vannesTimerActive = true;
  vannesOffTime = millis();
}

void setBras(uint16_t b0, uint16_t b1, uint16_t b2, uint16_t b3)
{
  message.compteur[8] = b0;
  message.compteur[9] = b1;
  message.compteur[10] = b2;
  message.compteur[11] = b3;
}

void bras2cmAuDessus()
{
  setBras(450, 450, 460, 480);
}

void brasAccroche()
{
  setBras(495, 500, 500, 520);
}

void setTapis(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t3)
{
  message.compteur[12] = t0;
  message.compteur[13] = t1;
  message.compteur[14] = t2;
  message.compteur[15] = t3;
}

void tapisEnAvant()
{
  setTapis(200, 200, 200, 200);
}

void tapisEnArriere()
{
  setTapis(400, 400, 400, 400);
}

void updateVannesTimer()
{
  if (vannesTimerActive && (millis() - vannesOffTime >= VANNES_DELAY))
  {
    vannesOff();
    vannesTimerActive = false;
  }
}
