#include "actionneurs.h"
#include "message.h"

const uint16_t BRAS_2CM[4] = {450, 450, 460, 480};
const uint16_t BRAS_ACCROCHE[4] = {495, 500, 500, 520};
const uint16_t BRAS_RETOURNE[4] = {140, 140, 140, 160};

static void setGroup(uint8_t base, const uint16_t vals[4])
{
  for (uint8_t i = 0; i < 4; i++) message.compteur[base + i] = vals[i];
}

// Vannes (indices 0-3)
void vanneOn(uint8_t i)  { if (i < 4) message.compteur[i] = 4095; }
void vanneOff(uint8_t i) { if (i < 4) message.compteur[i] = 0; }

// Pompes (indices 4-7)
void pompeOn(uint8_t i)  { if (i < 4) message.compteur[4 + i] = 4095; }
void pompeOff(uint8_t i) { if (i < 4) message.compteur[4 + i] = 0; }

// Bras (indices 8-11)
void setBrasIndex(uint8_t i, uint16_t val) { if (i < 4) message.compteur[8 + i] = val; }
void bras2cmAuDessus() { setGroup(8, BRAS_2CM); }
void brasAccroche()    { setGroup(8, BRAS_ACCROCHE); }

// Tapis (indices 12-15)
void setTapisIndex(uint8_t i, uint16_t val) { if (i < 4) message.compteur[12 + i] = val; }
void tapisOff() { for (uint8_t i = 0; i < 4; i++) message.compteur[12 + i] = 0; }
