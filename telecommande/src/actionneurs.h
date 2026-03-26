#pragma once
#include <Arduino.h>

// Positions bras
extern const uint16_t BRAS_2CM[4];
extern const uint16_t BRAS_ACCROCHE[4];
extern const uint16_t BRAS_RETOURNE[4];

// Vannes (indices 0-3)
void vannesOn();
void vannesOff();
void vanneOn(uint8_t i);
void vanneOff(uint8_t i);

// Pompes (indices 4-7)
void pompesOn();
void pompesOff();
void pompeOn(uint8_t i);
void pompeOff(uint8_t i);

// Bras (indices 8-11)
void setBras(uint16_t b0, uint16_t b1, uint16_t b2, uint16_t b3);
void setBrasIndex(uint8_t i, uint16_t val);
void bras2cmAuDessus();
void brasAccroche();

// Tapis (indices 12-15)
void setTapis(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t3);
void setTapisIndex(uint8_t i, uint16_t val);
void tapisEnAvant();
void tapisEnArriere();

// Timers vannes
void updateVannesTimer();
void pompeOffAvecVanne(uint8_t i);
void updateVannesIndivTimer();
