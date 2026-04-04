#pragma once
#include <Arduino.h>

extern const uint16_t BRAS_2CM[4];
extern const uint16_t BRAS_ACCROCHE[4];
extern const uint16_t BRAS_RETOURNE[4];

void vanneOn(uint8_t i);
void vanneOff(uint8_t i);

void pompeOn(uint8_t i);
void pompeOff(uint8_t i);

void setBrasIndex(uint8_t i, uint16_t val);
void bras2cmAuDessus();
void brasAccroche();

void setTapisIndex(uint8_t i, uint16_t val);
void tapisOff();
