#pragma once
#include <Arduino.h>

void vannesOn();
void vannesOff();
void pompesOn();
void pompesOff();
void setBras(uint16_t b0, uint16_t b1, uint16_t b2, uint16_t b3);
void bras2cmAuDessus();
void brasAccroche();
void setTapis(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t3);
void tapisEnAvant();
void tapisEnArriere();
void updateVannesTimer();
