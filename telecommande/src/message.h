#pragma once
#include <Arduino.h>

struct __attribute__((packed)) STRUCT
{
  int16_t x;
  int16_t y;
  int16_t z;
  // Vannes: 0-3
  // Pompes: 4-7
  // Servo bras: 8-11
  // Servo tapis: 12-15
  uint16_t compteur[32];
};

extern STRUCT message;
