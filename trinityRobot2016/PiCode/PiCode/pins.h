#ifndef PINS_H
#define PINS_H

#define BASE 100
#define SPI_CHAN 0


constexpr int ACCELEROMETERPIN = 0;
constexpr int SONARPIN1 = 1;
constexpr int SONARPIN2 = 2;
constexpr int SONARPIN3 = 3;
constexpr int SONARPIN4 = 4;
constexpr int FLAMESENSORPIN = 5;

constexpr int FLAMELEDPIN = 6;
constexpr int VISIONLEDPIN = 7;
constexpr int SOUNDLED = 8;

#define A0 BASE
#define A1 BASE+1
#define A2 BASE+2
#define A3 BASE+3
#define A4 BASE+4
#define A5 BASE+5
#define A6 BASE+6
#define A7 BASE+7

#define SONIC1_TRIG 18
#define SONIC1_ECHO 17
#define SONIC2_TRIG 23
#define SONIC2_ECHO 27
#define SONIC3_TRIG 24
#define SONIC3_ECHO 22
#define SONIC4_TRIG 25
#define SONIC4_ECHO 5

#endif