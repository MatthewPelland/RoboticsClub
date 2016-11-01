#ifndef wheel_h
#define wheel_h
#include "Arduino.h"

class Wheel {
  public: 
    Wheel(int eP)
    driveInches(float inches, int power, int motor);

  private:
    int encoderValue;
    int encoderPin;
    int lastEncoder;
    int currentEncoder;
    const int PWM = {2,3,4,5,6}; //PWM motor pins
    const int BRAKE = {7,8,9,10}; //brake motor pins
    float encoder_val[4];
    updateEncoder();
    inchesToEncoder(int inches);//converts inch values to encoder values
    encoderToInches(int encoder);//converst encoder values to inch values

    float pi = 3.14159;
    
};

#endif
