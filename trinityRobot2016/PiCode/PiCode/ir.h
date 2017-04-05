#ifndef IR_H
#define IR_H

class IRSensor {
public:
    IRSensor(int pin);
    bool fireDetected();
private:
    int pin;
};

#endif


