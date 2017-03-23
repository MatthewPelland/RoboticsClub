#ifndef IR_H
#define IR_H

class IRSensor {
public:
	IRSensor(int analogIn);
	int getFireIntensity();
private:
	int analogPin;
};

#endif

