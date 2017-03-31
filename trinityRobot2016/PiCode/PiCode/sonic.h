class SonicSensor {
public:
	SonicSensor(int trig, int echo);
	double getDistance();
private:
	int trig, echo;
};
