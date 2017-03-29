#include "drive.h"

/**
* change "wheel" to "drive" [done]
* add a 2nd encoder for the drive function [done]
* add turning (all wheels at once) [mostly done] **
* edit constructor [done]
* dump driveinches [done]
* add encoder x, encoder y [done]
* add finding speed w encoder [done]
* get speed from accelerometer [done]
* 	figure out how to calibrate **
* 	convert output to m/s^2 [done]
* add getting gyro speed - angular velocity**
* Find constants for PID - how??
*
* figure in having the encoders constantly recording [mostly done] **
* how does turning figure into distance?  [done]
* 	encoder is recording but we're not going forward :( [reset encoders at end of turning] [done]
*
* other:
* color sensor - detect crossing a white line [done] ?
*
*/
Drive::Drive()
{
	color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X); //color sensor object

	for (int i = 0; i < 4; i++) {
		currentEncoder[i] = 0; //current encoder ticks
	}
	for (int i = 0; i < 8; i++) {
		lastEncoder[i] = 0; //last raw encoder value
		encoderValue[i] = 0; //raw encoder value
	}
	time = 0; //time since the program began running
	lastTime = 0; //previous time
	inRoom = false; //whether we are in a room
	currentXPos = 0; //current x
	currentYPos = 0; //current y
	totalDeg = 0; //total degrees the robot has turned from its initial position
}

//updates the encoder value of the specified encoder
//num 0, 1 - x axis
//num 2, 3 - y axis
void Drive::updateEncoder(int encoderNum) {
	int encoderPinA = ENCODER_PIN4A;
	int encoderPinB = ENCODER_PIN4B;

	if (encoderNum == 0) {
		encoderPinA = ENCODER_PIN1A;
		encoderPinB = ENCODER_PIN1B;
	}
	else if (encoderNum == 1) {
		encoderPinA = ENCODER_PIN2A;
		encoderPinB = ENCODER_PIN2B;
	}
	else if (encoderNum == 2) {
		encoderPinA = ENCODER_PIN3A;
		encoderPinB = ENCODER_PIN3B;
	}

	currentEncoder[encoderNum] = digitalRead(encoderPinA);
	currentEncoder[encoderNum + 4] = digitalRead(encoderPinB);

	if (lastEncoder[encoderNum] != currentEncoder[encoderNum]) {
		if (currentEncoder[encoderNum] == currentEncoder[encoderNum + 4]) {
			encoderValue[encoderNum]++;
		}
		else {
			encoderValue[encoderNum]--;
		}
	}
	else if (lastEncoder[encoderNum + 4] != currentEncoder[encoderNum + 4]) {
		if (currentEncoder[encoderNum] == currentEncoder[encoderNum + 4]) {
			encoderValue[encoderNum]--;
		}
		else {
			encoderValue[encoderNum]++;
		}
	}
	/*if (lastEncoder[encoderNum] == LOW && currentEncoder[encoderNum] == HIGH) {
	if(currentEncoder[encoderNum + 4] == LOW){
	encoderValue[encoderNum]--;
	} else {
	encoderValue[encoderNum]++;
	}

	}*/
	lastEncoder[encoderNum] = currentEncoder[encoderNum];
	lastEncoder[encoderNum + 4] = currentEncoder[encoderNum + 4];

	/*Serial.print("encoderValue ");
	Serial.print(encoderNum);
	Serial.print("= ");
	Serial.println(encoderValue[encoderNum]);*/
}

//update the current time and last time
void Drive::updateTime() {
	lastTime = time;
	time = millis();
}

/**
* @brief Get the linear speed of the robot using the motor encoder
* @param axis: speed along x or y axis (of the robot)
* @return instantaneous linear speed
*/
double Drive::getLinearSpeedEncoder(char axis) {
	double deltas[4];
	double speeds[4];
	for (int i = 0; i < 4; i++) {
		deltas[i] = encoderToCm(currentEncoder[i] - lastEncoder[i]);
		if (time - lastTime == 0) {
			speeds[i] = 0;
		}
		else {
			speeds[i] = deltas[i] / (time - lastTime);
		}
	}
	if (axis == 'x')
		return speeds[0] * sqrt(2) + speeds[1] * sqrt(2) - speeds[2] * sqrt(2) - speeds[3] * sqrt(2);
	else
		return -speeds[0] * sqrt(2) + speeds[1] * sqrt(2) + speeds[2] * sqrt(2) - speeds[3] * sqrt(2);
}

double Drive::getAngularSpeedEncoder(char axis) {
	return getLinearSpeedEncoder(axis) / ROBOT_RADIUS; //???
}

void Drive::setInitialPos(int x, int y, int deg) {
	currentXPos = x;
	currentYPos = y;
	totalDeg = deg;
}

/**
* FIX TO INCLUDE
* Calibration (?) **
*/
/**
* @brief get the instantaneous velocity from the accelerometer
* @param axis: 'x' or 'y'
* @param vel: initial velocity
* @return new velocity
*/
double Drive::getSpeedAccel(char axis, double vel) {

	//v = v_0 + at
	return vel + getAccelerometerData(axis) * (time - lastTime);
}

/**
* @brief Get the angular velocity from the gyro sensor
* @return angular velocity
*/
double Drive::getSpeedGyro() { //change to angular velocity
	return getGyroData();
}

/**
* @brief Convert cm to encoder ticks
* @param cm: length in centimeters
* @return encoder ticks
*/
double Drive::cmToEncoder(int cm) {
	double circ = 2 * M_PI * WHEEL_RADIUS;

	return (double) cm / circ * 64; //or 16??
}

/**
* @brief Convert encoder ticks to cm
* @param encoder: encoder ticks
* @return length in centimeters
*/
double Drive::encoderToCm(int encoder) {
	double circ = 2 * M_PI * WHEEL_RADIUS;

	return (double) encoder / 64 * circ;
}

/**
* @brief Check to see if we are in a room
* 			If the color sensor sees white, we are
* 			Otherwise, we are not
*/
void Drive::updateInRoom() {
	uint16_t r, g, b, c;
	color.getRawData(&r, &g, &b, &c);

	if (r > 250 && g > 250 && b > 250) { //white -- update these values
		inRoom = true;
	}
	else {
		inRoom = false;
	}
}

/**
* @brief Get data from the accelerometer
* @param axis: 'x', 'y', or 'z'
* @return acceleration in m/s^2
*/
double Drive::getAccelerometerData(char axis) {
	//raw data / 2^14 * 9.8
	double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
	const int MPU_addr = 0x68;
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
	AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
	AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	if (axis == 'x')
		return AcX / pow(2, 14) * 9.8;
	else
		return AcY / pow(2, 14) * 9.8;
}

/**
* @brief Get data from the gyro
* @return angular velocity (in ?? units) --FIX
*/
double Drive::getGyroData() {
	double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
	const int MPU_addr = 0x68;
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
	AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
	AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	return GyZ; //CHANGE TO ANGULAR VEL
}

////drives the robot
///**
//* @brief Drive the robot sideways/forward along the x and y axis (relative to the robot)
//* @param x: x distance in cm
//* @parem y: y distance in cm
//*/
//void Drive::drive(int x, int y, int max_speed)
//{
//	/*Serial.println(x);
//	Serial.println(y);
//	Serial.println("---");*/
//
//	int initialEnc[] = { encoderValue[0], encoderValue[1], encoderValue[2], encoderValue[3] };
//
//	//initially 0
//	double vAccelX = 0;
//	double vAccelY = 0;
//	double vEnc = 0;
//
//	//PID constants !!FIND THESE!!
//	double posKP = 0.5;
//	double posKI = 0;
//	double posKD = 0;
//
//	double velKP = 0.5;
//	double velKI = 0;
//	double velKD = 0;
//
//	const double MAX_P = 255; //max motor power
//
//
//							  //calculate and correct for error n number of times
//	for (int n = 0; n < 100; n++)
//	{
//		//Update Current Position
//		updateTime();
//		//updateInRoom();
//
//		for (int i = 0; i < 4; i++)
//			updateEncoder(i);
//		//updateCurrent position and velocity
//		double currentXVel = getLinearSpeedEncoder('x');
//		double currentYVel = getLinearSpeedEncoder('y');
//
//		currentXPos += currentXVel * (time - lastTime);
//		currentYPos += currentYVel * (time - lastTime);
//
//		//get x and y error
//		double xError = x - currentXPos;
//		double yError = y = currentYPos;
//
//		//get target x and y velocity
//		double targetXVel = abs(posKP * xError) > maxSpeed ? maxSpeed : posKP * xError;
//		double targetYVel = abs(posKP * yError) > maxSpeed ? maxSpeed : posKP * yError;
//
//		//get xVel and yVel error
//		double xVelError = targetXVel - currentXVel;
//		double yVelError = targetYVel - currentYVel;
//
//		//get target x and y power
//		double xPower = abs(xVelError * velKP) > MAX_P ? MAX_P : xVelError * velKP;
//		double yPower = abs(yVelError * velKP) > MAX_P ? MAX_P : yVelError * velKP;
//
//		double power1 = xPower / sqrt(2) - yPower / sqrt(2);
//		double power2 = yPower / sqrt(2) + xPower / sqrt(2);
//		double power3 = yPower / sqrt(2) - xPower / sqrt(2);
//		double power4 = -xPower / sqrt(2) - yPower / sqrt(2);
//
//		motor1.setSpeed(abs(power1) * 100);
//		motor2.setSpeed(abs(power2) * 100);
//		motor3.setSpeed(abs(power3) * 100);
//		motor4.setSpeed(abs(power4) * 100);
//
//		if (power1 < 0)
//			motor1.run(BACKWARD);
//		else
//			motor1.run(FORWARD);
//		if (power2 < 0)
//			motor2.run(BACKWARD);
//		else
//			motor2.run(FORWARD);
//		if (power3 < 0)
//			motor3.run(BACKWARD);
//		else
//			motor3.run(FORWARD);
//		if (power4 < 0)
//			motor4.run(BACKWARD);
//		else
//			motor4.run(FORWARD);
//	}
//	Serial.print("encoderValue ");
//	Serial.print(1);
//	Serial.print("= ");
//	Serial.println(encoderValue[0]);
//
//	motor1.run(RELEASE);
//	motor2.run(RELEASE);
//	motor3.run(RELEASE);
//	motor4.run(RELEASE);
//	Serial.println("END");
//
//	//Update global current position variables + correct for current robot angle
//	//what the heck is this?
//	currentXPos += xCurrent * cos(totalDeg * M_PI / 180);
//	currentYPos += yCurrent * cos(totalDeg * M_PI / 180);
//
//	//Print whether we are in the room, and current position
//	//Serial.print(inRoom); REMEMBER TO UNCOMMENT ***************
//	//Serial.print((int) currentXPos);
//	//Serial.print((int) currentYPos);
//}
//
///**
//* @brief Turn the robot the specified angle
//* @param angle; angle in degrees
//*/
//void Drive::turn(int degrees, int max_speed) {
//	//Find the linear distance the robot needs to go
//	int linear_distance = ROBOT_RADIUS * degrees * M_PI / 180;
//	totalDeg += degrees;
//
//	//current pos
//	int current = 0;
//	int initialEnc[] = { encoderValue[0], encoderValue[1], encoderValue[2], encoderValue[3] };
//
//	//save initial values
//	int initialLastEnc[] = { lastEncoder[0], lastEncoder[1], lastEncoder[2], lastEncoder[3], lastEncoder[6],
//		lastEncoder[5], lastEncoder[6], lastEncoder[7] };
//	int initialCEnc[] = { currentEncoder[0], currentEncoder[1], currentEncoder[2], currentEncoder[3], currentEncoder[4], currentEncoder[5],
//		currentEncoder[6], currentEncoder[7] };
//
//	//initially 0
//	double vGyro = 0;
//	double vEnc = 0;
//
//	//PID constants !!FIND THESE!!
//	double posKP = 0;
//	double posKI = 0;
//	double posKD = 0;
//
//	double velKP = 0;
//	double velKI = 0;
//	double velKD = 0;
//
//	const double MAX_P = 255; //max motor power
//
//							  //calculate and correct for error n number of times
//	for (int n = 0; n < 100; n++)
//	{
//		//Update Current Position
//		updateTime();
//		updateInRoom();
//		for (int i = 0; i < 4; i++)
//			updateEncoder(i);
//		current = encoderToCm(encoderValue[0]) - encoderToCm(initialEnc[0]);
//
//		//FIND TARGET VELOCITY (LINEAR (?))
//		double input, output, setpoint;
//		input = current;
//		setpoint = linear_distance;
//		PID myPID = PID(&input, &output, &setpoint, posKP, posKI, posKD, AUTOMATIC); //error in x distance
//		myPID.SetOutputLimits(0, max_speed);
//		myPID.Compute();
//		double target_v = output;
//
//		//convert vel to angular velocity
//		target_v = target_v / ROBOT_RADIUS;
//
//		//GET CURRENT ANGULAR VELOCITY
//		vGyro = getSpeedGyro();
//		vEnc = getAngularSpeedEncoder('x');
//		double current_v = (vGyro + vEnc) / 2; //angular
//
//
//											   //FIND TARGET MOTOR POWER
//		input = current_v;
//		setpoint = target_v;
//		myPID.SetTunings(velKP, velKI, velKD);
//		myPID.SetOutputLimits(0, 255);
//		myPID.Compute();
//		double power = output;
//
//
//		//adjust speed based on speed error - set motor power
//		//all wheels have the same power because we're spinning
//		//power x = wheel set 1
//		//power y = wheel set 2
//		//analogWrite(MOTOR_PIN1, power);
//		//analogWrite(MOTOR_PIN2, power);
//		//analogWrite(MOTOR_PIN3, power);
//		//analogWrite(MOTOR_PIN4, power);
//
//		motor1.setSpeed(power);
//		motor2.setSpeed(power);
//		motor3.setSpeed(power);
//		motor4.setSpeed(power);
//
//		motor1.run(FORWARD);
//		motor2.run(FORWARD);
//		motor3.run(FORWARD);
//		motor4.run(FORWARD);
//	}
//
//	//turning is not included in normal encoder values -reset
//	for (int i = 0; i < 4; i++)
//		encoderValue[i] = initialEnc[i];
//	for (int i = 0; i < 8; i++) {
//		lastEncoder[i] = initialLastEnc[i];
//		currentEncoder[i] = initialCEnc[i];
//	}
//
//	//done turning
//	Serial.print(1);
//}

void Drive::go(double xTarget, double yTarget) {
	double xError = xTarget - currentXPos;
	double yError = yTarget - currentYPos;
	double angleDelta = 0;
	double epsilon = 1;
	int powers[] = { 200, 200, 200, 200 };
	int xPower = 0, yPower = 0;

	while (xError > epsilon || yError > epsilon) {
		updateTime();
		for(int i = 0; i < 4; i++)
			updateEncoder(i);
		double currentXVel = getLinearSpeedEncoder('x');
		double currentYVel = getLinearSpeedEncoder('y');
		currentXPos += currentXVel * (time - lastTime);
		currentYPos += currentYVel * (time - lastTime);
		xError = xTarget - currentXPos;
		yError = yTarget - currentYPos;

		if (xError == 0)
			xPower = 0;
		else if (xError > 0)
			xPower = 340;
		else
			xPower = -340;
		if (yError == 0)
			yPower = 0;
		else if (yError > 0)
			yPower = 340;
		else
			yPower = -340;

		powers[0] = xPower / sqrt(2) - yPower / sqrt(2);
		powers[1] = yPower / sqrt(2) + xPower / sqrt(2);
		powers[2] = yPower / sqrt(2) - xPower / sqrt(2);
		powers[3] = -1 * xPower / sqrt(2) - yPower / sqrt(2);
		angleDelta += getGyroData() * (time - lastTime);

		if (angleDelta > 0)
			for (int i = 0; i < 4; i++)
				powers[i] += 20;
		else if (angleDelta < 0)
			for (int i = 0; i < 4; i++)
				powers[i] -= 20;
		for (int i = 0; i < 4; i++)
			Serial.println(powers[i]);

		motor1.setSpeed(abs(powers[0]));
		motor2.setSpeed(abs(powers[1]));
		motor3.setSpeed(abs(powers[2]));
		motor4.setSpeed(abs(powers[3]));

		if (powers[0] < 0)
			motor1.run(BACKWARD);
		else
			motor1.run(FORWARD);
		if (powers[1] < 0)
			motor2.run(BACKWARD);
		else
			motor2.run(FORWARD);
		if (powers[2] < 0)
			motor3.run(BACKWARD);
		else
			motor3.run(FORWARD);
		if (powers[3] < 0)
			motor4.run(BACKWARD);
		else
			motor4.run(FORWARD);
	}

	motor1.run(RELEASE);
	motor2.run(RELEASE);
	motor3.run(RELEASE);
	motor4.run(RELEASE);
	Serial.println(currentXPos);

}