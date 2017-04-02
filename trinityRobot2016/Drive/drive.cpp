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

int tickCount[4];

void updateEncoder1HIGH(void);
void updateEncoder1LOW(void);
void updateEncoder2HIGH(void);
void updateEncoder2LOW(void);
void updateEncoder3HIGH(void);
void updateEncoder3LOW(void);
void updateEncoder4HIGH(void);
void updateEncoder4LOW(void);

Drive::Drive()
{
	//colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X); //color sensor object

	for (int i = 0; i < 4; i++) {
		currentEncoder[i] = 0; //current encoder value (since last checked)
		lastEncoder[i] = 0; //last encoder value
		tickCount[i] = 0; //constantly updated encoder value
	}

	time = 0; //time since the program began running
	lastTime = 0; //previous time
	inRoom = true; //whether we are in a room
	lastColor = 0; //not white
	color = 0; //not white
	currentXPos = 0; //current x
	currentYPos = 0; //current y
	totalDeg = 0; //total degrees the robot has turned from its initial position

	//encoder Interrupts
	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1A), updateEncoder1HIGH, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1A), updateEncoder1LOW, FALLING);

	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN2A), updateEncoder2HIGH, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN2A), updateEncoder2LOW, FALLING);

	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN3A), updateEncoder3HIGH, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN3A), updateEncoder3LOW, FALLING);

	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN4A), updateEncoder4HIGH, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN4A), updateEncoder4LOW, FALLING);

	//i2c_init();
}

//int readOneVal(bool last) {
//	uint8_t msb, lsb;
//	lsb = i2c_read(false);
//	msb = i2c_read(last);
//	if (last) i2c_stop();
//	return (int)((msb << 8) | lsb) / 64;
//}
//
//bool setControlBits(uint16_t cntr) {
//	if (!i2c_start(I2CADDR | I2C_WRITE)) {
//		return false;
//	}
//	if (!i2c_write(0x0A)) {
//		return false;
//	}
//	if (!i2c_write(cntr)) {
//		return false;
//	}
//	i2c_stop();
//	return true;
//}

//updates the encoder value of the specified encoder
//num 0, 1 - x axis
//num 2, 3 - y axis
void updateEncoder1HIGH() {
	if (digitalRead(ENCODER_PIN1B) == HIGH)
		tickCount[0] ++;
	else
		tickCount[0] --;
}
void updateEncoder1LOW() {
	if (digitalRead(ENCODER_PIN1B) == LOW)
		tickCount[0] ++;
	else
		tickCount[0] --;
}

void updateEncoder2HIGH() {
	if (digitalRead(ENCODER_PIN2B) == HIGH)
		tickCount[1] ++;
	else
		tickCount[1] --;
}
void updateEncoder2LOW() {
	if (digitalRead(ENCODER_PIN2B) == LOW)
		tickCount[1] ++;
	else
		tickCount[1] --;
}

void updateEncoder3HIGH() {
	if (digitalRead(ENCODER_PIN3B) == HIGH)
		tickCount[2] ++;
	else
		tickCount[2] --;
}
void updateEncoder3LOW() {
	if (digitalRead(ENCODER_PIN3B) == LOW)
		tickCount[2] ++;
	else
		tickCount[2] --;
}

void updateEncoder4HIGH() {
	if (digitalRead(ENCODER_PIN4B) == HIGH)
		tickCount[3] ++;
	else
		tickCount[3] --;
}
void updateEncoder4LOW() {
	if (digitalRead(ENCODER_PIN4B) == LOW)
		tickCount[3] ++;
	else
		tickCount[3] --;
}


//update the current time and last time
void Drive::updateTime() {
	lastTime = time;
	for (int i = 0; i < 4; i++) {
		lastEncoder[i] = currentEncoder[i];
		currentEncoder[i] = tickCount[i];
	}
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

	return (double)cm / circ * 1024;
}

/**
* @brief Convert encoder ticks to cm
* @param encoder: encoder ticks
* @return length in centimeters
*/
double Drive::encoderToCm(int ticks) {
	double circ = 2 * M_PI * WHEEL_RADIUS;

	return (double)ticks / 1024 * circ;
}

/**
* @brief Check to see if we are in a room
* 			If the color sensor sees white, we are
* 			Otherwise, we are not
*/
//void Drive::updateInRoom() {
//	uint16_t r, g, b, c;
//	colorSensor.getRawData(&r, &g, &b, &c);
//
//	if (r > 250 && g > 250 && b > 250) { //white -- update these values
//		color = 1; //white
//		if (color != lastColor) {
//			inRoom = !inRoom;
//		} 
//	}
//	else {
//		color = 0; //not white
//	}
//	lastColor = color;
//}

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

	return GyZ; //currently in degrees/sec
}

//drives the robot
/**
* @brief Drive the robot sideways/forward along the x and y axis (relative to the robot)
* @param x: x distance in cm
* @parem y: y distance in cm
*/
void Drive::drive(int x, int y, int max_speed)
{
	double xCurrent = 0; //current x position
	double yCurrent = 0; //current y position

						 /*Serial.println(x);
						 Serial.println(y);
						 Serial.println("---");*/

	int initialEnc[] = { tickCount[0], tickCount[1], tickCount[2], tickCount[3] };

	//initially 0
	double vAccelX = 0;
	double vAccelY = 0;
	double vEnc = 0;

	//PID constants !!FIND THESE!!
	double posKP = 100;
	double posKI = 0;
	double posKD = 0;

	double velKP = 5;
	double velKI = 0;
	double velKD = 0;

	const double MAX_P = 255; //max motor power

	double xError = x - currentXPos;
	double yError = y - currentYPos;
	double epsilon = 1;


	//calculate and correct for error n number of times
	while (!(abs(xError) < epsilon && abs(yError) < epsilon))
	{
		/*//Update Current Position
		updateTime();
		//updateInRoom();

		xCurrent = encoderToCm(currentEncoder[0]) - encoderToCm(initialEnc[0]);
		yCurrent = encoderToCm(currentEncoder[2]) - encoderToCm(initialEnc[2]);

		//Serial.print("xCurrent=");
		//Serial.println(xCurrent);
		//Serial.print("yCurrent=");
		//Serial.println(yCurrent);
		//FIND TARGET VELOCITY
		double posInputX = xCurrent;
		double posOutputX = 0;
		double posSetpointX = x;
		PID posXPID = PID(&posInputX, &posOutputX, &posSetpointX, posKP, posKI, posKD, 0); //error in x distance
		posXPID.SetMode(AUTOMATIC);
		posXPID.SetOutputLimits(0, max_speed);
		posXPID.Compute();
		double target_v_x = posOutputX;

		double posInputY = yCurrent;
		double posOutputY = 0;
		double posSetpointY = y;
		PID posYPID = PID(&posInputY, &posOutputY, &posSetpointY, posKP, posKI, posKD, 0);
		posYPID.SetMode(AUTOMATIC);
		posYPID.SetOutputLimits(0, max_speed);
		posYPID.Compute();
		double target_v_y = posOutputY;

		//GET CURRENT X/Y VELOCITY
		//vAccelX = getSpeedAccel(x, vAccelX);
		vEnc = getLinearSpeedEncoder('x');
		double current_v_x = vEnc;
		//vAccelY = getSpeedAccel(y, vAccelY);
		vEnc = getLinearSpeedEncoder('y');
		double current_v_y = vEnc;
		//FIND TARGET MOTOR POWER
		double velInputX = current_v_x;
		double velOutputX = 0;
		double velSetpointX = target_v_x;
		PID velXPID = PID(&velInputX, &velOutputX, &velSetpointX, velKP, velKI, velKD, 0);
		velXPID.SetMode(AUTOMATIC);
		velXPID.SetOutputLimits(0, 255);
		velXPID.Compute();
		double power_x = velOutputX;

		double velInputY = current_v_y;
		double velOutputY = 0;
		double velSetpointY = target_v_y;
		PID velYPID = PID(&velInputY, &velOutputY, &velSetpointY, velKP, velKI, velKD, 0);
		velYPID.SetMode(AUTOMATIC);
		velYPID.SetOutputLimits(0, 255);
		velYPID.Compute();
		double power_y = velOutputY;

		//adjust speed based on speed error - set motor power
		//power x = wheel set 1
		//power y = wheel set 2

		/*Serial.print("power x: ");
		Serial.println(power_x);
		Serial.print("power y: ");
		Serial.println(power_y);

		double power1 = power_x / sqrt(2) - power_y / sqrt(2);
		double power2 = power_y / sqrt(2) + power_x / sqrt(2);
		double power3 = power_y / sqrt(2) - power_x / sqrt(2);
		double power4 = -1 * power_x / sqrt(2) - power_y / sqrt(2);
		/*Serial.print("power 1: ");
		Serial.println(power1);
		Serial.print("power 2: ");
		Serial.println(power2);
		Serial.print("power 3: ");
		Serial.println(power3);
		Serial.print("power 4: ");
		Serial.println(power4);
		motor1.setSpeed(abs(power1) * 100);
		motor2.setSpeed(abs(power2) * 100);
		motor3.setSpeed(abs(power3) * 100);
		motor4.setSpeed(abs(power4) * 100);

		if(power1 < 0)
		motor1.run(BACKWARD);
		else
		motor1.run(FORWARD);
		if(power2 < 0)
		motor2.run(BACKWARD);
		else
		motor2.run(FORWARD);
		if(power3 < 0)
		motor3.run(BACKWARD);
		else
		motor3.run(FORWARD);
		if(power4 < 0)
		motor4.run(BACKWARD);
		else
		motor4.run(FORWARD);*/

		//Update Current Position
		updateTime();
		//updateInRoom();

		//updateCurrent position and velocity
		double currentXVel = getLinearSpeedEncoder('x');
		double currentYVel = getLinearSpeedEncoder('y');

		currentXPos += currentXVel * (time - lastTime);
		currentYPos += currentYVel * (time - lastTime);

		//get x and y error
		xError = x - currentXPos;
		yError = y - currentYPos;

		//get target x and y velocity
		double targetXVel = abs(posKP * xError) > max_speed ? max_speed : posKP * xError;
		double targetYVel = abs(posKP * yError) > max_speed ? max_speed : posKP * yError;

		//get xVel and yVel error
		double xVelError = targetXVel - currentXVel;
		double yVelError = targetYVel - currentYVel;

		//get target x and y power
		double xPower = abs(xVelError * velKP) > MAX_P ? MAX_P : xVelError * velKP;
		double yPower = abs(yVelError * velKP) > MAX_P ? MAX_P : yVelError * velKP;

		double power1 = xPower - yPower;
		double power2 = yPower + xPower;
		double power3 = yPower - xPower;
		double power4 = -xPower - yPower;

		Serial.println(power2);

		motor1.setSpeed(abs(power1) * 100 > 255 ? 255 : abs(power1) * 100);
		motor2.setSpeed(abs(power2) * 100 > 255 ? 255 : abs(power2) * 100);
		motor3.setSpeed(abs(power3) * 100 > 255 ? 255 : abs(power3) * 100);
		motor4.setSpeed(abs(power4) * 100 > 255 ? 255 : abs(power4) * 100);

		if (power1 < 0)
			motor1.run(BACKWARD);
		else
			motor1.run(FORWARD);
		if (power2 < 0)
			motor2.run(BACKWARD);
		else
			motor2.run(FORWARD);
		if (power3 < 0)
			motor3.run(BACKWARD);
		else
			motor3.run(FORWARD);
		if (power4 < 0)
			motor4.run(BACKWARD);
		else
			motor4.run(FORWARD);
	}
	Serial.print("encoderValue ");
	Serial.print(1);
	Serial.print("= ");
	Serial.println(tickCount[0]);

	motor1.run(RELEASE);
	motor2.run(RELEASE);
	motor3.run(RELEASE);
	motor4.run(RELEASE);
	Serial.println("END");

	//Update global current position variables + correct for current robot angle
	//currentXPos += xCurrent * cos(totalDeg * M_PI / 180);
	//currentYPos += yCurrent * cos(totalDeg * M_PI / 180);

	//Print whether we are in the room, and current position
	//Serial.print(inRoom); REMEMBER TO UNCOMMENT ***************
	Serial.print((int)currentXPos);
	//Serial.print((int) currentYPos);
}

/**
* @brief Turn the robot the specified angle
* @param angle; angle in degrees
*/
void Drive::turn(int degrees, int max_speed) {
	double ticksToGo = (degrees * M_PI / 180 * ROBOT_RADIUS) / (2 * M_PI * WHEEL_RADIUS) * 1024 / 1.5;//3000
	/*Serial.print("degrees: ");
	Serial.println(degrees);
	Serial.print("arcLength: ");
	Serial.println(degrees * M_PI / 180 * ROBOT_RADIUS);
	Serial.print("ticksToGo: ");
	Serial.println(ticksToGo);*/

	int targetTicks = tickCount[0] + ticksToGo; //3000
	double error = ticksToGo;//3000
	double epsilon = 30;
	motor1.setSpeed(150);
	motor2.setSpeed(150);
	motor3.setSpeed(150);
	motor4.setSpeed(150);
	if (error < 0) {
		motor1.run(BACKWARD);
		motor2.run(BACKWARD);
		motor3.run(BACKWARD);
		motor4.run(BACKWARD);
		while (error < -epsilon) {
			error = targetTicks - tickCount[0];
			delay(1);
		}
	}
	else {
		motor1.run(FORWARD);
		motor2.run(FORWARD);
		motor3.run(FORWARD);
		motor4.run(FORWARD);
		while (error > epsilon) {
			error = targetTicks - tickCount[0];
			delay(1);
		}
	}
	motor1.run(RELEASE);
	motor2.run(RELEASE);
	motor3.run(RELEASE);
	motor4.run(RELEASE);

	Serial.print(1); //done
}

void Drive::go(double xTarget, double yTarget) {
	double xError = xTarget - currentXPos;
	double yError = yTarget - currentYPos;
	double epsilon = 0.50;
	double powers[] = { 0,0,0,0 };
	double xPower = 0, yPower = 0;
	updateTime();
	while (abs(xError) > epsilon || abs(yError) > epsilon) {
		updateTime();

		double deltaX = ((double)(currentEncoder[0] - lastEncoder[0]) / sqrt(2) + (double)(currentEncoder[1] - lastEncoder[1]) / sqrt(2) - (double)(currentEncoder[2] - lastEncoder[2]) / sqrt(2) - (double)(currentEncoder[3] - lastEncoder[3]) / sqrt(2))*WHEEL_RADIUS * 2 * M_PI / 1024 / 2;
		double deltaY = (-(double)(currentEncoder[0] - lastEncoder[0]) / sqrt(2) + (double)(currentEncoder[1] - lastEncoder[1]) / sqrt(2) + (double)(currentEncoder[2] - lastEncoder[2]) / sqrt(2) - (double)(currentEncoder[3] - lastEncoder[3]) / sqrt(2))*WHEEL_RADIUS * 2 * M_PI / 1024 / 2;
		currentXPos += deltaX;
		currentYPos += deltaY;

		xError = xTarget - currentXPos;
		yError = yTarget - currentYPos;

		if (xError < epsilon && xError > -epsilon)
			xPower = 0;
		else if (xError > 0)
			xPower = 300;
		else
			xPower = -300;
		if (yError < epsilon && yError > -epsilon)
			yPower = 0;
		else if (yError > 0)
			yPower = 300;
		else
			yPower = -300;

		powers[0] = xPower / sqrt(2) - yPower / sqrt(2);
		powers[1] = yPower / sqrt(2) + xPower / sqrt(2);
		powers[2] = yPower / sqrt(2) - xPower / sqrt(2);
		powers[3] = -1 * xPower / sqrt(2) - yPower / sqrt(2);


		motor1.setSpeed(abs(powers[0]) + 40);
		motor2.setSpeed(abs(powers[1]) + 32);
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

	Serial.print(currentXPos);
	Serial.print(" ");
	Serial.println(currentYPos);
}

void Drive::driveCm(int cm) {
	double error = cm;
	double epsilon = 0.50;
	double powers[] = { 0,0,0,0 };
	double xPower = 0, yPower = 0;
	updateTime();
	while (abs(error) > epsilon ) {
		updateTime();
		//updateInRoom();

		double delta = (-(double)(currentEncoder[0] - lastEncoder[0]) / sqrt(2) + (double)(currentEncoder[1] - lastEncoder[1]) / sqrt(2) + (double)(currentEncoder[2] - lastEncoder[2]) / sqrt(2) - (double)(currentEncoder[3] - lastEncoder[3]) / sqrt(2))*WHEEL_RADIUS * 2 * M_PI / 1024 / 2;

		error -= delta;

		yPower = 300;

		powers[0] = xPower / sqrt(2) - yPower / sqrt(2);
		powers[1] = yPower / sqrt(2) + xPower / sqrt(2);
		powers[2] = yPower / sqrt(2) - xPower / sqrt(2);
		powers[3] = -1 * xPower / sqrt(2) - yPower / sqrt(2);


		motor1.setSpeed(abs(powers[0]) + 40);
		motor2.setSpeed(abs(powers[1]) + 32);
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
}