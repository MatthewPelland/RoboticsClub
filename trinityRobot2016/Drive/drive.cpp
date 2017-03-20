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
drive::Drive()
{
	color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X); //color sensor object
	currentEncoder = {0, 0, 0, 0}; //current encoder ticks
	lastEncoder = {0, 0, 0, 0, 0, 0, 0, 0}; //last encoder ticks
	encoderValue = {0, 0, 0, 0, 0, 0, 0, 0}; //raw encoder value
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
void drive::updateEncoder(int encoderNum) {
	int encoderPinA = ENCODER_PIN4A;
	int encoderPinB = ENCODER_PIN4B;
	
	if(encoderNum == 0){
		encoderPinA = ENCODER_PIN1A;
		encoderPinB = ENCODER_PIN1B;
	} else if(encoderNum == 1){
		encoderPinA = ENCODER_PIN2A;
		encoderPinB = ENCODER_PIN2B;
	} else if(encoderNum == 2){
		encoderPinA = ENCODER_PIN3A;
		encoderPinB = ENCODER_PIN3B;
	}
		
	currentEncoder[encoderNum] = digitalRead(encoderPinA);
	currentEncoder[encoderNum + 4] = digitalRead(encoderPinB);
	if (lastEncoder[encoderNum] == LOW && currentEncoder[encoderNum] == HIGH) {
		if(currentEncoder[encoderNum + 4] == LOW){
			encoderValue[encoderNum]--;
		} else {
			encoderValue[encoderNum]++;
		}
		
	}
	lastEncoder[encoderNum] = currentEncoder[encoderNum];
	lastEncoder[encoderNum + 4] = currentEncoder[encoderNum + 4];
}

//update the current time and last time
void drive::updateTime(){
	lastTime = time;
	time = millis();
}

/**
 * @brief Get the linear speed of the robot using the motor encoder
 * @param axis: speed along x or y axis (of the robot)
 * @return instantaneous linear speed
 */
float drive::getLinearSpeedEncoder(char axis){ 
	int encoderNum = 0;
	if(axis == 'x')
		encoderNum = 0;
	else
		encoderNum = 2;
	int lastPos = encoderToCm(lastEncoder[encoderNum]);
	int pos = encoderToCm(currentEncoder[encoderNum]);
	
	return (pos - lastPos) / (time - lastTime) * sqrt(2);
}

float drive::getAngularSpeedEncoder(char axis){ 
	return getLinearSpeedEncoder(axis) / ROBOT_RADIUS; //???
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
float drive::getSpeedAccel(char axis, float vel){ 
	
	//v = v_0 + at
	return vel + getAccelerometerData(axis) * (time - lastTime);
}

/**
 * @brief Get the angular velocity from the gyro sensor
 * @return angular velocity
 */
float drive::getSpeedGyro(){ //change to angular velocity
	return getGyroData();
}

/**
 * @brief Convert cm to encoder ticks
 * @param cm: length in centimeters
 * @return encoder ticks
 */
int drive::cmToEncoder(int cm){ 
	float circ = 2 * M_PI * WHEEL_RADIUS;
		
	return cm / circ * 64; //or 16??
}

/**
 * @brief Convert encoder ticks to cm
 * @param encoder: encoder ticks
 * @return length in centimeters
 */
int drive::encoderToCm(int encoder){ 
	float circ = 2 * M_PI * WHEEL_RADIUS;
	
	return encoder / 64 * circ;
}

/**
 * @brief Check to see if we are in a room
 * 			If the color sensor sees white, we are
 * 			Otherwise, we are not
 */
void drive::updateInRoom(){
	uint16_t r, g, b, c;
	color.getRawData(&r, &g, &b, &c);
	
	if(r > 250 && g > 250 && b > 250){ //white -- update these values
		inRoom = true;
	} else {
		inRoom = false;
	}
}

/**
 * @brief Get data from the accelerometer
 * @param axis: 'x', 'y', or 'z'
 * @return acceleration in m/s^2
 */
float drive::getAccelerometerData(char axis){
	//raw data / 2^14 * 9.8
	float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
	AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
	AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	if(axis == 'x')
		return AcX / pow(2, 14) * 9.8;
	else
		return AcY / pow(2, 14) * 9.8;
}

/**
 * @brief Get data from the gyro
 * @return angular velocity (in ?? units) --FIX
 */
float drive::getGyroData(){
	float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
	AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
	AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	
	return GyZ; //CHANGE TO ANGULAR VEL
}

//drives the robot
/**
 * @brief Drive the robot sideways/forward along the x and y axis (relative to the robot)
 * @param x: x distance in cm
 * @parem y: y distance in cm
 */
void drive::drive(int x, int y)
{	
	float xCurrent = 0; //current x position
	float yCurrent = 0; //current y position
	
	int initialEnc = { encoderValue[0], encoderValue[1], encoderValue[2], encoderValue[3] };
	
	//initially 0
	float vAccelX = 0;
	float vAccelY = 0;
	float vEnc = 0;
  
	//PID constants !!FIND THESE!!
	const float posKP = 0;	
	const float posKI = 0;
	const float posKD = 0;
  
	const float velKP = 0;
	const float velKI = 0;
	const float velKD = 0;
	
	const float MAX_V = 0; //max velocity
	const float MAX_P = 255; //max motor power
	
	PID myPID; //object for calculating PID 
	
	//calculate and correct for error n number of times
	for (int n = 0; n < 100; n++)
	{
		//Update Current Position
		updateTime();
		updateInRoom();
		for(int i = 0; i < 4; i++)
			updateEncoder(i);
		xCurrent = encoderToCm(encoderValue[0] - encoderToCm(initialEnc[0]);
		yCurrent = encoderToCm(encoderValue[2] - encoderToCm(initialEnc[2]));

		//FIND TARGET VELOCITY
		float input, output, setpoint;
		input = xCurrent;
		setpoint = x;
		myPID = PID(&input, &output, &setpoint, posKP, posKI, posKD, AUTOMATIC); //error in x distance
		myPID.Compute();
		float target_v_x = output;
		input = yCurrent;
		setpoint = y;
		myPID.Compute();
		float target_v_y = output;
	
		//GET CURRENT X/Y VELOCITY
		vAccelX = getSpeedAccel(x, vAccelX); 
		vEnc = getLinearSpeedEncoder(x);
		float current_v_x = (vAccelX + vEnc) / 2;
		vAccelY = getSpeedAccel(y, vAccelY);
		vEnc = getLinearSpeedEncoder(y);
		float current_v_y = (vAccelY + vEnc) / 2;
	
		//FIND TARGET MOTOR POWER
		input = current_v_x;
		setpoint = target_v_x;
		myPID.setTunings(velKP, velKI, velKD);
		myPID.Compute();
		float power_x = output;
		input = current_v_y;
		setpoint = target_v_y;
		myPID.Compute();
		float power_y = output;
	
		//adjust speed based on speed error - set motor power
		//power x = wheel set 1
		//power y = wheel set 2
		analogWrite(MOTOR_PIN1, power_x);
		analogWrite(MOTOR_PIN2, power_x);
		analogWrite(MOTOR_PIN3, power_y);
		analogWrite(MOTOR_PIN4, power_y);
	}
	
	//Update global current position variables + correct for current robot angle
	currentXPos += xCurrent * cos(totalDeg * M_PI / 180);
	currentYPos += yCurrent * cos(totalDeg * M_PI / 180);
	
	//Print whether we are in the room, and current position
	Serial.print("In Room: "); Serial.print(inRoom);
	Serial.print("\nX: "); Serial.print(currentXPos);
	Serial.print("\nY: "); Serial.print(currentYPos);
}

/**
 * @brief Turn the robot the specified angle
 * @param angle; angle in degrees
 */
void turn(int degrees){
	//Find the linear distance the robot needs to go
	int linear_distance = ROBOT_RADIUS * degrees * M_PI / 180;
	totalDeg += degrees;
	
	//current pos
	int current = 0;
	int initialEnc = { encoderValue[0], encoderValue[1], encoderValue[2], encoderValue[3] };
	
	//save initial values
	int initialLastEnc = { lastEncoder[0], lastEncoder[1], lastEncoder[2], lastEncoder[3] };
	int initialCEnc = { currentEncoder[0], currentEncoder[1], currentEncoder[2], currentEncoder[3] };
	
	//initially 0
	float vGyro = 0;
	float vEnc = 0;
  
	//PID constants !!FIND THESE!!
	const float posKP = 0;	
	const float posKI = 0;
	const float posKD = 0;
  
	const float velKP = 0;
	const float velKI = 0;
	const float velKD = 0;
	
	const float MAX_V = 0; //max velocity
	const float MAX_P = 255; //max motor power
	
	//calculate and correct for error n number of times
	for (int n = 0; n < 100; n++)
	{
		//Update Current Position
		updateTime();
		updateInRoom();
		for(int i = 0; i < 4; i++)
			updateEncoder(i);
		current = encoderToCm(encoderValue[0]) - encoderToCm(initialEnc[0]);

		//FIND TARGET VELOCITY (LINEAR (?))
		float input, output, setpoint;
		input = current;
		setpoint = linear_distance;
		myPID = PID(&input, &output, &setpoint, posKP, posKI, posKD, AUTOMATIC); //error in x distance
		myPID.Compute();
		float target_v = output;
		
		//convert vel to angular velocity
		target_v = target_v / ROBOT_RADIUS;

		//GET CURRENT ANGULAR VELOCITY
		vGyro = getSpeedGyro(); 
		vEnc = getAngularSpeedEncoder(x); 
		float current_v_x = (vGyro + vEnc) / 2; //angular

	
		//FIND TARGET MOTOR POWER
		input = current_v;
		setpoint = target_v;
		myPID.setTunings(velKP, velKI, velKD);
		myPID.Compute();
		float power = output;

	
		//adjust speed based on speed error - set motor power
		//all wheels have the same power because we're spinning
		//power x = wheel set 1
		//power y = wheel set 2
		analogWrite(MOTOR_PIN1, power);
		analogWrite(MOTOR_PIN2, power);
		analogWrite(MOTOR_PIN3, power);
		analogWrite(MOTOR_PIN4, power);
	}
	
	//turning is not included in normal encoder values -reset
	encoderValue = initalEnc;
	lastEncoder = initialLastEnc;
	currentEncoder = initialCEnc;
}

