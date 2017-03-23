#include "Arduino.h"
#include "math.h"
#include "wheel.h"

wheel::Wheel(int eP, float ROBOT_RADIUS)
{
  encoderPin = eP;
  pinMode(encoderPin, INPUT);

  lastEncoder = digitalRead(encoderPin);
  currentEncoder = lastEncoder;
  encoderValue = 0;
}

//drives motors; I don't know how this will work yet
//power must be 0<=power<=255
void wheel::driveInches(float inches, int power, int motor) {

  pinMode(BRAKE[motor], OUTPUT);  // Brake pin on
  digitalWrite(BRAKE[motor], LOW);  // setting brake LOW disable motor brake

  analogWrite(PWM[motor], power);     // Set the speed of the motor
  encoderValue = 0;
  while (encoderValue < inchesToEncoder(inches)) {
    updateEncoder();
  }
  digitalWrite(BRAKE[motor], HIGH);  // raise the brake
  encoder_val[motor] = encoderValue; //final encoder value of each motor
}

//updates the encoder value (instantaneous; not the count)
void wheel::updateEncoder() {
  currentEncoder = digitalRead(encoderPin);
  if (currentEncoder != lastEncoder) {
    encoderValue++;
  }
  lastEncoder = currentEncoder;
}

/**
 * @brief PID Control Function
 * @param current: current value
 * @param setpoint: target value
 * @param KI: integral coefficient / constant
 * @param KP: position coefficient / constant
 * @param KD: derivative coefficient / constant
 * @param MAX_OUTPUT: maximum desired output
 * @return a corrected value based on the input
 * 			for position, a desired velocity
 * 			for velocity, a desired motor power
 */
float wheel::PID(float current, float setpoint, float KI, float KP, float KD, float MAX_OUTPUT)
{

  return 0; //adjusted value
}

//the distance between 2 points
float wheel::distance(float xi, float yi, float xf, float yf)
{
  return sqrt (((xi - xf) ^ 2) + ((yi - yf) ^ 2));
}

//the angle between 2 points in rad
float wheel::angle(float xi, float yi, float xf, float yf)
{
  return atan((yi - yf) / (xi - xf));
}

//the distance traveled by the robot in x based on wheel movements
float wheel::distancex(float rad) {
  float d1 = encoderToInches(encoder_val[1]);
  float d3 = encoderToInches(encoder_val[3]);
  return (d1 + d3) / (2 * cos(rad));
}

//the distance traveled by the robot in y based on wheel movements
float wheel::distancey(float rad) {
  float d2 = encoderToInches(encoder_val[2]);
  float d4 = encoderToInches(encoder_val[4]);
  return (d2 + d4) / (2 * cos(rad));
}

float wheel::getLinearSpeedEncoder(){ //implement

}

float wheel::getAngularSpeedEncoder(){ //implement

}

float wheel::getSpeedAccel(){ //implement

}

float wheel::getSpeedGyro(){ //implment
	
}


//drives the robot
/**
 * @brief 
 * @param rad: angle in radians
 * @param inches: inches to move (the hypotenuse)
 */
void wheel::drive(float rad, float inches)
{
  time_init = millis();

  float distancexTarget = inches * cos(rad); //distance in inches, x direction
  float distanceyTarget = inches * sin(rad); //distance in inches, y direction
  
  float xInitial = 0; //initial x position
  float yInitial = 0; //initial y position

  float xCurrent = 0; //current x position
  float yCurrent = 0; //current y position
  
  const float posKP = 0;
  const float posKI = 0;
  const float posKD = 0;
  
  const float velKP = 0;
  const float velKI = 0;
  const float velKD = 0;
  
  const float MAX_V = 0; //max velocity
  const float MAX_P = 100; //max motor power

  //calculate and correct for error n number of times
  for (int n = 0; n < 100; n++)
  {
	//Update Current Position
	xCurrent = xInitial + distancex(rad); 
	yCurrent = yInitial + distancey(rad);

    //FIND TARGET VELOCITY
    float target_v_x = PID(xCurrent, distancexTarget, posKP, posKI, posKD, MAX_V); //error in x distance
    float target_v_y = PID(yCurrent, distanceyTarget, posKP, posKI, posKD, MAX_V); //error in y distance
	
	//float target_v = sqrt(pow(target_v_x, 2) + pow(target_v_y,2)); //target velocity (m/s)
	//float target_v_ang = target_v / ROBOT_RADIUS; //target angular velocity (rad/s)
	
	float current_v = (getLinearSpeedEncoder() + getSpeedAccel()) / 2;
	float current_v_x = current_v * cos(rad);
	float current_v_y = current_v * sin(rad);
	
	//FIND MOTOR POWER
	float power_x = PID(current_v_x, target_v_x, velKP, velKI, velKD, MAX_P);
	float power_y = PID(current_v_y, target_v_y, velKP, velKI, velKD, MAX_P);
	
	//adjust speed based on speed error - set motor power
	//power x = wheel set 1
	//power y = wheel set 2
	
 }
}

