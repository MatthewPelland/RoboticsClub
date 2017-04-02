#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drive.h>
#include <Servo.h>

#define cradleDIRPin1 41
#define cradleDIRPin2 43 
#define cradlePWMPin1 15
#define cradlePWMPin2 16
#define firePin 53
#define babyPin 51

Servo fire;
Servo baby;

void extinguishFire();
void getCradle(int cm);
void shoveBabyOutWindow(int cm);

Drive d;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  fire.attach(firePin);
  baby.attach(babyPin);
  fire.write(0);
  baby.write(0);

  pinMode(cradleDIRPin1, OUTPUT);
  pinMode(cradleDIRPin2, OUTPUT);
  pinMode(cradlePWMPin1, OUTPUT);
  pinMode(cradlePWMPin2, OUTPUT);

  digitalWrite(cradleDIRPin1, LOW);
  digitalWrite(cradleDIRPin2, LOW);
  digitalWrite(cradlePWMPin1, LOW);
  digitalWrite(cradlePWMPin2, LOW);
  d = Drive();
  Serial.print(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  String s;
  int c;
  if(Serial.available() > 0){
    c = Serial.read();
    while(c != '\n' && c != '^'){
      if(Serial.available()){
        s += (char) c;
        c = Serial.read();
      }
    }
  }
  
  char* c_str = s.c_str();
  int x, y, deg, cm;
  switch(c_str[0]){
      case 'm':   
                sscanf(c_str + 2, "%d %d", &x, &y);
                d.go(x, y); //test this
                break;
      case 'r': int deg;
                sscanf(c_str + 2, "%d", &deg);
                d.turn(deg, 150); //test this
                break;
      case 'c':
                sscanf(c_str + 2, "%d", &cm);
                getCradle(cm);
                break;
      case 'w':
                sscanf(c_str + 2, "%d", &cm);
                shoveBabyOutWindow(cm);
                break;
      case 'e': Serial.print(5);
                extinguishFire();
                break;
      case 'z': 
                sscanf(c_str + 2, "%d %d %d", &deg, &x, &y);
                d.setInitialPos(x, y, deg);
                break;
      case 's': d.motor1.setSpeed(255);
                d.motor2.setSpeed(255);
                d.motor3.setSpeed(255);
                d.motor4.setSpeed(255);
                d.motor1.run(FORWARD);
                d.motor2.run(FORWARD);
                d.motor3.run(FORWARD);
                d.motor4.run(FORWARD);
  }
}

void extinguishFire(){
  //press button
  fire.write(38);
  Serial.print(7);
  for(int i = 0; i < 100; i++)
    delay(1);
  Serial.print(8);
  //unpress button
  fire.write(50);
  Serial.print(1);
}

void getCradle(int cm){
  //go forwards :)
  /*d.motor1.setSpeed(255);
  d.motor2.setSpeed(230);
  d.motor3.setSpeed(190);
  d.motor4.setSpeed(190);
  d.motor1.run(FORWARD);
  d.motor2.run(BACKWARD);
  d.motor3.run(BACKWARD);
  d.motor4.run(FORWARD);
  delay(2000);
  d.motor1.run(RELEASE);
  d.motor2.run(RELEASE);
  d.motor3.run(RELEASE);
  d.motor4.run(RELEASE);*/
  digitalWrite(cradleDIRPin1, LOW);
  digitalWrite(cradleDIRPin2, LOW);
  digitalWrite(cradlePWMPin1, LOW);
  digitalWrite(cradlePWMPin2, LOW);  
  Serial.print(1);
  delay(150);
  digitalWrite(cradleDIRPin1, HIGH);
  digitalWrite(cradleDIRPin2, HIGH);
  analogWrite(cradlePWMPin1, 50);
  analogWrite(cradlePWMPin2, 50);
  Serial.print(2);
  delay(1000); //test this
  digitalWrite(cradleDIRPin1, LOW);
  digitalWrite(cradleDIRPin2, LOW);
  digitalWrite(cradlePWMPin1, LOW);
  digitalWrite(cradlePWMPin2, LOW);
  //d.driveCm(cm); //back up
  Serial.print(3); //done
}

void shoveBabyOutWindow(int cm){
  //go forwards :)
  d.motor1.setSpeed(255);
  d.motor2.setSpeed(230);
  d.motor3.setSpeed(190);
  d.motor4.setSpeed(190);
  d.motor1.run(FORWARD);
  d.motor2.run(BACKWARD);
  d.motor3.run(BACKWARD);
  d.motor4.run(FORWARD);
  delay(2000);
  d.motor1.run(RELEASE);
  d.motor2.run(RELEASE);
  d.motor3.run(RELEASE);
  d.motor4.run(RELEASE);
  baby.write(100);
  delay(500); //test this
  d.driveCm(cm); //back up
  Serial.print(1); //done
}


