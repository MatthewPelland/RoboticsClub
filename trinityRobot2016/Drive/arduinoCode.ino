#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drive.h>
#include <Servo.h>

#define FIRE_PIN 10
#define CRADLE_PIN 10

Servo fire;
Servo cradle;

void extinguishFire();
void getCradle(int cm);
void shoveBabyOutWindow(int cm);
Drive d;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  fire.attach(FIRE_PIN);
  cradle.attach(CRADLE_PIN);
  d = Drive();
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
  char* test = "a 8 7 2";
  int x, y;
  char tmp;
  int temp1, temp2, temp3;
  int inc = 2;
  switch(c_str[0]){
      case 'm':   
                  while(sscanf(c_str + inc, "%d %d", &x, &y) == 2){
                    Serial.print("here1 :)\n");
                    d.go(x, y); //test this
                    //Serial.print("here2 :)\n");
                    inc += floor (log10 (abs (x))) + 1 + floor (log10 (abs (y))) + 1 + 1;
                  }
                  Serial.println("...");
                  
//                  d.go(175, 175, 175, 175);

                  //d.motor1.setSpeed(255);
                  //d.motor1.run(FORWARD);
                  break;
      case 'r': int deg;
                sscanf(c_str, "%d", &deg);
                d.turn(deg, 50); //test this
                break;
      case 'c': //getCradle();
                break;
      case 'w': int cm;
                sscanf(c_str, "%d", &cm);
                shoveBabyOutWindow(cm);
                break;
      case 'e': extinguishFire();
                break;
      case 'z': int angle;
                sscanf(c_str, "%d %d %d", &angle, &x, &y);
                d.setInitialPos(x, y, angle);
                break;
      case 's': d.motor1.run(RELEASE);
  }
}

void extinguishFire(){
  //press button
  fire.write(20);
  delay(2000); //2 seconds
  //unpress button
  fire.write(100);
}

void getCradle(int cm){
  //go forwards :)
  //analogWrite(MOTOR_PIN1, 255);
  //analogWrite(MOTOR_PIN2, -255);
  //analogWrite(MOTOR_PIN3, 255);
  //analogWrite(MOTOR_PIN4, -255);
  delay(1000); //1 second
  cradle.write(100);
  delay(500); //test this
//  d.drive(0, cm, 50); //test this - 2 cm 50 cm/s
}

void shoveBabyOutWindow(int cm){
  //move servo
}


