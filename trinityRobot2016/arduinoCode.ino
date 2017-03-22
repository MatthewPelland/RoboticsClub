#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drive.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  String s;
  int c;
  Drive d = Drive();
  if(Serial.available() > 0){
    c = Serial.read();
    while(c != '\n'){
      s += c;
      c = Serial.read();
    }
  }
  switch(s[0]){
      case 'm': int x, y;
                for(int i = 0; i < (s.length() - 1) / 4; i++){
                  sscanf(s.c_str(), "%d %d", &x, &y);
                  d.drive(x, y);
                }
                break;
      case 'r': int deg;
                sscanf(s.c_str(), "%d", &deg);
                d.turn(deg);
                break;
      case 'c': //get cradle
                break;
      case 'w': //shove baby out the window
                break;
      case 'e': //activate fire extinguisher
                break;
      case 'z': int angle;
                sscanf(s.c_str(), "%d %d %d", &angle, &x, &y);
                d.setInitialPos(x, y, angle);
                break;
  }
}
