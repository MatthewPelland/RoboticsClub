/*****************************
Purpose: writes a character to serial port for arduino to receive
         sends character corresponding to key being pressed down.
*****************************/
import processing.serial.*;

Serial myPort;  
void setup() 
{
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
}

char sending = '0';
void draw()
{
  if (sending != '0') //not sure why but code doesn't work without this
    myPort.write(sending);  
}

void keyPressed(){
  sending = key;
}