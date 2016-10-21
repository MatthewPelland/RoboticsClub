Robit robot;
public enum Cell{UNKNOWN, WALL, CLEAR, BLOCKED};//blocked means too close to wall to move there


void setup(){
  size(1600, 800);
  background(0, 0, 0);
  robot = new Robit(new PVector(width/4, height/2), 0);
  maze = loadImage("mazeD.jpg");
  maze.resize(width/2, height);
}
boolean update = true;

boolean drawing = false;
PImage maze;
float precision = 5;
void draw(){
  background(128);
  image(maze, 0, 0);
  drawGrid(precision);
  robot.update();
  robot.display();
  update = !update;
}

void mouseClicked(){
  robot = new Robit(new PVector(mouseX, mouseY), 0);
}


void drawGrid(float precision){
  stroke(128, 128, 128);
  for(int i = 0; i < 800; i += precision/2){
    line(800, i, 1600, i);
    line(800 + i, 0, 800 + i, 800);
  }
}



void keyPressed(){
  switch(keyCode){
    case UP:
      robot.move(0, -1);
      break;
    case DOWN:
      robot.move(0, 1);
      break;
    case LEFT:
      robot.move(-1, 0);
      break;
    case RIGHT:
      robot.move(1, 0);
  }
  switch(key){
  case 'a':
  case 'b':
  case 'c':
  case 'd':
    
  }
}