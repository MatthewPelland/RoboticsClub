Robit robot;



void setup(){
  size(800, 800);
  background(0, 0, 0);
  robot = new Robit(new PVector(width/2, height/2), 0);
  maze = loadImage("mazeC.jpg");
  maze.resize(width, height);
}

boolean drawing = false;
PImage maze;

void draw(){
  background(maze);
  if (drawing){
    line(pmouseX, pmouseY, mouseX, mouseY);
  }
  robot.update();
  print(robot.determineRoom(), '\n');
  robot.display();
  color myColor = get(mouseX, mouseY);
}

void mouseClicked(){
  robot = new Robit(new PVector(mouseX, mouseY), 0);
}