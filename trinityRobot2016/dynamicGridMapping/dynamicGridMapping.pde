/**********************

  a, b, c, d to choose maps, r to make random maze

**********************/






Robit robot;
public enum Cell{UNKNOWN, WALL, CLEAR};//blocked means too close to wall to move there
boolean mazeGeneration = false;
boolean generating  = false, go = false;
Maze myMaze;
PGraphics mazeImg;

void setup(){
  frameRate(100000000);
  size(1600, 800);
  background(0, 0, 0);
  robot = new Robit(new PVector(width/4, height/2), 0);
  maze = loadImage("mazeD.jpg");
  mazeImg = createGraphics(width/2, height);
  myMaze = new Maze(width, height, 50, maze);
  mazeImg.beginDraw();
  mazeImg.background(0);
  mazeImg.stroke(255);
  mazeImg.line(0,0, 0,800);
  mazeImg.line(0,799, 800,799);
  mazeImg.line(800,800, 800,0);
  mazeImg.line(800,0, 0,0);
  mazeImg.endDraw();
  maze.resize(width/2, height);
}
boolean update = true;

boolean drawing = false;
PImage maze;
float precision = 5;
void draw(){
  background(128);
  if(mazeGeneration){
    if (generating){
      myMaze.generateWalls();
    }
    image(mazeImg, 0, 0);
  }
  else{
    image(maze, 0, 0);
  }
  if(!generating){
    drawGrid(precision);
    robot.update();
    //delay(100);
  }
  if(go)
  robot.update();
  robot.display();
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
    mazeGeneration = false;
    generating = false;
    String map = "maze" + Character.toUpperCase(key) + ".jpg";
    maze = loadImage(map);
    maze.resize(width/2, height);
    break;
  case 's':
    robot.scanSurroundings();
    break;
  case 'r':
    mazeGeneration = true;
    generating = true;
    break;
  case ' ':
    generating = false;
    break;
  case 'x':
    go = true;
    break;
  }
  
}