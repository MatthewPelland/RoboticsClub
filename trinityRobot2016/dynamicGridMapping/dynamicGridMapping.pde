/**********************

  a, b, c, d to choose maps, r to make random maze

**********************/






Robit robot;
public enum Cell{UNKNOWN, WALL, CLEAR};
boolean mazeGeneration = false;//using generated maze rather than arenas
boolean generating  = false, go = false;//go is override to tell robot to update, generating - generating maze
Maze myMaze;
PGraphics mazeImg;
boolean paused = false;
PImage arena;
float precision = 5;

void setup(){
  frameRate(100000000);
  size(1600, 800);
  background(0, 0, 0);
  robot = new Robit(new PVector(width/4, height/2), 0);
  arena = loadImage("mazeD.jpg");
  mazeImg = createGraphics(width/2, height);
  myMaze = new Maze(width, height, 50, arena);
  //construct outer border so robot can't exit screen
  mazeImg.beginDraw();
  mazeImg.background(0);
  mazeImg.stroke(255);
  mazeImg.line(0,0, 0,800);
  mazeImg.line(0,799, 800,799);
  mazeImg.line(800,800, 800,0);
  mazeImg.line(800,0, 0,0);
  mazeImg.endDraw();
  arena.resize(width/2, height);
}


void draw(){
  background(128);
  if(mazeGeneration){
    if (generating){
      myMaze.generateWalls();
    }
    image(mazeImg, 0, 0);
  }
  else{
    image(arena, 0, 0);
  }
  if(!generating && !paused){
    drawGrid(precision);
    robot.update();
  }
  if(go)
    robot.update();
  robot.display();
}

void mouseClicked(){ //reposition and reinitiate robot
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
  /*switch(keyCode){
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
  }*/
  switch(key){
  case 'a':
  case 'b':
  case 'c':
  case 'd':
    //load respective arena onto screen
    mazeGeneration = false;
    generating = false;
    String map = "maze" + Character.toUpperCase(key) + ".jpg";
    arena = loadImage(map);
    arena.resize(width/2, height);
    break;
  case 's':
    robot.scanSurroundings();
    break;
  case 'r': //switch from arena to maze generation
    mazeGeneration = true;
    generating = true;
    break;
  case ' ': //pause generating
    generating = false;
    break;
  case 'x': //override robot to update
    go = true;
    break;
  case 'p':
    paused = !paused;
  }
  
}