class Robit{
  PVector pos, vel, acc;
  int[] gridPos;
  float angle;
  float vAng;
  int sensorDist;//in pixels to approximate real world
  Cell[][] grid;
  int[][] distanceField;
  
  Robit(PVector initPos, float initAngle){
    pos = new PVector(initPos.x, initPos.y);
    vel = new PVector(0,0);
    acc = new PVector(0,0);
    angle = initAngle;
    sensorDist = int(800.0/244.0 * 100);
    grid = new Cell[int(1600/precision)][int(1600/precision)];
    for(int i = 0; i < grid.length; i ++)
      for(int j = 0; j < grid[0].length; j ++)
        grid[i][j] = Cell.UNKNOWN;
    gridPos = new int[2];
    gridPos[0] = gridPos[1] = grid.length/2;
  }
  
  
  void update(){
    scanSurroundings();//unrealistic temporary spotholder
  }
  
  void move(int x, int y){
    gridPos[0] += x;
    gridPos[1] += y;
    pos.x += x * precision;
    pos.y += y * precision;
  }
  
  void scanSurroundings(){
    //Robot rotates 360 degrees, recording distance data, and uses that data to construct where the walls are in the gridmap
    float temp = angle;
    for(float targetAngle = angle + 2*PI; angle < targetAngle; angle += PI/180){
      float distanceReading = computeDistance();
      if (distanceReading < sensorDist){
        int targetCellX = int((distanceReading * cos(angle))/precision + gridPos[0]);
        int targetCellY = int((distanceReading * sin(angle))/precision + gridPos[1]);
        grid[targetCellX][targetCellY] = Cell.WALL;
        for(int i = -2; i <= 2; i ++)
          for(int j = -2; j <= 2; j ++)
            if(grid[targetCellX + i][targetCellY + j] != Cell.WALL)
            grid[targetCellX + i][targetCellY + j] = Cell.BLOCKED;
      }
        for(int i = 0; i < distanceReading; i ++){
          int cellX = int(i * cos(angle)/precision + gridPos[0]);
          int cellY = int(i * sin(angle)/precision + gridPos[1]);
          if (grid[cellX][cellY] == Cell.UNKNOWN)
            grid[cellX][cellY] = Cell.CLEAR;
      }
    }
    angle = temp;
  }
  
  int computeDistance(){
    //returns distance to closest wall in direction robot is facing
    PVector looking = new PVector(cos(angle), sin(angle));
    for(int i = 10; i < sensorDist; i ++){
      looking.setMag(i);
      color maybeWall = get(int(looking.x + pos.x), int(looking.y + pos.y));
      if (isWall(maybeWall))
        return i;
    }
    return (int)sensorDist;
  }
  
  boolean isWall(color toCheck){
    //used to mimic sonar sensors
    return (red(toCheck) >= 128 && green(toCheck) >= 0 && blue(toCheck) >= 128);
  }
  
  void computeDistanceField(){
    
  }
  
  
  
  
  void display(){
    //display the robot
    fill(0, 255, 0);
    stroke(0, 255, 0);
    ellipse(pos.x, pos.y, 10, 10);
    ellipse(pos.x + 5*cos(angle), pos.y + 5*sin(angle), 5, 5);
    fill(0, 0, 0, 0);
    ellipse(pos.x, pos.y, 2*sensorDist, 2*sensorDist);
    gridDisplay();
    fill(0, 255, 0);
    ellipse(1200 + precision/4, 400 + precision/4, precision, precision);
  }
  
  void gridDisplay(){
    //display all data the robot has collected
    stroke(25);
    for(int i = 0; i < grid.length; i ++){
      for(int j = 0; j < grid[i].length; j ++){
        switch(grid[i][j]){
        case CLEAR:
          fill(0);
          break;
        case WALL:
          fill(255);
          break;
        case BLOCKED:
          fill(180);
          break;
        default:
          fill(128);
        }
        if((i-gridPos[0])*precision/2 > -400)
        rect(1200 + (i-gridPos[0])*precision/2, 400 + (j - gridPos[1])*precision/2, precision/2, precision/2);
      }
    }
  }

}