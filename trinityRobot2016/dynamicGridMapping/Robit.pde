class Robit{
  PVector pos, vel, acc;
  int[] gridPos;
  float angle;
  float vAng;
  int sensorDist;//in pixels to approximate real world
  Cell[][] grid;
  int[][] distanceField;
  boolean targetReached;
  int targetDiameter;
  int[] target;
  ArrayList<int[]> moves;
  Robit(PVector initPos, float initAngle){
    pos = new PVector(initPos.x, initPos.y);
    vel = new PVector(0,0);
    acc = new PVector(0,0);
    angle = initAngle;
    sensorDist = int(800.0/244.0 * 100);
    grid = new Cell[int(1600/precision)][int(1600/precision)];
    distanceField = new int[int(1600/precision)][int(1600/precision)];
    moves = new ArrayList<int[]>();
    for(int i = 0; i < grid.length; i ++)
      for(int j = 0; j < grid[0].length; j ++){
        distanceField[i][j] = -1;
        grid[i][j] = Cell.UNKNOWN;
      }
    gridPos = new int[2];
    gridPos[0] = gridPos[1] = grid.length/2;
    targetReached = true;
    targetDiameter = 0;
  }
  
  
  void update(){
    if(advance()){
      scanSurroundings();
      createTargetPath();
    }
  }
  
  boolean advance(){//advance towards the target
    if (moves.size() == 0)
      return true;
    else{
      int deltaX = moves.get(0)[0] - gridPos[0];
      int deltaY = moves.get(0)[1] - gridPos[1];
      gridPos[0] += deltaX;
      gridPos[1] += deltaY;
      pos.x += deltaX * precision;
      pos.y += deltaY * precision;
      moves.remove(0);
      return false;
    }
  }
  
  void move(int x, int y){
    gridPos[0] += x;
    gridPos[1] += y;
    pos.x += x * precision;
    pos.y += y * precision;
  }
  
  void scanSurroundings(){
    
    //   THIS IS A REALLY GOOD IDEA MUST DO |
    //                                      V
    
    //////////////////////////////////////////////////////////////////////////////////////////////
    //when there is a wall ignore readings from previous and next few, unless reading was a wall//
    //////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    //Robot rotates 360 degrees, recording distance data, and uses that data to construct where the walls are in the gridmap
    float temp = angle;
    for(float targetAngle = angle + 2*PI; angle < targetAngle; angle += PI/180){
      float distanceReading = computeDistance();
      if (distanceReading < sensorDist){
        int targetCellX = int((distanceReading * cos(angle))/precision + gridPos[0]);
        int targetCellY = int((distanceReading * sin(angle))/precision + gridPos[1]);
        grid[targetCellX][targetCellY] = Cell.WALL;
        for(int i = -1; i <= 1; i ++)
          for(int j = -1; j <= 1; j ++)
            if(grid[targetCellX + i][targetCellY + j] != Cell.WALL)
              grid[targetCellX + i][targetCellY + j] = Cell.WALL;
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
  
  int createTargetPath(){
    //fills moves with list of all moves necessary to reach target tile
    moves.clear(); 
    int[][] distanceFieldClone = distanceField.clone();
    target = findClosestUnknown();
    targetDiameter = 250;
    //if there are no more unknowns on the map
    if(target[0] == -1)
      return 1; //map completed
    //otherwise...
    int distance = distanceField[target[0]][target[1]];
    int currentDist = distance;
    int[] currentSquare = target;
    int[] initialDirection = {7, 7};
    boolean changedDirection = false;
    for(int reverseCount = 0; reverseCount < distance; reverseCount ++){
      ArrayList<int[]> openNeighbors = openNeighbors(currentSquare);
      for(int[] neighbor : openNeighbors){
        //if the neighbor is one square closer to robot than the current square
        if (distanceFieldClone[neighbor[0]][neighbor[1]] == currentDist-1){
          currentDist = distanceFieldClone[neighbor[0]][neighbor[1]];
          if(initialDirection[0] == 7){
            initialDirection[0] = neighbor[0] - currentSquare[0];
            initialDirection[1] = neighbor[1] - currentSquare[1];
          }
          
          if (changedDirection){
            moves.add(0, neighbor);
          }
          else{
            if (neighbor[0] - currentSquare[0] != initialDirection[0] || neighbor[1] - currentSquare[1] != initialDirection[1] 
                || neighbor[0] - target[0] > sensorDist || neighbor[1] - target[1] > sensorDist){
                  changedDirection = true;
                  //this loop runs from target to robot, so moves are inserted to the front as we come across them
                  moves.add(0, neighbor);
                }
          }
          
          
          ///////////////////////////////////////////////////
          //                                               //
          //         can still be improved...              //
          //                                               //
          ///////////////////////////////////////////////////
          
          
          ///////////////////////////////////////////////////////////
          // every time direction changes, create a waypoint       //
          // robot motion profiles to each waypoint, then corrects //
          ///////////////////////////////////////////////////////////
          
          

          currentSquare = neighbor;
        }
      }
    }
    return 0;//map not completed
  }
  
  float distance(int[] pos1, int[] pos2){
    return sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2));
  }
  
  int[] findClosestUnknown(){
    for(int i = 0; i < distanceField.length; i ++)
      for(int j = 0; j < distanceField[0].length; j ++)
        distanceField[i][j] = -1;
    //finds the closest unknown tile
    ArrayList<int[]> boundary = new ArrayList<int[]>();
    int[] robotCoords = {gridPos[0], gridPos[1]};
    boundary.add(robotCoords);
    distanceField[robotCoords[0]][robotCoords[1]] = 0;
    while(boundary.size() > 0){
      int[] checking = boundary.get(0);
      boundary.remove(0);
      ArrayList<int[]> openNeighbors = openNeighbors(checking);
      for(int[] neighbor : openNeighbors){
        if(grid[neighbor[0]][neighbor[1]] == Cell.UNKNOWN){//shouldn't keep going infinitely, this is awkward
          distanceField[neighbor[0]][neighbor[1]] = distanceField[checking[0]][checking[1]] + 1;
          return neighbor; //closest unknown tile
        }
        else{
          if(distanceField[neighbor[0]][neighbor[1]] == -1){
            boundary.add(neighbor);
            distanceField[neighbor[0]][neighbor[1]] = distanceField[checking[0]][checking[1]] + 1;
          }
        }
      }
    }
    int[] noneFound = {-1, -1};
    return noneFound;  //no unknown tiles left, maze completely mapped
  }
  
  ArrayList<int[]> openNeighbors(int[] coords){
    //returns list of coordinates neighboring input coordinates where robot could move
    ArrayList<int[]> openNeighbors = new ArrayList<int[]>();
    if(grid[coords[0] + 1][coords[1]] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0] + 1, coords[1]};
      openNeighbors.add(neighbor);
    }
    if(grid[coords[0] - 1][coords[1]] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0] - 1, coords[1]};
      openNeighbors.add(neighbor);
    }
    if(grid[coords[0]][coords[1] + 1] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0], coords[1] + 1};
      openNeighbors.add(neighbor);
    }
    if(grid[coords[0]][coords[1] - 1] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0], coords[1] - 1};
      openNeighbors.add(neighbor);
    }
    //For if we think we can move diagonally
    /*
    if(grid[coords[0] + 1][coords[1]+ 1] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0] + 1, coords[1] +1};
      openNeighbors.add(neighbor);
    }
    if(grid[coords[0] - 1][coords[1]-1] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0] - 1, coords[1]-1};
      openNeighbors.add(neighbor);
    }
    if(grid[coords[0]-1][coords[1] + 1] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0]-1, coords[1] + 1};
      openNeighbors.add(neighbor);
    }
    if(grid[coords[0]+1][coords[1] - 1] != Cell.WALL && grid[coords[0]][coords[1]] != Cell.BLOCKED){
      int[] neighbor = {coords[0] + 1, coords[1] - 1};
      openNeighbors.add(neighbor);
    }*/
    
    return openNeighbors;
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
    if((targetDiameter -= 25) > 0){
      stroke(255, 0, 0);
      noFill();
      ellipse(1200 + (target[0]-gridPos[0])*precision/2, 400 + (target[1] - gridPos[1])*precision/2, targetDiameter, targetDiameter);
    }
  }
  
  void gridDisplay(){
    //display all data the robot has collected
    stroke(25);
    for(int i = 0; i < grid.length; i ++){
      for(int j = 0; j < grid[i].length; j ++){
        switch(grid[i][j]){
        case CLEAR:
          if(distanceField[i][j] != -1){
            fill((distanceField[i][j] * 10)%255, 255 - (distanceField[i][j] * 10)%255, 0);
          }
          else{
            fill(0);
          }
          break;
        case WALL:
          fill(255);
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