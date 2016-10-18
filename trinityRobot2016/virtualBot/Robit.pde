class Robit{
  PVector pos, vel, acc;
  float angle;
  float vAng;
  int sensorDist;//in pixels to approximate real world
  Robit(PVector initPos, float initAngle){
    pos = new PVector(initPos.x, initPos.y);
    vel  =new PVector(0,0);
    acc = new PVector(0,0);
    angle = initAngle;
    sensorDist = int(800.0/244.0 * 100);
  }
  
  void update(){}
  
  
  int determineRoom(){
    ArrayList<Integer> distances = new ArrayList<Integer>();
    float temp = angle;
    for(float targetAngle = angle + 2*PI; angle < targetAngle; angle += 2*PI/360){
      distances.add(computeDistance());
    }
    angle = temp;
    int closestWallIndex = minimum(distances);
    int[] wallIndices = {closestWallIndex, (closestWallIndex + 90)%360, (closestWallIndex + 180)%360, (closestWallIndex + 270)%360};
    for(int i : wallIndices)
      print(distances.get(i), ' ');
    print(openingCount(distances), ' ');
    
    float averageDistance = 0;
    int count = 0;
    for(int i : wallIndices)
      if(distances.get(i) < 200 || true){
        count ++;
         averageDistance += distances.get(i);
      }
    averageDistance /= count;
         
    print("ave: ", averageDistance, ' ');
    if(averageDistance < 135)
      return 4;    
    else if (openingCount(distances) == 2)
      return 1; //only room with two openings
    else if(distances.get(wallIndices[0]) < 215 && distances.get(wallIndices[1]) < 215 && distances.get(wallIndices[2]) < 215 && distances.get(wallIndices[3]) < 215)
      return 2;
    else
      return 3;
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
  
  int openingCount(ArrayList<Integer> distances){
    int count = 0;
      for(int i = 0; i < 360; i++){
        if (distances.get(i) - distances.get((i + 1)%360) >= 25 || distances.get(i) == sensorDist && distances.get((i + 1)%360) < sensorDist)
          count ++;
      }
      return count;
  }
  
  void display(){
    fill(0, 255, 0);
    stroke(0, 255, 0);
    ellipse(pos.x, pos.y, 10, 10);
    ellipse(pos.x + 5*cos(angle), pos.y + 5*sin(angle), 5, 5);
    fill(0, 0, 0, 0);
    ellipse(pos.x, pos.y, 2*sensorDist, 2*sensorDist);
  }
  
  boolean isWall(color toCheck){
    return !(red(toCheck) == 0 && green(toCheck) == 0 && blue(toCheck) == 0);
  }
  
   int maximum(ArrayList<Integer> toFind){
    int best = 0;
    int index = -1;
  for(int i = 0; i < toFind.size(); i ++){
      print(toFind.get(i));
      if (toFind.get(i) > best){
        best = toFind.get(i);
        index = i;
      }
  }
    return index;
  }
  
   int minimum(ArrayList<Integer> toFind){
    int best = (int)sensorDist + 1;
    int index = -1;
    for(int i = 0; i < toFind.size(); i ++)
      if (toFind.get(i) < best && toFind.get((i-30+toFind.size())%toFind.size()) < 200 && toFind.get((i+30)%toFind.size()) < 200){
        best = toFind.get(i);
        index = i;
      }
    return index;
  }
}