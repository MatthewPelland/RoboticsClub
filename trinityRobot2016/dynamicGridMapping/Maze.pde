class Maze{
  int width, height;
  ArrayList<float[]> usingPoints = new ArrayList<float[]>();
  ArrayList<float[]> allPoints = new ArrayList<float[]>();
  MazePoints maze;
  float[] centerPoint = {0,0};
  int wallLength;
  PImage image;
  public Maze(int w, int h, int wLength, PImage img){
    width = w;
    height = h;
    centerPoint[0] = width/4;
    centerPoint[1] = height/2;
    usingPoints.add(centerPoint);
    wallLength = wLength;
    image = img;
    image.resize(width, height);
    maze = new MazePoints(centerPoint);
  }
  
  
  
  public void generateWalls(){
    if (usingPoints.size() > 0){
      float[] usingPoint = usingPoints.get(int(random(usingPoints.size())));
      ArrayList<float[]> neighbors = openNeighbors(usingPoint);
      if ( neighbors.size() == 0){
        usingPoints.remove(usingPoint);
      }
      else{
        float[] neighbor = neighbors.get(int(random(neighbors.size())));
        maze.addToPoint(usingPoint, neighbor);
        maze.addNewPoint(neighbor);
        maze.addToPoint(neighbor, usingPoint);
        usingPoints.add(neighbor);
        allPoints.add(neighbor);
       
        mazeImg.beginDraw();
        mazeImg.stroke(colorToUse(neighbor[0], neighbor[1]));
        mazeImg.line(usingPoint[0], usingPoint[1], neighbor[0], neighbor[1]);
        mazeImg.endDraw();
      }
    }
  }
  
   ArrayList<float[]> openNeighbors(float[] point){
     ArrayList<float[]> neighbors = new ArrayList<float[]>();
     for(float i = 0; i < 2*PI; i += PI/2){
       float[] neighbor = new float[]{point[0] + cos(i) * wallLength, point[1] + sin(i) * wallLength};
       if(!inPoints(neighbor) && neighbor[0] < 800 && neighbor[0] > 0 && neighbor[1] < 800 && neighbor[1] > 0){
         neighbors.add(neighbor);
       }
     }
     return neighbors;
   }
   
   boolean inPoints(float[] point){
     for(float[] i : allPoints){
       if (abs(i[0] - point[0]) < .9 && abs(i[1] - point[1]) < .9)
         return true;
     }
     return false;
   }
   
color colorToUse(float x, float y) {
  PImage img = image.get(int(x), int(y), wallLength, wallLength);
  img.loadPixels();
  int r = 0, g = 0, b = 0;
  for (int i=0; i<img.pixels.length; i++) {
    color c = img.pixels[i];
    r += c>>16&0xFF;
    g += c>>8&0xFF;
    b += c&0xFF;
  }
  r /= img.pixels.length;
  g /= img.pixels.length;
  b /= img.pixels.length;
  return 255;//color(r, g, b);
  }     
}