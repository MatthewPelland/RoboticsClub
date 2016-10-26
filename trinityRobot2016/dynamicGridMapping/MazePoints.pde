class MazePoints{
  ArrayList<ArrayList<float[]>> points = new ArrayList<ArrayList<float[]>>();
  
  MazePoints(float[] centerPoint){
    ArrayList<float[]> first = new ArrayList<float[]>();
    first.add(centerPoint);
    points.add(first);
  }
  
  
  void addNewPoint(float[] point){
    ArrayList<float[]> add = new ArrayList<float[]>();
    add.add(point);
    points.add(add);
  }
  
  void addToPoint(float[] point, float[] toAdd){
    int index = getIndexOf(point);
    ArrayList<float[]> modded = points.get(index);
    modded.add(toAdd);
    points.set(index, modded);
  }
  
  ArrayList<float[]> wallsFromPoint(float[] point){
    ArrayList<float[]> walls = copyArray(points.get(getIndexOf(point)));
    walls.remove(0);
    return walls;
  }
  
  ArrayList<float[]> copyArray(ArrayList<float[]> toCopy){
    ArrayList<float[]> copy = new ArrayList<float[]>();
    for(float[] i : toCopy)
      copy.add(i.clone());
    return copy;
  }
  
  int getIndexOf(float[] point){
    for(int i = 0; i < points.size(); i ++){
      ArrayList<float[]> checking = points.get(i);
      if (checking.get(0)[0] == point[0] && checking.get(0)[1] == point[1]){
        return i;
      }
    }
    return -1;
  }
}