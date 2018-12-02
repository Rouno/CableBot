/* Winch class describes winch attributes such as present position, velocity and load 
as well as goal position, velocity and load
*/

class Winch{
  
  private int id;
  private PVector coordinates;
  private float presentLength = 0;
  private float presentSpeed = 1023;
  private float presentLoad = 0;
  private float goalLength = MOUNT_LENGTH_in_mm;
  private float goalSpeed = MAX_WINCH_SPEED;
  private float goalLoad = MIN_WINCH_LOAD;
  private float zeroOffset = 0;
  private float radiusOffset = 0;
  private color c;
  
  Winch(int ID, PVector coord){
    this.id = ID;
    this.c = 255/(this.id+2);
    this.coordinates = new PVector();
    this.setCoordinates(coord);
  }
  
  Winch(int ID, float len){
    this.id = ID;
    this.c = 255/(this.id+2);
    this.setPresentLength(len);
    this.coordinates = new PVector(0,0,0);
  }
  
  void setCoordinates(PVector newCoord){
    this.coordinates = newCoord.copy();
  }
  
  void offsetCoordinates(PVector offset){
    this.coordinates.add(offset);
  }
  
  PVector getCoordinates(){
    return this.coordinates;
  }
  
  void setPresentValues(float len, float speed, float load){
    this.presentLength = len;
    this.presentSpeed = speed;
    this.presentLoad = load;
  }
  
  float[] getPresentValues(){
    float[] result = {this.presentLength,this.presentSpeed,this.presentLoad};
    return result;
  }
  
  float getPresentLength(){
    return this.presentLength;
  }
  
  void setPresentLength(float len){
    this.presentLength = len;
  }
  
  float getPresentSpeed(){
    return this.presentSpeed;
  }
  
  float getPresentLoad(){
    return this.presentLoad;
  }
  
  
  void setGoalValues(float len, float speed, float load){
    this.goalLength = len;
    this.goalSpeed = speed;
    this.goalLoad = load;
  }
  
  void setGoalLength(float len){
    this.goalLength = len;
  }
  
  float getGoalLength(){
    return this.goalLength;
  }
  
  void setGoalSpeed(float speed){
    this.goalSpeed = speed;
  }
  
  float getGoalSpeed(){
    return this.goalSpeed;
  }
  
  void setGoalLoad(float torquelm){
    this.goalLoad = torquelm;
  }
  
  float getGoalLoad(){
    return this.goalLoad;
  }
  
  float[] getGoalValues(){
    float[] result = {this.goalLength,this.goalSpeed,this.goalLoad};
    return result;
  }

  float getZeroOffset(){
    return this.zeroOffset;
  }

  void setZeroOffset(float offset){
    this.zeroOffset = offset;
  }
  
  color getColor(){
    return this.c;
  }
  
  void drawInfo(){
    float onScreenX = screenX(this.coordinates.x,this.coordinates.y,0);
    float onScreenY = screenY(this.coordinates.x,this.coordinates.y,0);
    pushMatrix();
    textAlign(LEFT);
    fill(this.c,100,100);
    textSize(TEXT_SIZE/2);
    camera();
    text(" winch " + this.id + " lenght "+ int(this.presentLength) + " speed " + 
    int(this.presentSpeed) + " load " + int(this.presentLoad), 0, height - this.id*TEXT_SIZE);
    fill(this.c,150,150);
    text(int(this.coordinates.x) + " " + int(this.coordinates.y),onScreenX,onScreenY);
    popMatrix();
  }
  
  void draw3d(){
    stroke(this.c,100,100);
    strokeWeight(3);
    line(this.coordinates.x, this.coordinates.y, this.coordinates.z, this.coordinates.x, this.coordinates.y, 0); 
  }
  
  void drawWinch(){
    this.drawInfo();
    this.draw3d();
  }
}
