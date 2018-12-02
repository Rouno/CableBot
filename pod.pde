/* Pod class store all attributes and function regarding the pod of 
cableBot which is the piece where all cable are connected
*/

class Pod{
  static final float MIN_POD_HEIGHT_mm = 150;
  private PVector presentCoordinates = new PVector(0,0,WINCH_PROTO_HEIGHT_mm);
  private PVector goalCoordinates = new PVector(0,0,WINCH_PROTO_HEIGHT_mm);
  private float presentSpeed;
  private float goalSpeed = 50;
  private color c = color(100,100,100);
  private int size = 80;
  private Button grabber = new Button(this.presentCoordinates.x,this.presentCoordinates.y,this.size);
  
  Pod(){
    
  }
  
  Pod(PVector coord){
    super();
    this.presentCoordinates = coord.copy();
    this.goalCoordinates = this.presentCoordinates.copy();
    this.grabber.coordinates.set(this.goalCoordinates.x,this.goalCoordinates.y);
  }
  
  void setPresentCoordinates(PVector coord){
    this.presentCoordinates = coord.copy();
  }
  
  PVector getPresentCoordinates(){
    return this.presentCoordinates.copy();
  }
  
  void setGoalCoordinates(PVector coord){
    this.goalCoordinates = coord.copy();
    this.goalCoordinates.y = coord.y;
    //if(this.goalCoordinates.z >WINCH_PROTO_HEIGHT_mm)this.goalCoordinates.z = WINCH_PROTO_HEIGHT_mm;
    this.grabber.coordinates.set(coord.x,coord.y);
  }
  
  void offsetGoalZ(int offset){
    this.goalCoordinates.z = max(this.goalCoordinates.z+offset,0);
  }
  
  PVector getGoalCoordinates(){
    return this.goalCoordinates.copy();
  }
  
  float getGoalSpeed(){
    return this.goalSpeed;
  }
  
  void draw(){
    stroke(50,0,150);
    noFill();
    //draw presenst pod coordinates
    translate(0, 0, this.presentCoordinates.z);
    ellipse(this.presentCoordinates.x, this.presentCoordinates.y, this.size, this.size);
    translate(0, 0, -this.presentCoordinates.z);
    
    //draw goal pod coordinates
    stroke(0,125,150);
    translate(0, 0, this.goalCoordinates.z);
    ellipse(this.goalCoordinates.x, this.goalCoordinates.y, this.size, this.size);
    translate(0, 0, -this.goalCoordinates.z);
    
    //draw grabber
    this.grabber.drawButton();
  }
  
}

class Button{
  PVector coordinates;
  int size;  //button size
  color rectColor, rectHighlight;  //color states
  boolean grabbed = false;
  
  Button(float x, float y, int tmp_size) {
    this.coordinates = new PVector(x,y);
    this.size = tmp_size;
    this.rectColor = 100;
    this.rectHighlight = 150;
  }
  void setGrab(boolean isGrabbed){
    this.grabbed = isGrabbed;
  }
  
  boolean getGrab(){
    return this.grabbed;
  }
  
  boolean isPointOverButton(PVector point){
    if(point.dist(this.coordinates)<this.size){  
      return true;
    }else{
      return false;
    }
  }
  
  void drawButton(){
    stroke(150);
    fill(50,0,150);
    ellipse(this.coordinates.x, this.coordinates.y, this.size, this.size);
  }


}
