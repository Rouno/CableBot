/*
*  Calibrator class contains methods for gradient descent optimization of multiple cablebot winch length samples
*/

class Calibrator{
  float[][] sample;
  int nbSamples = 50;
  float minSampleDistance;
  static final float EPSILON = 0.001;
  static final float L_RATE = 0.05; //learning rate of gradient descent
  CableBot mybot;
  
  Calibrator(CableBot abot){
    mybot = abot;
    this.minSampleDistance = maxFloatValue(this.mybot.getPresentLengths())/3; //divider factor is purely heuristic
    sample = new float[0][this.mybot.winchList.length];
  }
  
  void spreadWinchesCoord(){
    int n = this.mybot.winchList.length;
    PVector[] winchesCoord = new PVector[n];
    for (int i=0; i<n; i++) {
      PVector newcoord = new PVector();
      newcoord = PVector.fromAngle(-TWO_PI * i/n); //spread coordinates clockwise
      newcoord.setMag(this.mybot.winchList[i].getPresentLength());
      newcoord.z=WINCH_PROTO_HEIGHT_mm;
      this.mybot.winchList[i].setCoordinates(newcoord);
      winchesCoord[i]=this.mybot.winchList[i].getCoordinates();
    }
    alignAccordingToFstEdge(winchesCoord); //align and center winches along first 2 vetices (x axis convention)
  }
  
  void addSample(){
    if(isNewSampleFarEnough()){
      float[] newsample = new float[mybot.winchList.length];
      arrayCopy(this.mybot.getPresentLengths(),newsample);
      
      if(this.sample.length >= this.nbSamples){ //if sample array is full, delete last sample
        this.sample = (float[][]) reverse(this.sample);
        this.sample = (float[][]) shorten(this.sample);
        this.sample = (float[][]) reverse(this.sample);
      }
      this.sample = (float[][]) append(this.sample,newsample);
    }
  }
  
  boolean isNewSampleFarEnough(){
    boolean result;
    if(this.sample.length == 0){
      result = true;
    }else{
      PVector lastPod = this.mybot.getPodFromLengths(this.sample[this.sample.length-1]);
      if(lastPod.dist(this.mybot.pod.getPresentCoordinates())>this.minSampleDistance){
        result = true;
      }else{
        result = false;
      }
    }
    return result;
  }
  
  float costFunction() {
    float cost=0;
    trilaterationContainer tril_cont;
    PVector[] winchcoords = this.mybot.copyWinchCoords();
    for (float[] one_sample : this.sample) {
      tril_cont = trilateration(one_sample, winchcoords);
      for(int i=0;i<winchcoords.length;i++){
        float predictedWinchLength = tril_cont.coordinates.copy().sub(winchcoords[i]).mag();
        cost+= sq(predictedWinchLength - one_sample[i]);
      }
      if(tril_cont.coordinates.z <0) cost+= sq(tril_cont.coordinates.z);
    }
    cost/=2 * this.mybot.winchList.length;
    return cost;
  }
  
  PVector[] costGradient(){
    float cost = this.costFunction();
    PVector[] gradient = new PVector[this.mybot.winchList.length]; //for each winch, we use a 3d PVector for winch coordinates and, TODO a 2D PVector for zero offset and radius offset
    for(int i=0;i<this.mybot.winchList.length;i++){
      float dfx, dfy, dfz;
      PVector offset = new PVector();
      
      offset.set(EPSILON,0,0);
      this.mybot.winchList[i].offsetCoordinates(offset);
      dfx = (this.costFunction()-cost)/EPSILON;
      
      offset.set(-EPSILON,EPSILON,0);
      this.mybot.winchList[i].offsetCoordinates(offset);
      dfy = (this.costFunction()-cost)/EPSILON;
      
      offset.set(0,-EPSILON,EPSILON);
      this.mybot.winchList[i].offsetCoordinates(offset);
      dfz = 0;//(this.costFunction()-cost)/EPSILON; winch z coordinate is fixed, doesn't need calibration
      
      offset.set(0,0,-EPSILON);
      this.mybot.winchList[i].offsetCoordinates(offset);
      gradient[i] = new PVector(dfx, dfy, dfz);
    }
    return gradient;
  }
  
  void optimize(){
    PVector[] gradient = this.costGradient();
    PVector[] winchesCoord = new PVector[gradient.length];
    for(int i=0;i<gradient.length;i++){
      gradient[i].mult(-L_RATE);
      if(i==0){
        gradient[i].x=0;
        gradient[i].y=0;
      }
      if(i==1)gradient[i].y=0;
      //println("gradient value "+gradient[i]);
      this.mybot.winchList[i].offsetCoordinates(gradient[i]);
      winchesCoord[i] = this.mybot.winchList[i].getCoordinates();
    }
    this.drawGradient(gradient);
    centerVectorArray(winchesCoord);
  }
  
  void setMinSampleDistance(float distance){
    this.minSampleDistance = distance;
  }
  
  void drawSamples(){
    strokeWeight(3);
    stroke(0,0,255);
    for(float[] s : this.sample){
      PVector pod = new PVector();
      pod = this.mybot.getPodFromLengths(s).copy();
      point(pod.x,pod.y,pod.z);
    }
  }
  
  void drawCostValue(){
    pushMatrix();
    textAlign(RIGHT);
    fill(30,100,100);
    textSize(TEXT_SIZE/2);
    camera();
    text("Calibration cost : " + this.costFunction(), width, TEXT_SIZE);
    popMatrix();
  }
  
  void drawGradient(PVector[] gradient){
    for(int i=0;i<gradient.length;i++){
      pushMatrix();
      PVector gradientOrigin = this.mybot.winchList[i].getCoordinates();
      translate(gradientOrigin.x,gradientOrigin.y);
      stroke(this.mybot.winchList[i].c,80,80);
      fill(this.mybot.winchList[i].c,80,80);
      line(0,0,-gradient[i].x,-gradient[i].y);
      translate(-gradient[i].x,-gradient[i].y);
      rotate(gradient[i].heading());
      triangle(-width/80,-width/160,0,width/60,width/80,width/160);
      popMatrix();
    }
  }
}
