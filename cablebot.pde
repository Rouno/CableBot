/* 
 CableBot related class, with methods for x y and z pod set & read positions 
 */

class CableBot {

  final float GRID_SIZE = 5000; //GRID_SIZE 1px = 1mm
  final float GRID_RES = 100; //resoltion in between grid lines in mm

  Winch[] winchList; //store each pillar's x y coordinates and height
  Pod pod;
  ReactShape footprint;

  /*
  *  constructor for simulation
   */
  CableBot(PVector[] coords) {
    this.pod = new Pod();
    this.winchList = new Winch[coords.length];
    for (int i=0; i<coords.length; i++) {
      this.winchList[i] = new Winch(i, coords[i]);
    }
    this.footprint = new ReactShape(this.getShape());
    this.setFootPrint();
    this.setPresentLengthFromPod();
  }

  CableBot(int nbWinch) {
    this.pod = new Pod(new PVector(0.0, 0.0, WINCH_PROTO_HEIGHT_mm));
    this.winchList = new Winch[nbWinch];
    for (int i=0; i<nbWinch; i++) {
      this.winchList[i] = new Winch(i, 0);
    }
    this.footprint = new ReactShape(nbWinch);
  }

  /*
  *  Load & Save cablebot model, file used "data/cableBotModel.json"
   */

  void saveModel() {
    JSONArray model;
    model = new JSONArray();
    for (int i=0; i<this.winchList.length; i++) {
      JSONObject winchdata;
      PVector coordinates = this.winchList[i].getCoordinates();
      winchdata = new JSONObject();
      winchdata.setInt("id", i);
      winchdata.setString("x", str(coordinates.x));
      winchdata.setString("y", str(coordinates.y));
      winchdata.setString("z", str(coordinates.z));
      model.setJSONObject(i, winchdata);
    }
    saveJSONArray(model, "data/cableBotModel.json");
    println("cableBot model saved");
  }

  void loadModel() {
    try {
      JSONArray model;
      model = loadJSONArray("data/cableBotModel.json");
      for (int i = 0; i < model.size(); i++) {
        JSONObject winchdata = model.getJSONObject(i);
        int id = winchdata.getInt("id");
        float x = float(winchdata.getString("x"));
        float y = float(winchdata.getString("y"));
        float z = float(winchdata.getString("z"));
        PVector coordinates = new PVector(x, y, z);
        this.winchList[id].setCoordinates(coordinates);
      }
      println("cableBot model loaded");
    } 
    catch(Exception e) {
      println("cablebot model loading failed");
    }
  }

  /*
  *  Build and receive frame for serial updates
   */
  //TODO : write winch method to convert metric data into word data
  String buildFrame() { 
    String result = "";// = new int[SerialBridge.num_data];
    int nWinch = this.winchList.length;
    for (int i =0; i<nWinch; i++) {
      result += int( (this.winchList[i].getGoalLength() + this.winchList[i].getZeroOffset() - MOUNT_LENGTH_in_mm)/POSITION_TO_FLOAT_mm );
      result += ',';
      result += int(this.winchList[i].getGoalSpeed());
      result += ',';
      result += int(this.winchList[i].getGoalLoad());
      result += ',';
    }
    return result;
  }

  //TODO : write winch methods to convert word values in metric values
  void receiveFrame(String[] tokens) {
    for (int i = 0; i<this.winchList.length; i++) {
      float len = float(tokens[i*SerialBridge.nb_data_per_actuator]) * POSITION_TO_FLOAT_mm - this.winchList[i].getZeroOffset() + MOUNT_LENGTH_in_mm;
      float speed = float(tokens[i*SerialBridge.nb_data_per_actuator+1]);
      float load = float(tokens[i*SerialBridge.nb_data_per_actuator+2]);
      this.winchList[i].setPresentValues(len, speed, load);
    }
  } 


  /*
  *  update pod present coordinates from new winch lengths reading 
   *  &
   *  updates goal winch length from new pod goal coordinates
   */

  void setPresentPodFromWinchLengths() {
    if (this.winchList.length<3) {
      println("number of winches needs to be at least 3");
      this.pod.setPresentCoordinates(new PVector(0, 0, 0));
    } else {
      float[] lengths = new float[this.winchList.length];
      PVector[] poses = new PVector[this.winchList.length];
      for (int i=0; i<this.winchList.length; i++) {
        lengths[i] = this.winchList[i].getPresentLength();
        poses[i] = this.winchList[i].getCoordinates();
      }
      this.pod.setPresentCoordinates(trilateration(lengths, poses).coordinates);
    }
  }

  PVector getPodFromLengths(float[] lengths) {
    PVector[] poses = copyWinchCoords();
    return trilateration(lengths, poses).coordinates;
  }

  //used only for simulation, use presentPod to compute
  void setPresentLengthFromPod() { 
    for (Winch w : this.winchList) {
      float Li = w.getCoordinates().dist(this.pod.getPresentCoordinates());
      w.setPresentLength(Li);
    }
  }

  //used only when no serial available
  void setPresentLengthFromLengths(Winch[] wincharray) { 
    for (int i=0; i<wincharray.length; i++) {
      this.winchList[i].setPresentLength(wincharray[i].getPresentLength());
    }
  }

  void setGoalLengthFromPod() {
    for (Winch w : this.winchList) {
      float Li = w.getCoordinates().dist(this.pod.getGoalCoordinates());
      w.setGoalLength(Li);
    }
  }

  float[] getGoalLengths() {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      result[i]=this.winchList[i].getGoalLength();
    }
    return result;
  }

  float[] getGoalLoads() {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<result.length; i++) {
      result[i] = this.winchList[i].getGoalLoad();
    }
    return result;
  }

  float[] getPresentLoads() {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<result.length; i++) {
      result[i] = this.winchList[i].getPresentLoad();
    }
    return result;
  }

  float [] getLengthsFromPoint(PVector point) {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<result.length; i++) {
      result[i]=point.dist(this.winchList[i].getCoordinates());
    }
    return result;
  }

  float[] getPresentLengths() {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      result[i]=this.winchList[i].getPresentLength();
    }
    return result;
  }

  PVector[] copyWinchCoords() {
    PVector[] result = new PVector[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      result[i] = new PVector();
      result[i]=this.winchList[i].getCoordinates().copy();
    }
    return result;
  }


  /*
  *  UI functions
   */

  void grabPod() {
    this.pod.grabber.setGrab(true);
  }

  void releaseGrab() {
    this.pod.grabber.setGrab(false);
  }

  boolean isGrabbed() {
    boolean result = this.pod.grabber.getGrab();
    return result;
  }

  boolean isPointOverGrabber(PVector point) {
    return this.pod.grabber.isPointOverButton(point);
  }


  /*
  *  Actuator driving functions
   */
  void setGoalLoad(float load){
    for (Winch w : this.winchList) {
      w.setGoalLoad(load);
    }
  }

  void setCompliantActuators() {
    int loadoffset = 50;
    for (Winch w : this.winchList) {
      if (w.getGoalLoad()<w.getPresentLoad()) {
        float newTorque = max(w.getGoalLoad()-loadoffset, 0);
        w.setGoalLoad(newTorque);
      } else {
        float newTorque = min(w.getGoalLoad()+loadoffset, MIN_WINCH_LOAD);
        w.setGoalLoad(newTorque);
      }
    }
  }

  void driveActuatorByLoad() {
    float load = this.pod.getGoalCoordinates().z/WINCH_PROTO_HEIGHT_mm * MAX_WINCH_LOAD;

    for (Winch w : this.winchList) {
      PVector goalPodxy = this.pod.getGoalCoordinates().copy();
      goalPodxy.z = 0;
      PVector winchxy = w.getCoordinates().copy();
      winchxy.z = 0;
      w.setGoalLoad(min(load * winchxy.mag() / (2*goalPodxy.dist(winchxy)), MAX_WINCH_LOAD));
    }
  }

  void movePodToGoalPosition() {
    PVector displacement = new PVector();
    displacement = this.pod.getGoalCoordinates().sub(this.pod.getPresentCoordinates());
    displacement.setMag(this.pod.getGoalSpeed());
    displacement.add(this.pod.getPresentCoordinates());
    float[] newLengths= this.getLengthsFromPoint(this.pod.getGoalCoordinates());
    for (int i = 0; i<this.winchList.length; i++) {
      this.winchList[i].setGoalLength(newLengths[i]);
    }
  }

  void movePodToGoalSpeed() {
    for(int i = 0;i<this.winchList.length; i++) {
      this.winchList[i].setGoalSpeed(getCableVariations()[i]*MAX_WINCH_SPEED);
      this.winchList[i].setGoalLength(this.pod.getGoalCoordinates().dist(this.winchList[i].getCoordinates()));
    }
  }

  float[] getCableVariations() {
    float[] result = new float[this.winchList.length];
    PVector goalDirection = this.pod.getGoalCoordinates().sub(this.pod.getPresentCoordinates());
    if (goalDirection.mag()!=0) {
      goalDirection.div(goalDirection.mag());
      PVector d_goalDirection = this.pod.getPresentCoordinates().add(goalDirection);
      float[] currentCableLength = getPresentLengths();
      float[] d_targetCableLength = getLengthsFromPoint(d_goalDirection);
      float maxCableVariation = 0;

      for (int i = 0; i<result.length; i++) {
        result[i] = d_targetCableLength[i] - currentCableLength[i];
        if (abs(result[i]) > maxCableVariation) maxCableVariation = abs(result[i]);
      }
      for (int i = 0; i<result.length; i++) {
        if (maxCableVariation!= 0) result[i] /= maxCableVariation;
      }
    } else {
      for (int i = 0; i<result.length; i++) result[i]=0;
    }
    return result;
  }

  /*
  *  drawing functions
   */

  void drawGrid() {
    stroke(50);
    strokeWeight(1);
    for (int i=-(int)GRID_SIZE/2; i<(int)GRID_SIZE/2; i+=GRID_RES) {
      line(i, GRID_SIZE/2, i, -GRID_SIZE/2);
      line(GRID_SIZE/2, i, -GRID_SIZE/2, i);
    }
  }

  void drawCursorAxis() {
    strokeWeight(1);
    stroke(255, 0, 255);
    PVector[] podXYbound = this.footprint.getUpDownLeftRightbounds(this.pod.getGoalCoordinates());
    line(podXYbound[0].x, podXYbound[0].y, podXYbound[2].x, podXYbound[2].y);
    line(podXYbound[1].x, podXYbound[1].y, podXYbound[3].x, podXYbound[3].y);
  }

  void drawCables() {
    for (Winch w : this.winchList) {
      stroke(w.c, 100, 100);
      PVector winchcoord = w.getCoordinates();
      PVector podcoord = this.pod.getPresentCoordinates();
      line(winchcoord.x, winchcoord.y, winchcoord.z, podcoord.x, podcoord.y, podcoord.z);
    }
  }

  void setFootPrint() {
    this.footprint = new ReactShape(this.getShape());
  }

  PVector[] getShape() {
    PVector[] shape = new PVector[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      shape[i] = this.winchList[i].getCoordinates();
    }
    return shape;
  }

  void drawWinches() {
    for (Winch w : this.winchList) {
      w.drawWinch();
    }
  }

  void drawBot() {
    this.drawWinches();
    this.drawGrid();
    this.pod.draw();
    this.drawCables();

    if (this.footprint != null) {
      this.footprint.drawShape();
      this.drawCursorAxis();
    }
  }
}
