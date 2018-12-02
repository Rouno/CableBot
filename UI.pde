static class UI {
  static final float PODSHIFTMAG = 1;
  static State state = State.COMPLIANT;
  static String stateName = "COMPLIANT";

  static boolean[] arrowKeyTable = {false, false, false, false, false, false};

  static void updateUserInputs(CameraControlManager mycam, CableBot mybot, boolean isMousePressed) {
    mycam.updateMouse();
    if (isMousePressed) {
      if (mybot.isGrabbed()) {
        PVector mouseCoordInsideShape = mybot.footprint.getClosestPointInsideShape(mycam.mouseOnGroundPlane);
        PVector newgoal = new PVector(mouseCoordInsideShape.x, mouseCoordInsideShape.y, mybot.pod.getGoalCoordinates().z);
        mybot.pod.setGoalCoordinates(newgoal);
      } else {
        mycam.updateOrbitAngle();
      }
    }
    mycam.updateCamera();
    PVector podShift = new PVector((arrowKeyTable[3]?1:0)*PODSHIFTMAG-(arrowKeyTable[2]?1:0)*PODSHIFTMAG, 
      (arrowKeyTable[1]?1:0)*PODSHIFTMAG-(arrowKeyTable[0]?1:0)*PODSHIFTMAG, (arrowKeyTable[5]?1:0)*PODSHIFTMAG-(arrowKeyTable[4]?1:0)*PODSHIFTMAG);

    PVector newGoalPod = mybot.pod.getGoalCoordinates().add(podShift);
    if (newGoalPod.z<0)newGoalPod.z=0;
    newGoalPod = mybot.footprint.getClosestPointInsideShape(newGoalPod);
    mybot.pod.setGoalCoordinates(newGoalPod);
  }

  static void updateBotOutput(CableBot mybot, Calibrator calibrator) {
    switch (UI.state) {
    case COMPLIANT :
      UI.stateName = "COMPLIANT";
      mybot.setCompliantActuators();
      mybot.setPresentPodFromWinchLengths();
      calibrator.spreadWinchesCoord();
      break;
    case CALIBRATION :
      UI.stateName = "CALIBRATION";
      //mybot.setCompliantActuators();
      mybot.driveActuatorByLoad();
      mybot.setPresentPodFromWinchLengths();
      calibrator.addSample();
      calibrator.drawSamples();
      calibrator.drawCostValue();
      calibrator.optimize();
      break;
    case OPERATION :
      UI.stateName = "OPERATION";
      mybot.setPresentPodFromWinchLengths();
      mybot.movePodToGoalPosition();
      //mybot.movePodToGoalSpeed();
      break;
    }
  }

  static void mousePressedCallback(CameraControlManager mycam, CableBot mybot) {
    mycam.lastMouseClickedXY = mycam.mouseXY.copy();
    if (mybot.isPointOverGrabber(mycam.mouseOnGroundPlane)) {
      mybot.grabPod();
    }
  }

  static void mouseReleasedCallback(CameraControlManager mycam, CableBot mybot) {
    if (mybot.isGrabbed()) {
      mybot.pod.grabber.setGrab(false);
    } else {
      mycam.updateLastMouseReleased();
    }
  }

  static void mouseWheelCallback(CameraControlManager mycam, CableBot mybot, int wheelcount) {
    if (mybot.isPointOverGrabber(mycam.mouseOnGroundPlane)) {
      mybot.pod.offsetGoalZ(wheelcount);
    } else {
      mycam.orbitRadius += wheelcount;
    }
  }

  static void keyPressedCallback(CableBot mybot, Calibrator calibrator) {
    switch (UI.state) {
    case COMPLIANT :
      float minSampleDist = maxFloatValue(mybot.getPresentLengths())/3;
      calibrator.setMinSampleDistance(minSampleDist);
      mybot.setFootPrint();
      UI.state = State.CALIBRATION;
      break;
    case CALIBRATION :
      mybot.setFootPrint();
      mybot.setGoalLoad(MAX_WINCH_LOAD);
      UI.state = State.OPERATION;
      break;
    case OPERATION :
      break;
    }
  }
}

void drawInfo(String str) {
  pushMatrix();
  textAlign(LEFT);
  fill(100, 100, 100);
  textSize(TEXT_SIZE/2);
  camera();
  text(str, 0, TEXT_SIZE);
  popMatrix();
}

class Plotter {
  color c;
  float[] values;
  float min, max;
  int x_offset = 0;
  int y_offset = 0;
  int hght;
  int wdth;

  Plotter(color col, int w, int h, float m, float M, int xoff, int yoff) {
    this.c = col;
    this.hght = h;
    this.wdth = w;
    this.min = m;
    this.max = M;
    this.x_offset = xoff;
    this.y_offset = yoff;
    this.values = new float[w];
    for (int i=0; i<w; i++) {
      this.values[i]=0;
    }
  }

  void addValue(float val) {
    this.values = shorten(this.values);
    this.values = reverse(this.values);
    this.values = append(this.values, val);
    this.values = reverse(this.values);
  }

  void plot() {
    pushMatrix();
    stroke(this.c, 100, 100);
    camera();
    for (int i=0; i<this.wdth; i++) {
      float j = map(this.values[i], this.min, this.max, this.hght, 0);
      point(i+x_offset, int(j)+y_offset);
    }
    float j = map(this.values[0], this.min, this.max, this.hght, 0);
    fill(this.c, 100, 100);
    textAlign(LEFT);
    text(int(this.values[0]), x_offset, int(j)+y_offset);
    popMatrix();
  }
}

class BotPlotter {
  Plotter[] plotterList;

  BotPlotter(CableBot mybot, int minval, int maxval) {
    this.plotterList = new Plotter[mybot.winchList.length];
    for (int i=0; i<this.plotterList.length; i++) {
      this.plotterList[i] = new Plotter(mybot.winchList[i].getColor(), width/4, height/24, minval, maxval, 3*width/4, height/24);
    }
  }

  void addValues(float[] values) {
    for (int i = 0; i<this.plotterList.length; i++) {
      this.plotterList[i].addValue(values[i]);
    }
  }

  void plot() {
    for (int i = 0; i<this.plotterList.length; i++) {
      this.plotterList[i].plot();
    }
  }
}
