CameraControlManager cameraControls;
CableBot myCableBot; //used to simulate bot when serial not available
CableBot simulatedBot;
Calibrator myCalibrator;
BotPlotter goalPlotter;
BotPlotter presentPlotter;

static final int NB_ACTUATORS = 4;
static final float POSITION_TO_FLOAT_mm = 0.075;
static final float WINCH_PROTO_HEIGHT_mm = 483;
static final float MOUNT_LENGTH_in_mm = 70;
static final float MAX_WINCH_LOAD = 1000;
static final float MIN_WINCH_LOAD = 200;
static final float MAX_WINCH_SPEED = 500;

PVector[] botCoords = {new PVector(1400, -1000, WINCH_PROTO_HEIGHT_mm), new PVector(-500, -1000, WINCH_PROTO_HEIGHT_mm), new PVector(-1600, 1000, WINCH_PROTO_HEIGHT_mm), new PVector(1200, 1000, WINCH_PROTO_HEIGHT_mm)};

static int TEXT_SIZE; //size of text used

enum State {
  COMPLIANT, CALIBRATION, OPERATION
};

void setup() {
  //setup() should always start with serial init to determine if we need to simulate cablebot or not
  try {
    SerialBridge.myPort = new Serial(this, "/dev/tty.usbmodem1411", 57600);
    SerialBridge.myPort.bufferUntil(SerialBridge.NEW_LINE);
    myCableBot = new CableBot(NB_ACTUATORS);
    SerialBridge.myPort.write('K');
  }
  catch (Exception e) {
    println("Serial port initialization failed, running simulation instead");
    simulatedBot = new CableBot(botCoords);
    alignAccordingToFstEdge(simulatedBot.getShape());
    myCableBot = new CableBot(simulatedBot.winchList.length);
    myCableBot.footprint = new ReactShape(simulatedBot.getShape());
    myCableBot.setPresentLengthFromLengths(simulatedBot.winchList);
  }

  myCalibrator = new Calibrator(myCableBot);

  size(1280, 720, P3D);
  //size(800, 600, P3D);
  TEXT_SIZE = height/20;
  rectMode(CENTER);
  colorMode(HSB, 100);
  cameraControls = new CameraControlManager((PGraphicsOpenGL) this.g);

  goalPlotter = new BotPlotter(myCableBot, 0, 1500);
  presentPlotter = new BotPlotter(myCableBot, 0, 1500);
}

void draw() {
  background(0);
  UI.updateUserInputs(cameraControls, myCableBot, mousePressed);
  UI.updateBotOutput(myCableBot, myCalibrator);
  myCableBot.drawBot();
  drawInfo(UI.stateName);

  goalPlotter.addValues(myCableBot.getGoalLengths());
  goalPlotter.plot();
  presentPlotter.addValues(myCableBot.getPresentLengths());
  presentPlotter.plot();

  if (simulatedBot!=null) {
    simulatedBot.pod.setPresentCoordinates(myCableBot.pod.getGoalCoordinates());
    simulatedBot.setPresentLengthFromPod();
    myCableBot.setPresentLengthFromLengths(simulatedBot.winchList);
  }
}

void serialEvent(Serial myPort) {
  SerialBridge.serialCallBack(myCableBot, myPort);
}

void mousePressed() {
  UI.mousePressedCallback(cameraControls, myCableBot);
}

void mouseReleased() {
  UI.mouseReleasedCallback(cameraControls, myCableBot);
}

void mouseWheel(MouseEvent event) {
  UI.mouseWheelCallback(cameraControls, myCableBot, event.getCount());
}

void keyPressed() {
  switch (key) {
  case 's' :
    myCableBot.saveModel();
    break;
  case 'l' : 
    myCableBot.loadModel();
    break;
  case CODED :
    switch (keyCode) {
    case UP:
      UI.arrowKeyTable[0] = true;
      break;
    case DOWN:
      UI.arrowKeyTable[1] = true;
      break;
    case LEFT:
      UI.arrowKeyTable[2] = true;
      break;
    case RIGHT:
      UI.arrowKeyTable[3] = true;
      break;
    case CONTROL:
      UI.arrowKeyTable[4] = true;
      break;
    case SHIFT:
      UI.arrowKeyTable[5] = true;
      break;
    }
    break;
  default :
    UI.keyPressedCallback(myCableBot, myCalibrator);
  }
}

void keyReleased() {
  if (key== CODED) {
    switch (keyCode) {
    case UP:
      UI.arrowKeyTable[0] = false;
      break;
    case DOWN:
      UI.arrowKeyTable[1] = false;
      break;
    case LEFT:
      UI.arrowKeyTable[2] = false;
      break;
    case RIGHT:
      UI.arrowKeyTable[3] = false;
      break;
    case CONTROL:
      UI.arrowKeyTable[4] = false;
      break;
    case SHIFT:
      UI.arrowKeyTable[5] = false;
      break;
    }
  }
}
