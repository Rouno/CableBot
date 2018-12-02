/* 
 * Trying to implement a camera_control class
 */

class CameraControlManager{
  
  PGraphicsOpenGL gl;
  PMatrix3D S = new PMatrix3D(); //camera space to screen space matrix
  PMatrix3D P = new PMatrix3D(); //projection of model to camera
  PMatrix3D M = new PMatrix3D(); //projection of world to model
  PMatrix3D Si = new PMatrix3D(); //screen to camera
  PMatrix3D Pi = new PMatrix3D(); //camera to model
  PMatrix3D Mi = new PMatrix3D(); //model to world
  
  PVector mouseXY = new PVector(0,0);   //used to store mouse coordinate in 2D vector
  PVector lastMouseClickedXY = new PVector(0,0); //mouse coordinates when clicked for the last time 
  PVector lastMouseReleaseXY = new PVector(1300,500); //mouse coordinates when released for the last time
  PVector mouseOnGroundPlane = new PVector(0,0); //used to store 2D mouse projection on (x,y,0) plane

  PVector orbitAngle = new PVector(1300,500);
  float orbitRadius = 1500, lastOrbitRadius=1500;
  
  CameraControlManager(PGraphicsOpenGL openglObject){
    //get the graphics object which contains camera projection/model view
    this.gl = (PGraphicsOpenGL) openglObject;
    this.init_camera_matrices();
  }

  void init_camera_matrices(){
    //set the fixed camera to screen matrix
    this.S.set(width/2, 0, 0, width/2,
          0, -height/2, 0, height/2,
          0, 0, 0.5, 0.5,
          0, 0, 0, 1);
          
    //get inverse matrix
    this.Si.set(this.S);
    this.Si.invert();
    
    //set camera position
    camera(0, 0, width, 0, 0, 0, 0, 1, 0);
    
    //get current camera projection and inverse
    this.P.set(this.gl.projection);
    this.Pi.set(this.P);
    this.Pi.invert();
  }
  
  void updateCamera(){
    this.camera_orbit(this.orbitRadius, this.orbitAngle);
  }
  
  void updateMouse(){
    this.mouseXY.set(mouseX,mouseY); //store current mouse coordinates in a vector 
    this.mouseOnGroundPlane.set(this.worldCoords(this.mouseXY.x, this.mouseXY.y, 0)); //get 3D coordinates on ground plane which correspond to the 2D position of the mouse on the screen
  }
  
  void updateOrbitAngle(){
    this.orbitAngle = this.lastMouseReleaseXY.copy().add(mouseXY).sub(this.lastMouseClickedXY);
}
  
  void updateLastMouseReleased(){
    this.lastMouseReleaseXY.sub(this.lastMouseClickedXY).add(this.mouseXY);
  }
  
  //"lastMouseReleaseXY.x + mouseX - lastMouseClickedXYX" is used to preserve last cursor position while mouse was dragged 
  void camera_orbit(float my_orbitRadius,PVector my_orbitAngle){
    float xpos = cos(radians(my_orbitAngle.x - width/2)/10)*my_orbitRadius;
    float ypos = sin(radians(my_orbitAngle.x - width/2)/10)*my_orbitRadius;
    float zpos = my_orbitRadius / width * my_orbitAngle.y;
    camera(xpos, ypos, zpos, 0, 0, 0, 0, 0, -1);
  }
  
  boolean orbit_radius_has_changed(){
    return (this.orbitRadius != this.lastOrbitRadius);
  }
  
  
  //project 2d mouse x y on 3D point belonging on h z-plane
  float[] worldCoords(float x, float y, float h){
    //get the current model view matrix and inverse
    this.M.set(gl.modelview);
    this.Mi.set(this.M);
    this.Mi.invert();
    
    //inversion of 2D screen coordinates to 3D world coordinates with z-plane constraint
    PMatrix3D SPMi = new PMatrix3D();
    SPMi.set(this.Si);
    SPMi.preApply(this.Pi);
    SPMi.preApply(this.Mi);
    
    //solve for z needed to end up with world-z = h
    float z = ((SPMi.m20 - h*SPMi.m30)*x + (SPMi.m21 - h*SPMi.m31)*y + (SPMi.m23 - h*SPMi.m33)) / (h*SPMi.m32 - SPMi.m22);
    
    float[] mouse = {x, y, z, 1};
    float[] orig = new float[4];
    
    SPMi.mult(mouse, orig);
    
    for(int i = 0; i < 4; i++){
      orig[i] /= orig[3];
    }
    float[] result = {orig[0],orig[1],orig[2]};
    return result;
  }

}
