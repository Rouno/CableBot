//class made of any custom shape to know if a point is inside the shape
class ReactShape {
  PShape customShape;
  color c = color(0,0,100);
  PGraphics pg; //create offscreen buffer to test if a point is within shape

  ReactShape(PVector[] vertices) {
    this.pg = createGraphics(int(2 * maxWidth(vertices)), int(2 * maxHeight(vertices)));
    println("footprint maxwidth: "+maxWidth(vertices)+ " footprint maxheight: "+maxHeight(vertices));
    this.pg.colorMode(HSB,100);
    this.setShape(vertices); 
  }
  
  ReactShape(int nbVertice){
    PVector[] vertices = new PVector[nbVertice];
    for (int i=0; i<nbVertice; i++) {
      vertices[i] = new PVector();
      vertices[i] = PVector.fromAngle(-TWO_PI * i/nbVertice); //spread coordinates clockwise
      vertices[i].setMag(MOUNT_LENGTH_in_mm);
    }
    this.pg = createGraphics(int(2 * maxWidth(vertices)), int(2 * maxHeight(vertices)));
    println("footprint maxwidth: "+maxWidth(vertices)+ " footprint maxheight: "+maxHeight(vertices));
    this.pg.colorMode(HSB,100);
    this.setShape(vertices); 
  }

  void setShape(PVector[] shapeVertices){
    this.customShape = createShape();
    this.customShape.beginShape();
    for (PVector vect : shapeVertices) {
      this.customShape.vertex(vect.x, vect.y);
    }
    this.customShape.stroke(c);
    this.customShape.endShape(CLOSE);
    
    this.pg.beginDraw();
    this.pg.background(0);
    this.customShape.setFill(c);
    this.pg.shape(this.customShape, pg.width/2, pg.height/2);
    this.pg.endDraw();
  }

  //return true if point over shape
  boolean isOverFootprint(PVector point) {
    boolean result;
    if (this.pg.get(int(point.x + pg.width/2), int(point.y+pg.height/2)) == c) { //using color(O,O,100) in HSB 
      result = true;
    } else {
      result = false;
    }
    return result;
  }

  //return the closest coordinates inside shape from point using dicotomy, origin must be inside shape
  PVector getClosestPointInsideShape(PVector point) {
    PVector pt = point.copy();
    if (this.isOverFootprint(pt)) return pt;
    float dicotomy_mag = pt.mag()/2;
    while (dicotomy_mag >= 1) {  //while pixel diff between point and result > 1 pixel
      if (this.isOverFootprint(pt)) {
        pt.setMag(pt.mag()+dicotomy_mag);
      } else {
        pt.setMag(pt.mag()-dicotomy_mag);
      }
      dicotomy_mag /= 2;
    }
    return pt;
  }

  PVector[] getUpDownLeftRightbounds(PVector point) {
    PVector[] result=new PVector[4]; //4 vectors : 4 boundaries along +x +y -x -y
    PVector unitVector = new PVector(1, 0);
    for (int i=0; i<4; i++) {
      float dicotomy_mag = max(pg.width/2, pg.height/2);
      result[i] = point.copy();
      while (dicotomy_mag > 1) {  //while pixel diff between point and result > 1 pixel
        if (this.isOverFootprint(result[i])) {
          result[i].add(unitVector.copy().mult(dicotomy_mag));
        } else {
          result[i].sub(unitVector.copy().mult(dicotomy_mag));
        }
        dicotomy_mag /= 2;
      }
      unitVector.rotate(HALF_PI);
    }
    return result;
  }
  
  void drawShape(){
    this.customShape.setFill(0);
    this.customShape.setStroke(c);
    shape(this.customShape);
  }
}

class trilaterationContainer{
  PVector coordinates;
  float error;
  trilaterationContainer(PVector avector,float afloat){
    this.coordinates = avector;
    this.error = afloat;
  }
}

//Compute pod 3d position from cableLengthData measurements and, takes only 3 points and 3 distances
//from https://en.wikipedia.org/wiki/Trilateration
//TODO : write code to manage 1 point 1 distance and 2 points 2 distances
trilaterationContainer trilateration(float[] radius, PVector[] center) {
  float error=0;
  float r1 = radius[0];
  float r2 = radius[1];
  float r3 = radius[2];
  PVector P1, P2, P3, P1P2, P1P3 = new PVector();

  P1 = center[0].copy();
  P2 = center[1].copy();
  P3 = center[2].copy();
  P1P2 = P2.copy().sub(P1);
  P1P3 = P3.copy().sub(P1);

  //Compute unit vectors derived from first 3 first pillars coordinates
  PVector Ex = P1P2.copy().div(P1P2.mag());
  float i = Ex.copy().dot(P3.copy().sub(P1));
  PVector Ey = P1P3.copy().sub(Ex.copy().mult(i)).div(P1P3.copy().sub(Ex.copy().mult(i)).mag());
  PVector Ez = Ex.copy().cross(Ey);

  float d = P1P2.mag();
  float j = Ey.copy().dot(P3.copy().sub(P1));

  float x = (r1*r1 - r2*r2 + d*d)/(2*d);
  float y = (r1*r1 - r3*r3 + i*i + j*j)/(2*j) - i/j*x;

  PVector result = new PVector();
  float z = 0;
  if (r1*r1 - x*x - y*y >=0 ) {
    z =  sqrt(r1*r1 - x*x - y*y);
    result = P1.add(Ex.mult(x)).add(Ey.mult(y)).add(Ez.mult(z)); //there may be an error on z.Ez sign !!!!!!!!!!!!!
  } else {
    //if 3 spheres intersection has no solution, grab closest solution following Al Kashi theoreme 
    //http://kuartin.math.pagesperso-orange.fr/theoremealkashi.htm
    //error from closest solution is stored in error
    error = sqrt(-r1*r1 + x*x + y*y);
    float cosalpha = (sq(d)-sq(r2)+sq(r1))/(2*r1*d);
    cosalpha = max(min(cosalpha, 1), -1); //ensure unique cosalpha solution by caping cosalpha between -1 and 1, appears when r1 & r2 cableLengthData cross each others
    float sinalpha = sqrt(1 - sq(cosalpha));
    result = P1.add(Ex.mult(cosalpha*r1).add(Ey.mult(sinalpha*r1)));
  }
  trilaterationContainer container = new trilaterationContainer(result, error);
  return container;
}

//returns mean vector of a PVector array
static PVector pvectorMean(PVector[] vector_array) {
  PVector result = new PVector(0, 0, 0);
  for (PVector vect : vector_array) {
    result = result.add(vect);
  }
  result = result.div(vector_array.length);
  return result;
}

//center vector array aouround x & y mean vector
//TODO : should be part of cablebot class
static void centerVectorArray(PVector[] vector_array) {
  PVector mean = pvectorMean(vector_array);
  mean.z=0;
  for (PVector vect : vector_array) {
    vect.sub(mean);
  }
}

//align and center a set of PVector according to the first edge along x axis
//TODO : should be part of cablebot class
static void alignAccordingToFstEdge(PVector[] vector_to_align) {
  centerVectorArray(vector_to_align);
  PVector Ex = new PVector(1, 0);
  PVector axis_to_align = vector_to_align[1].copy().sub(vector_to_align[0]);
  float angle = PVector.angleBetween(axis_to_align, Ex) - PI;
  for (PVector vect : vector_to_align) {
    vect.rotate(angle);
  }
}

//returns max over z coordinates of PVector array
float maxZcoordinates(PVector[] vector_array) {
  float result=0; //assume z coordinates >= 0
  for (PVector vect : vector_array) {
    if (vect.z > result) result = vect.z;
  }
  return result;
}

//returns width of the centered rectangle bounding vector_array
float maxWidth(PVector[] vector_array) {
  float max;
  max=0;
  for (PVector vect : vector_array) {
    if (abs(vect.x) >= max) max = abs(vect.x);
  }
  return 2*max;
}

//returns height of the centered rectangle bounding vector_array
float maxHeight(PVector[] vector_array) {
  float max;
  max=0;
  for (PVector vect : vector_array) {
    if (abs(vect.y) >= max) max = abs(vect.y);
  }
  return 2*max;
}

static float maxFloatValue(float[] float_array) {
  float result=0;
  for (float afloat : float_array) {
    if (afloat>result) result=afloat;
  }
  return result;
}


//TODO test cycle functions
void cycleFloatArray(float[] anArray){
  float lastvalue = anArray[anArray.length-1];
  anArray = shorten(anArray);
  anArray = reverse(anArray);
  anArray = append(anArray,lastvalue);
  anArray = reverse(anArray);
}

void cyclePVectArray(PVector[] anArray){
  PVector lastvalue = anArray[anArray.length-1];
  anArray = (PVector[]) shorten(anArray);
  anArray = (PVector[]) reverse(anArray);
  anArray = (PVector[]) append(anArray,lastvalue);
  anArray = (PVector[]) reverse(anArray);
}
