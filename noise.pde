// based on:
// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com
// Flow Field Following
// Via Reynolds: http://www.red3d.com/cwr/steer/FlowFollow.htm

int gridW = 12; //how many units in one direction of the grid?
float gridL;
float gridT;
float gridR;
float gridB;
int x = 26; // base unit of grid
float originX;
float originY;


FlowField flowfield;

// An ArrayList of vehicles
ArrayList<Vehicle> vehicles;
float angle=0;

void setup() {
  size(1000, 400);
  //fullScreen();
  frameRate(30);
  ellipseMode(CENTER);

  originX = width/2;
  originY = height/2;
  gridL = originX-((gridW*x)/2);
  gridT = originY-((gridW*x)/2);
  gridR = originX+((gridW*x)/2);
  gridB = originY+((gridW*x)/2);

  // Make a new flow field, specify width and height of grid cell
  //lower #, denser flow field, more lag
  //higher #, less variety, less lag
  flowfield = new FlowField(20);
  vehicles = new ArrayList<Vehicle>();

  // Make a whole bunch of vehicles with random maxspeed and maxforce values
  //Make a 2D array of vehicles
  for (int i = 0; i <gridW-1; i++) {
    for (int j = 0; j <gridW-1; j++) {
      //vehicles.add(new Vehicle(new PVector(random(width), random(height)), random(2, 5), random(0.1, 0.5)));
      vehicles.add(new Vehicle(new PVector(gridL+((i+1)*x), gridT+((j+1)*x)), random(2, 5), random(0.1, 0.5)));
    }
  }

  background(0);
}

void draw() {

  //transparent box
  //blendMode(SUBTRACT);
  fill(0,.99); //lower fill, longer trails
  rect(0, 0, width, height);
  //blendMode(BLEND);
  

  flowfield.update();

  // Display the flowfield in "debug" mode
  //if (debug) flowfield.display();
  // Tell all the vehicles to follow the flow field
  for (Vehicle v : vehicles) {
    v.follow(flowfield);
    v.run();
  }
}

class FlowField {

  // A flow field is a two dimensional array of PVectors
  PVector[][] field;
  int cols, rows; // Columns and Rows
  int resolution; // How large is each "cell" of the flow field

  float zoff = 0.0; // 3rd dimension of noise

  FlowField(int r) {
    resolution = r;
    // Determine the number of columns and rows based on sketch's width and height
    cols = width/resolution;
    rows = height/resolution;
    //cols = gridW;
    //rows = gridW;
    field = new PVector[cols][rows];
    update();
  }

  void update() {
    float xoff = 0;
    for (int i = 0; i < cols; i++) {
      float yoff = 0;
      for (int j = 0; j < rows; j++) {
        float theta = map(noise(xoff, yoff, zoff), 0, 1, 0, TWO_PI);
        // Make a vector from an angle
        field[i][j] = PVector.fromAngle(theta);
        yoff += 0.1;
      }
      xoff += 0.1;
    }
    zoff += 0.005;
    //println(zoff);
  }

  // Draw every vector
  void display() {
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        drawVector(field[i][j], i*resolution, j*resolution, resolution-2);
      }
    }
  }

  // Renders a vector object 'v' as an arrow and a location 'x,y'
  void drawVector(PVector v, float x, float y, float scayl) {
    pushMatrix();
    float arrowsize = 4;
    // Translate to location to render vector
    translate(x, y);
    stroke(150, 255);
    // Call vector heading function to get direction (note that pointing up is a heading of 0) and rotate
    rotate(v.heading2D());
    // Calculate length of vector & scale it to be bigger or smaller if necessary
    float len = v.mag()*scayl;
    // Draw three lines to make an arrow (draw pointing up since we've rotate to the proper direction)
    line(0, 0, len, 0);
    //line(len,0,len-arrowsize,+arrowsize/2);
    //line(len,0,len-arrowsize,-arrowsize/2);
    popMatrix();
  }

  PVector lookup(PVector lookup) {
    int column = int(constrain(lookup.x/resolution, 0, cols-1));
    int row = int(constrain(lookup.y/resolution, 0, rows-1));
    return field[column][row].get();
  }
}

class Vehicle {

  // The usual stuff
  PVector location;
  PVector velocity;
  PVector acceleration;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed

  Vehicle(PVector l, float ms, float mf) {
    location = l.get();
    r = 3.0;
    maxspeed = ms;
    maxforce = mf;
    acceleration = new PVector(0, 0);
    velocity = new PVector(0, 0);
  }

  public void run() {
    update();
    borders();
    display();
  }


  // Implementing Reynolds' flow field following algorithm
  // http://www.red3d.com/cwr/steer/FlowFollow.html

  void follow(FlowField flow) {
    // What is the vector at that spot in the flow field?
    PVector desired = flow.lookup(location);
    // Scale it up by maxspeed
    desired.mult(maxspeed);
    // Steering is desired minus velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    applyForce(steer);
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acceleration.add(force);
  }

  // Method to update location
  void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    location.add(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
  }

  void display() {
    fill(255);
    noStroke();
    pushMatrix();
    translate(location.x, location.y);
    ellipse(0, 0, 3, 3);
    popMatrix();
  }

  // Wraparound
  void borders() {
    //if (location.x < -r) location.x = width+r;
    //if (location.y < -r) location.y = height+r;
    //if (location.x > width+r) location.x = -r;
    //if (location.y > height+r) location.y = -r;

    if (location.x < gridL) location.x = gridR+r;
    if (location.y < gridT) location.y = gridB+r;
    if (location.x > gridR+r) location.x = gridL;
    if (location.y > gridB+r) location.y = gridT;
  }
}