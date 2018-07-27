/*** //<>//
Matthew Chun
Practicing how to make a spring using Fisica (In-Progress)
Current status: Have to be touching the blue circle and press the 'g' button to anchor to the spring.
Made on original Haply (circa Summer 2017)
Last modified: July 27, 2018
***/


/* library imports *****************************************************************************************************/ 

import processing.serial.*;
import com.dhchoi.CountdownTimer;
import com.dhchoi.CountdownTimerService;

/* Device block definitions ********************************************************************************************/
Device            haply_2DoF;
byte              deviceID                   = 5; //Unique Device ID needed by hAPI
Board             haply_board;
DeviceType        degreesOfFreedom;
boolean           rendering_force            = false; //flag to check if we should be rendering force out or not, otherwise just show graphics only


/* Simulation Speed Parameters ****************************************************************************************/
final long        SIMULATION_PERIOD          = 1; //ms
final long        HOUR_IN_MILLIS             = 36000000; //1 hour for haptic simulation timer, can change later if needed
CountdownTimer    haptic_timer;
float             dt                        = SIMULATION_PERIOD/1000.0; //maybe not used 


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                    = new PVector(0, 0);
PVector           torques                   = new PVector(0, 0);

/* task space */
PVector           pos_ee                    = new PVector(0, 0); //position of end effector for Haply
PVector           pos_ee_last               = new PVector(0, 0); //position of end effector for Haply (last known)
PVector           f_ee                      = new PVector(0, 0); //force for end effector for Haply

/* Graphic objects */
float pixelsPerCentimeter= 91.4; //this is the resolution of my screen divided by the number of centimeters  i.e. a 2560x1440 x 800px display with a 28 wide screen -> 91.4 pixels/cm
FWorld world; //world to set everything in

//desired world boundaries (can later visually see with black borders)
float worldWidth = 16.0;  
float worldHeight = 10.0; 

float edgeTopLeftX = 0.0; //presumably in CM as well
float edgeTopLeftY = 0.0;  //presumably in CM as well
float edgeBottomRightX = worldWidth; 
float edgeBottomRightY = worldHeight; 

//avatar parameters (haptics, graphics)
HVirtualCoupling avatarHaptics;
PImage avatarGraphics; 

//joint that can be toggled
FDistanceJoint jointBtoAvatar;
//for button toggling of avatar anchoring to the spring
boolean grabMode; 
boolean isTouching; //to check if avatar is touching the blue circle for potential "grabbing"
FCircle testCircleB; //target "grabbable" blue circle (end of spring)

void setup() { //one time setup for initial parameters
  
  size(1462, 914); // (worldWidth*pixelsPerCentimeter, worldHeight*pixelsPerCentimeter) must input as number
  frameRate(60); //60 and above for smooth performance in theory

  /* BOARD */
  haply_board = new Board(this, Serial.list()[0], 0); 
  /* DEVICE */
  haply_2DoF = new Device(degreesOfFreedom.HaplyTwoDOF, deviceID, haply_board);

  //world setup
  hAPI_Fisica.init(this); //create instance of Fisica to reference later
  hAPI_Fisica.setScale(pixelsPerCentimeter); //presumably sets Fisica scaling based on our desired screen dimensions
  world = new FWorld(); //create an instance of a Fisica world to place our Fisica virtual objects in
 //<>//
 
  //avatar code to play around with
  avatarHaptics = new HVirtualCoupling((1)); //constructor takes a "size" in float (size of circle in docs...) for reference?  
  avatarHaptics.h_avatar.setDensity(1.0); //referencing HVirtualCoupling object's (avatarHaptics) properity of density (total mass of body based on given float ...) might have to play in future
  //avatarHaptics.h_avatar.setFill(255,0,0); //sets color of avatar .... despite using a pre-loaded image, you can comment this out if you want, otherwise it is a white circle shown
  avatarHaptics.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+4); //initialize position of avatar in rough centre of window screen, the plus 4 is in cm, measured and yes it is 4 cm below window border
  avatarGraphics = loadImage("../img/Haply_avatar.png"); //replace avatar look with another image 
  avatarGraphics.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1))); //resize image to fit based on screen settings, based on scale factor I guess ... also if it's too big, Haply handle goes crazy for some reason
  avatarHaptics.h_avatar.attachImage(avatarGraphics); //do the replacement of the avatar look
  
  
  //trying to fix bottom left, shake -> vibration bug
  //println("Default damping setting: " + avatarHaptics.getVirtualCouplingDamping()); //700 by default? In dyne seconds per meter ...
  avatarHaptics.setVirtualCouplingDamping(3000.0); //to help prevent that "jerky" bug, might still be present in light of springs, play around with later
  //println("New damping setting: " + avatarHaptics.getVirtualCouplingDamping());
  
  //set world parameters like boundaries, basic physics
  world.setGravity((0.0), (200.0)); //1000 cm/(s^2), parameters are horizontial and vertical component of gravity ... 0,0 means no gravity, maybe directional gravitational forces??? 1000cm is more or less "normal" earth gravity falling?
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); //important for "black borders" actual bounds of interaction, important for subsequent position referencing in world terms? Going from top left corner (x,y) to bottom right (x,y)
  world.setEdgesRestitution(0.4); //restitution is apparently amount of energy turned to "heat" or work done to deform objects when they collide, set as a factor here? yup higher values makes the Haply and avatar "bounce" back on impact
  world.setEdgesFriction(3.5); //edge friction as stated, but to be honest, hard to distinguish even "higher values" like 5.5 and example value of 0.5, sometimes if you "really grind" then maybe
  
  //other test virtual object initialization
  FCircle testCircleA = new FCircle(1.5); //sets diameter in real world cm
  //testCircleA.setPosition((1462/2), (914/2)); //set this circle in the "centre" of screen window (half point of width, height res) but also causes "jerky" movement due to object being off screen?
  testCircleA.setPosition(8, 2); //is based on CM not pixel positions, if you mess up and it "falls" out of world edges, then movement can be jerky
  testCircleA.setDensity(2); //or value of 0 is also "static", for mass
  testCircleA.setStatic(true); //static body means no physics on this object? Means it won't move, but you can still feel the edge of it, this is our "hanging point" so it should be static
  testCircleA.setFill(255,0,0); //rgb values for red
  //testCircleA.setDrawable(false); //actually still renders haptically, but no longer visible ... just invisible
  world.add(testCircleA); //add this red circle to the world
  
  //seems like you need "steps" in btw joints to make it more spring like, let's test that idea out
  //one "middle" guy graphic
  FCircle midCircle = new FCircle(1.5); //sets diameter in real world cm
  midCircle.setDensity(2); //or value of 0 is also "static", for mass
  midCircle.setPosition(8,5); //pretty much halfway on the window screen, but in the "middle" btw the red and blue circles
  midCircle.setFill(0,255,0); //green in rgb
  world.add(midCircle); //add this green circle to the world
  
  //adding second virtual object for the "wrecking ball" for test Circle A (testing FDistanceJoint)
  testCircleB = new FCircle(1.5); //sets diameter in real world cm
  testCircleB.setPosition(8,7); //is based on CM not pixel positions, if you mess up and it "falls" out of world edges, then movement can be jerky
  testCircleB.setDensity(8); //or value of 0 is also "static", for mass
  //testCircleB.setStatic(true); //was testing to see if this "hanging" blue ball could still move around, or prevent "snapping" when adding a joint to it
  testCircleB.setFill(0,0,255); //blue in rgb
  world.add(testCircleB); //add this blue circle to the world
  
  //FDistanceJoint testing - IN-PROGRESS
  FDistanceJoint jointAtoMid = new FDistanceJoint(testCircleA, midCircle); //sets up a joint between these virtual objects, auto sets anchor of joints to centre of bodies
  jointAtoMid.setFrequency(10); //controls "bounce", 100 was the default but was way too much -> might be diff when using more than one joint
  jointAtoMid.setDamping(2); //"slow down" rate
  //println("Starting Anchor 1 Coordinates: " + jointAtoMid.getAnchor1X() + "," + jointAtoMid.getAnchor1Y());
  //println("Starting Anchor 2 Coordinates: " + jointAtoMid.getAnchor2X() + "," + jointAtoMid.getAnchor2Y());
  //jointAtoMid.setLength(4); //set defined distance in theory btw joint bodies
  jointAtoMid.calculateLength(); //sets the current distance btw joints, for "spring stretch" perhaps?
  world.add(jointAtoMid); //add this first joint to the world
  
  FDistanceJoint jointMidtoB = new FDistanceJoint (midCircle, testCircleB); //sets up a joint between these virtual objects, auto sets anchor of joints to centre of bodies
  jointMidtoB.setFrequency(10); //controls "bounce", 100 was the default but was way too much -> might be diff when using more than one joint
  jointMidtoB.setDamping(2); //"slow down" rate
  jointMidtoB.calculateLength(); //sets the current distance btw joints, for "spring stretch" perhaps?
  //jointMidtoB.setLength(4); //set defined distance in theory btw joint bodies, but doesn't seem to work since joints tend to "snap" towards each other
  world.add(jointMidtoB); //add this second joint to the world
  
  //avatar anchored to spring simple example
  jointBtoAvatar = new FDistanceJoint(testCircleB,avatarHaptics.h_avatar); //make a joint between the blue circle (end of spring) and our avatar
  jointBtoAvatar.setFrequency(7); //or maybe this value to prevent jerkiness, but more about "weight"
  jointBtoAvatar.setDamping(8); //play around here to try to prevent jerky on contact with blue circle, higher seems safer, "rate of return"
  jointBtoAvatar.calculateLength(); //sets the current distance btw joints, for "spring stretch" perhaps?
  //jointBtoAvatar.setLength(6);
  //world.add(jointBtoAvatar); //add this joint btw the avatar and the blue circle to the world
  
  //for button toggling
  grabMode = false;
  //for detecting if avatar is touching blue circle (spring end) at this point
  isTouching = false; 
  
  //start graphics and haptics sim loops
  world.draw(); //render the world and initialized objects defined above on the screen
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start(); //start the haptic simulation loop
  
   
}

void draw() {
  background(255); //used to "flush" past graphics on update, otherwise you get "inception" effect!
  
  //get current status of whether avatar and blue circle (end of spring) are "touching" or not
  isTouching = avatarHaptics.h_avatar.isTouchingBody(testCircleB);
  println("Is avatar touching blue circle?: " + isTouching);
  //println(frameRate); //show current framerate, but for some reason, seems to prevent keypress detection? 
  //debugging vibration bug
  //println("Current damping setting: " + avatarHaptics.getVirtualCouplingDamping());
   
  if(!rendering_force){ //why would we need this? Seems safe without it
    
    //s.drawContactVectors(this); 
    
   }
    world.draw(); //update the world graphics on the screen
  
}

//button toggling to allow for avatar anchoring on/off to spring
void keyPressed(){
  if(key == 'g' || key == 'G'){
    if(grabMode == false){
      if(isTouching == true){
        println("Grab mode curently ON");
        world.add(jointBtoAvatar);
      }
      grabMode = true;
    
    }else{
      world.remove(jointBtoAvatar);
      println("Grab mode curently OFF");
      grabMode = false;
    }
  }
}

/**********************************************************************************************************************
 * Haptics simulation event, engages state of physical mechanism, calculates and updates physics simulation conditions
 **********************************************************************************************************************/ 

void onTickEvent(CountdownTimer t, long timeLeftUntilFinish){
  
  rendering_force = true; //initiate haptics (flag) in theory
  
  //if there happens to be new information to play with
  if (haply_board.data_available()) {
    /* GET END-EFFECTOR STATE (TASK SPACE) */
        
    angles.set(haply_2DoF.get_device_angles()); 
    pos_ee.set( haply_2DoF.get_device_position(angles.array())); //print out?
    pos_ee.set(pos_ee.copy().mult(100)); //since position of end effector was an array
    
  }
  
  //based on current board info presumably, update avatar information such as current position and if any force should be rendered 
  avatarHaptics.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+1.0, edgeTopLeftY+(pos_ee).y); 
  avatarHaptics.updateCouplingForce();
 
  f_ee.set(-avatarHaptics.getVCforceX(), avatarHaptics.getVCforceY()); //I think this is "opposite" force that is "reflected" back on contact
 
  f_ee.div(10000); //
  haply_2DoF.set_device_torques(f_ee.array());
  torques.set(haply_2DoF.mechanisms.get_torque());
  haply_2DoF.device_write_torques();
  
  
  world.step(1.0f/1000.0f);
  
  rendering_force = false;
}


/* Timer control event functions **************************************************************************************/

/**
 * haptic timer reset
 */
void onFinishEvent(CountdownTimer t){
  println("Resetting timer...");
  haptic_timer.reset();
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
}