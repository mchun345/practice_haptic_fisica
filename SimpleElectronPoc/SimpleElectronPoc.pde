/*** //<>//
Matthew Chun
Practicing how to make a water analogy for electrical current 
Current status: Generation of certain "electrons" that move along a pipe like object. Quick and dirty example (fixed positions, assertionerror sometimes)
Made on original Haply (circa Summer 2017)
Last modified: Aug 1, 2018
***/


/* library imports *****************************************************************************************************/ //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>//

import processing.serial.*;
import com.dhchoi.CountdownTimer;
import com.dhchoi.CountdownTimerService;

/* Device block definitions ********************************************************************************************/
Device            haply_2DoF;
byte              deviceID                   = 5;
Board             haply_board;
DeviceType        degreesOfFreedom;
boolean           rendering_force                 = false;


/* Simulation Speed Parameters ****************************************************************************************/
final long        SIMULATION_PERIOD          = 1; //ms
final long        HOUR_IN_MILLIS             = 36000000;
CountdownTimer    haptic_timer;
float             dt                        = SIMULATION_PERIOD/1000.0; 


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                    = new PVector(0, 0);
PVector           torques                   = new PVector(0, 0);

/* task space */
PVector           pos_ee                    = new PVector(0, 0);
PVector           pos_ee_last               = new PVector(0, 0); 
PVector           f_ee                      = new PVector(0, 0); 

/* Graphic objects */
float pixelsPerCentimeter= 91.4; //this is the resolution of my screen divided by the number of centimeters  i.e. a 2560x1440 x 800px display with a 28 wide screen -> 91.4 pixels/cm
FWorld world; //world to set everything in

//desired world boundaries (can later visually see with black borders), maybe adjust for "pipe" like view
float worldWidth = 16.0;  
float worldHeight = 10.0; 
float edgeTopLeftX = 0.0; 
float edgeTopLeftY = 0.0; 
float edgeBottomRightX = worldWidth; 
float edgeBottomRightY = worldHeight; 

//avatar parameters (haptics, graphics)
HVirtualCoupling avatarHaptics;
PImage avatarGraphics; 

//our electrons
FCircle testCircleA; 
FCircle testCircleB;
FCircle testCircleC; 
FCircle testCircleD;
FCircle testCircleE;
FCircle testCircleF; 
FBlob testBlobA; 
 

void setup() {
  
  size(1462, 914); // (worldWidth*pixelsPerCentimeter, worldHeight*pixelsPerCentimeter) must input as number
  frameRate(60); //60 and above for smooth performance in theory
  

  /* BOARD */
  haply_board = new Board(this, Serial.list()[0], 0); 
  

  /* DEVICE */
  haply_2DoF = new Device(degreesOfFreedom.HaplyTwoDOF, deviceID, haply_board);

  //world initialize
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world = new FWorld(); //<>//

  //avatar code to play around with
  avatarHaptics = new HVirtualCoupling((1)); //constructor takes a "size" in float (size of circle in docs...) for reference?  
  avatarHaptics.h_avatar.setDensity(1.0); //referencing HVirtualCoupling object's (avatarHaptics) properity of density (total mass of body based on given float ...) might have to play in future
  //avatarHaptics.h_avatar.setFill(255,0,0); //sets color of avatar .... despite using a pre-loaded image, you can comment this out if you want, otherwise it is a white circle shown
  avatarHaptics.init(world, worldWidth/2, worldHeight/2); //initialize position of avatar in rough centre of window screen, the plus 4 is in cm, measured and yes it is 4 cm below window border
  avatarGraphics = loadImage("../img/Haply_avatar.png"); //replace avatar look with another image 
  avatarGraphics.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1))); //resize image to fit based on screen settings, based on scale factor I guess ... also if it's too big, Haply handle goes crazy for some reason
  avatarHaptics.h_avatar.attachImage(avatarGraphics); //do the replacement of the avatar look 
  avatarHaptics.setVirtualCouplingDamping(3000.0); //to help prevent that "jerky" bug
  
  //pipe walls
  FBox pipeTop = new FBox(14.5,2);
  pipeTop.setFill(154,161,173); //gray-ish colour
  //pipeTop.setPosition(hAPI_Fisica.screenToWorld(69), hAPI_Fisica.screenToWorld(69)); //done in pixels not CM
  //println(hAPI_Fisica.screenToWorld(68)); //seems "off" like way too small eg. 0.74 CM?
  pipeTop.setPosition(8,1.75); //doc says in pixels ... but it really seems like CM here, lots of trial/error
  pipeTop.setStatic(true);
  world.add(pipeTop);
  
  FBox pipeBottom = new FBox(14.5,2);
  pipeBottom.setFill(154,161,173);
  pipeBottom.setPosition(8, 8.25);
  pipeBottom.setStatic(true);
  world.add(pipeBottom);
  
  //testing circle objects to move across pipe
  testCircleA = new FCircle(1);
  testCircleA.setFill(0,0,255); //blue
  testCircleA.setPosition(1.5,3.75); //make sure to set the initial position to not be outside the world boundaries, otherwise will "fall off"
  testCircleA.setDensity(20);
  //testCircleA.setFriction(0);
  //testCircleA.setStatic(true);
  world.add(testCircleA);
  
  testCircleB = new FCircle(1); 
  testCircleB.setFill(0,0,255);
  testCircleB.setPosition(1.5, 5.0);
  testCircleB.setDensity(20); 
  //testCircleB.setStatic(true); 
  world.add(testCircleB); 
  
  testCircleC = new FCircle(1);
  testCircleC.setFill(0,0,255);
  testCircleC.setPosition(1.5, 5.75);
  testCircleC.setDensity(20); 
  world.add(testCircleC); 
  
  testCircleD = new FCircle(1);
  testCircleD.setFill(0,0,255);
  testCircleD.setPosition(2.5, 3.85);
  testCircleD.setDensity(20);
  //testCircleD.setStatic(true);
  world.add(testCircleD);
  
  testCircleE = new FCircle(1);
  testCircleE.setFill(0,0,255);
  testCircleE.setPosition(2.5, 5.15);
  testCircleE.setDensity(20);
  //testCircleE.setStatic(true);
  world.add(testCircleE); 
  
  testCircleF = new FCircle(1);
  testCircleF.setFill(0,0,255);
  testCircleF.setPosition(2.5, 6);
  testCircleF.setDensity(20);
  //testCircleF.setStatic(true);
  world.add(testCircleF); 
  
  testBlobA = new FBlob();
  testBlobA.setAsCircle(1.5, 5.75, 1);
  testBlobA.setFill(0,0,255); //blue
  //testBlobA.setFriction(0); //bounce?
  testBlobA.setDensity(4); 
  //world.add(testBlobA); 
  
  world.setGravity((0.0), (0.0)); //1000 cm/(s^2) //was (0.0), (200.0)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
 
  
  //start graphics and haptics sim loops
  world.draw();
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
  
}

void draw() {
  
  //for debugging, graphic positioning
  //println("Current mouse position in x,y: " + mouseX + " , " + mouseY);
  
  background(255); 
   
  if(!rendering_force){
    
    //s.drawContactVectors(this); 
    
   }
   
    //move our electrons in the pipe
    testCircleA.setVelocity(10,0); //for these sorts of functions, you can specify the "amount of force" on a particular x,y coordinate
    testCircleB.setVelocity(10,0);
    testCircleC.setVelocity(10,0);
    testCircleD.setVelocity(11,0); 
    testCircleE.setVelocity(11,0); 
    testCircleF.setVelocity(11,0); 
    //testCircleA.addImpulse(5,0);
    //testBlobA.setVelocity(1,0);
    //testBlobA.addImpulse(5,0); 
    
    //detect if electron is at end of pipe
    //println("Current testCircleA x pos: " + testCircleA.getX());
    //println("World width is : " + worldWidth);
    
    if(testCircleA.getX() > 14.7){ //get X is in world CM, can't be exactly world edge due to the black borders ...
      println("testCircleA is at end of pipe");
      world.remove(testCircleA);
      world.add(testCircleA); 
    }
    
    if(testCircleB.getX() > 14.7){ //get X is in world CM, can't be exactly world edge due to the black borders ...
      println("testCircleB is at end of pipe");
      world.remove(testCircleB);
      world.add(testCircleB); 
    }
    
    if(testCircleC.getX() > 14.7){ //get X is in world CM, can't be exactly world edge due to the black borders ...
      println("testCircleC is at end of pipe");
      world.remove(testCircleC);
      world.add(testCircleC); 
    }
    
    if(testCircleD.getX() > 14.7){ //get X is in world CM, can't be exactly world edge due to the black borders ...
      println("testCircleD is at end of pipe");
      world.remove(testCircleD);
      world.add(testCircleD); 
    }
    
    if(testCircleE.getX() > 14.7){ //get X is in world CM, can't be exactly world edge due to the black borders ...
      println("testCircleE is at end of pipe");
      world.remove(testCircleE);
      world.add(testCircleE); 
    }
    
    if(testCircleF.getX() > 14.7){ //get X is in world CM, can't be exactly world edge due to the black borders ...
      println("testCircleF is at end of pipe");
      world.remove(testCircleF);
      world.add(testCircleF); 
    }
    
    
   
    world.draw(); //update the world graphics on the screen
   
}

/**********************************************************************************************************************
 * Haptics simulation event, engages state of physical mechanism, calculates and updates physics simulation conditions
 **********************************************************************************************************************/ 

void onTickEvent(CountdownTimer t, long timeLeftUntilFinish){
  
  rendering_force = true;
   
  if (haply_board.data_available()) {
    /* GET END-EFFECTOR STATE (TASK SPACE) */
        
    angles.set(haply_2DoF.get_device_angles()); 
    pos_ee.set( haply_2DoF.get_device_position(angles.array()));
    pos_ee.set(pos_ee.copy().mult(100)); 
    
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