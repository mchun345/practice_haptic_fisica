/*** //<>//
Matthew Chun
Practicing how to make a spring using Fisica
Last modified: July 24, 2018
***/


/* library imports *****************************************************************************************************/ 

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
FCircle spring_joint_a; //first circle to represent joint of spring
//FCircle spring_joint_b; //second circlce to represent joint of spring
HVirtualCoupling spring_joint_b; //will ostensibly act as second joint of spring (to pull from)
PImage spring_joint_b_avatar; //will take visual position of second joint  
PImage spring; //image of the spring that we will distort accordingly 

//desired world boundaries (can later visually see with black borders)
float worldWidth = 20.0;  
float worldHeight = 15.0; 

float edgeTopLeftX = 0.0; 
float edgeTopLeftY = 0.0; 
float edgeBottomRightX = worldWidth; 
float edgeBottomRightY = worldHeight; 

void setup() { //one time setup for initial parameters
  
  size(1828, 1371); // (worldWidth*pixelsPerCentimeter, worldHeight*pixelsPerCentimeter) must input as number
  frameRate(60);

  /* BOARD */
  haply_board = new Board(this, Serial.list()[0], 0); 
  /* DEVICE */
  haply_2DoF = new Device(degreesOfFreedom.HaplyTwoDOF, deviceID, haply_board);

  //world setup
  hAPI_Fisica.init(this); //create instance of Fisica to reference later
  hAPI_Fisica.setScale(pixelsPerCentimeter); //presumably sets Fisica scaling based on our desired screen dimensions
  world = new FWorld(); //create an instance of a Fisica world to place our Fisica virtual objects in
 //<>//
  //spring setup
  //spring_joint_a = new FCircle(1.0);
  //spring_joint_a.setPosition(2010/2, 1188/2);
  //spring_joint_a.setFill(247,49.49); //red colour-ish
  //world.add(spring_joint_a);
  
  //spring setup but in place of our haptic avatar
  
  //tmp avatar code
  spring_joint_b = new HVirtualCoupling((1)); 
  spring_joint_b.h_avatar.setDensity(2); 
  spring_joint_b.h_avatar.setFill(255,0,0); 
  spring_joint_b.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  spring_joint_b_avatar = loadImage("../img/Haply_avatar.png"); 
  spring_joint_b_avatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  spring_joint_b.h_avatar.attachImage(spring_joint_b_avatar); 
 

  //set world parameters like boundaries, basic physics
  //world.setGravity((0.0), (200.0)); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); //important for "black borders" actual bounds of interaction
  //world.setEdgesRestitution(.4);
  //world.setEdgesFriction(0.5);
  
  world.draw();
  
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
  
   
}

void draw() {
  background(255); 
   
  if(!rendering_force){
    
    //s.drawContactVectors(this); 
    
   }
    world.draw();
    //world.drawDebug();  
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

  spring_joint_b.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+1.0, edgeTopLeftY+(pos_ee).y); 
  spring_joint_b.updateCouplingForce();
 
  f_ee.set(-spring_joint_b.getVCforceX(), spring_joint_b.getVCforceY());
 
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