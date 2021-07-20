// File:            flippy_controller.cpp
// Description:     Standard controller code for each "flippy" robot
// Authors:         J. Zerez (Spring 2021), J. Brettle (Summer 2021)

// Include standard Webots libraries
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Device.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <webots/Connector.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>

// Include standard C++ libraries
#include <string>
#include <iostream>
#include <cstring>
#include <math.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Declare simulation parameters
double goalCoords[1][4] = {0.0, 0.0, -2.0, 0.5}; // {x, y, z, tolerance} for each consecutive goal point flippies steer towards
  // TODO: use more than just the first goal point
double slowCoords[1][4] = {0.0, -0.8, -1.0, 0.1}; // {x, y, z, range multiplier} for each point source of flip delay - see auto calcFlipDelay()
  // TODO: use more than just the first slowdown point
const int FLIP_DELAY_MIN = 10; // minimum number of timesteps flippy will wait between flips - see auto calcFlipDelay()
const int FLIP_DELAY_MAX = 1000; // maximum number of timesteps flippy will wait between flips - see auto calcFlipDelay()
const int BRIDGE_DELAY = 1100; // number of timesteps flippy will wait in bridge state after no longer being walked on
float velocity = 5.0; // flipping motor speed
const float TURNING_VELOCITY = 3.0; // turning motor speed
const float CORRECTION_VELOCITY = 90.0; // flipping and turning motor speeds when reset to position = 0


// Declare intermediate variables used for calculations
const double PI = 3.141592653589;
double deltaX; // flippy's x distance from point
double deltaY; // flippy's y distance from point
double deltaZ; // flippy's z distance from point
double theta;  // angle flippy should be headed to point at goal (radians)
double adjustedYaw; // adjusted angle if flippy is upside down (radians)
double turnAngle; // angle flippy should turn now to point at goal (radians)
double dist; // radial distance from flippy to slowCoords point
int flipDelay; // calculated delay between flips based on dist from slowCoords point
int flipDelayCounter = 0; // counts up to flipDelay
int bridgeSteps = 0; // counts up to BRIDGE_DELAY

// Declare state variables used to pass infomation between loops
int movingMode = 0;   // Which sphere is moving. 0 for none, 1 for sphere1, 2 for sphere2
int oldMovingMode = 0;  // Used for knowing which movingMode to go back to after a bridge state (movingMode = 4)
int wasWalkedOn = 0;   // Tracks if flippy has been walked on but has not yet entered bridge state

// Webots comments:
// This is the main program of your controller.
// It creates an instance of the Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.

// Webots comments:
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, const char *argv[]) {

  std::cout << "INITIALIZING CONTROLLER" << std::endl;
  // Create the Robot instance.
  Robot* robot = new Robot();

  // Get name of robot for console output
  std::string robotName = robot->getName();

  // Generate the names of the 2 robot spheres/nodes
  // If the robot's ID (specified by its name field) is F###, then then spheres
  // are named F###_S1 and F###_S2.
  std::string s1Name = robot->getName().append("_S1");
  std::string s2Name = robot->getName().append("_S2");


  // Get the time step of the current world (in milliseconds)
  int timeStep = (int)robot->getBasicTimeStep();

  // Initialize motors (pointers)
  Motor* m1 = robot->getMotor("M1");
  Motor* m2 = robot->getMotor("M2");
  Motor* rm1 = robot->getMotor("RM1");
  Motor* rm2 = robot->getMotor("RM2");

  // Initialize motor operating modes (INFINITY means velocity driven)
  m1->setPosition(INFINITY);
  m2->setPosition(INFINITY);

  // Initialize position sensors
  PositionSensor* p1 = robot->getPositionSensor("P1");
  PositionSensor* p2 = robot->getPositionSensor("P2"); 
  PositionSensor* rp1 = robot->getPositionSensor("RP1");
  PositionSensor* rp2 = robot->getPositionSensor("RP2"); 
  
  // Enable position sensors
  p1->enable(timeStep);
  p2->enable(timeStep);
  rp1->enable(timeStep);
  rp2->enable(timeStep);

  // Initialize touch sensors (pointers)
  TouchSensor* t11 = robot->getTouchSensor("T11"); // light green
  TouchSensor* t12 = robot->getTouchSensor("T12"); // dark green
  TouchSensor* t21 = robot->getTouchSensor("T21"); // light blue
  TouchSensor* t22 = robot->getTouchSensor("T22"); // dark blue

  // Enable touch sensors. Sets refresh frequency to timeStep  
  t11->enable(timeStep);
  t12->enable(timeStep);
  t21->enable(timeStep);
  t22->enable(timeStep);

  // Initialize and enable the reciever (pointer)
  Receiver* r1 = robot->getReceiver("R1");
  r1->enable(timeStep);

  // Initialize the emitter (pointer). No need to enable.
  Emitter* e1 = robot->getEmitter("E1");

  // Initialize and enable the inertial unit (pointer)
  InertialUnit* iu = robot->getInertialUnit("IU");
  iu->enable(timeStep);

  // Initialize and enable the GPS (pointer)
  GPS* gps = robot->getGPS("GPS");
  gps->enable(timeStep);


  auto steer = [&](int movingSphere, bool showOutput) {
    // Calculates ideal heading and turns flippy based on which flippy sphere is moving
    // and has ability to print calculation data to console if showOutput = true

    const double *rpy = iu->getRollPitchYaw();
    const double *gpsCoords = gps->getValues();
    deltaX = goalCoords[0][0] - gpsCoords[0];
    deltaY = goalCoords[0][1] - gpsCoords[1];
    deltaZ = goalCoords[0][2] - gpsCoords[2];
   // Calculate correct yaw based on current position and goalCoords
    theta = acos(deltaX/sqrt(pow(deltaX,2.0)+pow(deltaZ,2.0)))*-deltaZ/abs(deltaZ);
    if (movingSphere == 1){
      turnAngle = theta - rpy[2];
      rm1->setPosition(turnAngle);
    } else {                        // if movingSphere = 2, Inertial Unit is upside down and yaw needs to be corrected
      if (rpy[2] < 0) {
        adjustedYaw = rpy[2] + PI;
      } else {
        adjustedYaw = rpy[2] - PI;
      }
      turnAngle = theta - adjustedYaw;
      rm2->setPosition(turnAngle);
    }
    if (showOutput){
        std::cout << "goal detas:               " << deltaX << "  " << deltaY << "  " << deltaZ << std::endl;
        std::cout << "theta:                  " << theta << std::endl;
        std::cout << "turnAngle:                " << turnAngle << std::endl;
      }
  };
  auto calcFlipDelay = [&]() {
    // Calculates and returns number of steps flippy should wait before next flip
    // based on direct distance from slowCoords using exp equation
    const double *gpsCoords = gps->getValues();
    deltaX = slowCoords[0][0] - gpsCoords[0];
    deltaY = slowCoords[0][1] - gpsCoords[1];
    deltaZ = slowCoords[0][2] - gpsCoords[2];
    dist = sqrt(pow(deltaX,2.0)+pow(deltaY,2.0)+pow(deltaZ,2.0));
   // This exp function keeps output between given MIN and MAX values
   // slowCoords[][3] defines the range of the effect (value < 1 for shorter range and > 1 for longer range)
    return (FLIP_DELAY_MAX-FLIP_DELAY_MIN)*exp(-dist/slowCoords[0][3])+FLIP_DELAY_MIN;

  };
  auto setFixedSphere = [&](int movingSphere) { 
    // Automates communicating which bodies to fix and unfix with the physics plugin
    if (movingSphere == 1){
      // Send the name of the stationary sphere to the physics plugin
      e1->send(s2Name.c_str(), sizeof(s2Name.c_str()));
      // Send the name of the moving sphere to the physics plugin
      e1->send(s1Name.c_str(), sizeof(s1Name.c_str()));
    } else {
      // Send the name of the stationary sphere to the physics plugin
      e1->send(s1Name.c_str(), sizeof(s1Name.c_str()));
      // Send the name of the moving sphere to the physics plugin
      e1->send(s2Name.c_str(), sizeof(s2Name.c_str()));
    }
  };
  auto isWalkedOn = [&](int moveMode) { 
    // Checks all touch sensors except the ones currently being used to walk
    if (moveMode == 101 && t12->getValue() == 1) {
      return true;
    } else if (moveMode == 201 && t21->getValue() == 1) {
      return true;
    } else if (t11->getValue() == 1 || t22->getValue() == 1) {
      return true;
    } else { 
      return false; 
    }
  };
  auto isDoneFlipping = [&](int moveMode) { 
    // Checks the touch sensor being used to sense the floor
    if (moveMode == 1 && t12->getValue() == 1) {
      return true;
    } else if (moveMode == 104 && t11->getValue() == 1) {
      return true;
    } else if (moveMode == 2 && t21->getValue() == 1) {
      return true;
    } else if (moveMode == 204 && t22->getValue() == 1) {
      return true;
    } else { 
      return false; 
    }
  };
  auto isBetweenFlips = [&]() {
    // Checks both flipping touch sensors
    if (t12->getValue() == 1 && t21->getValue() == 1) {
      return true;
    } else if (t12->getValue() == 1 && t21->getValue() == 1) {
      return true;
    } else {
      return false;
    }
  };


  // The first argument in controllerArgs specifies which sphere is moving
  if (strcmp(argv[1], "0") == 0) {
    // State 0: Robot is staionary. No movement.
    movingMode = 0;
    m1->setVelocity(0.0);
    m2->setVelocity(0.0);
    std::cout << "STATE IS: 0. NO MOVEMENT" << std::endl;
  } else if (strcmp(argv[1], "1") == 0) {
    // State 1: Sphere1 will move first. Robot will pivot around Sphere2
    movingMode = 1;
    m1->setVelocity(0.0);
    m2->setVelocity(velocity);
    std::cout << "STATE IS: 1. SPHERE 1 WILL MOVE FIRST" << std::endl;
  } else if (strcmp(argv[1], "2") == 0) {
    // State 2: Sphere2 will move first. Robot will pivot around Sphere1
    movingMode = 2;
    m1->setVelocity(velocity);
    m2->setVelocity(0.0);
    std::cout << "STATE IS: 2. SPHERE 2 WILL MOVE FIRST" << std::endl;
  } else {
    // Default state: Sphere1 will move first. Robot will pivot around Sphere2
    movingMode = 1;
    m1->setVelocity(0.0);
    m2->setVelocity(velocity);
    std::cout << "STATE IS: DEFAULT. SPHERE 1 WILL MOVE FIRST" << std::endl;
  }

  // Main loop: Perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    
   // Send start-of-step robot information to console. Comment out data not being currently used for speed.
    std::cout << robotName << " start step" << std::endl;
    std::cout << movingMode << "  " << t11->getValue() << t12->getValue() << t21->getValue() << t22->getValue() << "  " << wasWalkedOn << std::endl;
    // std::cout << "flipping motor positions: " << p1->getValue() << "  " << p2->getValue() << std::endl;
    // std::cout << "turning motor positions:  " << rp1->getValue() << "  " << rp2->getValue() << std::endl;
    // const double *rpy = iu->getRollPitchYaw();
    // std::cout << "roll, pitch, and yaw:     " << rpy[0] << "  " << rpy[1] << "  " << rpy[2] << std::endl;
    // const double *gpsCoords = gps->getValues();
    // std::cout << "gpsCoords:                " << gpsCoords[0] << "  " << gpsCoords[1] << "  " << gpsCoords[2] << std::endl;
    

   // flippy getting walked on protocols
    if (isWalkedOn(movingMode) == 1) {  // check the touch sensors that corespond to being walked on for the current moving mode
      wasWalkedOn = 1;
    }
    if ( isBetweenFlips() && wasWalkedOn == 1) { // if recorded being walked on and is at a sphere movement switch, go to case 4
     // make sure motors are stopped
      m1->setPosition(INFINITY);
      m1->setVelocity(0.0);
      m2->setPosition(INFINITY);
      m2->setVelocity(0.0);

      bridgeSteps = 0; // make sure delay clock is reset if flippy is still being stepped on
      wasWalkedOn = 0; // reset record so that new bump by other flippy can be recorded
      movingMode = 4; // go to case 4 to add to delay timer for every step a being stepped on isn't detected
    }

    switch (movingMode) {
      case 0: // for starting the flippy walk based on first touch sensor touched
        if (t12->getValue() == 1 || t11->getValue() == 1) { // if touch is sensed on sphere 1 then start moving sphere 2 about sphere 1 (like case 2)
         // stop m2 (stop any flipping)
          m2->setPosition(INFINITY);
          m2->setVelocity(0.0);

          setFixedSphere(1); 

         // start m1 (start moving sphere 2)
          m1->setPosition(INFINITY);
          m1->setVelocity(velocity);

         // rotate sphere 2 back to starting position (correction)
          m2->setVelocity(CORRECTION_VELOCITY);
          m2->setPosition(0.0);
          rm2->setPosition(0.0);

         // when next robot step starts, watch for touch on t21 (in case 2)
          movingMode = 201;
          oldMovingMode = 201;
        } else if (t21->getValue() == 1 || t22->getValue() == 1) { // if touch is sensed on sphere 2 then start moving sphere 1 about sphere 2 (like case 1)
         // stop m1 (stop flipping)
          m1->setPosition(INFINITY);
          m1->setVelocity(0.0);

          setFixedSphere(2);

         // start m2 (start flipping around other sphere)
          m2->setPosition(INFINITY);
          m2->setVelocity(velocity);

         // rotate sphere 1 back to starting position (correction)
          m1->setVelocity(CORRECTION_VELOCITY);
          m1->setPosition(0.0);
          rm1->setPosition(0.0);
          
         // when next robot step starts, watch for touch on t12 (in case 1)
          movingMode = 101;
          oldMovingMode = 101;
        }
        break;
      case 4:
        // in bridge state
        bridgeSteps = bridgeSteps + 1;
        // std::cout << "loop case 4. bridgeSteps = " << bridgeSteps << std::endl;
        if (bridgeSteps >= BRIDGE_DELAY){
          movingMode = oldMovingMode;
        }
        break;

      case 100:
        // Sphere1 is moving about sphere2
        // This is for the very short period right before Sphere1 lifts off the ground so that 
        // the touch sensor doesn't trigger anything

        if (p2->getValue() >= 0.1) { // when sphere 1 lifts a tiny bit off the ground, switch to movingMode 101
          movingMode = 101;
          oldMovingMode = 101;
        }
        break;
      case 101:
        // Sphere1 is moving about sphere2
        // If flippy is walked on, rotate backwards to closest bridge state and switch to movingMode 104
        // When flippy gets halfway through its flip, switch to movingMode 102

        if (wasWalkedOn == 1) {
          m2->setPosition(INFINITY);
          m2->setVelocity(-velocity);

          movingMode = 104;
        }

        if (p2->getValue() >= PI/2 - 0.1) {  
          m2->setPosition(INFINITY);
          m2->setVelocity(velocity);

          movingMode = 102;
          oldMovingMode = 102;
        }

        break;
      case 102:
        // Sphere1 is moving about sphere2
        // If the robot registers a touch through the touch sensor, stop and start delay state (movingMode 103)

        if (isDoneFlipping(1)) {  // if moving touch sensor closest to floor senses touch
         // stop m2 (stop flipping)
          m2->setPosition(INFINITY);
          m2->setVelocity(0.0);

          movingMode = 103;
          oldMovingMode = 103;

          flipDelayCounter = 1;
          flipDelay = calcFlipDelay();
        }

        break;
      case 103:
        // Flip delay state
        // When counted up to flipDelay, start the next flip and switch to movingMode 200
        
        flipDelayCounter = flipDelayCounter + 1;
        std::cout << flipDelayCounter << std::endl;

        if (flipDelayCounter >= flipDelay){
          flipDelayCounter = 0;
          
          setFixedSphere(1);

          // start m1 (start flipping around other sphere)
          m1->setVelocity(velocity);
          m1->setPosition(PI/2);

          steer(1,true);

         // rotate sphere 2 back to starting position (correction)
          m2->setVelocity(CORRECTION_VELOCITY);
          m2->setPosition(0.0);
          rm2->setPosition(0.0);

          movingMode = 200;
          oldMovingMode = 200;
        }
        
        break;
      case 104:
        // Sphere1 is moving BACKWARDS about sphere2
        // Flippy sensed being walked on while in first half of flip, so is going back the way it came

        if (isDoneFlipping(104)) {  // if moving touch sensor closest to floor senses touch
         // stop m2 (stop flipping)
          m2->setPosition(INFINITY);
          m2->setVelocity(0.0);

          movingMode = 4;
        }
        break;
      case 200:
        // Sphere2 is moving about sphere1
        // This is for the very short period right before Sphere2 lifts off the ground so that 
        // the touch sensor doesn't trigger anything
        if (p1->getValue() >= 0.1) {
          movingMode = 201;
          oldMovingMode = 201;
        }
        break;
      case 201:
        // Sphere2 is moving about sphere1
        // If flippy is walked on, rotate backwards to closest bridge state and switch to movingMode 204
        // When flippy gets halfway through its flip, switch to movingMode 202

        if (p1->getValue() >= PI/2 - 0.1) {
         
          m1->setPosition(INFINITY);
          m1->setVelocity(velocity);

          movingMode = 202;
          oldMovingMode = 202;
        }

        break;
      case 202:
        // Sphere2 is moving about sphere1
        // If the robot registers a touch through the touch sensor, stop and start delay state (movingMode 203)

        if (isDoneFlipping(2)) {  // if moving touch sensor closest to floor senses touch
         // stop m2 (stop flipping)
          m1->setPosition(INFINITY);
          m1->setVelocity(0.0);

          movingMode = 203;
          oldMovingMode = 203;

          flipDelayCounter = 1;
          flipDelay = calcFlipDelay();
        }

        break;
      case 203:
        // Flip delay state
        // When counted up to flipDelay, start the next flip and switch to movingMode 100

        flipDelayCounter = flipDelayCounter + 1;
        std::cout << flipDelayCounter << std::endl;
        
        if (flipDelayCounter >= flipDelay){
          flipDelayCounter = 0;
          
          setFixedSphere(2);

          // start m1 (start flipping around other sphere)
          m2->setVelocity(velocity);
          m2->setPosition(PI/2);

          steer(1,true);

         // rotate sphere 2 back to starting position (correction)
          m1->setVelocity(CORRECTION_VELOCITY);
          m1->setPosition(0.0);
          rm1->setPosition(0.0);

          movingMode = 100;
          oldMovingMode = 100;
        }
        
        break;
      case 204:
        // Sphere2 is moving BACKWARDS about sphere1
        // Flippy sensed being walked on while in first half of flip, so is going back the way it came

        if (isDoneFlipping(204)) {  // if moving touch sensor closest to floor senses touch
         // stop m1 (stop flipping)
          m1->setPosition(INFINITY);
          m1->setVelocity(0.0);

          movingMode = 4;
        }
        break;

    } // end of `switch (movingMode) {`

    // Recieve messages from the physics plugin
    if (r1->getQueueLength() > 0) {
      // The line below prints the incomming data from the plugin (for debuging)
      // std::cout<<r1->getData()<<std::endl;
      r1->nextPacket();
    }

   // send end-of-step robot information to console. Comment out data not being currently used for speed.
    std::cout << movingMode << "  " << t11->getValue() << t12->getValue() << t21->getValue() << t22->getValue() << "  " << wasWalkedOn << std::endl;
  
  }; // end of robot control loop

  // Exit cleanup code.
  delete robot;
  return 0;
}
