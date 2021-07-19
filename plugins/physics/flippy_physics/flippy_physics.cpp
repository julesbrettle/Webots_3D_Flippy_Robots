/*
 * File: flippy_physics.cpp
 * Date: Spring, 2021
 * Description: Custom Physics plugin for flippy. Handles crawling over terrain
 * Author: Jonathan Zerez
 * Modifications:
 */

#include <ode/ode.h>
#include <plugins/physics.h>
#include <iostream>
#include <unordered_set>

static pthread_mutex_t mutex; // needed to run with multi-threaded version of ODE

typedef enum {STATIONARY_1, STATIONARY_2} states;

static dBodyID floorBody {};
std::unordered_set<dJointID> joints {};



/*
 * Note: This plugin will become operational only after it was compiled and associated with the current world (.wbt).
 * To associate this plugin with the world follow these steps:
 *  1. In the Scene Tree, expand the "WorldInfo" node and select its "physics" field
 *  2. Then hit the [Select] button at the bottom of the Scene Tree
 *  3. In the list choose the name of this plugin (same as this file without the extention)
 *  4. Then save the .wbt by hitting the "Save" button in the toolbar of the 3D view
 *  5. Then reload the world: the plugin should now load and execute with the current simulation
 */

std::string flippyBodyName(int flippyNum, int sphereNum){
  std::string fnum = std::to_string(flippyNum);
  std::string snum = std::to_string(sphereNum);
  if (flippyNum < 10){
    return "F00" + fnum + "_S" + snum;
  } else if (flippyNum < 100){
    return "F0" + fnum + "_S" + snum;
  } else {
    return "F" + fnum + "_S" + snum;
  }
}

// Removes all user-created joints from a specified body DEF
void removeJoints(const char* bodyDef) {
  dBodyID body = dWebotsGetBodyFromDEF(bodyDef);
  int numJoints = dBodyGetNumJoints(body);
  for (int i = 0; i < numJoints; i++) {
    dJointID joint = dBodyGetJoint(body, i);
    // Only user-created joints will be in the joints set
    if (joints.find(joint) != joints.end()) {
      joints.erase(joint);
      dJointDisable(joint);
    }

  }
}

// Creates a fixed joints between two specified bodies
void addFixedJoint(dBodyID b1, dBodyID b2) {
  dWorldID joint_world = dBodyGetWorld(b1);
  pthread_mutex_lock(&mutex);
  dJointID joint = dJointCreateFixed(joint_world, 0);
  dJointAttach(joint, b1, b2);
  dJointSetFixed(joint);
  joints.insert(joint);
  pthread_mutex_unlock(&mutex);
}

void webots_physics_init() {

  // int numRobots = 4;

  pthread_mutex_init(&mutex, NULL);

  dWebotsConsolePrintf("INITIALIZING PHYSICS...");

  // Initialize Simulation Bodies
  // TODO: Need to initialize bodies in a for loop when it comes to multi-agents
  
  floorBody = dWebotsGetBodyFromDEF("FLOOR");
  
  // dBodyID flippy_bodies[numRobots][2];
  // for (int i = 0; i < numRobots; i++) {
  //   flippy_bodies[i][0] = dWebotsGetBodyFromDEF(flippyBodyName(i,1).c_str());
  //   flippy_bodies[i][1] = dWebotsGetBodyFromDEF(flippyBodyName(i,2).c_str());
  // }
  
  // dWorldID world = dBodyGetWorld(flippy_bodies[0][0]);

  // Create initial joint
  // TODO: create joint according to robot state only.
  dWebotsConsolePrintf("creating joints....");

  pthread_mutex_lock(&mutex);
  
  // for(int i = 0; i < numRobots; i++) {
    // dJointID joint = dJointCreateFixed(world, 0);
    // dJointAttach(joint, flippy_bodies[i][1], floorBody);
    // dJointSetFixed(joint);
    // joints.insert(joint);
  // }

  pthread_mutex_unlock(&mutex);

}

void webots_physics_step() {
  /*
   * Do here what needs to be done at every time step, e.g. add forces to bodies
   *   dBodyAddForce(body1, f[0], f[1], f[2]);
   *   ...
   */

  // Read data incomming from the controller
  int dataSize;
  const char* data = (const char*)dWebotsReceive(&dataSize);
  if (dataSize > 0) {
    // This code parses the messages in the packet
    char *msg = new char[dataSize];
    int count = 1, i = 0, j = 0;
    for ( ; i < dataSize; ++i) {
      char c = data[i];
      if (c == '\0') {
        msg[j] = c;
        if (count == 1) {
          // This is the first message in the package.
          // Specifies the body (by DEF) to unfix
          dWebotsConsolePrintf("REMOVING JOINTS FROM: ");
          dWebotsConsolePrintf(msg);
          removeJoints(msg);
        } else {
          // This is the second message in the package.
          // Specifies the body (by DEF) to fix
          dWebotsConsolePrintf("ADDING JOINT TO: ");
          dWebotsConsolePrintf(msg);
          dBodyID body1 = dWebotsGetBodyFromDEF(msg);
          dBodyID body2 = dWebotsGetBodyFromDEF("FLOOR");
          addFixedJoint(body1, body2);
        }
        ++count;
        j = 0;
      } else {
        msg[j] = c;
        ++j;
      }
    }

  }


}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  /*
   * This function needs to be implemented if you want to overide Webots collision detection.
   * It must return 1 if the collision was handled and 0 otherwise.
   * Note that contact joints should be added to the contactJoint_group which can change over the time, e.g.
   *   n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
   *   dJointGroupID contactJoint_group = dWebotsGetContactJointGroup();
   *   dWorldID world = dBodyGetWorld(body1);
   *   ...
   *   pthread_mutex_lock(&mutex);
   *   dJointCreateContact(world, contactJoint_group, &contact[i])
   *   dJointAttach(contactJoint, body1, body2);
   *   pthread_mutex_unlock(&mutex);
   *   ...
   */
  //  dBodyID b1 = dGeomGetBody(g1);
  //  dBodyID b2 = dGeomGetBody(g2);

  // if ((!dAreGeomsSame(g1, floor_geom) && dBodyGetNumJoints(b1) == 0) ||
       // (!dAreGeomsSame(g2, floor_geom) && dBodyGetNumJoints(b2) == 0)) {
    // addFixedJoint(g1, g2);
    // return 1;
  // }
  
  return 0;
}

void webots_physics_cleanup() {
  /*
   * Here you need to free any memory you allocated in above, close files, etc.
   * You do not need to free any ODE object, they will be freed by Webots.
   */
  pthread_mutex_destroy(&mutex);
}