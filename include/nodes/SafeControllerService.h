//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    The fall-back controller that keeps the Crazyflie safe
//
//    ----------------------------------------------------------------------------------




//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These various headers need to be included so that this controller service can be
// connected with the D-FaLL system.

#include <math.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "crazy_pkg/IntWithHeader.h"
#include "crazy_pkg/CrazyflieData.h"
#include "crazy_pkg/Setpoint.h"
#include "crazy_pkg/ControlCommand.h"
#include "crazy_pkg/Controller.h"


// Include the shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"





// Namespacing the package
using namespace crazy_pkg;





//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.


// NOTE: these constants are already defined in the "Constant.h" header file
//       and are repeated here for convenience

// These constants define the modes that can be used for controller this is
// running on-board the Crazyflie 2.0.
// Therefore, the constants defined here need to be in agreement with those
// defined in the firmware running on-board the Crazyflie 2.0.
// The following is a short description about each mode:
//
// CF_COMMAND_TYPE_MOTORS
//     In this mode the Crazyflie will apply the requested 16-bit per motor
//     command directly to each of the motors
//
// CF_COMMAND_TYPE_RATE
//     In this mode the Crazyflie will apply the requested 16-bit per motor
//     command directly to each of the motors, and additionally request the
//     body frame roll, pitch, and yaw angular rates from the PID rate
//     controllers implemented in the Crazyflie 2.0 firmware.
//
// CF_COMMAND_TYPE_ANGLE
//     In this mode the Crazyflie will apply the requested 16-bit per motor
//     command directly to each of the motors, and additionally request the
//     body frame roll, pitch, and yaw angles from the PID attitude
//     controllers implemented in the Crazyflie 2.0 firmware.
//#define CF_COMMAND_TYPE_MOTORS 6
//#define CF_COMMAND_TYPE_RATE   7
//#define CF_COMMAND_TYPE_ANGLE  8






//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// The Load Yaml Files Service
ros::ServiceClient loadYamlFilesService_own_agent;

std::vector<float>  ffThrust(4);
std::vector<float>  feedforwardMotor(4);
float cf_mass;
float gravity_force;
std::vector<float>  motorPoly(3);

std::vector<float>  gainMatrixRoll(9);
std::vector<float>  gainMatrixPitch(9);
std::vector<float>  gainMatrixYaw(9);
std::vector<float>  gainMatrixThrust(9);

//K_infinite of feedback
std::vector<float> filterGain(6);
//only for velocity calculation
std::vector<float> estimatorMatrix(2);
float prevEstimate[9];

std::vector<float>  setpoint(4);
std::vector<float> defaultSetpoint(4);
float saturationThrust;

CrazyflieData previousLocation;


// Variable for the namespaces for the paramter services
// > For the paramter service of this agent
std::string namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string namespace_to_coordinator_parameter_service;



//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    PPPP   RRRR    OOO   TTTTT   OOO   TTTTT  Y   Y  PPPP   EEEEE   SSSS
//    P   P  R   R  O   O    T    O   O    T     Y Y   P   P  E      S
//    PPPP   RRRR   O   O    T    O   O    T      Y    PPPP   EEE     SSS
//    P      R  R   O   O    T    O   O    T      Y    P      E          S
//    P      R   R   OOO     T     OOO     T      Y    P      EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These function prototypes are not strictly required for this code to complile, but
// adding the function prototypes here means the the functions can be written below in
// any order. If the function prototypes are not included then the function need to
// written below in an order that ensure each function is implemented before it is
// called from another function, hence why the "main" function is at the bottom.

// > For the CONTROL LOOP
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);
void convertIntoBodyFrame(float est[9], float (&state)[9], float yaw_measured);
float computeMotorPolyBackward(float thrust);
void estimateState(Controller::Request &request, float (&est)[9]);

// > For the LOAD PARAMETERS
void yamlReadyForFetchCallback(const std_msgs::Int32& msg);
void fetchYamlParameters(ros::NodeHandle& nodeHandle);
void processFetchedParameters();

// > For the GETPARAM()
float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name);
void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length);
int getParameterInt(ros::NodeHandle& nodeHandle, std::string name);
void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length);
int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val);
bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name);
