//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat, Angel Romero, Cyrill Burgener, Marco Mueller, Philipp Friedli
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
//    ROS node that manages the student's setup.
//
//    ----------------------------------------------------------------------------------





//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

#include "ros/ros.h"
#include <stdlib.h>
#include <std_msgs/String.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "crazy_pkg/IntWithHeader.h"
#include "crazy_pkg/ViconData.h"
#include "crazy_pkg/CrazyflieData.h"
#include "crazy_pkg/ControlCommand.h"
#include "crazy_pkg/CrazyflieContext.h"
#include "crazy_pkg/Setpoint.h"

// Include the DFALL service types
#include "crazy_pkg/Controller.h"
#include "crazy_pkg/CMQuery.h"
#include "crazy_pkg/IntIntService.h"

// Include the shared definitions
#include "nodes/Constants.h"
#include "nodes/DefaultControllerConstants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Need for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>





// Namespacing the package
using namespace crazy_pkg;





//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------









//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// The ID of the agent that this node is monitoring
int m_agentID;

// The ID of the agent that can coordinate this node
int m_coordID;

// NAMESPACES FOR THE PARAMETER SERVICES
// > For the paramter service of this agent
std::string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string m_namespace_to_coordinator_parameter_service;



// VARIABLES FOR THE MOTION CAPTURE DATA
// > The index for which element in the "motion capture
//   data" array is expected to match the name in
//   "m_context", (negative numbers indicate unknown)
int m_poseDataIndex = -1;
// > Boolen indicating if the Mocap data is availble
bool m_isAvailable_mocap_data = false;
// > Boolen indicating if the object is "long term" occuled
bool m_isOcculded_mocap_data = false;
// > Number of consecutive occulsions that trigger the
//   "m_isOcculded_mocap_data" variable to be "true"
int yaml_consecutive_occulsions_threshold = 30;
// > Timer that when triggered determines that the
//   "m_isAvailable_mocap_data" variable becomes true
ros::Timer m_timer_mocap_timeout_check;
// > Time out duration after which Mocap is considered unavailable
float yaml_mocap_timeout_duration = 1.0;

// VARIABLES FOR GETTING MOTION CAPTURE DATA OF OTHER OBJECTS
int m_otherObjectPoseDataIndex = -1;



// VARIABLES FOR STORING THE PARAMTER OF THE POSITION
// AND TILT ANGLE SAFTY CHECKS
// > Boolean indicating whether the tilt angle check
//   should be performed
bool yaml_isEnabled_strictSafety = true;
// > The maximum allowed tilt angle, in degrees and radians
float yaml_maxTiltAngle_for_strictSafety_degrees = 70;
float yaml_maxTiltAngle_for_strictSafety_radians = 70 * DEG2RAD;



// VARIABLES FOR MANAGING THE FLYING STATE
// > Integer that is the current flying state
int m_flying_state;
// > Booleans for whether the {take-off,landing} should
//   be performed with the default controller
bool yaml_shouldPerfom_takeOff_with_defaultController = true;
bool yaml_shouldPerfom_landing_with_defaultController = true;
// > Service Clients for requesting the Default controller
//   to perform a {take-off,landing} maneouvre
ros::ServiceClient m_defaultController_requestManoeuvre;
// > Timer that fire when the {take-off,landing} is complete
ros::Timer m_timer_takeoff_complete;
ros::Timer m_timer_land_complete;



// VARIABLES RELATING TO CONTROLLER SELECTION
// > Integer indicating the controller that is to be
//   used in when motion capture data is received
int m_instant_controller;
// > Pointer to the controller service client that
//   agrees with the "m_instant_controller" variable
ros::ServiceClient* m_instant_controller_service_client;
// > Boolean indicating that the controller service clients
//   have been successfully created
bool m_controllers_avialble = false;
// > Timer for creating the controller service client after
//   some delay
ros::Timer m_timer_for_createAllControllerServiceClients;
// > Integer indicating the controller that has been
//   requested. This controller is used during the "flying"
//   state, and the "Default" controller is used during the
//   "take-off" and "landing" states.
int m_controller_nominally_selected;



// VARIABLES FOR THE CONTROLER SERVIVCE CLIENTS
// The default controller specified in the FlyingAgentClientConfig.yaml
ros::ServiceClient m_defaultController;
// The Student controller specified in the FlyingAgentClientConfig.yaml
ros::ServiceClient m_studentController;
// The Tuning controller specified in the FlyingAgentClientConfig.yaml
ros::ServiceClient m_tuningController;
// The Picker controller specified in the FlyingAgentClientConfig.yaml
ros::ServiceClient m_pickerController;
// The Remote controller specified in the FlyingAgentClientConfig.yaml
ros::ServiceClient m_remoteController;
// The Template controller specified in the FlyingAgentClientConfig.yaml
ros::ServiceClient m_templateController;

// The Test Mtoros controller specified in the FlyingAgentClientConfig.yaml
ros::ServiceClient m_testMotorsController;






// variable for crazyradio status
int m_crazyradio_status;

//describes the area of the crazyflie and other parameters
CrazyflieContext m_context;

// Service Client from which the "CrazyflieContext" is loaded
ros::ServiceClient m_centralManager;

// Publisher for the control actions to be sent on
// to the Crazyflie (the CrazyRadio node listen to this
// publisher and actually send the commands)
// {onboardControllerType,roll,pitch,yaw,motorCmd1,motorCmd2,motorCmd3,motorCmd4}
ros::Publisher m_commandForSendingToCrazyfliePublisher;

// Publisher for the current flying state of this Flying Agent Client
ros::Publisher m_flyingStatePublisher;

// Publisher for the commands of:
// {take-off,land,motors-off, and which controller to use}
//ros::Publisher commandPublisher;

// Publisher Communication with crazyRadio node. Connect and disconnect
ros::Publisher m_crazyRadioCommandPublisher;

// Publisher for which controller is currently being used
ros::Publisher m_controllerUsedPublisher;





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



// FUNCTIONS FOR HANDLING THE MOTION CAPTURE DATA
// > Callback run every time new motion capture
//   data is available
void viconCallback(const CrazyflieData& crazyflieData);
void attitudeCallback(const ViconData& viconData);
// > For extracting the pose data of an specific
//   object by name
//int getPoseDataForObjectNameWithExpectedIndex(const ViconData& viconData, std::string name , int expected_index ,CrazyflieData& pose);
// > For converting the global frame motion capture
//   data to the local frame of this agent
//void coordinatesToLocal(CrazyflieData& cf);
// > Callback run when motion capture data has not
//   been receive for a specified time
//void timerCallback_mocap_timeout_check(const ros::TimerEvent&);
// > For sending a command, via the CrazyRadio, that
//   the motors should be set to zero
void sendZeroOutputCommandForMotors();



// FOR THE SAFETY CHECKS ON POSITION AND TILT ANGLE
bool safetyCheck_on_positionAndTilt(CrazyflieData data);



// FUNCTIONS THAT MANAGE THE CHANGES TO THE FLYING STATE
// > For changing between possible state of:
//   {take-off,flying,take-off,motors-off}
//void requestChangeFlyingStateTo(int new_state);
// > For changing to take-off
///void requestChangeFlyingStateToTakeOff();
// > For changing to land
//void requestChangeFlyingStateToLand();
// > Callback that the take off phase is complete
//void takeOffTimerCallback(const ros::TimerEvent&);
// > Callback that the landing phase is complete
//void landTimerCallback(const ros::TimerEvent&);
// > Callback that the Default Controller complete a manoeuvre
//void defaultControllerManoeuvreCompleteCallback(const IntWithHeader & msg);



// FUNCTIONS FOR SELECTING WHICH CONTROLLER TO USE
// > For setting the controller that is actually used
void setInstantController(int controller);
// > For retrieving the value of the class variable
int getInstantController();
// > For setting the controller that it to be used
//   during the normal "flying" state
void setControllerNominallySelected(int controller);
// > For retrieving the value of the class variable
int getControllerNominallySelected();



// THE CALLBACK THAT A NEW FLYING STATE WAS REQUESTED
// > These requests come from the "Flying Agent GUI"
// > The options are: {take-off,land,motor-off,controller}
//void flyingStateOrControllerRequestCallback(const IntWithHeader & commandMsg);



// THE CALLBACK THAT THE CRAZY RADIO STATUS CHANGED
//void crazyRadioStatusCallback(const std_msgs::Int32& msg);



// THE CALLBACK THAT AN EMERGENCY STOP MESSAGE WAS RECEIVED
//void emergencyStopReceivedCallback(const IntWithHeader & msg);



// THE SERVICE CALLBACK REQUESTING THE CURRENT FLYING STATE
//bool getCurrentFlyingStateServiceCallback(IntIntService::Request &request, IntIntService::Response &response);



// FOR THE BATTERY STATE CALLBACK
//void batteryMonitorStateChangedCallback(std_msgs::Int32 msg);



// FUNCTIONS FOR THE CONTEXT OF THIS AGENT
// > Callback that the context database changed
//void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg);
// > For loading the context of this agent
//void loadCrazyflieContext();



// FUNCTIONS FOR CREATING THE CONTROLLER SERVICE CLIENT
// > For creating a service client from the name
//   of the YAML parameter
void createControllerServiceClientFromParameterName( std::string paramter_name , ros::ServiceClient& serviceClient );
// > For creating an IntInt service client from the
//   name of a YAML paramter
void createIntIntServiceClientFromParameterName( std::string paramter_name , ros::ServiceClient& serviceClient );
// > Callback for the timer so that all services servers
//   exists before we try to create the clients
void timerCallback_for_createAllcontrollerServiceClients(const ros::TimerEvent&);



// FOR LOADING THE YAML PARAMETERS
//void isReadyFlyingAgentClientConfigYamlCallback(const IntWithHeader & msg);
//void fetchFlyingAgentClientConfigYamlParameters(ros::NodeHandle& nodeHandle);