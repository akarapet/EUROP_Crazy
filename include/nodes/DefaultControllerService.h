//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
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
//    The fall-back controller
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

//some useful libraries
#include <math.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <std_msgs/String.h>

// Include the DFALL message types
#include "crazy_pkg/IntWithHeader.h"
//#include "dfall_pkg/StringWithHeader.h"
#include "crazy_pkg/SetpointWithHeader.h"
#include "crazy_pkg/CustomButtonWithHeader.h"
#include "crazy_pkg/ViconData.h"
#include "crazy_pkg/Setpoint.h"
#include "crazy_pkg/ControlCommand.h"
#include "crazy_pkg/Controller.h"
#include "crazy_pkg/DebugMsg.h"

// Include the DFALL service types
#include "crazy_pkg/IntIntService.h"
#include "crazy_pkg/LoadYamlFromFilename.h"
#include "crazy_pkg/GetSetpointService.h"
#include "crazy_pkg/CMQuery.h"

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

// These constants are defined to make the code more readable and adaptable.

// NOTE: many constants are already defined in the "Constant.h" header file

// These constants define the method used for computing
// the control actions from the state error estimates.
// The following is a short description about each mode:
//
// CONTROLLER_METHOD_RATES
//       Uses the poisition, linear velocity and angle
//       error estimates to compute the rates
//
// CONTROLLER_METHOD_RATE_ANGLE_NESTED
//       Uses the position and linear velocity error
//       estimates to compute an angle, and then uses
//       this as a reference to construct an angle error
//       estimate and compute from that the rates
//
#define CONTROLLER_METHOD_RATES               1
#define CONTROLLER_METHOD_RATE_ANGLE_NESTED   2   // (DEFAULT)


// These constants define the method used for estimating
// the Inertial frame state.
// All methods are run at all times, this flag indicates
// which estimate is passed onto the controller.
// The following is a short description about each mode:
//
// ESTIMATOR_METHOD_FINITE_DIFFERENCE
//       Takes the poisition and angles directly as measured,
//       and estimates the velocities as a finite different to the
//       previous measurement
//
// ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION
//       Uses a 2nd order random walk estimator independently for
//       each of (x,y,z,roll,pitch,yaw)
//
// ESTIMATOR_METHOD_QUADROTOR_MODEL_BASED
//       Uses the model of the quad-rotor and the previous inputs
//
#define ESTIMATOR_METHOD_FINITE_DIFFERENCE          1
#define ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION   2   // (DEFAULT)


// These constants deine the behaviour of the intergrator
#define INTEGRATOR_FLAG_ON       1
#define INTEGRATOR_FLAG_OFF      2
#define INTEGRATOR_FLAG_RESET    3




//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------


// VARIABLES FOR PERFORMING THE TAKE-OFF AND LANDING MANOEUVRES

// The current state of the Default Controller
int m_current_state = DEFAULT_CONTROLLER_STATE_STANDBY;

// A flag for when the state is changed, this is used
// so that a "one-off" operation can be performed
// the first time after changing that state
bool m_current_state_changed = false;

// The elapased time, incremented by counting the motion
// capture callbacks
float m_time_in_seconds = 0.0;

// PARAMETERS FROM THE YAML FILE

// Max setpoint change per second
float yaml_max_setpoint_change_per_second_horizontal = 0.60;
float yaml_max_setpoint_change_per_second_vertical   = 0.40;

// Max error for z
float yaml_max_setpoint_error_z = 0.4;

// Max error for xy
float yaml_max_setpoint_error_xy = 0.3;

// Max {roll,pitch} angle request
float yaml_max_roll_pitch_request_degrees = 30.0;
float yaml_max_roll_pitch_request_radians = 30.0 * DEG2RAD;

// Max error for yaw angle
float yaml_max_setpoint_error_yaw_degrees = 60.0;
float yaml_max_setpoint_error_yaw_radians = 60.0 * DEG2RAD;

// Theshold for {roll,pitch} angle beyond
// which the motors are turned off
float yaml_threshold_roll_pitch_for_turn_off_degrees = 70.0;
float yaml_threshold_roll_pitch_for_turn_off_radians = 70.0 * DEG2RAD;

// The thrust for take off spin motors
float yaml_takeoff_spin_motors_thrust = 8000;
// The time for: take off spin(-up) motors
float yaml_takoff_spin_motors_time = 0.8;

// Height change for the take off move-up
float yaml_takeoff_move_up_start_height = 0.1;
float yaml_takeoff_move_up_end_height   = 0.4;
// The time for: take off spin motors
float yaml_takoff_move_up_time = 2.0;

// Minimum and maximum allowed time for: take off goto setpoint
float yaml_takoff_goto_setpoint_nearby_time = 1.0;
float yaml_takoff_goto_setpoint_max_time    = 4.0;

// Box within which to keep the integrator on
// > Units of [meters]
// > The box consider is plus/minus this value
float yaml_takoff_integrator_on_box_horizontal = 0.25;
float yaml_takoff_integrator_on_box_vertical   = 0.15;
// The time for: take off integrator-on
float yaml_takoff_integrator_on_time = 2.0;


// Height change for the landing move-down
float yaml_landing_move_down_end_height_setpoint  = 0.05;
float yaml_landing_move_down_end_height_threshold = 0.10;
// The time for: landing move-down
float yaml_landing_move_down_time_max = 5.0;

// The thrust for landing spin motors
float yaml_landing_spin_motors_thrust = 10000;
// The time for: landing spin motors
float yaml_landing_spin_motors_time = 1.5;




// The setpoint to be tracked, the ordering is (x,y,z,yaw),
// with units [meters,meters,meters,radians]
float m_setpoint[4] = {0.0,0.0,0.3,0.0};

// The setpoint that is actually used by the controller, this
// differs from the setpoint when smoothing is turned on
float m_setpoint_for_controller[4] = {0.0,0.0,0.3,0.0};

float m_setpoint_offset[4] = {0.0,0.0,0.0,0.0};

// Boolean for whether to limit rate of change of the setpoint
bool m_shouldSmoothSetpointChanges = true;





// ------------------------------------------------------
// VARIABLES THAT ARE STANDARD FOR A "CONTROLLER SERVICE"

// The ID of the agent that this node is monitoring
int m_agentID;

// The ID of the agent that can coordinate this node
int m_coordID;

// NAMESPACES FOR THE PARAMETER SERVICES
// > For the paramter service of this agent
std::string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string m_namespace_to_coordinator_parameter_service;




// VARAIBLES FOR VALUES LOADED FROM THE YAML FILE
// > the mass of the crazyflie, in [grams]
float yaml_cf_mass_in_grams = 27.0;
// > the weight of the Crazyflie in Newtons, i.e., mg
float m_cf_weight_in_newtons = yaml_cf_mass_in_grams * 9.81 / 1000.0;

// > the frequency at which the controller is running
float yaml_control_frequency = 200.0;
float m_control_deltaT = (1.0/200.0);

// > the coefficients of the 16-bit command to thrust conversion
std::vector<float> yaml_motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};

// > the min and max for saturating 16 bit thrust commands
float yaml_command_sixteenbit_min = 1000;
float yaml_command_sixteenbit_max = 60000;

// > the default setpoint, the ordering is (x,y,z,yaw),
//   with units [meters,meters,meters,radians]
std::vector<float> yaml_default_setpoint = {0.0,0.0,0.2,0.0};

// Boolean indiciating whether the "Debug Message" of this
// agent should be published or not
bool yaml_shouldPublishDebugMessage = false;

// Boolean indiciating whether the debugging ROS_INFO_STREAM
// should be displayed or not
bool yaml_shouldDisplayDebugInfo = false;



// VARIABLES FOR THE CONTROLLER

// > A flag for which controller to use:
int yaml_controller_method = CONTROLLER_METHOD_RATES ;
//double k = 2.0f;
// The LQR Controller parameters for z-height
//std::vector<float> yaml_gainMatrixThrust_2StateVector     =  { 0.65, 0.24};
//std::vector<float> yaml_gainMatrixThrust_2StateVector     =  { 0.98*0.30, 0.24*1.50};

//std::vector<float> yaml_gainMatrixThrust_2StateVector     =  { 0.98*0.4, 0.24*1.1};
//std::vector<float> yaml_gainMatrixThrust_2StateVector     =  { 0.4880, 0.2756}; //no integral optimal

std::vector<float> yaml_gainMatrixThrust_2StateVector     =  { 1.0715, 0.6}; // integral optimal

// The LQR Controller parameters for "CONTROLLER_METHOD_RATES"
//std::vector<float> yaml_gainMatrixRollRate_3StateVector   =  {-0.8163,-0.8193, 4.0930};
//std::vector<float> yaml_gainMatrixPitchRate_3StateVector  =  { 0.8163, 0.8193, 4.0930};

//std::vector<float> yaml_gainMatrixRollRate_3StateVector   =  {-1.5*2,-1.26*2, 5.2};
//std::vector<float> yaml_gainMatrixPitchRate_3StateVector  =  { 1.5*2, 1.26*2, 5.2};

//std::vector<float> yaml_gainMatrixRollRate_3StateVector   =  {-6.20*0.8,-3.00*1.1, 6.00};
//std::vector<float> yaml_gainMatrixPitchRate_3StateVector  =  { 6.20*0.8, 3.00*1.1, 6.00};

//std::vector<float> yaml_gainMatrixRollRate_3StateVector   =  {-6.1965,-3.2359, 8.0985}; // no integral optimal
//std::vector<float> yaml_gainMatrixPitchRate_3StateVector  =  { 6.1965, 3.2359, 8.17098554};

std::vector<float> yaml_gainMatrixRollRate_3StateVector   =  {-6.4509,-3.3003, 8.1754}; //  integral optimal
std::vector<float> yaml_gainMatrixPitchRate_3StateVector  =  { 6.4509, 3.3003, 8.1754};

//std::vector<float> yaml_gainMatrixRollRate_3StateVector   =  {-1.29,-1.08, 4.70};
//std::vector<float> yaml_gainMatrixPitchRate_3StateVector  =  { 1.29, 1.08, 4.70};


//std::vector<float> yaml_gainMatrixRollRate_3StateVector   =  {-6.20,-3.00, 5.20};
//std::vector<float> yaml_gainMatrixPitchRate_3StateVector  =  { 6.20, 3.00, 5.20};

// The LQR Controller parameters for "CONTROLLER_METHOD_RATE_ANGLE_NESTED"
std::vector<float> yaml_gainMatrixRollAngle_2StateVector  =  {-0.80,-0.50};
std::vector<float> yaml_gainMatrixPitchAngle_2StateVector =  { 0.80, 0.50};
float yaml_gainRollRate_fromAngle   =  -6.0;
float yaml_gainPitchRate_fromAngle  =  6.0;

float yaml_gainRollRate_fromAngle_integral   =  -3.0;
float yaml_gainPitchRate_fromAngle_integral  =  3.0;

// The LQR Controller parameters for yaw
//float yaml_gainYawRate_fromAngle    =  2.30;
float yaml_gainYawRate_fromAngle    =  2.3;
// Integrator gains
float yaml_integratorGain_forThrust = 0.3;//0.10;
float x_integrator_gain = 0.4898;
float y_integrator_gain = -0.4898;
float yaml_integratorGain_forTauXY  = 0.0;
float yaml_integratorGain_forTauYaw = 0.0;


float vel_diff[4] = {0.0,0.0,0.0,0.0};


float angle_diff[2] = {0.0,0.0};


// VARIABLES FOR THE ESTIMATOR

// Frequency at which the controller is running
float m_estimator_frequency = 200.0;

// > A flag for which estimator to use:
int yaml_estimator_method = ESTIMATOR_METHOD_FINITE_DIFFERENCE;

// > The current state interial estimate,
//   for use by the controller
float m_current_stateInertialEstimate[9];

// > The measurement of the Crazyflie at the "current" time step,
//   to avoid confusion
float m_current_xzy_rpy_measurement[6];

// > The measurement of the Crazyflie at the "previous" time step,
//   used for computing finite difference velocities
float m_previous_xzy_rpy_measurement[6];

// > The full 12 state estimate maintained by the finite
//   difference state estimator
float m_stateInterialEstimate_viaFiniteDifference[9];

// > The full 12 state estimate maintained by the point mass
//   kalman filter state estimator
float m_stateInterialEstimate_viaPointMassKalmanFilter[9];

// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
// > For the (x,y,z) position
std::vector<float> yaml_PMKF_Ahat_row1_for_positions  =  {  0.6723, 0.0034};
std::vector<float> yaml_PMKF_Ahat_row2_for_positions  =  {-12.9648, 0.9352};
std::vector<float> yaml_PMKF_Kinf_for_positions       =  {  0.3277,12.9648};


// VARIABLES RELATING TO PUBLISHING

// ROS Publisher for debugging variables
ros::Publisher m_debugPublisher;

// ROS Publisher to inform the network about
// changes to the setpoint
ros::Publisher m_setpointChangedPublisher;

// ROS Publisher to inform the flying agent client
// when a requested manoeuvre is complete
ros::Publisher m_manoeuvreCompletePublisher;

// ROS Publisher for sending motors-off command
// to the flying agent client
ros::Publisher m_motorsOffToFlyingAgentClientPublisher;

// Describes the area of the crazyflie and other parameters
//CrazyflieContext m_context;

// Flag for whether a context is available or not
bool m_isAvailableContext = false;

// Variable for each coordinate
float m_symmetric_area_bounds_x = 0.5;
float m_symmetric_area_bounds_y = 0.5;
float m_symmetric_area_bounds_z = 0.5;

// Service Client from which the "CrazyflieContext" is loaded
ros::ServiceClient m_centralManager;







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

// These function prototypes are not strictly required for this code
// to complile, but adding the function prototypes here means the
// functions can be written in any order in the ".cpp" file.
// (If the function prototypes are not included then the functions
// need to written below in an order that ensure each function is
// implemented before it is called from another function)

// CALLBACK FOR THE REQUEST MANOEUVRE SERVICE
bool requestManoeuvreCallback(IntIntService::Request &request, IntIntService::Response &response);

// CONTROLLER COMPUTATIONS
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);

// > For the normal state
void computeResponse_for_normal(Controller::Response &response);
// > For the standby state (also used for unknown state)
void computeResponse_for_standby(Controller::Response &response);
// > For the take-off phases
void computeResponse_for_takeoff_move_up(Controller::Response &response);
void computeResponse_for_takeoff_spin_motors(Controller::Response &response);
void computeResponse_for_takeoff_goto_setpoint(Controller::Response &response);
void computeResponse_for_takeoff_integrator_on(Controller::Response &response);
// > For the landing phases
void computeResponse_for_landing_move_down(Controller::Response &response);
void computeResponse_for_landing_spin_motors(Controller::Response &response);

// SMOOTHING SETPOINT CHANGES
void smoothSetpointChanges( float target_setpoint[4] , float (&current_setpoint)[4] );
void smoothSetpointChangesOffset( float target_setpoint[4] , float setpoint_offset[4], float (&current_setpoint)[4] );

// > This function constructs the error in the body frame
//   before calling the appropriate control function
void calculateControlOutput_viaLQR_givenSetpoint(float setpoint[4], float stateInertial[9], Controller::Response &response, int integrator_flag);
// > The various functions that implement an LQR controller
void calculateControlOutput_viaLQR_givenError(float stateErrorBody[9], Controller::Response &response, int integrator_flag);

// ESTIMATOR COMPUTATIONS
void performEstimatorUpdate_forStateInterial(Controller::Request &request);
void performEstimatorUpdate_forStateInterial_viaFiniteDifference();
void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter();

// PUBLISHING OF THE DEBUG MESSAGE
void construct_and_publish_debug_message(Controller::Request &request, Controller::Response &response);

// TRANSFORMATION OF THE (x,y) INERTIAL FRAME ERROR
// INTO AN (x,y) BODY FRAME ERROR
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured);

// CONVERSION FROM THRUST IN NEWTONS TO 16-BIT COMMAND
float computeMotorPolyBackward(float thrust);

// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const Setpoint& newSetpoint);

// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw);

// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response);

// PUBLISH THE CURRENT SETPOINT AND STATE
void publishCurrentSetpointAndState();

// CLIP TO MIN AND MAX
float clipToBounds(float val, float val_min, float val_max);

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived);

// PUBLISH MOTORS-OFF MESSAGE TO FLYING AGENT CLIENT
void publish_motors_off_to_flying_agent_client();

// FUNCTIONS FOR THE CONTEXT OF THIS AGENT
// > Callback that the context database changed
void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg);
// > For loading the context of this agent
void loadCrazyflieContext();

// LOADING OF YAML PARAMETERS
void timerCallback_initial_load_yaml(const ros::TimerEvent&);
void isReadyDefaultControllerYamlCallback(const IntWithHeader & msg);
//void fetchDefaultControllerYamlParameters(ros::NodeHandle& nodeHandle);