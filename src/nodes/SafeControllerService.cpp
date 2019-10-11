//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat, Cyrill Burgener, Marco Mueller, Philipp Friedli
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





// INCLUDE THE HEADER
#include "nodes/SafeControllerService.h"





//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    III M   M PPPP  L     EEEEE M   M EEEEE N   N TTTTT   A   TTTTT III  OOO  N   N
//     I  MM MM P   P L     E     MM MM E     NN  N   T    A A    T    I  O   O NN  N
//     I  M M M PPPP  L     EEE   M M M EEE   N N N   T   A   A   T    I  O   O N N N
//     I  M   M P     L     E     M   M E     N  NN   T   AAAAA   T    I  O   O N  NN
//    III M   M P     LLLLL EEEEE M   M EEEEE N   N   T   A   A   T   III  OOO  N   N
//    ----------------------------------------------------------------------------------




//    ------------------------------------------------------------------------------
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L           L       OOO    OOO   PPPP
//    C      O   O  NN  N    T    R   R  O   O  L           L      O   O  O   O  P   P
//    C      O   O  N N N    T    RRRR   O   O  L           L      O   O  O   O  PPPP
//    C      O   O  N  NN    T    R  R   O   O  L           L      O   O  O   O  P
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL       LLLLL   OOO    OOO   P
//    ----------------------------------------------------------------------------------

//simple derivative
/*
void estimateState(Controller::Request &request, float (&est)[9]) {
    est[0] = request.ownCrazyflie.x;
    est[1] = request.ownCrazyflie.y;
    est[2] = request.ownCrazyflie.z;

    est[3] = (request.ownCrazyflie.x - previousLocation.x) / request.ownCrazyflie.acquiringTime;
    est[4] = (request.ownCrazyflie.y - previousLocation.y) / request.ownCrazyflie.acquiringTime;
    est[5] = (request.ownCrazyflie.z - previousLocation.z) / request.ownCrazyflie.acquiringTime;

    est[6] = request.ownCrazyflie.roll;
    est[7] = request.ownCrazyflie.pitch;
    est[8] = request.ownCrazyflie.yaw;
}
*/


bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{
    //float yaw_measured = request.ownCrazyflie.yaw;
    float yaw_measured = request.partCrazyflie.yaw; 
    //move coordinate system to make setpoint origin
    request.ownCrazyflie.x -= setpoint[0];
    request.ownCrazyflie.y -= setpoint[1];
    request.ownCrazyflie.z -= setpoint[2];
    float yaw = request.partCrazyflie.yaw - setpoint[3];

    
    //bag.write("Offset", ros::Time::now(), request.ownCrazyflie);

    while(yaw > PI) {yaw -= 2 * PI;}
    while(yaw < -PI) {yaw += 2 * PI;}
    request.ownCrazyflie.yaw = yaw;

    float est[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    estimateState(request, est);
    float state[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    convertIntoBodyFrame(est, state, yaw_measured);

    //calculate feedback
    float outRoll = 0;
    float outPitch = 0;
    float outYaw = 0;
    float thrustIntermediate = 0;
    for(int i = 0; i < 9; ++i) {
        outRoll -= gainMatrixRoll[i] * state[i];
        outPitch -= gainMatrixPitch[i] * state[i];
        outYaw -= gainMatrixYaw[i] * state[i];
        thrustIntermediate -= gainMatrixThrust[i] * state[i];
    }
    // ROS_INFO_STREAM("thrustIntermediate = " << thrustIntermediate);
    //INFORMATION: this ugly fix was needed for the older firmware
    //outYaw *= 0.5;

    //ROS_INFO_STREAM("y-error      = " << state[1]);
    //ROS_INFO_STREAM("y-velocity   = " << state[4]);
    //ROS_INFO_STREAM("roll         = " << state[6]);
    //ROS_INFO_STREAM("rollRate     = " << outRoll );

    response.controlOutput.roll = outRoll;
    response.controlOutput.pitch = outPitch;
    response.controlOutput.yaw = outYaw;

    if(thrustIntermediate > saturationThrust)
        thrustIntermediate = saturationThrust;
    else if(thrustIntermediate < -saturationThrust)
        thrustIntermediate = -saturationThrust;

    response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustIntermediate + gravity_force);

    // ROS_INFO_STREAM("ffThrust" << ffThrust[0]);
    // ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
    // ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
    // ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
    // ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);

    response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;

    //previousLocation = request.ownCrazyflie;

    return true;
}

void convertIntoBodyFrame(float est[9], float (&state)[9], float yaw_measured)
{
    float sinYaw = sin(yaw_measured);
    float cosYaw = cos(yaw_measured);

    state[0] = est[0] * cosYaw + est[1] * sinYaw;
    state[1] = -est[0] * sinYaw + est[1] * cosYaw;
    state[2] = est[2];

    state[3] = est[3] * cosYaw + est[4] * sinYaw;
    state[4] = -est[3] * sinYaw + est[4] * cosYaw;
    state[5] = est[5];

    state[6] = est[6];
    state[7] = est[7];
    state[8] = est[8];
}



float computeMotorPolyBackward(float thrust)
{
    return (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);
}


//Kalman
void estimateState(Controller::Request &request, float (&est)[9])
{
    // attitude
    est[6] = request.partCrazyflie.roll;
    est[7] = request.partCrazyflie.pitch;
    est[8] = request.partCrazyflie.yaw;

    // //velocity & filtering
    // float ahat_x[6]; //estimator matrix times state (x, y, z, vx, vy, vz)
    // ahat_x[0] = 0; ahat_x[1]=0; ahat_x[2]=0;
    // ahat_x[3] = estimatorMatrix[0] * prevEstimate[0] + estimatorMatrix[1] * prevEstimate[3];
    // ahat_x[4] = estimatorMatrix[0] * prevEstimate[1] + estimatorMatrix[1] * prevEstimate[4];
    // ahat_x[5] = estimatorMatrix[0] * prevEstimate[2] + estimatorMatrix[1] * prevEstimate[5];

    
    // float k_x[6]; //filterGain times state
    // k_x[0] = request.ownCrazyflie.x * filterGain[0];
    // k_x[1] = request.ownCrazyflie.y * filterGain[1];
    // k_x[2] = request.ownCrazyflie.z * filterGain[2];
    // k_x[3] = request.ownCrazyflie.x * filterGain[3];
    // k_x[4] = request.ownCrazyflie.y * filterGain[4];
    // k_x[5] = request.ownCrazyflie.z * filterGain[5];
   
    est[0] = request.ownCrazyflie.x; //ahat_x[0] + k_x[0];
    est[1] = request.ownCrazyflie.y; //ahat_x[1] + k_x[1];
    est[2] = request.ownCrazyflie.z; //ahat_x[2] + k_x[2];
    est[3] = request.ownCrazyflie.vx; //ahat_x[3] + k_x[3];
    est[4] = request.ownCrazyflie.vy; //ahat_x[4] + k_x[4];
    est[5] = request.ownCrazyflie.vz; //ahat_x[5] + k_x[5];

    memcpy(prevEstimate, est, 9 * sizeof(float));
}





//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//    PPPP     A    RRRR     A    M   M  EEEEE  TTTTT  EEEEE  RRRR    SSSS
//    P   P   A A   R   R   A A   MM MM  E        T    E      R   R  S
//    PPPP   A   A  RRRR   A   A  M M M  EEE      T    EEE    RRRR    SSS
//    P      AAAAA  R  R   AAAAA  M   M  E        T    E      R  R       S
//    P      A   A  R   R  A   A  M   M  EEEEE    T    EEEEE  R   R  SSSS
//    ----------------------------------------------------------------------------------


// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
void yamlReadyForFetchCallback(const std_msgs::Int32& msg)
{
    // // Extract from the "msg" for which controller the and from where to fetch the YAML
    // // parameters
    // int controller_to_fetch_yaml = msg.data;

    // // Switch between fetching for the different controllers and from different locations
    // switch(controller_to_fetch_yaml)
    // {

    //     // > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
    //     case FETCH_YAML_SAFE_CONTROLLER_FROM_OWN_AGENT:
    //     {
    //         // Let the user know that this message was received
    //         ROS_INFO("[SAFE CONTROLLER] Received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from this machine.");
    //         // Create a node handle to the parameter service running on this agent's machine
    //         ros::NodeHandle nodeHandle_to_own_agent_parameter_service(namespace_to_own_agent_parameter_service);
    //         // Call the function that fetches the parameters
    //         fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);
    //         break;
    //     }

    //     // > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
    //     case FETCH_YAML_SAFE_CONTROLLER_FROM_COORDINATOR:
    //     {
    //         // Let the user know that this message was received
    //         ROS_INFO("[SAFE CONTROLLER] Received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from the coordinator.");
    //         // Create a node handle to the parameter service running on the coordinator machine
    //         ros::NodeHandle nodeHandle_to_coordinator_parameter_service(namespace_to_coordinator_parameter_service);
    //         // Call the function that fetches the parameters
    //         fetchYamlParameters(nodeHandle_to_coordinator_parameter_service);
    //         break;
    //     }

    //     default:
    //     {
    //         // Let the user know that the command was not relevant
    //         //ROS_INFO("The SafeControllerService received the message that YAML parameters were (re-)loaded.\r> However the parameters do not relate to this controller, hence nothing will be fetched.");
    //         break;
    //     }
    // }
}

void fetchYamlParameters(ros::NodeHandle& nodeHandle)
{

    // Here we load the parameters that are specified in the SafeController.yaml file

    // Add the "CustomController" namespace to the "nodeHandle"
    ros::NodeHandle nodeHandle_for_safeController(nodeHandle, "SafeController");

    // > The mass of the crazyflie
    cf_mass = getParameterFloat(nodeHandle_for_safeController, "mass");

    // > The co-efficients of the quadratic conversation from 16-bit motor command to
    //   thrust force in Newtons
    getParameterFloatVector(nodeHandle_for_safeController, "motorPoly", motorPoly, 3);
    
    
    // > The row of the LQR matrix that commands body frame roll rate
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixRoll", gainMatrixRoll, 9);
    // > The row of the LQR matrix that commands body frame pitch rate
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixPitch", gainMatrixPitch, 9);
    // > The row of the LQR matrix that commands body frame yaw rate
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixYaw", gainMatrixYaw, 9);
    // > The row of the LQR matrix that commands thrust adjustment
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixThrust", gainMatrixThrust, 9);

    // > The gains for the point-mass Kalman filter
    getParameterFloatVector(nodeHandle_for_safeController, "filterGain", filterGain, 6);
    getParameterFloatVector(nodeHandle_for_safeController, "estimatorMatrix", estimatorMatrix, 2);

    // > The defailt setpoint of the controller
    getParameterFloatVector(nodeHandle_for_safeController, "defaultSetpoint", defaultSetpoint, 4);

    // DEBUGGING: Print out one of the parameters that was loaded
    ROS_INFO_STREAM("[SAFE CONTROLLER] DEBUGGING: the fetched SafeController/mass = " << cf_mass);

    // Call the function that computes details an values that are needed from these
    // parameters loaded above
    processFetchedParameters();
}

void processFetchedParameters()
{
    // force that we need to counteract gravity (mg)
    gravity_force = cf_mass * 9.81/(1000*4); // in N
    // The maximum possible thrust
    saturationThrust = motorPoly[2] * 12000 * 12000 + motorPoly[1] * 12000 + motorPoly[0];
}




//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT  PPPP     A    RRRR     A    M   M   ( )
//    G      E        T    P   P   A A   R   R   A A   MM MM  (   )
//    G      EEE      T    PPPP   A   A  RRRR   A   A  M M M  (   )
//    G   G  E        T    P      AAAAA  R  R   AAAAA  M   M  (   )
//     GGGG  EEEEE    T    P      A   A  R   R  A   A  M   M   ( )
//    ----------------------------------------------------------------------------------


float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
{
    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}

void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}

int getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
{
    int val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}

void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}

int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val.size();
}

bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name)
{
    bool val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}





//    ----------------------------------------------------------------------------------
//    N   N  EEEEE  W     W        SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    NN  N  E      W     W       S      E        T    P   P  O   O   I   NN  N    T
//    N N N  EEE    W     W        SSS   EEE      T    PPPP   O   O   I   N N N    T
//    N  NN  E       W W W            S  E        T    P      O   O   I   N  NN    T
//    N   N  EEEEE    W W         SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     GGG   U   U  III        CCCC    A    L      L      BBBB     A     CCCC  K   K
//    G   G  U   U   I        C       A A   L      L      B   B   A A   C      K  K
//    G      U   U   I        C      A   A  L      L      BBBB   A   A  C      KKK
//    G   G  U   U   I        C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     GGGG   UUU   III        CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------

void setpointCallback(const Setpoint& newSetpoint) {
    setpoint[0] = newSetpoint.x;
    setpoint[1] = newSetpoint.y;
    setpoint[2] = newSetpoint.z;
    setpoint[3] = newSetpoint.yaw;
}





//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "SafeControllerService");

    ros::NodeHandle nodeHandle("~");


    // *********************************************************************************
    // EVERYTHING THAT RELATES TO FETCHING PARAMETERS FROM A YAML FILE


    // EVERYTHING FOR THE CONNECTION TO THIS AGENT's OWN PARAMETER SERVICE:

    // Get the namespace of this "SafeControllerService" node
    std::string m_namespace = ros::this_node::getNamespace();
    m_namespace = "/SafeController";
    // Set the class variable "namespace_to_own_agent_parameter_service" to be a the
    // namespace string for the parameter service that is running on the machine of this
    // agent
    namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";
    
    // Create a node handle to the parameter service running on this agent's machine
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(namespace_to_own_agent_parameter_service);

    // Instantiate the local variable "controllerYamlReadyForFetchSubscriber" to be a
    // "ros::Subscriber" type variable that subscribes to the "controllerYamlReadyForFetch" topic
    // and calls the class function "yamlReadyForFetchCallback" each time a message is
    // received on this topic and the message is passed as an input argument to the
    // "yamlReadyForFetchCallback" class function.
    ros::Subscriber controllerYamlReadyForFetchSubscriber_to_agent = nodeHandle_to_own_agent_parameter_service.subscribe("controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);


    // EVERYTHING FOR THE CONNECTION THE COORDINATOR'S PARAMETER SERVICE:

    // Set the class variable "nodeHandle_to_coordinator_parameter_service" to be a node handle
    // for the parameter service that is running on the coordinate machine
    // NOTE: the backslash here (i.e., "/") before the name of the node ("ParameterService")
    //       is very important because it specifies that the name is global
    //namespace_to_coordinator_parameter_service = "/ParameterService";

    // Create a node handle to the parameter service running on the coordinator machine
    //ros::NodeHandle nodeHandle_to_coordinator = ros::NodeHandle();
    //ros::NodeHandle nodeHandle_to_coordinator_parameter_service = ros::NodeHandle(namespace_to_own_agent_parameter_service);
    

    // Instantiate the local variable "controllerYamlReadyForFetchSubscriber" to be a
    // "ros::Subscriber" type variable that subscribes to the "controllerYamlReadyForFetch" topic
    // and calls the class function "yamlReadyForFetchCallback" each time a message is
    // received on this topic and the message is passed as an input argument to the
    // "yamlReadyForFetchCallback" class function.
    //ros::Subscriber controllerYamlReadyForFetchSubscriber_to_coordinator = nodeHandle_to_coordinator.subscribe("/ParameterService/controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);
    //ros::Subscriber controllerYamlReadyForFetchSubscriber_to_coordinator = nodeHandle_to_coordinator_parameter_service.subscribe("controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);


    // PRINT OUT SOME INFORMATION

    // Let the user know what namespaces are being used for linking to the parameter service
    ROS_INFO_STREAM("[SAFE CONTROLLER] The namespace string for accessing the Paramter Services are:");
    ROS_INFO_STREAM("[SAFE CONTROLLER] hello    =  " << m_namespace);
    ROS_INFO_STREAM("[SAFE CONTROLLER] namespace_to_own_agent_parameter_service    =  " << namespace_to_own_agent_parameter_service);
    //ROS_INFO_STREAM("[SAFE CONTROLLER] namespace_to_coordinator_parameter_service  =  " << namespace_to_coordinator_parameter_service);


    // FINALLY, FETCH ANY PARAMETERS REQUIRED FROM THESE "PARAMETER SERVICES"

    // Call the class function that loads the parameters for this class.

    // Sleep for some time (in seconds)
    // ros::Duration(1.0).sleep();

    // // Ask the Paramter Service to load the respective YAML file
    // std::string namespace_to_own_agent_loadYamlFiles_service = namespace_to_own_agent_parameter_service + "/LoadYamlFiles";
    // loadYamlFilesService_own_agent = ros::service::createClient<LoadYamlFiles>(namespace_to_own_agent_loadYamlFiles_service, true);
    // ROS_INFO_STREAM("[SAFE CONTROLLER] Loaded service: " << loadYamlFilesService_own_agent.getService());

    // LoadYamlFiles loadYamlFilesService;
    // std::vector<std::string> yamlFileNames_to_load = {"SafeController"};
    // loadYamlFilesService.request.yamlFileNames = yamlFileNames_to_load;
    // bool success = loadYamlFilesService_own_agent.call(loadYamlFilesService);

    // ROS_INFO_STREAM("[SAFE CONTROLLER] called Laod Yaml File service with success = " << success);

    // ros::Duration(2.0).sleep();


    // Call the class function that loads the parameters for this class.
    fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);


    // DEBUGGING
    // Add the "CustomController" namespace to the "nodeHandle"
    //ros::NodeHandle nodeHandle_for_safeController(nodeHandle, "SafeController");
    // > The mass of the crazyflie
    //float temp_mass = getParameterFloat(nodeHandle_for_safeController, "mass");
    //ROS_INFO_STREAM("[SAFE CONTROLLER] DEBUGGING: the fetched SafeController/mass = " << temp_mass);



    // *********************************************************************************

    
    setpoint = defaultSetpoint; // only first time setpoint is equal to default setpoint

    ros::Subscriber setpointSubscriber = nodeHandle.subscribe("Setpoint", 1, setpointCallback);

    ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());


    ros::ServiceServer service = nodeHandle.advertiseService("RateController", calculateControlOutput);
    ROS_INFO("[SAFE CONTROLLER] Service ready :-)");
    
	// std::string package_path;
	// package_path = ros::package::getPath("dfall_pkg") + "/";
	// ROS_INFO_STREAM(package_path);
	// std::string record_file = package_path + "LoggingSafeController.bag";
	// bag.open(record_file, rosbag::bagmode::Write);


    ros::spin();
	// bag.close();
	
	

    return 0;
}
