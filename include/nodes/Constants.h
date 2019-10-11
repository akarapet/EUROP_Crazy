//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
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
//    Constants that are used across multiple files
//
//    ----------------------------------------------------------------------------------





//    ----------------------------------------------------------------------------------
//    U   U
//    U   U
//    U   U
//    U   U
//     UUU
//    ----------------------------------------------------------------------------------


// Conversions between degrees and radians
#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0

// PI
#define PI 3.141592653589





//    ----------------------------------------------------------------------------------
//    
//    
//    
//    
//    
//    ----------------------------------------------------------------------------------



// The types, i.e., agent or coordinator
#define TYPE_INVALID      -1
#define TYPE_COORDINATOR   1
#define TYPE_AGENT         2





//    ----------------------------------------------------------------------------------
//    
//    
//    
//    
//    
//    ----------------------------------------------------------------------------------



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
#define CF_COMMAND_TYPE_MOTORS 9
#define CF_COMMAND_TYPE_RATE   10
#define CF_COMMAND_TYPE_ANGLE  11





//    ----------------------------------------------------------------------------------
//    
//    
//    
//    
//    
//    ----------------------------------------------------------------------------------

// Types of controllers being used:
#define DEFAULT_CONTROLLER   1
#define DEMO_CONTROLLER      2
#define STUDENT_CONTROLLER   3
#define MPC_CONTROLLER       4
#define REMOTE_CONTROLLER    5
#define TUNING_CONTROLLER    6
#define PICKER_CONTROLLER    7
#define TEMPLATE_CONTROLLER  8

#define TESTMOTORS_CONTROLLER  21

// The constants that "command" changes in the
// operation state of this agent
#define CMD_USE_DEFAULT_CONTROLLER   1
#define CMD_USE_DEMO_CONTROLLER      2
#define CMD_USE_STUDENT_CONTROLLER   3
#define CMD_USE_MPC_CONTROLLER       4
#define CMD_USE_REMOTE_CONTROLLER    5
#define CMD_USE_TUNING_CONTROLLER    6
#define CMD_USE_PICKER_CONTROLLER    7
#define CMD_USE_TEMPLATE_CONTROLLER  8


#define CMD_CRAZYFLY_TAKE_OFF        11
#define CMD_CRAZYFLY_LAND            12
#define CMD_CRAZYFLY_MOTORS_OFF      13

#define CMD_USE_TESTMOTORS_CONTROLLER 21

// Flying states
#define STATE_MOTORS_OFF     1
#define STATE_TAKE_OFF       2
#define STATE_FLYING         3
#define STATE_LAND           4
#define STATE_UNAVAILABLE    5


// Commands for CrazyRadio
#define CMD_RECONNECT  0
#define CMD_DISCONNECT 1


// CrazyRadio states:
#define CRAZY_RADIO_STATE_CONNECTED      0
#define CRAZY_RADIO_STATE_CONNECTING     1
#define CRAZY_RADIO_STATE_DISCONNECTED   2




//    ----------------------------------------------------------------------------------
//    BBBB     A    TTTTT  TTTTT  EEEEE  RRRR   Y   Y
//    B   B   A A     T      T    E      R   R   Y Y
//    BBBB   A   A    T      T    EEE    RRRR     Y
//    B   B  AAAAA    T      T    E      R   R    Y
//    BBBB   A   A    T      T    EEEEE  R   R    Y
//    ----------------------------------------------------------------------------------

// Battery levels
#define BATTERY_LEVEL_000            0
#define BATTERY_LEVEL_010            1
#define BATTERY_LEVEL_020            2
#define BATTERY_LEVEL_030            3
#define BATTERY_LEVEL_040            4
#define BATTERY_LEVEL_050            5
#define BATTERY_LEVEL_060            6
#define BATTERY_LEVEL_070            7
#define BATTERY_LEVEL_080            8
#define BATTERY_LEVEL_090            9
#define BATTERY_LEVEL_100           10
#define BATTERY_LEVEL_UNAVAILABLE   -1

// Battery states
#define BATTERY_STATE_NORMAL         0
#define BATTERY_STATE_LOW            1





//    ----------------------------------------------------------------------------------
//    Y   Y    A    M   M  L
//     Y Y    A A   MM MM  L
//      Y    A   A  M M M  L
//      Y    AAAAA  M   M  L
//      Y    A   A  M   M  LLLLL
//    ----------------------------------------------------------------------------------

// For where to load the yaml file from
#define LOAD_YAML_FROM_AGENT             1
#define LOAD_YAML_FROM_COORDINATOR       2






//    ----------------------------------------------------------------------------------
//    
//    
//    
//    
//    
//    ----------------------------------------------------------------------------------

// For standard buttons in the GUI
#define REQUEST_DEFAULT_SETPOINT_BUTTON_ID    100
