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





// INCLUDE THE HEADER
#include "nodes/DefaultControllerService.h"





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
//    RRRR   EEEEE   QQQ   U   U  EEEEE   SSSS  TTTTT
//    R   R  E      Q   Q  U   U  E      S        T
//    RRRR   EEE    Q   Q  U   U  EEE     SSS     T
//    R   R  E      Q  Q   U   U  E          S    T
//    R   R  EEEEE   QQ Q   UUU   EEEEE  SSSS     T
//    
//    M   M    A    N   N   OOO   EEEEE  U   U  V   V  RRRR   EEEEE
//    MM MM   A A   NN  N  O   O  E      U   U  V   V  R   R  E
//    M M M  A   A  N N N  O   O  EEE    U   U  V   V  RRRR   EEE
//    M   M  AAAAA  N  NN  O   O  E      U   U   V V   R   R  E
//    M   M  A   A  N   N   OOO   EEEEE   UUU     V    R   R  EEEEE
//    ------------------------------------------------------------------------------

// CALLBACK FOR THE REQUEST MANOEUVRE SERVICE

/*bool requestManoeuvreCallback(IntIntService::Request &request, IntIntService::Response &response)
{
	// Extract the requested manoeuvre
	int requestedManoeuvre = request.data;

	// Switch between the possible manoeuvres
	switch (requestedManoeuvre)
	{
		case DEFAULT_CONTROLLER_REQUEST_TAKEOFF:
		{
			// Inform the user
			ROS_INFO("[DEFAULT CONTROLLER] Received request to perform take-off manoeuvre. Switch to state: take-off spin motors");
			// Reset the time variable
			m_time_in_seconds = 0.0;
			// Update the state accordingly
			m_current_state = DEFAULT_CONTROLLER_STATE_TAKEOFF_SPIN_MOTORS;
			m_current_state_changed = true;
			// Fill in the response duration in milliseconds
			response.data = int(
					1000.0 * (
						+ yaml_takoff_spin_motors_time
						+ yaml_takoff_move_up_time
						+ yaml_takoff_goto_setpoint_max_time
						+ yaml_takoff_integrator_on_time
					)
				);
			break;
		}

		case DEFAULT_CONTROLLER_REQUEST_LANDING:
		{
			// Inform the user
			ROS_INFO("[DEFAULT CONTROLLER] Received request to perform landing manoeuvre. Switch to state: landing move down");
			// Reset the time variable
			m_time_in_seconds = 0.0;
			// Update the state accordingly
			m_current_state = DEFAULT_CONTROLLER_STATE_LANDING_MOVE_DOWN;
			m_current_state_changed = true;
			// Fill in the response duration in milliseconds
			response.data = int(
					1000 * (
						+ yaml_landing_move_down_time_max
						+ yaml_landing_spin_motors_time
					)
				);
			break;
		}

		default:
		{
			// Inform the user
			ROS_INFO("[DEFAULT CONTROLLER] The requested manoeuvre is not recognised. Hence switching to stand-by state.");
			// Update the state to standby
			m_current_state = DEFAULT_CONTROLLER_STATE_STANDBY;
			m_current_state_changed = true;
			// Fill in the response duration in milliseconds
			response.data = 0;
			break;
		}
	}

	// Publish the change
	publishCurrentSetpointAndState();

	// Return success
	return true;
}

*/
//    ------------------------------------------------------------------------------
//     OOO   U   U  TTTTT  EEEEE  RRRR 
//    O   O  U   U    T    E      R   R
//    O   O  U   U    T    EEE    RRRR
//    O   O  U   U    T    E      R  R
//     OOO    UUU     T    EEEEE  R   R
//
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L
//    C      O   O  NN  N    T    R   R  O   O  L
//    C      O   O  N N N    T    RRRR   O   O  L
//    C      O   O  N  NN    T    R  R   O   O  L
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL
//
//    L       OOO    OOO   PPPP
//    L      O   O  O   O  P   P
//    L      O   O  O   O  PPPP
//    L      O   O  O   O  P
//    LLLLL   OOO    OOO   P
//    ----------------------------------------------------------------------------------



// THE MAIN CONTROL FUNCTION CALLED FROM THE FLYING AGENT CLIENT
bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{

	// This is the "start" of the outer loop controller, add all your control
	// computation here, or you may find it convienient to create functions
	// to keep you code cleaner

	// Increment time
	m_time_in_seconds += m_control_deltaT;


	// PERFORM THE ESTIMATOR UPDATE FOR THE INTERIAL FRAME STATE
	// > After this function is complete the class variable
	//   "m_current_stateInertialEstimate" is updated and ready
	//   to be used for subsequent controller copmutations
	//performEstimatorUpdate_forStateInterial(request);

	m_current_stateInertialEstimate[0] = request.ownCrazyflie.x;
	m_current_stateInertialEstimate[1] = request.ownCrazyflie.y;
	m_current_stateInertialEstimate[2] = request.ownCrazyflie.z;

	m_current_stateInertialEstimate[3] = request.ownCrazyflie.vx;
	m_current_stateInertialEstimate[4] = request.ownCrazyflie.vy;
	m_current_stateInertialEstimate[5] = request.ownCrazyflie.vz;

	m_current_stateInertialEstimate[6] = request.partCrazyflie.roll;
	m_current_stateInertialEstimate[7] = request.partCrazyflie.pitch;
	m_current_stateInertialEstimate[8] = request.partCrazyflie.yaw;
    //std::cout<<"Yaw: "<<m_current_stateInertialEstimate[8]<<std::endl;
	//std::cout<<"Roll: "<<m_current_stateInertialEstimate[6] << std::endl;

	// Switch between the possible states
	switch (m_current_state)
	{
		case DEFAULT_CONTROLLER_STATE_NORMAL:
			computeResponse_for_normal(response);
			//std::cout<<"Hello"<<std::endl;
			break;

		case DEFAULT_CONTROLLER_STATE_TAKEOFF_SPIN_MOTORS:
			computeResponse_for_takeoff_spin_motors(response);
			break;

		case DEFAULT_CONTROLLER_STATE_TAKEOFF_MOVE_UP:
			computeResponse_for_takeoff_move_up(response);
			break;

		case DEFAULT_CONTROLLER_STATE_TAKEOFF_GOTO_SETPOINT:
			computeResponse_for_takeoff_goto_setpoint(response);
			break;

		case DEFAULT_CONTROLLER_STATE_TAKEOFF_INTEGRATOR_ON:
			computeResponse_for_takeoff_integrator_on(response);
			break;

		case DEFAULT_CONTROLLER_STATE_LANDING_MOVE_DOWN:
			computeResponse_for_landing_move_down(response);
			break;

		case DEFAULT_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
			computeResponse_for_landing_spin_motors(response);
			break;

		case DEFAULT_CONTROLLER_STATE_STANDBY:
		case DEFAULT_CONTROLLER_STATE_UNKNOWN:
		default:
			computeResponse_for_standby(response);
			break;
	}


	// Change to standby state if the {roll,pitch}
	// angles exceed the threshold
	if (
		(m_current_stateInertialEstimate[6] >  yaml_threshold_roll_pitch_for_turn_off_radians)
		or
		(m_current_stateInertialEstimate[6] < -yaml_threshold_roll_pitch_for_turn_off_radians)
		or
		(m_current_stateInertialEstimate[7] > yaml_threshold_roll_pitch_for_turn_off_radians)
		or
		(m_current_stateInertialEstimate[7] < -yaml_threshold_roll_pitch_for_turn_off_radians)
	)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Angle thereshold exceeded. Switch to state: standby");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_STANDBY;
		m_current_state_changed = true;
		// Publish a command to the "Flying Agent Client"
		// requesting the "MOTORS-OFF" state
		//publish_motors_off_to_flying_agent_client();
	}

	if (m_current_stateInertialEstimate[2]> 1.0){
		ROS_INFO("[DEFAULT CONTROLLER] Height thereshold exceeded. Switch to state: standby");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_STANDBY;
		m_current_state_changed = true;
	}

	// If the state changed,
	// then publish the setpoint so that the GUI is updated
	if (m_current_state_changed)
	{
		//publishCurrentSetpointAndState();
	}


	// PUBLISH THE DEBUG MESSAGE (if required)
	if (yaml_shouldPublishDebugMessage)
	{
		construct_and_publish_debug_message(request,response);
	}


	// Return "true" to indicate that the control computation was performed successfully
	return true;
}


void computeResponse_for_standby(Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Nothing to perform for this state
		// Set the change flag back to false
		m_current_state_changed = false;
	}

	// Fill in zero for the angle parts
	response.controlOutput.roll  = 0.0;
	response.controlOutput.pitch = 0.0;
	response.controlOutput.yaw   = 0.0;

	// Fill in all motor thrusts as zero
	response.controlOutput.motorCmd1 = 0.0;
	response.controlOutput.motorCmd2 = 0.0;
	response.controlOutput.motorCmd3 = 0.0;
	response.controlOutput.motorCmd4 = 0.0;

	// Specify that using a "motor type" of command
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS;
}


void computeResponse_for_normal(Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Set the "m_setpoint_for_controller" variable
		// to the current inertial estimate
		//m_setpoint_for_controller[0] = m_current_stateInertialEstimate[0];
		//m_setpoint_for_controller[1] = m_current_stateInertialEstimate[1];
		//m_setpoint_for_controller[2] = m_current_stateInertialEstimate[2];
		//m_setpoint_for_controller[3] = m_current_stateInertialEstimate[8];

		m_setpoint_offset[0] = m_current_stateInertialEstimate[0];
		m_setpoint_offset[1] = m_current_stateInertialEstimate[1];
		m_setpoint_offset[2] = m_current_stateInertialEstimate[2];
		m_setpoint_offset[3] = m_current_stateInertialEstimate[8];
		// Set the change flag back to false
		m_current_state_changed = false;
		// Inform the user
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] state \"normal\" started with \"m_setpoint_for_controller\" (x,y,z,yaw) =  ( " << m_setpoint_for_controller[0] << ", " << m_setpoint_for_controller[1] << ", " << m_setpoint_for_controller[2] << ", " << m_setpoint_for_controller[3] << ")");
	}

	// Smooth out any setpoint changes
	//smoothSetpointChangesOffset( m_setpoint , m_setpoint_offset, m_setpoint_for_controller );
	m_setpoint_for_controller[0] = m_setpoint[0] ;//+ m_setpoint_offset[0];
	m_setpoint_for_controller[1] = m_setpoint[1] ;//+ m_setpoint_offset[1];
	m_setpoint_for_controller[2] = m_setpoint[2] ;//+ m_setpoint_offset[2];
	m_setpoint_for_controller[3] = m_setpoint[3]; //+ m_setpoint_offset[3];
	// Call the LQR control function
	calculateControlOutput_viaLQR_givenSetpoint(m_setpoint_for_controller, m_current_stateInertialEstimate, response, INTEGRATOR_FLAG_ON);

}


void computeResponse_for_takeoff_spin_motors(Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Nothing to perform for this state
		// Set the change flag back to false
		m_current_state_changed = false;
	}

	// Compute the time elapsed as a proportion
	float time_elapsed_proportion = m_time_in_seconds / yaml_takoff_spin_motors_time;
	if (time_elapsed_proportion > 1.0)
		time_elapsed_proportion = 1.0;

	// Compute the "spinning" thrust
	float thrust_for_spinning =
		+ 1000.0
		+ time_elapsed_proportion * yaml_takeoff_spin_motors_thrust;

	// Fill in zero for the angle parts
	response.controlOutput.roll  = 0.0;
	response.controlOutput.pitch = 0.0;
	response.controlOutput.yaw   = 0.0;

	// Fill in all motor thrusts as the same
	response.controlOutput.motorCmd1 = thrust_for_spinning;
	response.controlOutput.motorCmd2 = thrust_for_spinning;
	response.controlOutput.motorCmd3 = thrust_for_spinning;
	response.controlOutput.motorCmd4 = thrust_for_spinning;

	// Specify that using a "motor type" of command
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS;

	// Change to next state after specified time
	/*if (m_time_in_seconds > yaml_takoff_spin_motors_time)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Switch to state: take-off move up");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_TAKEOFF_MOVE_UP;
		m_current_state_changed = true;
	}*/
}



void computeResponse_for_takeoff_move_up(Controller::Response &response)
{
	// Initialise a static variable for the starting height and yaw
	static float initial_height = 0.0;
	static float initial_yaw = 0.0;
	// Initialise a static variable for the yaw change
	static float yaw_start_to_end_diff = 0.0;

	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Set the current (x,y) location as the setpoint
		m_setpoint_for_controller[0] = m_current_stateInertialEstimate[0];
		m_setpoint_for_controller[1] = m_current_stateInertialEstimate[1];
		// Store the current (z,yaw)
		initial_height = m_current_stateInertialEstimate[2];
		initial_yaw = m_current_stateInertialEstimate[8];
		// Put these back into the setpoint
		m_setpoint_for_controller[2] = initial_height + yaml_takeoff_move_up_start_height;
		m_setpoint_for_controller[3] = initial_yaw;
		// Compute the yaw "start-to-end-difference" unwrapped
		yaw_start_to_end_diff = m_setpoint[3] - initial_yaw;
		while(yaw_start_to_end_diff > PI) {yaw_start_to_end_diff -= 2 * PI;}
		while(yaw_start_to_end_diff < -PI) {yaw_start_to_end_diff += 2 * PI;}
		// Set the change flag back to false
		m_current_state_changed = false;
		// Inform the user
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] state \"take-off move-up\" started with \"m_setpoint_for_controller\" (x,y,z,yaw) =  ( " << m_setpoint_for_controller[0] << ", " << m_setpoint_for_controller[1] << ", " << m_setpoint_for_controller[2] << ", " << m_setpoint_for_controller[3] << ")");
	}

	// Compute the time elapsed as a proportion
	float time_elapsed_proportion = m_time_in_seconds / (0.8*yaml_takoff_move_up_time);
	if (time_elapsed_proportion > 1.0)
		time_elapsed_proportion = 1.0;

	// Compute the z-height setpoint
	m_setpoint_for_controller[2] = initial_height + yaml_takeoff_move_up_start_height + time_elapsed_proportion * (yaml_takeoff_move_up_end_height-yaml_takeoff_move_up_start_height);

	// Compute the yaw-setpoint
	m_setpoint_for_controller[3] = initial_yaw + time_elapsed_proportion * yaw_start_to_end_diff;

	// Call the LQR control function
	calculateControlOutput_viaLQR_givenSetpoint(m_setpoint_for_controller, m_current_stateInertialEstimate, response, INTEGRATOR_FLAG_ON);

	// Change to next state after specified time
	if (m_time_in_seconds > yaml_takoff_move_up_time)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Switch to state: take-off goto setpoint");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_TAKEOFF_GOTO_SETPOINT;
		m_current_state_changed = true;
	}
}



void computeResponse_for_takeoff_goto_setpoint(Controller::Response &response)
{
	// Initialise a static variable for the time
	// to switch after entering the "box" around
	// the setpoint
	static float time_to_switch = 10.0;
	static bool box_reached = false;

	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Initialise the time to switch as greater than the 
		// max time
		time_to_switch = yaml_takoff_goto_setpoint_max_time + yaml_takoff_goto_setpoint_nearby_time;
		// Initialise the bool for whether the box was reached
		box_reached = false;
		// Set the change flag back to false
		m_current_state_changed = false;
		// Inform the user
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] state \"take-off goto-setpoint\" started with \"m_setpoint\" (x,y,z,yaw) =  ( " << m_setpoint[0] << ", " << m_setpoint[1] << ", " << m_setpoint[2] << ", " << m_setpoint[3] << ")");
	}

	// Smooth out any setpoint changes
	smoothSetpointChanges( m_setpoint , m_setpoint_for_controller );

	// Call the LQR control function
	calculateControlOutput_viaLQR_givenSetpoint(m_setpoint_for_controller, m_current_stateInertialEstimate, response, INTEGRATOR_FLAG_OFF);

	// Check if within the "integrator on" box of the setpoint
	// > First, compute the current errors
	float abs_error_x = m_setpoint[0] - m_current_stateInertialEstimate[0];
	float abs_error_y = m_setpoint[1] - m_current_stateInertialEstimate[1];
	float abs_error_z = m_setpoint[2] - m_current_stateInertialEstimate[2];
	if (abs_error_x<0.0){abs_error_x=-abs_error_x;}
	if (abs_error_y<0.0){abs_error_y=-abs_error_y;}
	if (abs_error_z<0.0){abs_error_z=-abs_error_z;}
	// > Then perform the check
	if (
		(!box_reached)
		and
		(abs_error_x < yaml_takoff_integrator_on_box_horizontal)
		and
		(abs_error_y < yaml_takoff_integrator_on_box_horizontal)
		and
		(abs_error_z < yaml_takoff_integrator_on_box_vertical)
	)
	{
		// Give it another "yaml_takoff_goto_setpoint_nearby_time"
		// seconds before changing to the integrator
		time_to_switch = m_time_in_seconds + yaml_takoff_goto_setpoint_nearby_time;
		// Set the bool that the box was reached
		box_reached = true;
	}

	if (m_time_in_seconds > time_to_switch)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Switch to state: take-off integrator on");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_TAKEOFF_INTEGRATOR_ON;
		m_current_state_changed = true;
	}

	// Change to normal if the timeout is reched
	if (m_time_in_seconds > yaml_takoff_goto_setpoint_max_time)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Did not reached the setpoint within the \"take-off goto setpoint\" allowed time. Switch to state: normal");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_NORMAL;
		m_current_state_changed = true;
	}
}



void computeResponse_for_takeoff_integrator_on(Controller::Response &response)
{
	// Initialise static variables for only running the
	// integrator once, and adjusting the time
	// accordingly
	static bool static_integrator_complete_once = false;
	static float static_integrator_on_time = yaml_takoff_integrator_on_time;
	static int static_integrator_flag = INTEGRATOR_FLAG_ON;

	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Set the "m_setpoint_for_controller" variable
		// to the current setpoint
		m_setpoint_for_controller[0] = m_setpoint[0];
		m_setpoint_for_controller[1] = m_setpoint[1];
		m_setpoint_for_controller[2] = m_setpoint[2];
		m_setpoint_for_controller[3] = m_setpoint[3];
		// Adjust the integrator on/off and time
		if (static_integrator_complete_once)
		{
			static_integrator_flag = INTEGRATOR_FLAG_OFF;
			static_integrator_on_time = 0.5;
		}
		else
		{
			static_integrator_flag = INTEGRATOR_FLAG_ON;
			static_integrator_on_time = yaml_takoff_integrator_on_time;
		}
		// Set the change flag back to false
		m_current_state_changed = false;
		// Inform the user
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] state \"take-off integrator-on\" started with \"m_setpoint\" (x,y,z,yaw) =  ( " << m_setpoint[0] << ", " << m_setpoint[1] << ", " << m_setpoint[2] << ", " << m_setpoint[3] << ")");
	}

	// Call the LQR control function
	calculateControlOutput_viaLQR_givenSetpoint(m_setpoint_for_controller, m_current_stateInertialEstimate, response , static_integrator_flag);

	// Change to next state after specified time
	if (m_time_in_seconds > static_integrator_on_time)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Publish message that take-off is complete, and  switch to state: normal");
		// Set that the integrator cylce was completed
		static_integrator_complete_once = true;
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_NORMAL;
		m_current_state_changed = true;
		// Publish a message that the take-off is complete
		//IntWithHeader msg;
		//msg.data = DEFAULT_CONTROLLER_TAKEOFF_COMPLETE;
		//m_manoeuvreCompletePublisher.publish(msg);
	}
}





void computeResponse_for_landing_move_down(Controller::Response &response)
{
	// Initialise a static variable for the starting height and yaw
	static float target_landing_setpoint[4] = {0.0,0.0,0.4,0.0};

	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Set the current (x,y,z,yaw) location as the setpoint
		m_setpoint_for_controller[0] = m_current_stateInertialEstimate[0];
		m_setpoint_for_controller[1] = m_current_stateInertialEstimate[1];
		m_setpoint_for_controller[2] = m_current_stateInertialEstimate[2];
		m_setpoint_for_controller[3] = m_current_stateInertialEstimate[8];
		// Make the target setpoint the same for (x,y,yaw)
		target_landing_setpoint[0] = m_setpoint_for_controller[0];
		target_landing_setpoint[1] = m_setpoint_for_controller[1];
		target_landing_setpoint[3] = m_setpoint_for_controller[3];
		// Set the target height
		target_landing_setpoint[2] = yaml_landing_move_down_end_height_setpoint;
		// Set the change flag back to false
		m_current_state_changed = false;
		// Inform the user
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] state \"landing move-down\" started with \"m_setpoint_for_controller\" (x,y,z,yaw) =  ( " << m_setpoint_for_controller[0] << ", " << m_setpoint_for_controller[1] << ", " << m_setpoint_for_controller[2] << ", " << m_setpoint_for_controller[3] << ")");
	}

	// Smooth out any setpoint changes
	smoothSetpointChanges( target_landing_setpoint , m_setpoint_for_controller );

	// Call the LQR control function
	calculateControlOutput_viaLQR_givenSetpoint(m_setpoint_for_controller, m_current_stateInertialEstimate, response, INTEGRATOR_FLAG_OFF);

	// Check if within the threshold of zero
	if (m_current_stateInertialEstimate[2] < yaml_landing_move_down_end_height_threshold)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Switch to state: landing spin motors");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_LANDING_SPIN_MOTORS;
		m_current_state_changed = true;
	}

	// Change to landing spin motors if the timeout is reached
	if ( m_time_in_seconds > yaml_landing_move_down_time_max )
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Did not reached the setpoint within the \"landing move down\" allowed time. Switch to state: landing spin motors");
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Update the state accordingly
		m_current_state = DEFAULT_CONTROLLER_STATE_LANDING_SPIN_MOTORS;
		m_current_state_changed = true;
	}
}


void computeResponse_for_landing_spin_motors(Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Set the z setpoint
		m_setpoint_for_controller[2] = yaml_landing_move_down_end_height_setpoint;
		// Set the change flag back to false
		m_current_state_changed = false;
		// Inform the user
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] state \"landing spin motors\" started with \"m_setpoint_for_controller\" (x,y,z,yaw) =  ( " << m_setpoint_for_controller[0] << ", " << m_setpoint_for_controller[1] << ", " << m_setpoint_for_controller[2] << ", " << m_setpoint_for_controller[3] << ")");
	}

	// Compute the time elapsed as a proportion
	float time_elapsed_proportion = m_time_in_seconds / yaml_landing_spin_motors_time;
	if (time_elapsed_proportion > 1.0)
		time_elapsed_proportion = 1.0;


	// Start by using the controller and reducing the thrust
	if (time_elapsed_proportion<0.5)
	{
		// Call the LQR control function
		calculateControlOutput_viaLQR_givenSetpoint(m_setpoint_for_controller, m_current_stateInertialEstimate, response, INTEGRATOR_FLAG_OFF);
		// Compute the desired "spinning" thrust
		float thrust_for_spinning =
			(1.0-time_elapsed_proportion)
			*
			computeMotorPolyBackward(m_cf_weight_in_newtons/4.0);
		// Adjust the motor commands
		response.controlOutput.motorCmd1 = thrust_for_spinning;
		response.controlOutput.motorCmd2 = thrust_for_spinning;
		response.controlOutput.motorCmd3 = thrust_for_spinning;
		response.controlOutput.motorCmd4 = thrust_for_spinning;
	}
	// Next stop using the controller and just spin the motors
	else
	{
		// Fill in zero for the angle parts
		response.controlOutput.roll  = 0.0;
		response.controlOutput.pitch = 0.0;
		response.controlOutput.yaw   = 0.0;
		// Fill in all motor thrusts as the same
		response.controlOutput.motorCmd1 = yaml_landing_spin_motors_thrust;
		response.controlOutput.motorCmd2 = yaml_landing_spin_motors_thrust;
		response.controlOutput.motorCmd3 = yaml_landing_spin_motors_thrust;
		response.controlOutput.motorCmd4 = yaml_landing_spin_motors_thrust;
		// Specify that using a "motor type" of command
		response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS;

		// Change to next state after specified time
		if ( m_time_in_seconds > (0.7*yaml_landing_spin_motors_time) )
		{
			// Inform the user
			ROS_INFO("[DEFAULT CONTROLLER] Publish message that landing is complete, and switch to state: standby");
			// Reset the time variable
			m_time_in_seconds = 0.0;
			// Update the state accordingly
			m_current_state = DEFAULT_CONTROLLER_STATE_STANDBY;
			m_current_state_changed = true;
			// Publish a message that the take-off is complete
			IntWithHeader msg;
			msg.data = DEFAULT_CONTROLLER_LANDING_COMPLETE;
			m_manoeuvreCompletePublisher.publish(msg);
		}
	}	
}





//    ------------------------------------------------------------------------------
//     SSSS  M   M   OOO    OOO   TTTTT  H   H
//    S      MM MM  O   O  O   O    T    H   H
//     SSS   M M M  O   O  O   O    T    HHHHH
//        S  M   M  O   O  O   O    T    H   H
//    SSSS   M   M   OOO    OOO     T    H   H
//
//     SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    S      E        T    P   P  O   O   I   NN  N    T
//     SSS   EEE      T    PPPP   O   O   I   N N N    T
//        S  E        T    P      O   O   I   N  NN    T
//    SSSS   EEEEE    T    P       OOO   III  N   N    T
//    ------------------------------------------------------------------------------



void smoothSetpointChanges( float target_setpoint[4] , float (&current_setpoint)[4] )
{
	// NO SMOOTHING IS APPLIED TO THE YAW-COORDINATE
	// > Hence copy it across directly
	current_setpoint[3] = target_setpoint[3];

	// SMOOTH THE Z-COORIDINATE
	// > Compute the max allowed change
	float max_for_z = yaml_max_setpoint_change_per_second_vertical / yaml_control_frequency;
	// > Compute the current difference
	float diff_for_z = target_setpoint[2] - current_setpoint[2] ;

	// SMOOTH THE X-Y-COORIDINATES
	// > Compute the max allowed change
	float max_for_xy = yaml_max_setpoint_change_per_second_horizontal / yaml_control_frequency;
	// > Compute the current difference
	float diff_for_x  = target_setpoint[0] - current_setpoint[0] ;
	float diff_for_y  = target_setpoint[1] - current_setpoint[1] ;
	float diff_for_xy = sqrt( diff_for_x*diff_for_x + diff_for_y*diff_for_y );

	// > Compute if outside the allowed ellipse
	float ellipse_value = (diff_for_xy*diff_for_xy)/(max_for_xy*max_for_xy) + (diff_for_z*diff_for_z)/(max_for_z*max_for_z);

	// > Clip the difference with outside the allowed ellispe
	if (ellipse_value > 1.0f)
	{
		// > Compute the proportion
		float proportion_xyz = 1.0f / sqrt(ellipse_value);
		// > Update the current setpoint
		current_setpoint[0] += proportion_xyz * diff_for_x;
		current_setpoint[1] += proportion_xyz * diff_for_y;
		current_setpoint[2] += proportion_xyz * diff_for_z;
	}
	else
	{
		// > Update the current setpoint to be the
		//   the target setpoint because it is within
		//   reach
		current_setpoint[0] = target_setpoint[0];
		current_setpoint[1] = target_setpoint[1];
		current_setpoint[2] = target_setpoint[2];
	}

}



void smoothSetpointChangesOffset( float target_setpoint[4] , float setpoint_offset[4] , float (&current_setpoint)[4] )
{
	// NO SMOOTHING IS APPLIED TO THE YAW-COORDINATE
	// > Hence copy it across directly
	current_setpoint[3] = target_setpoint[3];

	// SMOOTH THE Z-COORIDINATE
	// > Compute the max allowed change
	float max_for_z = yaml_max_setpoint_change_per_second_vertical / yaml_control_frequency;
	// > Compute the current difference
	float diff_for_z = target_setpoint[2] - current_setpoint[2] - setpoint_offset[2];

	// SMOOTH THE X-Y-COORIDINATES
	// > Compute the max allowed change
	float max_for_xy = yaml_max_setpoint_change_per_second_horizontal / yaml_control_frequency;
	// > Compute the current difference
	float diff_for_x  = target_setpoint[0] - current_setpoint[0] - setpoint_offset[0];
	float diff_for_y  = target_setpoint[1] - current_setpoint[1] - setpoint_offset[1];
	float diff_for_xy = sqrt( diff_for_x*diff_for_x + diff_for_y*diff_for_y );

	// > Compute if outside the allowed ellipse
	float ellipse_value = (diff_for_xy*diff_for_xy)/(max_for_xy*max_for_xy) + (diff_for_z*diff_for_z)/(max_for_z*max_for_z);

	// > Clip the difference with outside the allowed ellispe
	if (ellipse_value > 1.0f)
	{
		// > Compute the proportion
		float proportion_xyz = 1.0f / sqrt(ellipse_value);
		// > Update the current setpoint
		current_setpoint[0] += proportion_xyz * diff_for_x;
		current_setpoint[1] += proportion_xyz * diff_for_y;
		current_setpoint[2] += proportion_xyz * diff_for_z;
	}
	else
	{
		// > Update the current setpoint to be the
		//   the target setpoint because it is within
		//   reach
		current_setpoint[0] = target_setpoint[0] + setpoint_offset[0];
		current_setpoint[1] = target_setpoint[1] + setpoint_offset[1];
		current_setpoint[2] = target_setpoint[2] + setpoint_offset[2];
	}

}







//    ------------------------------------------------------------------------------
//    EEEEE   SSSS  TTTTT  III  M   M    A    TTTTT  III   OOO   N   N
//    E      S        T     I   MM MM   A A     T     I   O   O  NN  N
//    EEE     SSS     T     I   M M M  A   A    T     I   O   O  N N N
//    E          S    T     I   M   M  AAAAA    T     I   O   O  N  NN
//    EEEEE  SSSS     T    III  M   M  A   A    T    III   OOO   N   N
//    ------------------------------------------------------------------------------
void performEstimatorUpdate_forStateInterial(Controller::Request &request)
{

	// PUT THE CURRENT MEASURED DATA INTO THE CLASS VARIABLE
	// > for (x,y,z) position
	m_current_xzy_rpy_measurement[0] = request.ownCrazyflie.x;
	m_current_xzy_rpy_measurement[1] = request.ownCrazyflie.y;
	m_current_xzy_rpy_measurement[2] = request.ownCrazyflie.z;
	// > for (roll,pitch,yaw) angles
	m_current_xzy_rpy_measurement[3] = request.ownCrazyflie.roll;
	m_current_xzy_rpy_measurement[4] = request.ownCrazyflie.pitch;
	m_current_xzy_rpy_measurement[5] = request.ownCrazyflie.yaw;


	// RUN THE FINITE DIFFERENCE ESTIMATOR
	performEstimatorUpdate_forStateInterial_viaFiniteDifference();


	// RUN THE POINT MASS KALMAN FILTER ESTIMATOR
	performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter();


	// FILLE IN THE STATE INERTIAL ESTIMATE TO BE USED FOR CONTROL
	switch (yaml_estimator_method)
	{
		// Estimator based on finte differences
		case ESTIMATOR_METHOD_FINITE_DIFFERENCE:
		{
			// Transfer the estimate
			for(int i = 0; i < 9; ++i)
			{
				m_current_stateInertialEstimate[i]  = m_stateInterialEstimate_viaFiniteDifference[i];
			}
			break;
		}
		// Estimator based on Point Mass Kalman Filter
		case ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION:
		{
			// Transfer the estimate
			for(int i = 0; i < 9; ++i)
			{
				m_current_stateInertialEstimate[i]  = m_stateInterialEstimate_viaPointMassKalmanFilter[i];
			}
			break;
		}
		// Handle the exception
		default:
		{
			// Display that the "yaml_estimator_method" was not recognised
			ROS_INFO_STREAM("[DEFAULT CONTROLLER] ERROR: in the 'calculateControlOutput' function of the 'DefaultControllerService': the 'yaml_estimator_method' is not recognised.");
			// Transfer the finite difference estimate by default
			for(int i = 0; i < 9; ++i)
			{
				m_current_stateInertialEstimate[i]  = m_stateInterialEstimate_viaFiniteDifference[i];
			}
			break;
		}
	}


	// NOW THAT THE ESTIMATORS HAVE ALL BEEN RUN, PUT THE CURRENT
	// MEASURED DATA INTO THE CLASS VARIABLE FOR THE PREVIOUS 
	// > for (x,y,z) position
	m_previous_xzy_rpy_measurement[0] = m_current_xzy_rpy_measurement[0];
	m_previous_xzy_rpy_measurement[1] = m_current_xzy_rpy_measurement[1];
	m_previous_xzy_rpy_measurement[2] = m_current_xzy_rpy_measurement[2];
	// > for (roll,pitch,yaw) angles
	m_previous_xzy_rpy_measurement[3] = m_current_xzy_rpy_measurement[3];
	m_previous_xzy_rpy_measurement[4] = m_current_xzy_rpy_measurement[4];
	m_previous_xzy_rpy_measurement[5] = m_current_xzy_rpy_measurement[5];
}



void performEstimatorUpdate_forStateInterial_viaFiniteDifference()
{
	// PUT IN THE CURRENT MEASUREMENT DIRECTLY
	// > for (x,y,z) position
	m_stateInterialEstimate_viaFiniteDifference[0]  = m_current_xzy_rpy_measurement[0];
	m_stateInterialEstimate_viaFiniteDifference[1]  = m_current_xzy_rpy_measurement[1];
	m_stateInterialEstimate_viaFiniteDifference[2]  = m_current_xzy_rpy_measurement[2];
	// > for (roll,pitch,yaw) angles
	m_stateInterialEstimate_viaFiniteDifference[6]  = m_current_xzy_rpy_measurement[3];
	m_stateInterialEstimate_viaFiniteDifference[7]  = m_current_xzy_rpy_measurement[4];
	m_stateInterialEstimate_viaFiniteDifference[8]  = m_current_xzy_rpy_measurement[5];

	// COMPUTE THE VELOCITIES VIA FINITE DIFFERENCE
	// > for (x,y,z) velocities
	m_stateInterialEstimate_viaFiniteDifference[3]  = (m_current_xzy_rpy_measurement[0] - m_previous_xzy_rpy_measurement[0]) * m_estimator_frequency;
	m_stateInterialEstimate_viaFiniteDifference[4]  = (m_current_xzy_rpy_measurement[1] - m_previous_xzy_rpy_measurement[1]) * m_estimator_frequency;
	m_stateInterialEstimate_viaFiniteDifference[5]  = (m_current_xzy_rpy_measurement[2] - m_previous_xzy_rpy_measurement[2]) * m_estimator_frequency;
}



void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter()
{
	// PERFORM THE KALMAN FILTER UPDATE STEP
	// > First take a copy of the estimator state
	float temp_PMKFstate[9];
	for(int i = 0; i < 9; ++i)
	{
		temp_PMKFstate[i]  = m_stateInterialEstimate_viaPointMassKalmanFilter[i];
	}
	// > Now perform update for:
	// > x position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[0] = yaml_PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[0] + yaml_PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[3] + yaml_PMKF_Kinf_for_positions[0]*m_current_xzy_rpy_measurement[0];
	m_stateInterialEstimate_viaPointMassKalmanFilter[3] = yaml_PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[0] + yaml_PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[3] + yaml_PMKF_Kinf_for_positions[1]*m_current_xzy_rpy_measurement[0];
	// > y position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[1] = yaml_PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[1] + yaml_PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[4] + yaml_PMKF_Kinf_for_positions[0]*m_current_xzy_rpy_measurement[1];
	m_stateInterialEstimate_viaPointMassKalmanFilter[4] = yaml_PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[1] + yaml_PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[4] + yaml_PMKF_Kinf_for_positions[1]*m_current_xzy_rpy_measurement[1];
	// > z position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[2] = yaml_PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[2] + yaml_PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[5] + yaml_PMKF_Kinf_for_positions[0]*m_current_xzy_rpy_measurement[2];
	m_stateInterialEstimate_viaPointMassKalmanFilter[5] = yaml_PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[2] + yaml_PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[5] + yaml_PMKF_Kinf_for_positions[1]*m_current_xzy_rpy_measurement[2];

	// > for (roll,pitch,yaw) angles
	//   (taken directly from the measurement):
	m_stateInterialEstimate_viaPointMassKalmanFilter[6]  = m_current_xzy_rpy_measurement[3];
	m_stateInterialEstimate_viaPointMassKalmanFilter[7]  = m_current_xzy_rpy_measurement[4];
	m_stateInterialEstimate_viaPointMassKalmanFilter[8]  = m_current_xzy_rpy_measurement[5];
}	





//    ----------------------------------------------------------------------------------
//    L       QQQ   RRRR
//    L      Q   Q  R   R
//    L      Q   Q  RRRR
//    L      Q  Q   R  R
//    LLLLL   QQ Q  R   R
//    ----------------------------------------------------------------------------------

// > This function constructs the error in the body frame
//   before calling the appropriate control function                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
void calculateControlOutput_viaLQR_givenSetpoint(float setpoint[4], float stateInertial[9], Controller::Response &response, int integrator_flag )
{
	// Store the current YAW in a local variable
	float temp_stateInertial_yaw = stateInertial[8];

	// Initialise an array for the state error in the inertial frame
	float stateInertialError[9];

	// Adjust the INERTIAL (x,y,z) position for the setpoint
	stateInertialError[0] =  stateInertial[0] - setpoint[0];
	stateInertialError[1] =  stateInertial[1] - setpoint[1];
	stateInertialError[2] =  stateInertial[2] - setpoint[2];

	// The linear velocities can be directly copied across
	stateInertialError[3] = stateInertial[3];
	stateInertialError[4] = stateInertial[4];
	stateInertialError[5] = stateInertial[5];

	// The angular velocities for {roll,pitch} can be directly
	// copied across
	stateInertialError[6] = stateInertial[6];
	stateInertialError[7] = stateInertial[7];

	// Clip the x-error to within the specified bounds
	if (stateInertialError[0] > yaml_max_setpoint_error_xy)
		stateInertialError[0] = yaml_max_setpoint_error_xy;
	else if (stateInertialError[0] < -yaml_max_setpoint_error_xy)
		stateInertialError[0] = -yaml_max_setpoint_error_xy;

	// Clip the y-error to within the specified bounds
	if (stateInertialError[1] > yaml_max_setpoint_error_xy)
		stateInertialError[1] = yaml_max_setpoint_error_xy;
	else if (stateInertialError[1] < -yaml_max_setpoint_error_xy)
		stateInertialError[1] = -yaml_max_setpoint_error_xy;

	// Clip the z-error to within the specified bounds
	if (stateInertialError[2] > yaml_max_setpoint_error_z)
		stateInertialError[2] = yaml_max_setpoint_error_z;
	else if (stateInertialError[2] < -yaml_max_setpoint_error_z)
		stateInertialError[2] = -yaml_max_setpoint_error_z;

	// Fill in the yaw angle error
	// > This error should be "unwrapped" to be in the range
	//   ( -pi , pi )
	// > Det the yaw error into a local variable
	float yawError = stateInertial[8] - setpoint[3];
	// > "Unwrap" the yaw error to the interval ( -pi , pi )
	while(yawError > PI) {yawError -= 2 * PI;}
	while(yawError < -PI) {yawError += 2 * PI;}
	// Clip the error to within the specified bounds
	if (yawError > yaml_max_setpoint_error_yaw_radians)
		yawError = yaml_max_setpoint_error_yaw_radians;
	else if (yawError < -yaml_max_setpoint_error_yaw_radians)
		yawError = -yaml_max_setpoint_error_yaw_radians;

	// > Finally, put the "yawError" into the "stateError" variable
	stateInertialError[8] = yawError;

	// CONVERSION INTO BODY FRAME
	// Initialise a variable for the body frame error
	float bodyFrameError[9];
	// Call the function to convert the state erorr from
	// the Inertial frame into the Body frame
	convertIntoBodyFrame(stateInertialError, bodyFrameError, temp_stateInertial_yaw);

	calculateControlOutput_viaLQR_givenError(bodyFrameError, response, integrator_flag);
}

void calculateControlOutput_viaLQR_givenError(float stateErrorBody[9], Controller::Response &response, int integrator_flag)
{
	// PERFORM THE "u=Kx" LQR CONTROLLER COMPUTATION

	// Initialise static variables for the integral
	static float newtons_int = 0.0;
	static float tau_x = 0.0;
	static float tau_y = 0.0;
	static float int_x = 0.0;
	static float int_y = 0.0;
	static float tau_z = 0.0;


	std::cout<< "X Error: "<<stateErrorBody[0] <<std::endl;
	std::cout<< "Y Error: "<<stateErrorBody[1] <<std::endl;
	std::cout<< "Z Error: "<<stateErrorBody[2] <<std::endl;

	// Compute the Z-CONTROLLER
	// > provides the total thrust adjustment
	float thrustAdjustment =
		- yaml_gainMatrixThrust_2StateVector[0] * stateErrorBody[2]
		- yaml_gainMatrixThrust_2StateVector[1] * stateErrorBody[5];

	// Compute the YAW-CONTROLLER
	// > provides the body frame yaw rate
	float yawRate_forResponse =
		- yaml_gainYawRate_fromAngle * stateErrorBody[8];

	// Instantiate the local variables for the following:
	// > body frame roll rate,
	// > body frame pitch rate,
	float rollRate_forResponse = 0.0;
	float pitchRate_forResponse = 0.0;

	// Switch between the differnt control method for
	// the X-Y-CONTROLLER
	switch (yaml_controller_method)
	{
		case CONTROLLER_METHOD_RATES:
		{
			// Compute the X-CONTROLLER
			// > provides the body frame pitch rate
			pitchRate_forResponse = 
				- yaml_gainMatrixPitchRate_3StateVector[0] * stateErrorBody[0]
				- yaml_gainMatrixPitchRate_3StateVector[1] * stateErrorBody[3]
				- yaml_gainMatrixPitchRate_3StateVector[2] * stateErrorBody[7];

			// Compute the Y-CONTROLLER
			// > provides the body frame roll rate
			rollRate_forResponse = 
				- yaml_gainMatrixRollRate_3StateVector[0] * stateErrorBody[1]
				- yaml_gainMatrixRollRate_3StateVector[1] * stateErrorBody[4]
				- yaml_gainMatrixRollRate_3StateVector[2] * stateErrorBody[6];
			break;
		}

		case CONTROLLER_METHOD_RATE_ANGLE_NESTED:
		{
			// Compute the X-CONTROLLER
			// > Compute the desired pitch angle
			vel_diff[0] = stateErrorBody[0]*2 - stateErrorBody[3];
			vel_diff[1] = stateErrorBody[1]*2 - stateErrorBody[4];

			vel_diff[2] += vel_diff[0]/yaml_control_frequency;
			vel_diff[3] += vel_diff[1]/yaml_control_frequency;

			float pitchAngle_desired = 
				- 25 * vel_diff[0]
				- 1 * vel_diff[2];
			// Clip the request to within the specified limits
			if (pitchAngle_desired > yaml_max_roll_pitch_request_radians)
			{
				pitchAngle_desired = yaml_max_roll_pitch_request_radians;
				//ROS_ERROR_STREAM("[DEFAULT CONTROLLER] pitchAngle_desired = " << pitchAngle_desired);
			}
			else if (pitchAngle_desired < -yaml_max_roll_pitch_request_radians)
			{
				pitchAngle_desired = -yaml_max_roll_pitch_request_radians;
				//ROS_ERROR_STREAM("[DEFAULT CONTROLLER] pitchAngle_desired = " << pitchAngle_desired);
			}
			// Compute the Y-CONTROLLER
			// > Compute the desired roll angle
			float rollAngle_desired = 
				- 25 * vel_diff[1]
				- 1 * vel_diff[3];
			// Clip the request to within the specified limits
			if (rollAngle_desired > yaml_max_roll_pitch_request_radians)
				rollAngle_desired = yaml_max_roll_pitch_request_radians;
			else if (rollAngle_desired < -yaml_max_roll_pitch_request_radians)
				rollAngle_desired = -yaml_max_roll_pitch_request_radians;



            angle_diff[0] +=(-stateErrorBody[6] + rollAngle_desired)/yaml_control_frequency;
            angle_diff[1] +=(-stateErrorBody[7] + pitchAngle_desired)/yaml_control_frequency;
			

			// > Compute the pitch rate
			pitchRate_forResponse =
				- yaml_gainPitchRate_fromAngle * (-stateErrorBody[7] + pitchAngle_desired)
				- yaml_gainPitchRate_fromAngle_integral * angle_diff[1];


			// > Compute the roll rate
			rollRate_forResponse =
				- yaml_gainRollRate_fromAngle * (-stateErrorBody[6] + rollAngle_desired)
				- yaml_gainRollRate_fromAngle_integral * angle_diff[0] ;

			break;
		}

		default:
		{
			// Inform the user of the error
			ROS_ERROR("[DEFAULT CONTROLLER] The variable \"yaml_controller_method\" is not recognised.");
			break;
		}
	}


	// PERFORM THE INTEGRATOR COMPUTATIONS
	if (integrator_flag == INTEGRATOR_FLAG_ON)
	{
		newtons_int -= yaml_integratorGain_forThrust * stateErrorBody[2] / yaml_control_frequency;
		tau_x       += yaml_integratorGain_forTauXY  * stateErrorBody[1] / yaml_control_frequency;
		int_y       -= y_integrator_gain  * stateErrorBody[1] / yaml_control_frequency;
		tau_y       -= yaml_integratorGain_forTauXY  * stateErrorBody[0] / yaml_control_frequency;
		int_x       -= x_integrator_gain  * stateErrorBody[0] / yaml_control_frequency;
		tau_z       -= yaml_integratorGain_forTauYaw * stateErrorBody[8] / yaml_control_frequency;
	}
	else if (integrator_flag == INTEGRATOR_FLAG_RESET)
	{
		newtons_int = 0.0;
		tau_x       = 0.0;
		tau_y       = 0.0;
		tau_z	    = 0.0;
		int_y       = 0.0;
		int_x       = 0.0;
	}

	// UPDATE THE "RETURN" THE VARIABLE NAMED "response"

	// Put the computed rates and thrust into the "response" variable
	// > For roll, pitch, and yaw:
	response.controlOutput.roll  = rollRate_forResponse + int_y;
	response.controlOutput.pitch = pitchRate_forResponse + int_x;
	response.controlOutput.yaw   = yawRate_forResponse;
	// > For the thrust adjustment we must add the feed-forward thrust to counter-act gravity.
	// > NOTE: remember that the thrust is commanded per motor, so you sohuld
	//         consider whether the "thrustAdjustment" computed by your
	//         controller needed to be divided by 4 or not.
	thrustAdjustment = thrustAdjustment / 4.0;
	// > Compute the feed-forward force
	float feed_forward_thrust_per_motor = m_cf_weight_in_newtons / 4.0 + newtons_int;
	std::cout<<"FF Thrust: "<<computeMotorPolyBackward(feed_forward_thrust_per_motor)<<std::endl;
	// > Put in the per motor commands
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor - tau_x - tau_y - tau_z);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor - tau_x + tau_y + tau_z);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor + tau_x + tau_y - tau_z);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor + tau_x - tau_y + tau_z);

	
	// Specify that this controller is a rate controller
	 //response.controlOutput.onboardControllerType = //CF_COMMAND_TYPE_MOTOR;
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;
	 //response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_ANGLE;


	// An alternate debugging technique is to print out data directly to the
	// command line from which this node was launched.
	if (yaml_shouldDisplayDebugInfo)
	{
		// An example of "printing out" the control actions computed.
		ROS_INFO_STREAM("thrustAdjustment = " << thrustAdjustment);
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);

		// An example of "printing out" the "thrust-to-command" conversion parameters.
		ROS_INFO_STREAM("motorPoly 0:" << yaml_motorPoly[0]);
		ROS_INFO_STREAM("motorPoly 1:" << yaml_motorPoly[1]);
		ROS_INFO_STREAM("motorPoly 2:" << yaml_motorPoly[2]);

		// An example of "printing out" the per motor 16-bit command computed.
		ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);
	}
}



//  ***********************************************************
//  DDDD   EEEEE  BBBB   U   U   GGGG       M   M   SSSS   GGGG
//  D   D  E      B   B  U   U  G           MM MM  S      G
//  D   D  EEE    BBBB   U   U  G           M M M   SSS   G
//  D   D  E      B   B  U   U  G   G       M   M      S  G   G
//  DDDD   EEEEE  BBBB    UUU    GGGG       M   M  SSSS    GGGG

void construct_and_publish_debug_message(Controller::Request &request, Controller::Response &response)
{
	
    // DEBUGGING CODE:
    // As part of the D-FaLL-System we have defined a message type names"DebugMsg".
    // By fill this message with data and publishing it you can display the data in
    // real time using rpt plots. Instructions for using rqt plots can be found on
    // the wiki of the D-FaLL-System repository
    
	// Instantiate a local variable of type "DebugMsg", see the file "DebugMsg.msg"
	// (located in the "msg" folder) to see the full structure of this message.
	DebugMsg debugMsg;

	// Fill the debugging message with the data provided by Vicon
	debugMsg.vicon_x      = request.ownCrazyflie.x;
	debugMsg.vicon_y      = request.ownCrazyflie.y;
	debugMsg.vicon_z      = request.ownCrazyflie.z;
	debugMsg.vicon_roll   = request.ownCrazyflie.roll;
	debugMsg.vicon_pitch  = request.ownCrazyflie.pitch;
	debugMsg.vicon_yaw    = request.ownCrazyflie.yaw;

	// Fill in the debugging message with any other data you would like to display
	// in real time. For example, it might be useful to display the thrust
	// adjustment computed by the z-altitude controller.
	// The "DebugMsg" type has 10 properties from "value_1" to "value_10", all of
	// type "float64" that you can fill in with data you would like to plot in
	// real-time.
	// debugMsg.value_1 = thrustAdjustment;
	// ......................
	// debugMsg.value_10 = your_variable_name;

	// Publish the "debugMsg"
	m_debugPublisher.publish(debugMsg);
}





//    ------------------------------------------------------------------------------
//    RRRR    OOO   TTTTT    A    TTTTT  EEEEE       III  N   N  TTTTT   OOO
//    R   R  O   O    T     A A     T    E            I   NN  N    T    O   O
//    RRRR   O   O    T    A   A    T    EEE          I   N N N    T    O   O
//    R  R   O   O    T    AAAAA    T    E            I   N  NN    T    O   O
//    R   R   OOO     T    A   A    T    EEEEE       III  N   N    T     OOO
//
//    BBBB    OOO   DDDD   Y   Y       FFFFF  RRRR     A    M   M  EEEEE
//    B   B  O   O  D   D   Y Y        F      R   R   A A   MM MM  E
//    BBBB   O   O  D   D    Y         FFF    RRRR   A   A  M M M  EEE
//    B   B  O   O  D   D    Y         F      R  R   AAAAA  M   M  E
//    BBBB    OOO   DDDD     Y         F      R   R  A   A  M   M  EEEEE
//    ----------------------------------------------------------------------------------

// ROTATES THE (x,y) COMPONENTS BY THE PROVIDED "yaw" ANGLE
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured)
{

	float sinYaw = sin(yaw_measured);
	float cosYaw = cos(yaw_measured);

	// Fill in the (x,y,z) position estimates to be returned
	stateBody[0] = stateInertial[0] * cosYaw  +  stateInertial[1] * sinYaw;
	stateBody[1] = stateInertial[1] * cosYaw  -  stateInertial[0] * sinYaw;
	stateBody[2] = stateInertial[2];

	// Fill in the (x,y,z) velocity estimates to be returned
	stateBody[3] = stateInertial[3] * cosYaw  +  stateInertial[4] * sinYaw;
	stateBody[4] = stateInertial[4] * cosYaw  -  stateInertial[3] * sinYaw;
	stateBody[5] = stateInertial[5];

	// Fill in the (roll,pitch,yaw) estimates to be returned
	stateBody[6] = stateInertial[6];
	stateBody[7] = stateInertial[7];
	stateBody[8] = stateInertial[8];
}





//    ------------------------------------------------------------------------------
//    N   N  EEEEE  W     W   TTTTT   OOO   N   N        CCCC  M   M  DDDD
//    NN  N  E      W     W     T    O   O  NN  N       C      MM MM  D   D
//    N N N  EEE    W     W     T    O   O  N N N  -->  C      M M M  D   D
//    N  NN  E       W W W      T    O   O  N  NN       C      M   M  D   D
//    N   N  EEEEE    W W       T     OOO   N   N        CCCC  M   M  DDDD
//
//     CCCC   OOO   N   N  V   V  EEEEE  RRRR    SSSS  III   OOO   N   N
//    C      O   O  NN  N  V   V  E      R   R  S       I   O   O  NN  N
//    C      O   O  N N N  V   V  EEE    RRRR    SSS    I   O   O  N N N
//    C      O   O  N  NN   V V   E      R  R       S   I   O   O  N  NN
//     CCCC   OOO   N   N    V    EEEEE  R   R  SSSS   III   OOO   N   N
//    ----------------------------------------------------------------------------------

// CONVERTS A THURST IN NEWTONS TO A 16-BIT NUMBER
float computeMotorPolyBackward(float thrust)
{
	// Compute the 16-bit command that would produce the requested
	// "thrust" based on the quadratic mapping that is described
	// by the coefficients in the "yaml_motorPoly" variable.
	float cmd_16bit = (-yaml_motorPoly[1] + sqrt(yaml_motorPoly[1] * yaml_motorPoly[1] - 4 * yaml_motorPoly[2] * (yaml_motorPoly[0] - thrust))) / (2 * yaml_motorPoly[2]);

	// Saturate the signal to be 0 or in the range [1000,65000]
	if (cmd_16bit < yaml_command_sixteenbit_min)
	{
		cmd_16bit = 0;
	}
	else if (cmd_16bit > yaml_command_sixteenbit_max)
	{
		cmd_16bit = yaml_command_sixteenbit_max;
	}

	// Return the result
	return cmd_16bit;
}





//    ----------------------------------------------------------------------------------
//    N   N  EEEEE  W     W        SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    NN  N  E      W     W       S      E        T    P   P  O   O   I   NN  N    T
//    N N N  EEE    W     W        SSS   EEE      T    PPPP   O   O   I   N N N    T
//    N  NN  E       W W W            S  E        T    P      O   O   I   N  NN    T
//    N   N  EEEEE    W W         SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------


// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const Setpoint& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = false; //checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );


			m_setpoint[0]	=	newSetpoint.x;
			m_setpoint[1]	=	newSetpoint.y;
			m_setpoint[2]	=	newSetpoint.z;
			m_setpoint[3]	=	newSetpoint.yaw;
ROS_INFO("[DEFAULT CONTROLLER] New setpoint requested");

	// Continue if the message is relevant
/*	if (isRevelant)
	{
		// Check if the request if for the default setpoint
		if (newSetpoint.buttonID == REQUEST_DEFAULT_SETPOINT_BUTTON_ID)
		{
			setNewSetpoint(
					yaml_default_setpoint[0],
					yaml_default_setpoint[1],
					yaml_default_setpoint[2],
					yaml_default_setpoint[3]
				);
		}
		else
		{
			// Call the function for actually setting the setpoint
			setNewSetpoint(
					newSetpoint.x,
					newSetpoint.y,
					newSetpoint.z,
					newSetpoint.yaw
				);
		}
	}*/
}



// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw)
{
/*	if (!m_isAvailableContext)
	{
		ROS_ERROR("[DEFAULT CONTROLLER] New setpoint requested, however there is no context available, setting default setpoint instead.");
		m_setpoint[0] = yaml_default_setpoint[0];
		m_setpoint[1] = yaml_default_setpoint[1];
		m_setpoint[2] = yaml_default_setpoint[2];
		m_setpoint[3] = yaml_default_setpoint[3];
	}
	else
	{
		// Put the new setpoint into the class variable
		m_setpoint[0] = clipToBounds( x , -m_symmetric_area_bounds_x , m_symmetric_area_bounds_x );
		m_setpoint[1] = clipToBounds( y , -m_symmetric_area_bounds_y , m_symmetric_area_bounds_y );;
		m_setpoint[2] = clipToBounds( z , -m_symmetric_area_bounds_z , m_symmetric_area_bounds_z );;
		m_setpoint[3] = yaw;
	}

	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)

	// Instantiate a local variable of type "SetpointWithHeader"
	SetpointWithHeader msg;
	// Fill in the setpoint
	msg.x   = m_setpoint[0];
	msg.y   = m_setpoint[1];
	msg.z   = m_setpoint[2];
	msg.yaw = m_setpoint[3];
	// Publish the message
	m_setpointChangedPublisher.publish(msg);*/
}


// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response)
{
	// Directly put the current setpoint into the response
	response.setpointWithHeader.x   = m_setpoint[0];
	response.setpointWithHeader.y   = m_setpoint[1];
	response.setpointWithHeader.z   = m_setpoint[2];
	response.setpointWithHeader.yaw = m_setpoint[3];
	// Put the current state into the "buttonID" field
	response.setpointWithHeader.buttonID = m_current_state;
	// Return
	return true;
}


// PUBLISH THE CURRENT SETPOINT SO THAT THE NETWORK IS UPDATED
void publishCurrentSetpointAndState()
{
	// Instantiate a local variable of type "SetpointWithHeader"
	SetpointWithHeader msg;
	// Fill in the setpoint
	msg.x   = m_setpoint[0];
	msg.y   = m_setpoint[1];
	msg.z   = m_setpoint[2];
	msg.yaw = m_setpoint[3];
	// Put the current state into the "buttonID" field
	msg.buttonID = m_current_state;
	// Publish the message
	m_setpointChangedPublisher.publish(msg);
}

float clipToBounds(float val, float val_min, float val_max)
{
	float return_val = val;
	if (return_val < val_min)
		return_val = val_min;
	if (return_val > val_max)
		return_val = val_max;

	return return_val;
}




//    ----------------------------------------------------------------------------------
//     CCCC  U   U   SSSS  TTTTT   OOO   M   M
//    C      U   U  S        T    O   O  MM MM
//    C      U   U   SSS     T    O   O  M M M
//    C      U   U      S    T    O   O  M   M
//     CCCC   UUU   SSSS     T     OOO   M   M
//
//     CCCC   OOO   M   M  M   M    A    N   N  DDDD
//    C      O   O  MM MM  MM MM   A A   NN  N  D   D
//    C      O   O  M M M  M M M  A   A  N N N  D   D
//    C      O   O  M   M  M   M  AAAAA  N  NN  D   D
//     CCCC   OOO   M   M  M   M  A   A  N   N  DDDD
//    ----------------------------------------------------------------------------------

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived)
{
	// Check whether the message is relevant
	bool isRevelant = false;//checkMessageHeader( m_agentID , commandReceived.shouldCheckForAgentID , commandReceived.agentIDs );

	if (isRevelant)
	{
		// Extract the data from the message
		int custom_button_index   = commandReceived.button_index;
		float custom_command_code = commandReceived.float_data;

		// Switch between the button pressed
		switch(custom_button_index)
		{

			// > FOR CUSTOM BUTTON 1
			case 1:
			{
				// Inform the user
				ROS_INFO("[DEFAULT CONTROLLER] Received request to run integrator. Switch to state: take-off goto-setpoint");
				// Reset the time variable
				m_time_in_seconds = 0.0;
				// Update the state accordingly
				m_current_state = DEFAULT_CONTROLLER_STATE_TAKEOFF_GOTO_SETPOINT;
				m_current_state_changed = true;
				break;
			}

			// > FOR CUSTOM BUTTON 2
			case 2:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO("[DEFAULT CONTROLLER] Received request to reset the integrator.");
				// Call the controller function with the reset flag
				float tempStateBodyError[9];
				Controller::Response temp_response;
				calculateControlOutput_viaLQR_givenError(tempStateBodyError, temp_response, INTEGRATOR_FLAG_RESET);

				break;
			}

			// > FOR CUSTOM BUTTON 3
			case 3:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[DEFAULT CONTROLLER] Button 3 received in controller, with command code:" << custom_command_code );
				// Code here to respond to custom button 3

				break;
			}

			default:
			{
				// Let the user know that the command was not recognised
				ROS_INFO_STREAM("[DEMO CONTROLLER] A button clicked command was received in the controller but not recognised, message.button_index = " << custom_button_index << ", and message.command_code = " << custom_command_code );
				break;
			}
		}
	}
}






// PUBLISH MOTORS-OFF MESSAGE TO FLYING AGENT CLIENT
void publish_motors_off_to_flying_agent_client()
{
	// Instantiate a local variable of type "IntWithHeader"
	IntWithHeader msg;
	// Fill in the MOTORS-OFF command
	msg.data = CMD_CRAZYFLY_MOTORS_OFF;
	// Fill in the header that this applies
	msg.shouldCheckForAgentID = false;
	// Publish the message
	m_motorsOffToFlyingAgentClientPublisher.publish(msg);
}




//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//     CCCC   OOO   N   N  TTTTT  EEEEE  X   X  TTTTT
//    C      O   O  NN  N    T    E       X X     T
//    C      O   O  N N N    T    EEE      X      T
//    C      O   O  N  NN    T    E       X X     T
//     CCCC   OOO   N   N    T    EEEEE  X   X    T
//    ----------------------------------------------------------------------------------


void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg)
{
    ROS_INFO("[DEFAULT CONTROLLER] Received message that the Context Database Changed");
    loadCrazyflieContext();
}


void requestStateChangeCallback(const std_msgs::Int32& msg)
{
	std::cout<<msg.data<<std::endl;
	if (msg.data == 1)
	{
    ROS_INFO("[DEFAULT CONTROLLER] Received message to change state to normal");
	m_current_state = DEFAULT_CONTROLLER_STATE_NORMAL;
	m_current_state_changed = true;	
}
else{
	ROS_INFO("[DEFAULT CONTROLLER] Received message to change state to standby");
	m_current_state = DEFAULT_CONTROLLER_STATE_STANDBY;
	m_current_state_changed = true;
}
}


void loadCrazyflieContext()
{
    CMQuery contextCall;
    contextCall.request.studentID = m_agentID;
    ROS_INFO_STREAM("[DEFAULT CONTROLLER] AgentID:" << m_agentID);

    CrazyflieContext new_context;

    m_centralManager.waitForExistence(ros::Duration(-1));

    if(m_centralManager.call(contextCall)) {
        new_context = contextCall.response.crazyflieContext;
        //ROS_INFO_STREAM("[DEFAULT CONTROLLER] CrazyflieContext:\n" << new_context);

        //if((m_context.crazyflieName != "") && (new_context.crazyflieName != m_context.crazyflieName)) //linked crazyflie name changed and it was not empty before
        //{

            // Motors off is done in python script now everytime we disconnect

            // send motors OFF and disconnect before setting m_context = new_context
            // std_msgs::Int32 msg;
            // msg.data = CMD_CRAZYFLY_MOTORS_OFF;
            // commandPublisher.publish(msg);

            // ROS_INFO("[DEFAULT CONTROLLER] CF is now different for this student. Disconnect and turn it off");

            // IntWithHeader msg;
            // msg.shouldCheckForAgentID = false;
            // msg.data = CMD_DISCONNECT;
            // m_crazyRadioCommandPublisher.publish(msg);
        //}

        m_symmetric_area_bounds_x = 0.5 * (new_context.localArea.xmax - new_context.localArea.xmin);
        m_symmetric_area_bounds_y = 0.5 * (new_context.localArea.ymax - new_context.localArea.ymin);
        m_symmetric_area_bounds_z = 0.5 * (new_context.localArea.zmax - new_context.localArea.zmin);
        m_isAvailableContext = true;

    }
    else
    {
    	m_isAvailableContext = false;
        ROS_ERROR("[DEFAULT CONTROLLER] Failed to load context. Waiting for next Save in DB by teacher");
    }
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


// TIMER CALLBACK FOR SENDING THE LOAD YAML REQUEST
void timerCallback_initial_load_yaml(const ros::TimerEvent&)
{
	// Create a node handle to the selected parameter service
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	// Create the service client as a local variable
	ros::ServiceClient requestLoadYamlFilenameServiceClient = nodeHandle_to_own_agent_parameter_service.serviceClient<LoadYamlFromFilename>("requestLoadYamlFilename", false);
	// Create the service call as a local variable
	LoadYamlFromFilename loadYamlFromFilenameCall;
	// Specify the Yaml filename as a string
	loadYamlFromFilenameCall.request.stringWithHeader.data = "DefaultController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForAgentID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyDefaultControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
	// Inform the user
		ROS_ERROR("[DEFAULT CONTROLLER] The request load yaml file service call failed.");
	}
}


// LOADING OF YAML PARAMETERS
void isReadyDefaultControllerYamlCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = false;//heckMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the data
		int parameter_service_to_load_from = msg.data;
		// Initialise a local variable for the namespace
		std::string namespace_to_use;
		// Load from the respective parameter service
		switch(parameter_service_to_load_from)
		{
			// > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
			case LOAD_YAML_FROM_AGENT:
			{
				ROS_INFO("[DEFAULT CONTROLLER] Now fetching the DefaultController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[DEFAULT CONTROLLER] Now fetching the DefaultController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[DEFAULT CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		//fetchDefaultControllerYamlParameters(nodeHandle_to_use);
	}
}


// LOADING OF YAML PARAMETERS
/*void fetchDefaultControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// DefaultController.yaml

	// Add the "DefaultController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "DefaultController");



	// GET THE PARAMETERS:

	// ------------------------------------------------------
	// PARAMTERS FOR THE TAKE-OFF AND LANDING MANOEUVRES

	// Max setpoint change per second
	yaml_max_setpoint_change_per_second_horizontal = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_change_per_second_horizontal");
	yaml_max_setpoint_change_per_second_vertical = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_change_per_second_vertical");
	
	// Max error for z
	yaml_max_setpoint_error_z = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_error_z");

	// Max error for xy
	yaml_max_setpoint_error_xy = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_error_xy");
	
	// Max {roll,pitch} angle request
	yaml_max_roll_pitch_request_degrees = getParameterFloat(nodeHandle_for_paramaters , "max_roll_pitch_request_degrees");

	// Max error for yaw angle
	yaml_max_setpoint_error_yaw_degrees = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_error_yaw_degrees");

	// Theshold for {roll,pitch} angle beyond
	// which the motors are turned off
	yaml_threshold_roll_pitch_for_turn_off_degrees = getParameterFloat(nodeHandle_for_paramaters , "threshold_roll_pitch_for_turn_off_degrees");

	// The thrust for take off spin motors
	yaml_takeoff_spin_motors_thrust = getParameterFloat(nodeHandle_for_paramaters , "takeoff_spin_motors_thrust");
	// The time for the take off spin(-up) motors
	yaml_takoff_spin_motors_time = getParameterFloat(nodeHandle_for_paramaters , "takoff_spin_motors_time");

	// Height change for the take off move-up
	yaml_takeoff_move_up_start_height = getParameterFloat(nodeHandle_for_paramaters , "takeoff_move_up_start_height");
	yaml_takeoff_move_up_end_height = getParameterFloat(nodeHandle_for_paramaters , "takeoff_move_up_end_height");
	// The time for the take off spin motors
	yaml_takoff_move_up_time = getParameterFloat(nodeHandle_for_paramaters , "takoff_move_up_time");

	// Minimum and maximum allowed time for: take off goto setpoint
	yaml_takoff_goto_setpoint_nearby_time = getParameterFloat(nodeHandle_for_paramaters , "takoff_goto_setpoint_nearby_time");
	yaml_takoff_goto_setpoint_max_time    = getParameterFloat(nodeHandle_for_paramaters , "takoff_goto_setpoint_max_time");

	// Box within which to keep the integrator on
	// > Units of [meters]
	// > The box consider is plus/minus this value
	yaml_takoff_integrator_on_box_horizontal = getParameterFloat(nodeHandle_for_paramaters , "takoff_integrator_on_box_horizontal");
	yaml_takoff_integrator_on_box_vertical   = getParameterFloat(nodeHandle_for_paramaters , "takoff_integrator_on_box_vertical");
	// The time for: take off integrator-on
	yaml_takoff_integrator_on_time = getParameterFloat(nodeHandle_for_paramaters , "takoff_integrator_on_time");

	// Height change for the landing move-down
	yaml_landing_move_down_end_height_setpoint  = getParameterFloat(nodeHandle_for_paramaters , "landing_move_down_end_height_setpoint");
	yaml_landing_move_down_end_height_threshold = getParameterFloat(nodeHandle_for_paramaters , "landing_move_down_end_height_threshold");
	// The time for: landing move-down
	yaml_landing_move_down_time_max = getParameterFloat(nodeHandle_for_paramaters , "landing_move_down_time_max");

	// The thrust for landing spin motors
	yaml_landing_spin_motors_thrust = getParameterFloat(nodeHandle_for_paramaters , "landing_spin_motors_thrust");
	// The time for: landing spin motors
	yaml_landing_spin_motors_time = getParameterFloat(nodeHandle_for_paramaters , "landing_spin_motors_time");




	// ------------------------------------------------------
	// PARAMTERS THAT ARE STANDARD FOR A "CONTROLLER SERVICE"

	// The mass of the crazyflie, in [grams]
	yaml_cf_mass_in_grams = getParameterFloat(nodeHandle_for_paramaters , "mass");

	// > The frequency at which the "computeControlOutput" function
	//   is being called, as determined by the frequency at which
	//   the Motion Caption (Vicon) system provides pose data, i.e.,
	//   measurement of (x,y,z) position and (roll,pitch,yaw) attitude.
	yaml_control_frequency = getParameterFloat(nodeHandle_for_paramaters, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit
	//   motor command to thrust force in Newtons
	getParameterFloatVector(nodeHandle_for_paramaters, "motorPoly", yaml_motorPoly, 3);

	// The min and max for saturating 16 bit thrust commands
	yaml_command_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	yaml_command_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");

	// The default setpoint, the ordering is (x,y,z,yaw),
	// with unit [meters,meters,meters,radians]
	getParameterFloatVector(nodeHandle_for_paramaters, "default_setpoint", yaml_default_setpoint, 4);

	// Boolean indiciating whether the "Debug Message" of this agent should be published or not
	yaml_shouldPublishDebugMessage = getParameterBool(nodeHandle_for_paramaters, "shouldPublishDebugMessage");

	// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
	yaml_shouldDisplayDebugInfo = getParameterBool(nodeHandle_for_paramaters, "shouldDisplayDebugInfo");
	
	// > A flag for which controller to use:
	yaml_controller_method = getParameterInt( nodeHandle_for_paramaters , "controller_method" );	

	// The LQR Controller parameters for z-height
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixThrust_2StateVector", yaml_gainMatrixThrust_2StateVector, 2);
	// The LQR Controller parameters for "CONTROLLER_MODE_LQR_RATE"
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollRate_3StateVector",   yaml_gainMatrixRollRate_3StateVector,  3);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchRate_3StateVector",  yaml_gainMatrixPitchRate_3StateVector, 3);
	// The LQR Controller parameters for "CONTROLLER_MODE_LQR_RATE"
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollAngle_2StateVector",   yaml_gainMatrixRollAngle_2StateVector,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchAngle_2StateVector",  yaml_gainMatrixPitchAngle_2StateVector, 2);
	yaml_gainRollRate_fromAngle  = getParameterFloat(nodeHandle_for_paramaters, "gainRollRate_fromAngle");
	yaml_gainPitchRate_fromAngle = getParameterFloat(nodeHandle_for_paramaters, "gainPitchRate_fromAngle");
	// The LQR Controller parameters for yaw
	yaml_gainYawRate_fromAngle = getParameterFloat(nodeHandle_for_paramaters, "gainYawRate_fromAngle");
	// Integrator gains
	yaml_integratorGain_forThrust = getParameterFloat(nodeHandle_for_paramaters, "integratorGain_forThrust");
	yaml_integratorGain_forTauXY  = getParameterFloat(nodeHandle_for_paramaters, "integratorGain_forTauXY");
	yaml_integratorGain_forTauYaw = getParameterFloat(nodeHandle_for_paramaters, "integratorGain_forTauYaw");

	
	// A flag for which estimator to use:
	yaml_estimator_method = getParameterInt( nodeHandle_for_paramaters , "estimator_method" );
	
	// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
	// > For the (x,y,z) position
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row1_for_positions",  yaml_PMKF_Ahat_row1_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row2_for_positions",  yaml_PMKF_Ahat_row2_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Kinf_for_positions"     ,  yaml_PMKF_Kinf_for_positions     ,  2);



	// > DEBUGGING: Print out one of the parameters that was loaded to
	//   debug if the fetching of parameters worked correctly
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] DEBUGGING: the fetched DefaultController/mass = " << yaml_cf_mass_in_grams);



	// PROCESS THE PARAMTERS

	// > Compute the feed-forward force that we need to counteract
	//   gravity (i.e., mg) in units of [Newtons]
	m_cf_weight_in_newtons = yaml_cf_mass_in_grams * 9.81/1000.0;

	// > Conver the control frequency to a delta T
	m_control_deltaT = 1.0 / yaml_control_frequency;

	// Max error for yaw angle
	yaml_max_setpoint_error_yaw_radians = DEG2RAD * yaml_max_setpoint_error_yaw_degrees;

	// Max {roll,pitch} angle request
	yaml_max_roll_pitch_request_radians = DEG2RAD * yaml_max_roll_pitch_request_degrees;

	// Theshold for {roll,pitch} angle beyond
	// which the motors are turned off
	yaml_threshold_roll_pitch_for_turn_off_radians = DEG2RAD * yaml_threshold_roll_pitch_for_turn_off_degrees;

	// DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] DEBUGGING: thus the weight of this agent in [Newtons] = " << m_cf_weight_in_newtons);
}
*/




//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------


int main(int argc, char* argv[])
{
	// Starting the ROS-node
	ros::init(argc, argv, "DefaultControllerService");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "DefaultControllerService" node
	std::string m_namespace = ros::this_node::getNamespace();
	m_namespace = "DefaultControllerService";
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



	// AGENT ID AND COORDINATOR ID

	// NOTES:
	// > If you look at the "Agent.launch" file in the "launch" folder,
	//   you will see the following line of code:
	//   <param name="agentID" value="$(optenv ROS_NAMESPACE)" />
	//   This line of code adds a parameter named "agentID" to the
	//   "FlyingAgentClient" node.
	// > Thus, to get access to this "agentID" paremeter, we first
	//   need to get a handle to the "FlyingAgentClient" node within which this
	//   controller service is nested.


	// Get the ID of the agent and its coordinator
	/*bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/FlyingAgentClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[DEFAULT CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
	}

*/

	// PARAMETER SERVICE NAMESPACE AND NODEHANDLES:

	// NOTES:
	// > The parameters that are specified thorugh the *.yaml files
	//   are managed by a separate node called the "Parameter Service"
	// > A separate node is used for reasons of speed and generality
	// > To allow for a distirbuted architecture, there are two
	//   "ParamterService" nodes that are relevant:
	//   1) the one that is nested under the "m_agentID" namespace
	//   2) the one that is nested under the "m_coordID" namespace
	// > The following lines of code create the namespace (as strings)
	//   to there two relevant "ParameterService" nodes.
	// > The node handles are also created because they are needed
	//   for the ROS Subscriptions that follow.

	// Set the class variable "m_namespace_to_own_agent_parameter_service",
	// i.e., the namespace of parameter service for this agent
	m_namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";

	// Set the class variable "m_namespace_to_coordinator_parameter_service",
	// i.e., the namespace of parameter service for this agent's coordinator
	//constructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

	// Inform the user of what namespaces are being used
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	//ROS_INFO_STREAM("[DEFAULT CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	//ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	//ros::Subscriber safeContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "DefaultController", 1, isReadyDefaultControllerYamlCallback);
	//ros::Subscriber safeContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("DefaultController", 1, isReadyDefaultControllerYamlCallback);

	//fetchDefaultControllerYamlParameters(nodeHandle_to_own_agent_parameter_service);


	// GIVE YAML VARIABLES AN INITIAL VALUE

	// This can be done either here or as part of declaring the
	// variables in the header file



	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// The yaml files for the controllers are not added to
	// "Parameter Service" as part of launching.
	// The process for loading the yaml parameters is to send a
	// service call containing the filename of the *.yaml file,
	// and then a message will be received on the above subscribers
	// when the paramters are ready.
	// > NOTE IMPORTANTLY that by using a service client
	//   we stall the availability of this node until the
	//   paramter service is ready
	// > NOTE FURTHER that calling on the service directly from here
	//   often means the yaml file is not actually loaded. So we
	//   instead use a timer to delay the loading

	// Create a single-shot timer
	//ros::Timer timer_initial_load_yaml = nodeHandle.createTimer(ros::Duration(3.0), timerCallback_initial_load_yaml, true);
	//timer_initial_load_yaml.start();
	




	// PUBLISHERS AND SUBSCRIBERS

	// Instantiate the class variable "m_debugPublisher" to be a
	// "ros::Publisher". This variable advertises under the name
	// "DebugTopic" and is a message with the structure defined
	//  in the file "DebugMsg.msg" (located in the "msg" folder).
	//m_debugPublisher = nodeHandle.advertise<DebugMsg>("DebugTopic", 1);

	// Instantiate the local variable "requestSetpointChangeSubscriber"
	// to be a "ros::Subscriber" type variable that subscribes to the
	// "RequestSetpointChange" topic and calls the class function
	// "requestSetpointChangeCallback" each time a messaged is received
	// on this topic and the message is passed as an input argument to
	// the callback function. This subscriber will mainly receive
	// messages from the "flying agent GUI" when the setpoint is changed
	// by the user.
	ros::Subscriber requestSetpointChangeSubscriber = nodeHandle.subscribe("RequestSetpointChange", 1, requestSetpointChangeCallback);

	ros::Subscriber requestState = nodeHandle.subscribe("requestStateChange", 1, requestStateChangeCallback);


	// Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	//std::string namespace_to_coordinator;
	//constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	//ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	//ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("DefaultControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Instantiate the class variable "m_setpointChangedPublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "SetpointChanged" and is a message with the structure defined
	// in the file "SetpointWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// field that displays the current setpoint for this controller.
	//m_setpointChangedPublisher = nodeHandle.advertise<SetpointWithHeader>("SetpointChanged", 1);

	// Instantiate the local variable "getCurrentSetpointService" to be
	// a "ros::ServiceServer" type variable that advertises the service
	// called "GetCurrentSetpoint". This service has the input-output
	// behaviour defined in the "GetSetpointService.srv" file (located
	// in the "srv" folder). This service, when called, is provided with
	// an integer (that is essentially ignored), and is expected to respond
	// with the current setpoint of the controller. When a request is made
	// of this service the "getCurrentSetpointCallback" function is called.
	//ros::ServiceServer getCurrentSetpointService = nodeHandle.advertiseService("GetCurrentSetpoint", getCurrentSetpointCallback);



	// Instantiate the local variable "service" to be a "ros::ServiceServer" type
	// variable that advertises the service called "DefaultController". This service has
	// the input-output behaviour defined in the "Controller.srv" file (located in the
	// "srv" folder). This service, when called, is provided with the most recent
	// measurement of the Crazyflie and is expected to respond with the control action
	// that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
	// this is where the "outer loop" controller function starts. When a request is made
	// of this service the "calculateControlOutput" function is called.
	ros::ServiceServer controllerService = nodeHandle.advertiseService("DefaultController", calculateControlOutput);

	// Instantiate the local variable "customCommandSubscriber" to be a "ros::Subscriber"
	// type variable that subscribes to the "GUIButton" topic and calls the class
	// function "customCommandReceivedCallback" each time a messaged is received on this topic
	// and the message received is passed as an input argument to the callback function.
	//ros::Subscriber customCommandReceivedSubscriber = nodeHandle.subscribe("CustomButtonPressed", 1, customCommandReceivedCallback);



	// Instantiate the local variable "service" to be a "ros::ServiceServer"
	// type variable that advertises the service called:
	// >> "RequestManoeuvre"
	// This service has the input-output behaviour defined in the
	// "IntIntService.srv" file (located in the "srv" folder).
	// This service, when called, is provided with what manoeuvre
	// is requested and responds with the duration that menoeuvre
	// will take to perform (in milliseconds)
	//ros::ServiceServer requestManoeuvreService = nodeHandle.advertiseService("RequestManoeuvre", requestManoeuvreCallback);

	// Instantiate the class variable "m_manoeuvreCompletePublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "ManoeuvreComplete" and is a message with the structure defined
	// in the file "IntWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// flying state once the manoeuvre is complete.
	//m_manoeuvreCompletePublisher = nodeHandle.advertise<IntWithHeader>("ManoeuvreComplete", 1);


	// Instantiate the class variable "m_motorsOffToFlyingAgentClientPublisher"
	// to be a "ros::Publisher". This variable advertises under the
	// name space:
	// "FlyingAgentClient/Command"
	// meaning that it is mimicing messages send by the "Fying
	// Agent Client" node. The message sent has the structure
	// defined in the file "IntWithHeader.msg".
	// > First create a node handle to the base namespace
	//   i.e., the namespace: "/dfall/agentXXX/"
    //ros::NodeHandle base_nodeHandle(m_namespace);
    // > Now instantiate the publisher
	//m_motorsOffToFlyingAgentClientPublisher = base_nodeHandle.advertise<dfall_pkg::IntWithHeader>("FlyingAgentClient/Command", 1);



	// CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
	//ros::NodeHandle nodeHandle_dfall_root("/crazy");

	// LOADING OF THIS AGENT'S CONTEXT
	// Service cleint for loading the allocated flying
	// zone and other context details
	//ros::service::waitForService("/CentralManagerService/CentralManager");
	//m_centralManager = nodeHandle_dfall_root.serviceClient<CMQuery>("CentralManagerService/Query", false);
	// Call the class function that uses this service
	// client to load the context
	//loadCrazyflieContext();
	// Subscriber for when the Flying Zone Database changed
	//ros::Subscriber databaseChangedSubscriber = nodeHandle_dfall_root.subscribe("CentralManagerService/DBChanged", 1, crazyflieContextDatabaseChangedCallback);



	// Print out some information to the user.
	ROS_INFO("[DEFAULT CONTROLLER] Service ready :-)");

	// Enter an endless while loop to keep the node alive.
	ros::spin();

	// Return zero if the "ross::spin" is cancelled.
	return 0;
}
