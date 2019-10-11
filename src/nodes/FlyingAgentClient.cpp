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





// INCLUDE THE HEADER
#include "nodes/FlyingAgentClient.h"


float att[3];
//DEG2RAD = PI/180;

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




//    ----------------------------------------------------------------------------------
//    M   M   OOO   TTTTT  III   OOO   N   N
//    MM MM  O   O    T     I   O   O  NN  N
//    M M M  O   O    T     I   O   O  N N N
//    M   M  O   O    T     I   O   O  N  NN
//    M   M   OOO     T    III   OOO   N   N
//
//     CCCC    A    PPPP   TTTTT  U   U  RRRR   EEEEE
//    C       A A   P   P    T    U   U  R   R  E
//    C      A   A  PPPP     T    U   U  RRRR   EEE
//    C      AAAAA  P        T    U   U  R   R  E
//     CCCC  A   A  P        T     UUU   R   R  EEEEE
//    ----------------------------------------------------------------------------------

// CALLBACK RUN EVERY TIME NEW MOTION CAPTURE DATA IS RECEIVED


void attitudeCallback(const ViconData& viconData)
{
	att[0] = viconData.roll *  DEG2RAD;
	att[1] = -1*viconData.pitch * DEG2RAD;
	att[2] = -1* viconData.yaw * DEG2RAD;
	//att[2] = 0;
}


void viconCallback(const CrazyflieData& crazyflieData)
{
	// NOTE: THIS FUNCTION IS CALL AT THE FREQUENCY OF THE MOTION
	//       CAPTURE SYSTEM. HENCE ANY OPTERATIONS PERFORMED IN
	//       THIS FUNCTION MUST BE NON-BLOCKING.

	// Initialise a counter of consecutive frames of motion
	// capture data where the agent is occuled
	static int number_of_consecutive_occulsions = 0;

	// Initilise a variable for the pose data of this agent
	CrazyflieData poseDataForThisAgent = crazyflieData;
	//ViconData attitudeDataForThisAgent;

	// Extract the pose data for the allocated object from the
	// full motion capture array
	// NOTE: that if the return index is a negative then this
	//       indicates that the pose data was not found.
	//m_poseDataIndex = getPoseDataForObjectNameWithExpectedIndex( viconData, m_context.crazyflieName , m_poseDataIndex , poseDataForThisAgent );

	// Initilise a variable for the pose data of another agent
	//CrazyflieData poseDataForOtherAgent;
	//poseDataForOtherAgent.occluded = true;


	// Only continue if:
	// (1) the pose for this agent was found, and
	// (2) the controllers are actually available
	// (2) the flying state is something other than MOTORS-OFF
	if (1)
	{



		// Only continue if:
		// (1) the agent is NOT occulded
		if(1)
		{
			// Update the flag
			m_isOcculded_mocap_data = false;
			// Reset the "consecutive occulsions" flag
			number_of_consecutive_occulsions = 0;


			// Only continue if:
			// (1) The state is different from MOTORS-OFF
			if (1)
			{

				// Initliase the "Contrller" service call variable
				Controller controllerCall;

				// Fill in the pose data for this agent
				controllerCall.request.ownCrazyflie = poseDataForThisAgent;
				controllerCall.request.partCrazyflie.roll = att[0];
				controllerCall.request.partCrazyflie.pitch = att[1];
				controllerCall.request.partCrazyflie.yaw = att[2];

				// Fill in the pose data of another agaent, if the data
				// is avaialable
				

				// Initialise a local boolean success variable
				bool isSuccessful_controllerCall = false;

				// Call the controller service client
				isSuccessful_controllerCall = m_instant_controller_service_client->call(controllerCall);

				// Ensure success and enforce safety
				if(!isSuccessful_controllerCall)
				{
					// Let the user know that the controller call failed
					ROS_ERROR_STREAM("[FLYING Agent CLIENT] Failed to call controller, valid: " << m_instant_controller_service_client->isValid() << ", exists: " << m_instant_controller_service_client->exists());

					// Switch to the default controller,
					// if it was not already the active controller
					if ( getInstantController() != DEFAULT_CONTROLLER )
					{
						// Set the DEFAULT controller to be active
						setInstantController(DEFAULT_CONTROLLER);
						// Try the controller call
						isSuccessful_controllerCall = m_instant_controller_service_client->call(controllerCall);
						// Inform the user is not successful
						if ( !isSuccessful_controllerCall )
						{
							ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Also failed to call the DEFAULT controller, valid: " << m_instant_controller_service_client->isValid() << ", exists: " << m_instant_controller_service_client->exists());
						}
					}
				}

				// Send the command to the CrazyRadio
				// > IF SUCCESSFUL
				if (isSuccessful_controllerCall)
				{
					m_commandForSendingToCrazyfliePublisher.publish(controllerCall.response.controlOutput);
				}
				// > ELSE SEND ZERO OUTPUT COMMAND
				else
				{
					// Send the command to turn the motors off
                    ROS_ERROR_STREAM("1");
					sendZeroOutputCommandForMotors();
					// And change the state to motor-off
					//requestChangeFlyingStateTo(STATE_MOTORS_OFF);
				}
			}
			else
			{
				// Send the command to turn the motors off
				sendZeroOutputCommandForMotors();
			} // END OF: "if (m_flying_state != STATE_MOTORS_OFF)"
		}
		
	}
	else
	{
		// Send the command to turn the motors off
		sendZeroOutputCommandForMotors();

	} // END OF: "if ( (m_poseDataIndex >= 0) and (m_controllers_avialble) )"

}





/*int getPoseDataForObjectNameWithExpectedIndex(const ViconData& viconData, std::string name , int expected_index , CrazyflieData& pose_to_fill_in)
{
    // Initialise an integer for the index where the object
    // "name" is found
    // > Initialise an negative to indicate not found
    int object_index = -1;

    // Get the length of the "pose data array"
    int length_poseData = viconData.crazyflies.size();

    // If the "expected index" is non-negative and less than
    // the length of the data array, then attempt to check
    // for a name match
    if (
        (0 <= expected_index)
        and
        (expected_index < length_poseData)
    )
    {
        // Check if the names match
        if (viconData.crazyflies[expected_index].crazyflieName == name)
        {
            object_index = expected_index;
        }
    }

    // If not found, then iterate the data array looking
    // for a name match
    if (object_index < 0)
    {
        for( int i=0 ; i<length_poseData ; i++ )
        {    
            // Check if the names match
            if(viconData.crazyflies[i].crazyflieName == name)
            {
                object_index = i;
            }
        }
    }

    // If not found, then retrun, else fill in the pose data
    if (object_index < 0)
    {
        return object_index;
    }
    else
    {
        pose_to_fill_in = viconData.crazyflies[object_index];
        coordinatesToLocal(pose_to_fill_in);
        return object_index;
    }
}*/


/*void coordinatesToLocal(CrazyflieData& cf)
{
	// Get the area into a local variable
    AreaBounds area = m_context.localArea;

    // Shift the X-Y coordinates
    float originX = (area.xmin + area.xmax) / 2.0;
    float originY = (area.ymin + area.ymax) / 2.0;
    cf.x -= originX;
    cf.y -= originY;

    // Shift the Z coordinate
    // change Z origin to zero, i.e., to the table height, zero of global coordinates, instead of middle of the box
    //float originZ = 0.0;
    // float originZ = (area.zmin + area.zmax) / 2.0;
    //cf.z -= originZ;
}*/


/*void timerCallback_mocap_timeout_check(const ros::TimerEvent&)
{
	// Update the flag
	m_isAvailable_mocap_data = false;
	// Inform the user
	ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Motion Capture Data has been unavailable for " << yaml_mocap_timeout_duration << " seconds." );
	// Ensure that the motors are turned off
	if ( !(m_flying_state==STATE_MOTORS_OFF) )
	{
		// Send the command to turn the motors off
        sendZeroOutputCommandForMotors();
        // Update the flying state
        requestChangeFlyingStateTo( STATE_MOTORS_OFF );
	}
}*/


void sendZeroOutputCommandForMotors()
{
	ControlCommand zeroOutput = ControlCommand(); //everything set to zero
    zeroOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS; //set to motor_mode
    m_commandForSendingToCrazyfliePublisher.publish(zeroOutput);
}





//    ----------------------------------------------------------------------------------
//     SSSS    A    FFFFF  EEEEE  TTTTT  Y   Y
//    S       A A   F      E        T     Y Y
//     SSS   A   A  FFF    EEE      T      Y
//        S  AAAAA  F      E        T      Y
//    SSSS   A   A  F      EEEEE    T      Y
//
//     CCCC  H   H  EEEEE   CCCC  K   K   SSSS
//    C      H   H  E      C      K  K   S
//    C      HHHHH  EEE    C      KKK     SSS
//    C      H   H  E      C      K  K       S
//     CCCC  H   H  EEEEE   CCCC  K   K  SSSS
//    ----------------------------------------------------------------------------------

// Checks if crazyflie is within allowed area
bool safetyCheck_on_positionAndTilt(CrazyflieData data)
{
    // Check on the X position
    float symmetric_bound_x = 0.5 * (m_context.localArea.xmax-m_context.localArea.xmin);
    if((data.x < -symmetric_bound_x) or (data.x > symmetric_bound_x))
    {
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] x safety failed");
        return false;
    }
    // Check on the Y position
    float symmetric_bound_y = 0.5 * (m_context.localArea.ymax-m_context.localArea.ymin);
    if((data.y < -symmetric_bound_y) or (data.y > symmetric_bound_y))
    {
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] y safety failed");
        return false;
    }
    // Check on the Z position
    if((data.z < m_context.localArea.zmin) or (data.z > m_context.localArea.zmax))
    {
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] z safety failed");
        return false;
    }

    // Check the title angle (if required)
    // > The tilt anlge between the body frame and inertial frame z-axis is
    //   give by:
    //   tilt angle  =  1 / ( cos(roll)*cos(pitch) )
    // > But this would be too sensitve to a divide by zero error, so instead
    //   we just check if each angle separately exceeds the limit
    if(yaml_isEnabled_strictSafety)
    {
        // Check on the ROLL angle
        if(
            (data.roll > yaml_maxTiltAngle_for_strictSafety_radians)
            or
            (data.roll < -yaml_maxTiltAngle_for_strictSafety_radians)
        )
        {
            ROS_INFO_STREAM("[FLYING AGENT CLIENT] roll too big.");
            return false;
        }
        // Check on the PITCH angle
        if(
            (data.pitch > yaml_maxTiltAngle_for_strictSafety_radians)
            or
            (data.pitch < -yaml_maxTiltAngle_for_strictSafety_radians)
        )
        {
            ROS_INFO_STREAM("[FLYING AGENT CLIENT] pitch too big.");
            return false;
        }
    }

    // If the code makes it to here then all the safety checks passed,
    // Hence return "true"
    return true;
}








//    ----------------------------------------------------------------------------------
//    FFFFF L      Y   Y  III  N   N   GGGG
//    F     L       Y Y    I   NN  N  G
//    FFF   L        Y     I   N N N  G
//    F     L        Y     I   N  NN  G   G
//    F     LLLLL    Y    III  N   N   GGGG
//
//     SSSS  TTTTT    A    TTTTT  EEEEE
//    S        T     A A     T    E
//     SSS     T    A   A    T    EEE
//        S    T    AAAAA    T    E
//    SSSS     T    A   A    T    EEEEE
//    ----------------------------------------------------------------------------------


/*void requestChangeFlyingStateTo(int new_state)
{
    if(m_crazyradio_status != CRAZY_RADIO_STATE_CONNECTED)
    {
        ROS_INFO("[FLYING AGENT CLIENT] Disconnected and trying to change state. State goes to MOTORS OFF");
        m_flying_state = STATE_MOTORS_OFF;
    }
    else
    {
        // Switch between the possible new states
        switch (new_state)
        {
        	case STATE_TAKE_OFF:
        	{
        		requestChangeFlyingStateToTakeOff();
        		break;
        	}

        	case STATE_FLYING:
        	{
        		// This should never be called
        		break;
        	}

        	case STATE_LAND:
        	{
        		requestChangeFlyingStateToLand();
        		break;
        	}

        	case STATE_MOTORS_OFF:
        	{
        		ROS_INFO("[FLYING AGENT CLIENT] Change state to MOTORS OFF");
        		m_flying_state = new_state;
        		break;
        	}

        	default:
        	{
        		ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Request state of " << new_state << " not recognised. Hence changing to MOTORS OFF.");
        		m_flying_state = new_state;
        		break;
        	}
        }

    }

    // Publish a message with the new flying state
    std_msgs::Int32 flying_state_msg;
    flying_state_msg.data = m_flying_state;
    m_flyingStatePublisher.publish(flying_state_msg);
}
*/
/*void requestChangeFlyingStateToTakeOff()
{
	// Only allow taking off from the MOTORS OFF state
	if (m_flying_state != STATE_MOTORS_OFF)
	{
		ROS_ERROR("[FLYING AGENT CLIENT] Request to TAKE OFF was not implemented because the current state is NOT the MOTORS OFF state.");
	}
	else
	{
		// Check that the Motion Capture data is available
		if ( m_isAvailable_mocap_data and !m_isOcculded_mocap_data )
		{
			// Check whether a "controller" take-off should
			// be performed, and that not already in the
			// "take-off" state
			if (
				(yaml_shouldPerfom_takeOff_with_defaultController)
				and
				(m_flying_state != STATE_TAKE_OFF)
			)
			{
				// Call the service offered by the default
				// controller for how long a take-m_instant_controller_service_clientoff will take
				dfall_pkg::IntIntService requestManoeurveCall;
				requestManoeurveCall.request.data = DEFAULT_CONTROLLER_REQUEST_TAKEOFF;
				if(m_defaultController_requestManoeuvre.call(requestManoeurveCall))
				{
					// Extract the duration
					float take_off_duration = float(requestManoeurveCall.response.data) / 1000.0;
					// Start the timer
					// > Stop any previous instance
					m_timer_takeoff_complete.stop();
					// > Set the period again (second argument is reset)
					m_timer_takeoff_complete.setPeriod( ros::Duration(take_off_duration), true);
					// > Start the timer
					m_timer_takeoff_complete.start();
					// Inform the user
					ROS_INFO_STREAM("[FLYING AGENT CLIENT] Changed state to STATE_TAKE_OFF for a duration of " << take_off_duration << " seconds.");
					// Update the class variable
					m_flying_state = STATE_TAKE_OFF;
					// Set the Default controller as the instant controller
					setInstantController(DEFAULT_CONTROLLER);
				}
				else
				{
					// Inform the user
					ROS_INFO("[FLYING AGENT CLIENT] Failed to get take-off duration from Default controller. Switching to MOTORS-OFF.");
					// Update the class variable
					m_flying_state = STATE_MOTORS_OFF;
				}
			}
			// Otherwise, just switch straight to the
			// "flying" state
			else
			{
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Changed state directly to STATE_FLYING");
				// Update the class variable
				m_flying_state = STATE_FLYING;
			}
		}
		else
		{
			// Inform the user of the problem
			if (!m_isAvailable_mocap_data)
			{
				ROS_ERROR("[FLYING AGENT CLIENT] Take-off not possible because the motion capture data is unavailable.");
			}
			if (m_isOcculded_mocap_data)
			{
				ROS_ERROR("[FLYING AGENT CLIENT] Take-off not possible because the object is \"long-term\" occuled.");
			}
		}
	}
}


void requestChangeFlyingStateToLand()
{
	// Only allow landing from the TAKE-OFF and FLYING state
	if (
		(m_flying_state != STATE_TAKE_OFF)
		and
		(m_flying_state != STATE_FLYING)
	)
	{
		ROS_ERROR("[FLYING AGENT CLIENT] Request to LAND was not implemented because the current state is NOT the TAKE-OFF or FLYING state.");
	}
	else
	{
		// Check whether a "controller" take-off should
		// be performed, and that not already in the
		// "land" state
		if (
			(yaml_shouldPerfom_landing_with_defaultController)
			and
			(m_flying_state != STATE_LAND)
		)
		{
			// Call the service offered by the default
			// controller for how long a landing will take
			dfall_pkg::IntIntService requestManoeurveCall;
			requestManoeurveCall.request.data = DEFAULT_CONTROLLER_REQUEST_LANDING;
			if(m_defaultController_requestManoeuvre.call(requestManoeurveCall))
			{
				// Extract the duration
				float land_duration = float(requestManoeurveCall.response.data) / 1000.0;
				// Start the timer
				// > Stop any previous instance
				m_timer_land_complete.stop();
				// > Set the period again (second argument is reset)
				m_timer_land_complete.setPeriod( ros::Duration(land_duration), true);
				// > Start the timer
				m_timer_land_complete.start();
				// Inform the user
				ROS_INFO_STREAM("[FLYING AGENT CLIENT] Changed state to STATE_LAND for a duration of " << land_duration << " seconds.");
				// Update the class variable
				m_flying_state = STATE_LAND;
				// Set the Default controller as the instant controller
				setInstantController(DEFAULT_CONTROLLER);
			}
			else
			{
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Failed to get land duration from Default controller. Switching to MOTORS-OFF.");
				// Update the class variable
				m_flying_state = STATE_MOTORS_OFF;
			}
		}
		// Otherwise, just switch straight to the
		// "motors off" state
		else
		{
			// Inform the user
			ROS_INFO("[FLYING AGENT CLIENT] Changed state directly to STATE_MOTORS_OFF");
			// Update the class variable
			m_flying_state = STATE_MOTORS_OFF;
		}
	}
}


void timerCallback_takeoff_complete(const ros::TimerEvent&)
{
	// Only change to flying if still in the take-off state
	if (m_flying_state == STATE_TAKE_OFF)
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Take-off complete, changed state to STATE_FLYING");
		// Update the class variable
		m_flying_state = STATE_FLYING;
		// Publish a message with the new flying state
		std_msgs::Int32 flying_state_msg;
		flying_state_msg.data = m_flying_state;
		m_flyingStatePublisher.publish(flying_state_msg);
		// Change back to the nominal controller
		setInstantController( m_controller_nominally_selected );
	}
	else
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Take-off duration elapsed but the agent is no longer in STATE_TAKE_OFF.");
	}
}

void timerCallback_land_complete(const ros::TimerEvent&)
{
	// Only change to flying if still in the take-off state
	if (m_flying_state == STATE_LAND)
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Land complete, changed state to STATE_MOTORS_OFF");
		// Update the class variable
		m_flying_state = STATE_MOTORS_OFF;
		// Publish a message with the new flying state
		std_msgs::Int32 flying_state_msg;
		flying_state_msg.data = m_flying_state;
		m_flyingStatePublisher.publish(flying_state_msg);
		// Change back to the nominal controller
		setInstantController( m_controller_nominally_selected );
	}
	else
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Land duration elapsed but the agent is no longer in STATE_LAND.");
	}
}




void defaultControllerManoeuvreCompleteCallback(const IntWithHeader & msg)
{
	// Switch between the cases
	switch (msg.data)
	{
		case DEFAULT_CONTROLLER_TAKEOFF_COMPLETE:
		{
			// Only change to flying if still in the take-off state
			if (m_flying_state == STATE_TAKE_OFF)
			{
				// Stop the timer
				m_timer_takeoff_complete.stop();
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Take-off complete, changed state to STATE_FLYING");
				// Update the class variable
				m_flying_state = STATE_FLYING;
				// Publish a message with the new flying state
				std_msgs::Int32 flying_state_msg;
				flying_state_msg.data = m_flying_state;
				m_flyingStatePublisher.publish(flying_state_msg);
				// Change back to the nominal controller
				setInstantController( m_controller_nominally_selected );
			}
			else
			{
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Received a take-off complete message, BUT the agent is no longer in STATE_TAKE_OFF.");
			}
			break;
		}


		case DEFAULT_CONTROLLER_LANDING_COMPLETE:
		{
			// Only change to flying if still in the take-off state
			if (m_flying_state == STATE_LAND)
			{
				// Stop the timer
				m_timer_land_complete.stop();
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Land complete, changed state to STATE_MOTORS_OFF");
				// Update the class variable
				m_flying_state = STATE_MOTORS_OFF;
				// Publish a message with the new flying state
				std_msgs::Int32 flying_state_msg;
				flying_state_msg.data = m_flying_state;
				m_flyingStatePublisher.publish(flying_state_msg);
				// Change back to the nominal controller
				setInstantController( m_controller_nominally_selected );
			}
			else
			{
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Received a landing complete message, BUT the agent is no longer in STATE_LAND.");
			}
			break;
		}
	}
}



*/








//    ----------------------------------------------------------------------------------
//     SSSS  EEEEE  L      EEEEE   CCCC  TTTTT
//    S      E      L      E      C        T
//     SSS   EEE    L      EEE    C        T
//        S  E      L      E      C        T
//    SSSS   EEEEE  LLLLL  EEEEE   CCCC    T
//
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
//    C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
//    C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
//    C      O   O  N  NN    T    R   R  O   O  L      L      E      R   R
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
//    ----------------------------------------------------------------------------------


// THIS SETS THE CONTROLLER THAT IT ACTUALLY BEING USED
// > During take-off and landing this function is
//   called to switch to the "Default" controller
// > The highlighting in the "Flying Agent GUI" should
//   reflect the instant controller   
void setInstantController(int controller)
{
    // Update the class variable
    m_instant_controller = controller;

    // Point to the correct service client
    switch(controller)
    {
        case DEFAULT_CONTROLLER:
            m_instant_controller_service_client = &m_defaultController;
            break;
        case STUDENT_CONTROLLER:
            m_instant_controller_service_client = &m_studentController;
            break;
        case TUNING_CONTROLLER:
            m_instant_controller_service_client = &m_tuningController;
            break;
        case PICKER_CONTROLLER:
            m_instant_controller_service_client = &m_pickerController;
            break;
        case REMOTE_CONTROLLER:
            m_instant_controller_service_client = &m_remoteController;
            break;
        case TEMPLATE_CONTROLLER:
            m_instant_controller_service_client = &m_templateController;
            break;
        case TESTMOTORS_CONTROLLER:
            m_instant_controller_service_client = &m_defaultController;
            break;
        default:
            break;
    }

    // Publish a message that informs the "Flying Agent
    // GUI" about this update to the instant controller
    std_msgs::Int32 controller_used_msg;
    controller_used_msg.data = controller;
    //m_controllerUsedPublisher.publish(controller_used_msg);
}


// THIS SIMPLY RETRIEVES THE CLASS VARIABLE
int getInstantController()
{
    return m_instant_controller;
}


// THIS SETS THE CONTROLLER THAT IS NOMINALLY SELECTED
// > This is the controller that will be use as the
//   "instant controller" when in the "flying" state.
// > But during take-off and landing, the "Default"
//   controller is used, and this keeps track of which
//   controller to switch to after those phases are
//   complete
void setControllerNominallySelected(int controller)
{
    // Update the class variable
    m_controller_nominally_selected = controller;

    // If in state "MOTORS-OFF" or "FLYING",
    // then the change is instant.
    //if (
    //	(m_flying_state == STATE_MOTORS_OFF)
    //	or
    //	(m_flying_state == STATE_FLYING)
	//)
    {

        setInstantController(controller); 
    }
}


// THIS SIMPLY RETRIEVES THE CLASS VARIABLE
int getControllerNominallySelected()
{
    return m_controller_nominally_selected;
}










//    ----------------------------------------------------------------------------------
//     SSSS  TTTTT    A    TTTTT  EEEEE
//    S        T     A A     T    E
//     SSS     T    A   A    T    EEE
//        S    T    AAAAA    T    E
//    SSSS     T    A   A    T    EEEEE
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//
//    FFFFF  RRRR    OOO   M   M      GGGG  U   U  III
//    F      R   R  O   O  MM MM     G      U   U   I
//    FFF    RRRR   O   O  M M M     G      U   U   I
//    F      R   R  O   O  M   M     G   G  U   U   I
//    F      R   R   OOO   M   M      GGGG   UUU   III
//    ----------------------------------------------------------------------------------



// THE CALLBACK THAT A NEW FLYING STATE WAS REQUESTED
// > These requests come from the "Flying Agent GUI"
// > The options are: {take-off,land,motor-off,controller}
/*void flyingStateRequestCallback(const IntWithHeader & msg) {

    // Check whether the message is relevant
    bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

    // Continue if the message is relevant
    if (isRevelant)
    {
        // Extract the data
        int cmd = msg.data;

        switch(cmd) {
            case CMD_USE_DEFAULT_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_DEFAULT_CONTROLLER Command received");
                setControllerNominallySelected(DEFAULT_CONTROLLER);
                break;

            case CMD_USE_DEMO_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_DEMO_CONTROLLER Command received");
                setControllerNominallySelected(DEMO_CONTROLLER);
                break;

            case CMD_USE_STUDENT_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_STUDENT_CONTROLLER Command received");
                setControllerNominallySelected(STUDENT_CONTROLLER);
                break;

            case CMD_USE_MPC_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_MPC_CONTROLLER Command received");
                setControllerNominallySelected(MPC_CONTROLLER);
                break;

            case CMD_USE_REMOTE_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_REMOTE_CONTROLLER Command received");
                setControllerNominallySelected(REMOTE_CONTROLLER);
                break;

            case CMD_USE_TUNING_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_TUNING_CONTROLLER Command received");
                setControllerNominallySelected(TUNING_CONTROLLER);
                break;

            case CMD_USE_PICKER_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_PICKER_CONTROLLER Command received");
                setControllerNominallySelected(PICKER_CONTROLLER);
                break;

            case CMD_USE_TEMPLATE_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_TEMPLATE_CONTROLLER Command received");
                setControllerNominallySelected(TEMPLATE_CONTROLLER);
                break;


            case CMD_CRAZYFLY_TAKE_OFF:
                ROS_INFO("[FLYING AGENT CLIENT] TAKE_OFF Command received");
                requestChangeFlyingStateTo(STATE_TAKE_OFF);
                break;

            case CMD_CRAZYFLY_LAND:
                ROS_INFO("[FLYING AGENT CLIENT] LAND Command received");
                requestChangeFlyingStateTo(STATE_LAND);
                break;
            case CMD_CRAZYFLY_MOTORS_OFF:
                ROS_INFO("[FLYING AGENT CLIENT] MOTORS_OFF Command received");
                requestChangeFlyingStateTo(STATE_MOTORS_OFF);
                break;


            case CMD_USE_TESTMOTORS_CONTROLLER:
                if (m_flying_state == STATE_MOTORS_OFF)
                {
                    ROS_INFO("[FLYING AGENT CLIENT] USE_TEST_MOTORS_CONTROLLER Command received");
                    setControllerNominallySelected(TESTMOTORS_CONTROLLER);
                }
                else
                {
                    ROS_INFO("[FLYING AGENT CLIENT] USE_TEST_MOTORS_CONTROLLER Command received, but state is not currently STATE_MOTORS_OFF");
                }
                break;


            default:
                ROS_ERROR_STREAM("[FLYING AGENT CLIENT] unexpected command number: " << cmd);
                break;
        }
    }
}








void crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Received message with Crazy Radio Status = " << msg.data );
    m_crazyradio_status = msg.data;
}






void emergencyStopReceivedCallback(const IntWithHeader & msg)
{
    ROS_INFO("[FLYING AGENT CLIENT] Received message to EMERGENCY STOP");
    flyingStateRequestCallback(msg);
}




*/

//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT      SSSS  TTTTT    A    TTTTT  U   U   SSSS
//    G      E        T       S        T     A A     T    U   U  S
//    G      EEE      T        SSS     T    A   A    T    U   U   SSS
//    G   G  E        T           S    T    AAAAA    T    U   U      S
//     GGGG  EEEEE    T       SSSS     T    A   A    T     UUU   SSSS
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K   SSSS
//    C       A A   L      L      B   B   A A   C      K  K   S
//    C      A   A  L      L      BBBB   A   A  C      KKK     SSS
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K       S
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K  SSSS
//    ----------------------------------------------------------------------------------
/*
bool getCurrentFlyingStateServiceCallback(IntIntService::Request &request, IntIntService::Response &response)
{
    // Put the flying state into the response variable
    response.data = m_flying_state;
    // Return
    return true;
}


bool getInstantControllerServiceCallback(IntIntService::Request &request, IntIntService::Response &response)
{
    // Put the flying state into the response variable
    response.data = getInstantController();
    // Return
    return true;
}




//    ----------------------------------------------------------------------------------
//    BBBB     A    TTTTT  TTTTT  EEEEE  RRRR   Y   Y
//    B   B   A A     T      T    E      R   R   Y Y
//    BBBB   A   A    T      T    EEE    RRRR     Y
//    B   B  AAAAA    T      T    E      R   R    Y
//    BBBB   A   A    T      T    EEEEE  R   R    Y
//    ----------------------------------------------------------------------------------

void batteryMonitorStateChangedCallback(std_msgs::Int32 msg)
{
    // Extract the data
    int new_battery_state = msg.data;

    // Take action if changed to low battery state
    if (new_battery_state == BATTERY_STATE_LOW)
    {
        if (m_flying_state != STATE_MOTORS_OFF)
        {
            ROS_INFO("[FLYING AGENT CLIENT] low level battery triggered, now landing.");
            requestChangeFlyingStateTo(STATE_LAND);
        }
        else
        {
            ROS_INFO("[FLYING AGENT CLIENT] low level battery triggered, please turn off the Crazyflie.");
        }
    }
    else if (new_battery_state == BATTERY_STATE_NORMAL)
    {
        ROS_INFO("[FLYING AGENT CLIENT] received message that battery state changed to normal");
    }
    else
    {
        ROS_INFO("[FLYING AGENT CLIENT] received message that battery state changed to something unknown");
    }
    
}
*/












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


// void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg)
// {
//     ROS_INFO("[FLYING AGENT CLIENT] Received message that the Context Database Changed");
//     loadCrazyflieContext();
// }



// void loadCrazyflieContext()
// {
//     CMQuery contextCall;
//     contextCall.request.studentID = m_agentID;
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] AgentID:" << m_agentID);

//     CrazyflieContext new_context;

//     m_centralManager.waitForExistence(ros::Duration(-1));

//     if(m_centralManager.call(contextCall)) {
//         new_context = contextCall.response.crazyflieContext;
//         ROS_INFO_STREAM("[FLYING AGENT CLIENT] CrazyflieContext:\n" << new_context);

//         if((m_context.crazyflieName != "") && (new_context.crazyflieName != m_context.crazyflieName)) //linked crazyflie name changed and it was not empty before
//         {

//             // Motors off is done in python script now everytime we disconnect

//             // send motors OFF and disconnect before setting m_context = new_context
//             // std_msgs::Int32 msg;
//             // msg.data = CMD_CRAZYFLY_MOTORS_OFF;
//             // commandPublisher.publish(msg);

//             ROS_INFO("[FLYING AGENT CLIENT] CF is now different for this student. Disconnect and turn it off");

//             IntWithHeader msg;
//             msg.shouldCheckForAgentID = false;
//             msg.data = CMD_DISCONNECT;
//             m_crazyRadioCommandPublisher.publish(msg);
//         }

//         m_context = new_context;

//         ros::NodeHandle nh("CrazyRadio");
//         nh.setParam("crazyFlieAddress", m_context.crazyflieAddress);
//     }
//     else
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to load context. Waiting for next Save in DB by teacher");
//     }
// }





//    ----------------------------------------------------------------------------------
//     CCCC  RRRR   EEEEE    A    TTTTT  EEEEE
//    C      R   R  E       A A     T    E
//    C      RRRR   EEE    A   A    T    EEE
//    C      R   R  E      AAAAA    T    E
//     CCCC  R   R  EEEEE  A   A    T    EEEEE
//
//     SSSS  EEEEE  RRRR   V   V  III   CCCC  EEEEE
//    S      E      R   R  V   V   I   C      E
//     SSS   EEE    RRRR   V   V   I   C      EEE
//        S  E      R   R   V V    I   C      E
//    SSSS   EEEEE  R   R    V    III   CCCC  EEEEE
//
//     CCCC  L      III  EEEEE  N   N  TTTTT
//    C      L       I   E      NN  N    T
//    C      L       I   EEE    N N N    T
//    C      L       I   E      N  NN    T
//     CCCC  LLLLL  III  EEEEE  N   N    T
//    ----------------------------------------------------------------------------------


// CREATE A "CONTROLLER" TYPE SERVICE CLIENT
// NOTE: that in the "ros::service::createClient" function the
//       second argument is a boolean that specifies whether the 
//       service is persistent or not. In the ROS documentation a
//       persistent connection is described as:
//   "Persistent connections should be used carefully. They greatly
//    improve performance for repeated requests, but they also make
//    your client more fragile to service failures. Clients using
//    persistent connections should implement their own reconnection
//    logic in the event that the persistent connection fails."
void createControllerServiceClientFromParameterName( std::string paramter_name , ros::ServiceClient& serviceClient )
{
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "FlyingAgentClientConfig");

    std::string controllerName;
    if(!nodeHandle.getParam(paramter_name, controllerName))
    {
        ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Failed to get \"" << paramter_name << "\" paramter");
        return;
    }

    serviceClient = ros::service::createClient<Controller>(controllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded service: " << serviceClient.getService() <<  ", valid: " << serviceClient.isValid() << ", exists: " << serviceClient.exists() );
}


void createIntIntServiceClientFromParameterName( std::string paramter_name , ros::ServiceClient& serviceClient )
{
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "FlyingAgentClientConfig");

    std::string controllerName;
    if(!nodeHandle.getParam(paramter_name, controllerName))
    {
        ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Failed to get \"" << paramter_name << "\" paramter");
        return;
    }

    serviceClient = ros::service::createClient<IntIntService>(controllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded service: " << serviceClient.getService() <<  ", valid: " << serviceClient.isValid() << ", exists: " << serviceClient.exists() );
}



void timerCallback_for_createAllcontrollerServiceClients(const ros::TimerEvent&)
{
    // INITIALISE ALL THE CONTROLLER SERVICE CLIENTS
    createControllerServiceClientFromParameterName( "defaultController"  , m_defaultController );
    createControllerServiceClientFromParameterName( "studentController"  , m_studentController );
    createControllerServiceClientFromParameterName( "tuningController"   , m_tuningController );
    createControllerServiceClientFromParameterName( "pickerController"   , m_pickerController );
    createControllerServiceClientFromParameterName( "remoteController"   , m_remoteController );
    createControllerServiceClientFromParameterName( "templateController" , m_templateController );

    createControllerServiceClientFromParameterName( "testMotorsController" , m_testMotorsController );

    // INITIALISE THE SERVICE FOR REQUESTING THE DEFAULT
    // CONTROLLER TO PERFORM MANOEUVRES
    createIntIntServiceClientFromParameterName( "defaultController_requestManoeuvre" , m_defaultController_requestManoeuvre );

    // Check that at least the default controller is available
    // > Setting the flag accordingly
    if (m_defaultController)
    {
        m_controllers_avialble = true;
    }
    else
    {
        m_controllers_avialble = false;
        // Inform the user of the problem
        ROS_ERROR("[FLYING AGENT CLIENT] The default controller service client (and pressumably all other controllers) could NOT be created.");
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


/*void isReadyFlyingAgentClientConfigYamlCallback(const IntWithHeader & msg)
{
    // Check whether the message is relevant
    bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

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
                ROS_INFO("[FLYING AGENT CLIENT] Now fetching the FlyingAgentClientConfig YAML parameter values from this agent.");
                namespace_to_use = m_namespace_to_own_agent_parameter_service;
                break;
            }
            // > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
            case LOAD_YAML_FROM_COORDINATOR:
            {
                ROS_INFO("[FLYING AGENT CLIENT] Now fetching the FlyingAgentClientConfig YAML parameter values from this agent's coordinator.");
                namespace_to_use = m_namespace_to_coordinator_parameter_service;
                break;
            }

            default:
            {
                ROS_ERROR("[FLYING AGENT CLIENT] Paramter service to load from was NOT recognised.");
                namespace_to_use = m_namespace_to_own_agent_parameter_service;
                break;
            }
        }
        // Create a node handle to the selected parameter service
        ros::NodeHandle nodeHandle_to_use(namespace_to_use);
        // Call the function that fetches the parameters
        fetchFlyingAgentClientConfigYamlParameters(nodeHandle_to_use);
    }
}*/



// > Load the paramters from the Client Config YAML file
/*void fetchFlyingAgentClientConfigYamlParameters(ros::NodeHandle& nodeHandle)
{
    // Here we load the parameters that are specified in the file:
    // FlyingAgentClientConfig.yaml

    // Add the "FlyingAgentClientConfig" namespace to the "nodeHandle"
    ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "FlyingAgentClientConfig");

    // Flag for whether to use angle for switching to the Safe Controller
    yaml_isEnabled_strictSafety = getParameterBool(nodeHandle_for_paramaters,"isEnabled_strictSafety");
    yaml_maxTiltAngle_for_strictSafety_degrees = getParameterFloat(nodeHandle_for_paramaters,"maxTiltAngle_for_strictSafety_degrees");

    // Number of consecutive occulsions that will deem the
    // object as "long-term" occuled
	yaml_consecutive_occulsions_threshold = getParameterInt(nodeHandle_for_paramaters,"consecutive_occulsions_threshold");

	// Time out duration after which Mocap is considered unavailable
	yaml_mocap_timeout_duration = getParameterFloat(nodeHandle_for_paramaters,"mocap_timeout_duration");

	// Flag for whether the take-off should be performed
	//  with the default controller
	yaml_shouldPerfom_takeOff_with_defaultController = getParameterBool(nodeHandle_for_paramaters,"shouldPerfom_takeOff_with_defaultController");

	// Flag for whether the landing should be performed
	// with the default controller
	yaml_shouldPerfom_landing_with_defaultController = getParameterBool(nodeHandle_for_paramaters,"shouldPerfom_landing_with_defaultController");
    
    // DEBUGGING: Print out one of the parameters that was loaded
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] DEBUGGING: the fetched FlyingAgentClientConfig/isEnabled_strictSafety = " << yaml_isEnabled_strictSafety);



    // PROCESS THE PARAMTERS
    // Convert from degrees to radians
    yaml_maxTiltAngle_for_strictSafety_radians = DEG2RAD * yaml_maxTiltAngle_for_strictSafety_degrees;

    // DEBUGGING: Print out one of the processed values
    ROS_INFO_STREAM("[FLYING AGENT CLIENT CONTROLLER] DEBUGGING: after processing yaml_maxTiltAngle_for_strictSafety_radians = " << yaml_maxTiltAngle_for_strictSafety_radians);
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
	ros::init(argc, argv, "FlyingAgentClient");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this node
	std::string m_namespace = ros::this_node::getNamespace();
	m_namespace = "FlyingAgentClient";
	ROS_INFO_STREAM("[FLYING AGENT CLIENT] ros::this_node::getNamespace() =  " << m_namespace);



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
	//bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/FlyingAgentClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	//if ( !isValid_IDs )
	//{
	//	ROS_ERROR("[FLYING AGENT CLIENT] Node NOT FUNCTIONING :-)");
	//	ros::spin();
	//}
	//else
	//{
		ROS_INFO_STREAM("[FLYING AGENT CLIENT] loaded agentID = " << 1 << ", and coordID = " << 1);
	//}



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
	m_namespace_to_own_agent_parameter_service =m_namespace + "/ParameterService";

	// Set the class variable "m_namespace_to_coordinator_parameter_service",
	// i.e., the namespace of parameter service for this agent's coordinator
	//constructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

	// Inform the user of what namespaces are being used
	ROS_INFO_STREAM("[FLYING AGENT CLIENT] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	//ROS_INFO_STREAM("[FLYING AGENT CLIENT] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	//ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	//ros::Subscriber flyingAgentClientConfig_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "FlyingAgentClientConfig", 1, isReadyFlyingAgentClientConfigYamlCallback);
	//ros::Subscriber flyingAgentClientConfig_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("FlyingAgentClientConfig", 1, isReadyFlyingAgentClientConfigYamlCallback);


	// GIVE YAML VARIABLES AN INITIAL VALUE

	// This can be done either here or as part of declaring the
	// variables in the header file



	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// Call the class function that loads the parameters
	// from the "FlyingAgentClientConfig.yaml" file.
	// > This is possible because that YAML file is added
	//   to the parameter service when launched via the
	//   "agent.launch" file.
	//fetchFlyingAgentClientConfigYamlParameters(nodeHandle_to_own_agent_parameter_service);





	// INITIALISE ALL THE CONTROLLER SERVICE CLIENTS    
	// > This cannot be done directly here because the other nodes may
	//   be currently unavailable. Hence, we start a timer for a few
	//   seconds and in the call back all the controller service
	//   clients are created.
	m_controllers_avialble = false;
	m_timer_for_createAllControllerServiceClients = nodeHandle.createTimer(ros::Duration(3), timerCallback_for_createAllcontrollerServiceClients, true);



	// INITIALISE THE MOTION CAPTURE TIME-OUT TIMER
	// > And stop it immediately
	//m_isAvailable_mocap_data = false;
	//m_timer_mocap_timeout_check = nodeHandle.createTimer(ros::Duration(yaml_mocap_timeout_duration), timerCallback_mocap_timeout_check, true);
	//m_timer_mocap_timeout_check.stop();



	// INITIALISE THE TAKE-OFF AND LANDING COMPLETE TIMERS
	// > And stop it immediately
	//m_timer_takeoff_complete = nodeHandle.createTimer(ros::Duration(1.0), timerCallback_takeoff_complete, true);
	//m_timer_takeoff_complete.stop();
	//m_timer_land_complete = nodeHandle.createTimer(ros::Duration(1.0), timerCallback_land_complete, true);
	//m_timer_land_complete.stop();



	// INITIALISE THE CRAZY RADIO STATUS
	m_crazyradio_status = CRAZY_RADIO_STATE_DISCONNECTED;



	// INITIALISE THE FLYING STATE AND PUBLISH
	m_flying_state = STATE_MOTORS_OFF;





	// PUBLISHERS, SUBSCRIBERS, AND SERVICE CLIENTS


	// CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
	ros::NodeHandle nodeHandle_crazy_root("/crazy");


	// CREATE A NODE HANDLE TO THE COORDINATOR
	//std::string namespace_to_coordinator;
	//constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	//ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);


	// LOADING OF THIS AGENT'S CONTEXT
	// Service client for loading the allocated flying
	// zone and other context details
	//ros::service::waitForService("/CentralManagerService/CentralManager");
	//m_centralManager = nodeHandle_dfall_root.serviceClient<CMQuery>("CentralManagerService/Query", false);
	// Call the class function that uses this service
	// client to load the context
	//loadCrazyflieContext();
	// Subscriber for when the Flying Zone Database changed
	//ros::Subscriber databaseChangedSubscriber = nodeHandle_dfall_root.subscribe("CentralManagerService/DBChanged", 1, crazyflieContextDatabaseChangedCallback);


	// EMERGENCY STOP OF THE WHOLE "D-FaLL-System"
	//ros::Subscriber emergencyStopSubscriber = nodeHandle_dfall_root.subscribe("emergencyStop", 1, emergencyStopReceivedCallback);

    setControllerNominallySelected(DEFAULT_CONTROLLER);

	// LOCALISATION DATA FROM MOTION CAPTURE SYSTEM
	//keeps 100 messages because otherwise ViconDataPublisher would override the data immediately
	//ros::Subscriber viconSubscriber = nodeHandle_dfall_root.subscribe("ViconDataPublisher/ViconData", 100, viconCallback);
	ros::Subscriber viconSubscriber = nodeHandle_crazy_root.subscribe("/CrazyRadio/fullState",100, viconCallback);
	ros::Subscriber attitudeSubscriber = nodeHandle_crazy_root.subscribe("/CrazyRadio/attitude",100, attitudeCallback);

	// PUBLISHER FOR COMMANDING THE CRAZYFLIE
	// i.e., motorCmd{1,2,3,4}, roll, pitch, yaw, and onboardControllerType
	//ros::Publishers to advertise the control output
	m_commandForSendingToCrazyfliePublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);
    //m_commandForSendingToCrazyfliePublisher = nodeHandle.advertise <ControlCommand>("ControlTestCommand", 1);


	// SUBSCRIBER FOR THE CHANGE STATE COMMANDS
	// i.e., {TAKE-OFF,LAND,MOTORS-OFF,CONTROLLER SELECTION}
	// > for the agent GUI
	//ros::Subscriber commandSubscriber_to_agent = nodeHandle.subscribe("Command", 1, flyingStateRequestCallback);
	// > for the coord GUI
	//ros::Subscriber commandSubscriber_to_coord = nodeHandle_to_coordinator.subscribe("FlyingAgentClient/Command", 1, flyingStateRequestCallback);


	// PUBLISHER FOR THE CRAZYRADIO COMMANDS
	// i.e., {CONNECT,DISCONNECT}
	// This topic lets us use the terminal to communicate with
	// the crazyRadio node even when the GUI is not launched
	//m_crazyRadioCommandPublisher = nodeHandle.advertise<IntWithHeader>("crazyRadioCommand", 1);


	// PUBLISHER FOR THE FLYING STATE
	// Possible states: {MOTORS-OFF,TAKE-OFF,FLYING,LAND}
	// This topic will publish flying state whenever it changes.
	//m_flyingStatePublisher = nodeHandle.advertise<std_msgs::Int32>("flyingState", 1);


	// PUBLISHER FOR THE CONTROLLER CURRENTLY IN USE
	// This publishes the "m_instant_controller" variable
	// to reflect the controller that is actually called
	// when motion capture data is received.
	//m_controllerUsedPublisher = nodeHandle.advertise<std_msgs::Int32>("controllerUsed", 1);


	// SUBSCRIBER FOR CRAZY RADIO STATUS CHANGES
	// crazyradio status. Connected, connecting or disconnected
	//std::string namespace_to_crazy_radio = m_namespace + "/CrazyRadio";
	//ros::NodeHandle nodeHandle_to_crazy_radio(namespace_to_crazy_radio);
	//ros::Subscriber crazyRadioStatusSubscriber = nodeHandle_to_crazy_radio.subscribe("CrazyRadioStatus", 1, crazyRadioStatusCallback);


	// SUBSCRIBER FOR BATTERY STATE CHANGES
	// The battery state change message from the Battery
	// Monitor node
	//std::string namespace_to_battery_monitor = m_namespace + "/BatteryMonitor";
	//ros::NodeHandle nodeHandle_to_battery_monitor(namespace_to_battery_monitor);
	//ros::Subscriber CFBatterySubscriber = nodeHandle_to_battery_monitor.subscribe("ChangedStateTo", 1, batteryMonitorStateChangedCallback);


	// SUBSCRIBER FOR BATTERY STATE CHANGES
	// The battery state change message from the Battery
	// Monitor node
	//std::string namespace_to_default_contoller = m_namespace + "/DefaultControllerService";
	//ros::NodeHandle nodeHandle_to_default_controller(namespace_to_default_contoller);
	//ros::Subscriber ManoeuvreCompleteSubscriber = nodeHandle_to_default_controller.subscribe("ManoeuvreComplete", 1, defaultControllerManoeuvreCompleteCallback);



	// SERVICE SERVER FOR OTHERS TO GET THE CURRENT FLYING STATE
	// Advertise the service that return the "m_flying_state"
	// variable when called upon
	//ros::ServiceServer getCurrentFlyingStateService = nodeHandle.advertiseService("getCurrentFlyingState", getCurrentFlyingStateServiceCallback);


	// SERVICE SERVER FOR OTHERS TO GET THE INSTANT CONTROLLER
	// Advertise the service that return the "m_instant_controller"
	// variable when called upon
	//ros::ServiceServer getInstantControllerService = nodeHandle.advertiseService("getInstantcontroller", getInstantControllerServiceCallback);





	// PUBLISH THE FLYING STATE
	// Ideally the GUI receives this message
	//std_msgs::Int32 flying_state_msg;
	//flying_state_msg.data = m_flying_state;
	//m_flyingStatePublisher.publish(flying_state_msg);


	// SET THE INITIAL CONTROLLER
	// This cannot be done before the publishers because
	// the function also sets the "m_instant_controller"
	// (as the "m_flying_state" is "STATE_MOTORS_OFF")
	// and the function that sets the instant controller
	// publishes a message with the information.
	





	// Print out some information to the user.
    ROS_INFO("[FLYING AGENT CLIENT] Ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}


