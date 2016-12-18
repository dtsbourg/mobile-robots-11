#include "calibration_gr11.h"
#include "speed_regulation_gr11.h"
#include "odometry_gr11.h"
#include "useful_gr11.h"
#include "init_pos_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

#define CALIB_SPEED 10.0 ///< Robot wheel speed for calibration steps
#define WAIT_TIME   1.0  ///< Time to wait to ensure contact

#define WALL_X 		0.94 ///< Y coordinates of the teams base wall
#define WALL_Y 		1.44 ///< Y coordinates of the teams base wall

// Calibration FSM States
enum {
		CALIB_START,
		CALIB_STATE_A,
		CALIB_STATE_B,
		CALIB_STATE_C,
		CALIB_STATE_D,
		CALIB_STATE_E,
		CALIB_STATE_F,
		CALIB_STATE_G,
		CALIB_STATE_H,
		CALIB_FINISH
	};

/*! \brief calibration of the robot to calibrate its position
 *
 * \param[in,out] cvs controller main structure
 *
 * This FSM can be adapted, depending on the map and on the robots initial position.
 */
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	int team_id;
	double t;

	CtrlIn *inputs;
	RobotCalibration *calib;
	RobotPosition *rob_pos;

	// variables initialization
	inputs = cvs->inputs;
	calib  = cvs->calib;

	rob_pos = cvs->rob_pos;

	t = inputs->t;
	team_id = cvs->team_id;

	// finite state machine (FSM)
	switch (calib->flag)
	{
		case CALIB_START: // start calibration
			speed_regulation(cvs, 0.0, 0.0);

			calib->flag = CALIB_STATE_A; // directly go to state A
			calib->t_flag = t;
			break;

		// ----- Calibration State A ----- //
		/* - move backward to the top wall of the map
		 * - detect wall with micro switch and enter state B
		*/
		case CALIB_STATE_A: // state A:
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);

			if (inputs->u_switch[R_ID] && inputs->u_switch[L_ID])
			{   // Touched the wall
				calib->flag   = CALIB_STATE_B;
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State B ----- //
		/* - wait 1 second to make sure there is contact
		 * - set angle to -90(deg) / set Y position to 1.44[m] (top of the map minus wall)
		*/
		case CALIB_STATE_B: // state B
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);

			if (t - calib->t_flag > WAIT_TIME)
			{
				rob_pos->y     = WALL_Y;
				rob_pos->theta = -M_PI * 0.5;

				calib->flag   = CALIB_STATE_C;
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State C ----- //
		/* - move forward 1 seconds to center the robot
		*/
		case CALIB_STATE_C: // state C
			speed_regulation(cvs, CALIB_SPEED, CALIB_SPEED);

			if (t - calib->t_flag > WAIT_TIME)
			{
				calib->flag = CALIB_STATE_D;
			}
			break;

		// ----- Calibration State D ----- //
		/* - move in a round until 90° -> we want to put the robot in a -180° angle
		*/
		case CALIB_STATE_D:
			speed_regulation(cvs, -CALIB_SPEED, CALIB_SPEED);

			if (rob_pos->theta <= - M_PI)
			{
				calib->flag = CALIB_STATE_E;
			}
			break;

		// ----- Calibration State E ----- //
		/* - move backward to the top wall of the map
		 * - detect wall with micro switch and enter state F
		*/
		case CALIB_STATE_E: // state E:
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);

			if (inputs->u_switch[R_ID] && inputs->u_switch[L_ID])
			{	// Touched the wall
				calib->flag   = CALIB_STATE_F;
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State F ----- //
		/* - wait 1 seconds to make sure there is contact
		 * - set angle to -180(deg) / set X position to 0.94[m] (right of the map minus wall)
		*/
		case CALIB_STATE_F: // state F
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);

			if (t - calib->t_flag > WAIT_TIME)
			{
				rob_pos->x     = WALL_X;
				rob_pos->theta = -M_PI;

				calib->flag   = CALIB_STATE_G;
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State G ----- //
		/* - move forward 1 seconds to center the robot
		*/
		case CALIB_STATE_G: // state G
			speed_regulation(cvs, CALIB_SPEED, CALIB_SPEED);

			if (t - calib->t_flag > WAIT_TIME)
			{
				calib->flag = CALIB_STATE_H;
			}
			break;

		// ----- Calibration State H ----- //
		/* - move in a round until 90° -> we want to put the robot in a -90° angle with 1.0 degree margin
		*/
		case CALIB_STATE_H: // state H
			speed_regulation(cvs, CALIB_SPEED, -CALIB_SPEED);

			if (rob_pos->theta >= (-90.0 - 1.0) * DEG_TO_RAD)
			{
				calib->flag = CALIB_FINISH;
			}
			break;

		case CALIB_FINISH: // wait before the match is starting
			speed_regulation(cvs, 0.0, 0.0);
			cvs->main_state = WAIT_INIT_STATE;
			break;

		default:
			// printf("Error: unknown state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
