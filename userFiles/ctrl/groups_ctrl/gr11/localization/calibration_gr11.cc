#include "calibration_gr11.h"
#include "speed_regulation_gr11.h"
#include "odometry_gr11.h"
#include "useful_gr11.h"
#include "init_pos_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

#define CALIB_SPEED  10.0 ///< Robot wheel speed for calibration steps
#define WAIT_TIME    1.0  ///< Time to wait to ensure contact

#define WALL_X 		 0.94 ///< X coordinates of the teams base wall
#define SMALL_WALL_X 0.56 ///< X coordinates of the teams base small wall
#define WALL_Y 		 1.44 ///< Y coordinates of the teams base wall

#define ANGLE_OFFSET 0.6

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
	int robot_id;
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
	robot_id = cvs->robot_id;
	
	// finite state machine (FSM)
	switch (calib->flag)
	{
		case CALIB_START: // start calibration
			switch (robot_id)
			{
				case ROBOT_R:
				case ROBOT_W:
					speed_regulation(cvs, 0.0, 0.0);
					calib->flag = CALIB_STATE_A; // directly go to state A
					calib->t_flag = t;
					break;
				case ROBOT_B:
					speed_regulation(cvs, CALIB_SPEED, -CALIB_SPEED); // give a small angle to avoid collision
					if (rob_pos->theta >= 0.0 + ANGLE_OFFSET)
					{
						calib->flag = CALIB_STATE_A; // go to state A after calibration one
						calib->t_flag = t;			
					}
					break;
				case ROBOT_Y:
					speed_regulation(cvs, -CALIB_SPEED, CALIB_SPEED);
					if (rob_pos->theta <= 0.0 - ANGLE_OFFSET)
					{
						calib->flag = CALIB_STATE_A; // go to state A after calibration one
						calib->t_flag = t;			
					}
					break;
			}
			break;

		// ----- Calibration State A ----- //
		case CALIB_STATE_A: // state A:
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);
			if (inputs->u_switch[R_ID] && inputs->u_switch[L_ID])
			{   // Touched the wall
				calib->flag   = CALIB_STATE_B;
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State B ----- //
		case CALIB_STATE_B: // state B
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);
			if (t - calib->t_flag > WAIT_TIME)
			{
				switch (robot_id)
				{
					case ROBOT_R:
						rob_pos->y = WALL_Y;
						rob_pos->theta = -M_PI * 0.5;
						break;
					case ROBOT_B:
					case ROBOT_Y:
						rob_pos->x = SMALL_WALL_X;
						rob_pos->theta = 0.0;		
						break;
					case ROBOT_W:
						rob_pos->y = -WALL_Y;
						rob_pos->theta = M_PI * 0.5;
						break;
				}
				calib->flag = CALIB_STATE_C;
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State C ----- //
		case CALIB_STATE_C: // state C
			speed_regulation(cvs, CALIB_SPEED, CALIB_SPEED);
			if (t - calib->t_flag > 0.6)
			{
				calib->flag = CALIB_STATE_D;
			}
			break;

		// ----- Calibration State D ----- //
		case CALIB_STATE_D:
			
			if (team_id == TEAM_A)
				speed_regulation(cvs, -CALIB_SPEED, CALIB_SPEED);
			else
				speed_regulation(cvs, CALIB_SPEED, -CALIB_SPEED);

			switch (robot_id)
			{
				case ROBOT_R:
					if (rob_pos->theta <= - M_PI)
						calib->flag = CALIB_STATE_E;
					break;

				case ROBOT_W:
					if (rob_pos->theta >= M_PI)
						calib->flag = CALIB_STATE_E;
					break;

				case ROBOT_B:
					if (rob_pos->theta <= - M_PI * 0.5)
						calib->flag = CALIB_STATE_E;
					break;

				case ROBOT_Y:
					if (rob_pos->theta >= M_PI * 0.5)
						calib->flag = CALIB_STATE_E;
					break;
			}
			break;
			

		// ----- Calibration State E ----- //
		case CALIB_STATE_E: // state E:
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);
			if (inputs->u_switch[R_ID] && inputs->u_switch[L_ID])
			{	// Touched the wall
				calib->flag   = CALIB_STATE_F;
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State F ----- //
		case CALIB_STATE_F:
			speed_regulation(cvs, -CALIB_SPEED, -CALIB_SPEED);
			if (t - calib->t_flag > WAIT_TIME)
			{
				switch (robot_id)
				{
					case ROBOT_R:
						rob_pos->x = WALL_X;
						rob_pos->theta = -M_PI;
						calib->flag = CALIB_FINISH;
						break;

					case ROBOT_W:
						rob_pos->x = WALL_X;
						rob_pos->theta = M_PI;
						calib->flag = CALIB_FINISH;
						break;

					case ROBOT_B:
						rob_pos->y = WALL_Y;
						rob_pos->theta = -M_PI * 0.5;
						calib->flag = CALIB_STATE_G;
						break;

					case ROBOT_Y:
						rob_pos->y = -WALL_Y;
						rob_pos->theta = M_PI * 0.5;
						calib->flag = CALIB_STATE_G;
						break;
				}
				calib->t_flag = t;
			}
			break;

		// ----- Calibration State G ----- //
		case CALIB_STATE_G: // state G
			speed_regulation(cvs, CALIB_SPEED, CALIB_SPEED);
			if (t - calib->t_flag > 2 * WAIT_TIME)
				calib->flag = CALIB_FINISH;
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
