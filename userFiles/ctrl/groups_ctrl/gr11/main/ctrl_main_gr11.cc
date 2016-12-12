/*!
 * \author Group 11
 * \file controller_main_gr11.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr11.h"
#include "namespace_ctrl.h"
#include "init_pos_gr11.h"
#include "odometry_gr11.h"
#include "opp_pos_gr11.h"
#include "speed_regulation_gr11.h"
#include "calibration_gr11.h"
#include "triangulation_gr11.h"
#include "path_planning_gr11.h"
#include "strategy_gr11.h"
#include "kalman_gr11.h"
#include <time.h>

NAMESPACE_INIT(ctrlGr11);

/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;

	// robot ID
	cvs->robot_id = inputs->robot_id;

	// robot team
	switch (inputs->robot_id)
	{
		case ROBOT_B: cvs->team_id = TEAM_A; break;
		case ROBOT_R: cvs->team_id = TEAM_A; break;
		case ROBOT_Y: cvs->team_id = TEAM_B; break;
		case ROBOT_W: cvs->team_id = TEAM_B; break;

		default:
			printf("Error: unknown robot ID: %d !\n", inputs->robot_id);
			exit(EXIT_FAILURE);
	}

	// number of opponents
	cvs->nb_opp = inputs->nb_opponents;

	// robot initial position
	set_init_position(cvs->robot_id, cvs->rob_pos);
	cvs->rob_pos->last_t = t;

	// speed regulation
	cvs->sp_reg->last_t = t;
}

/*! \brief controller loop (called every time-step)
 *
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;
	CtrlOut *outputs;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;

	// time
	t = inputs->t;

	// update the robot odometry
	update_odometry(cvs);

	// triangulation
	triangulation(cvs);

	// opponents position
	opponents_tower(cvs);

	// tower control
	outputs->tower_command = 50.0;

	// Path planning testing
	static bool flag = 0;
	if (!flag)
	{
		flag = 1;
		// initial position
		Cell initial_pos;
		initial_pos.x = 16;
		initial_pos.y = 0;

		FILE *fp;
		fp = fopen("initial.txt", "w");
		fprintf(fp, "%d %d", initial_pos.x, initial_pos.y);
		fclose(fp);

		// goal position
		Cell final_pos;
		final_pos.x = 0;
		final_pos.y = 0;

		fp = fopen("final.txt", "w");
		fprintf(fp, "%d %d", final_pos.x, final_pos.y);
		fclose(fp);

		// fake map
		bool map[4][4] = {0};
		map[1][0] = 1;
		map[1][1] = 1;
		map[2][1] = 1;

		// Algo MIT test
		Path* test = (Path *)path_planning(initial_pos, final_pos, cvs->map, NULL, false);
		Path* tracker = test;

		while (tracker != NULL)
		{
			printf("(%d,%d) -> ", tracker->cell.x, tracker->cell.y);
			tracker = tracker->next;
		}
		printf("NULL\n");

		// Test to return distance:
		float* distance = (float *)path_planning(initial_pos, final_pos, cvs->map, NULL, true);
		printf("Distance: %f\n", (*distance));
		// do not forget to free distance:
		free(distance);
	}

	switch (cvs->main_state)
	{
		// calibration
		case CALIB_STATE:
			kalman(cvs);
			calibration(cvs);
			break;

		// wait before match beginning
		case WAIT_INIT_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			if (t > 0.0)
			{
				cvs->main_state = RUN_STATE;
				cvs->strat->main_state = GAME_STATE_A;
			}
			break;

		// during game
		case RUN_STATE:
			speed_regulation(cvs, 10.0, 11.0);

			main_strategy(cvs);
			if (t > 89.0) // 1 second safety
			{
				cvs->main_state = STOP_END_STATE;
			}
			break;

		// stop at the end of the game
		case STOP_END_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			outputs->flag_release = 1;
			break;

		case NB_MAIN_STATES:
			printf("Error: state NB_MAIN_STATES should not be reached !\n");
			exit(EXIT_FAILURE);
			break;

		default:
			printf("Error:unknown state : %d !\n", cvs->main_state);
			exit(EXIT_FAILURE);
	}
}

/*! \brief last controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

}

NAMESPACE_CLOSE();
