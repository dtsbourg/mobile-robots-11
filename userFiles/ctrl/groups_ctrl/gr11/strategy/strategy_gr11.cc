#include "strategy_gr11.h"
#include "path_planning_gr11.h"
#include "speed_regulation_gr11.h"
#include "path_regulation_gr11.h"
#include "init_pos_gr11.h"
#include "opp_pos_gr11.h"
#include "odometry_gr11.h"
#include <math.h>
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr11);


Cell get_home(CtrlStruct * cvs)
{
	Cell home;
	if (cvs->rob_pos->y > 0)
	{
		switch (cvs->team_id) {
			case TEAM_A:
				home.x = 16; home.y = 0;
				break;
			case TEAM_B:
				home.x = 0; home.y = 0;
				break;
			default:
				printf("Invalid team\n");
				break;
		}
	} else {
		switch (cvs->team_id) {
			case TEAM_A:
				home.x = 0; home.y = 26;
				break;
			case TEAM_B:
				home.x = 16; home.y = 26;
				break;
			default:
				printf("Invalid team\n");
				break;
		}
	}
	return home;
}

float get_target_dist(CtrlStruct *cvs, Target t)
{
	Cell initial_pos = { .x =   0, .y =  16 };
	Cell final_pos   = { .x = t.x, .y = t.y };

	float* distance = (float *)path_planning(initial_pos, final_pos, cvs->map, NULL, true);

	return *distance;
}

int dist_compare (void const * ta, void const * tb)
{
   return (((Target *)ta)->dist - ((Target *)tb)->dist);
}

/*! \brief intitialize the strategy structure
 *
 * \return strategy structure initialized
 */
Strategy* init_strategy(CtrlStruct *cvs)
{
	Strategy *strat;

	strat = (Strategy*) malloc(sizeof(Strategy));

	Target ts[N_TARGETS] = {
		{ .present = true, .points = 2, .x = 11, .y =  0, .dist = 0 },
		{ .present = true, .points = 1, .x = 16, .y =  7, .dist = 0 },
		{ .present = true, .points = 1, .x =  7, .y =  7, .dist = 0 },
		{ .present = true, .points = 3, .x =  0, .y = 13, .dist = 0 },
		{ .present = true, .points = 2, .x = 10, .y = 13, .dist = 0 },
		{ .present = true, .points = 1, .x =  4, .y = 19, .dist = 0 },
		{ .present = true, .points = 1, .x = 16, .y = 19, .dist = 0 },
		{ .present = true, .points = 2, .x = 11, .y = 26, .dist = 0 }
	};

	strat->tmp_targets = ts;

	for (int i = 0; i < N_TARGETS; i++)
	{
		ts[i].dist = get_target_dist(cvs, ts[i]);
		// printf("dist = %f \n", ts[i].dist);
	}

	qsort(ts, N_TARGETS, sizeof(Target), dist_compare);

	strat->targets = ts;
	strat->main_state = STATE_EMPTY;

	return strat;
}

/*! \brief release the strategy structure (memory released)
 *
 * \param[out] strat strategy structure to release
 */
void free_strategy(Strategy *strat)
{
	free(strat);
}

/*! \brief startegy during the game
 *
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	CtrlIn *inputs;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;

	int cell_x = (int)(cvs->rob_pos->x / 10.0);
	int cell_y = (int)(cvs->rob_pos->y / 10.0);
	Cell start = { .x = cell_x, .y = cell_y };
	Cell objective = { .x = strat->targets[0].x, .y = strat->targets[0].y };


	switch (strat->main_state)
	{
		case STATE_INIT:
			init_strategy(cvs);
			strat->main_state = STATE_EMPTY;
			break;

		case STATE_EMPTY:
			printf("x_start = %d ; y_start = %d \n", start.x, start.y);
			printf("x_obj = %d ; y_obj = %d \n", objective.x, objective.y);
			cvs->path = (Path *)path_planning(start, objective, cvs->map, cvs->path, false);
			follow_path(cvs);
			break;

		case STATE_ONE_DISK:
			// for (int i = 0; i < N_TARGETS; i++)
			// {
			// 	strat->tmp_targets[i].dist = get_target_dist(cvs, strat->tmp_targets[i]);
			// }
			// qsort(strat->tmp_targets, N_TARGETS, sizeof(Target), dist_compare);
			// objective.x = strat->tmp_targets[0].x; objective.y = strat->tmp_targets[0].y;
			// cvs->path = (Path *)path_planning(start, objective, cvs->map, cvs->path, false);
			// follow_path(cvs);
			break;

		case STATE_TWO_DISKS:
			// objective = get_home(cvs);
			// cvs->path = (Path *)path_planning(start , objective, cvs->map, cvs->path, false);
			// follow_path(cvs);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
