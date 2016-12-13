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

float get_target_dist(CtrlStruct *cvs, Target t)
{
	Cell initial_pos = { .x =   0, .y =  16 };
	Cell final_pos   = { .x = t.x, .y = t.y };

	printf("x : %d, y: %d\n", final_pos.x, final_pos.y);

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

	for (int i = 0; i < N_TARGETS; i++)
	{
		ts[i].dist = get_target_dist(cvs, ts[i]);
		printf("dist = %f \n", ts[i].dist);
	}

	qsort(ts, N_TARGETS, sizeof(Target), dist_compare);

	for (int i = 0; i < N_TARGETS; i++)
	{
		printf("sorted dist = %f \n", ts[i].dist);
	}

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

	switch (strat->main_state)
	{
		case STATE_INIT:
			init_strategy(cvs);
			strat->main_state = STATE_EMPTY;
			break;

		case STATE_EMPTY:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case STATE_ONE_DISK:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case STATE_TWO_DISKS:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
