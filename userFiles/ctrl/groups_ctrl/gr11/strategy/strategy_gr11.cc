#include "strategy_gr11.h"
#include "path_planning_gr11.h"
#include "speed_regulation_gr11.h"
#include "path_regulation_gr11.h"
#include "init_pos_gr11.h"
#include "opp_pos_gr11.h"
#include "odometry_gr11.h"
#include "useful_gr11.h"
#include <math.h>
#include <stdlib.h>
#include <memory.h>

NAMESPACE_INIT(ctrlGr11);

Cell get_home(CtrlStruct * cvs)
{
	Cell home;
	if (cvs->kal_pos->y > 0)
	{
		switch (cvs->team_id) {
			case TEAM_A:
				home.x = 15; home.y = 25;
				break;
			case TEAM_B:
				home.x = 3; home.y = 25;
				break;
			default:
				printf("Invalid team\n");
				break;
		}
	} else {
		switch (cvs->team_id) {
			case TEAM_A:
				home.x = 1; home.y = 2;
				break;
			case TEAM_B:
				home.x = 15; home.y = 2;
				break;
			default:
				printf("Invalid team\n");
				break;
		}
	}
	return home;
}

float get_target_dist(CtrlStruct *cvs, Target t, Cell robot_pos)
{
	Cell final_pos;
	final_pos.x = t.x;
	final_pos.y = t.y;

	float* distance = (float *)path_planning(robot_pos, final_pos, cvs->map, NULL, true);

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
	strat->tmp_nb_targets = 0;
	strat->targets = (Target *) malloc(N_TARGETS*sizeof(Target));

	Target ts[N_TARGETS];
	ts[0].present = true; ts[0].points = 2; ts[0].x = 10; ts[0].y =  1; ts[0].dist = 0;
	ts[1].present = true; ts[1].points = 1; ts[1].x = 15; ts[1].y =  7; ts[1].dist = 0;
	ts[2].present = true; ts[2].points = 1; ts[2].x =  4; ts[2].y =  7; ts[2].dist = 0;
	ts[3].present = true; ts[3].points = 3; ts[3].x =  0; ts[3].y = 13; ts[3].dist = 0;
	ts[4].present = true; ts[4].points = 2; ts[4].x =  9; ts[4].y = 13; ts[4].dist = 0;
	ts[5].present = true; ts[5].points = 1; ts[5].x =  4; ts[5].y = 19; ts[5].dist = 0;
	ts[6].present = true; ts[6].points = 1; ts[6].x = 15; ts[6].y = 19; ts[6].dist = 0;
	ts[7].present = true; ts[7].points = 2; ts[7].x = 10; ts[7].y = 25; ts[7].dist = 0;

	memcpy(strat->targets, ts, N_TARGETS*sizeof(Target));

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
	CtrlOut *outputs;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;
	outputs = cvs->outputs;

	int cell_x = world_to_map_x(cvs->kal_pos->x);
	int cell_y = world_to_map_y(cvs->kal_pos->y);

	Cell start;
	start.x = cell_x;
	start.y = cell_y;
	Cell objective;

	switch (strat->main_state)
	{
		case STATE_INIT:
			init_strategy(cvs);
			strat->main_state = STATE_LOOKING_CLOSEST_TARGET;
			break;

		case STATE_LOOKING_CLOSEST_TARGET:
			for (int i = 0; i < N_TARGETS; i++)
			{
				strat->targets[i].dist = get_target_dist(cvs, strat->targets[i], start);
			}
			qsort(strat->targets, N_TARGETS, sizeof(Target), dist_compare);

			for (int i = 0; i < N_TARGETS; i++)
			{
				if (strat->targets[i].present)
				{
					strat->current_target = i;
					objective.x = strat->targets[i].x;
					objective.y = strat->targets[i].y;
					cvs->path = (Path *)path_planning(start, objective, cvs->map, cvs->path, false);
					strat->main_state = STATE_MOVING_TO_TARGET;
					break;
				}
				if ( i == (N_TARGETS - 1))
				{
					// no more targets
					if (inputs->nb_targets > 0)
					{
						strat->main_state = STATE_TWO_DISKS;
					}
					else
					{
						strat->main_state = STATE_STRATEGY_FINISH;
					}
				}
			}
			break;

		case STATE_TWO_DISKS:
			objective = get_home(cvs);
			cvs->path = (Path *)path_planning(start , objective, cvs->map, cvs->path, false);
			strat->main_state = STATE_MOVING_HOME;
			break;

		case STATE_MOVING_TO_TARGET:
			if(check_opp_front(cvs))
			{
				cvs->path = NULL;
				follow_path(cvs);
				strat->main_state = STATE_LOOKING_CLOSEST_TARGET;
				break;
			}
			follow_path(cvs);
			if (cvs->path == NULL)
			{
				if (inputs->target_detected)
				{
					strat->tmp_nb_targets = inputs->nb_targets;
					strat->main_state = STATE_PICKUP_TARGET;
				}
				else
				{
					strat->targets[strat->current_target].present = false;
					strat->main_state = STATE_LOOKING_CLOSEST_TARGET;
				}
			}
			break;

		case STATE_MOVING_HOME:
			follow_path(cvs);
			if (cvs->path == NULL)
			{
				// we are home now
				outputs->flag_release = 1; // release target
				strat->main_state = STATE_LOOKING_CLOSEST_TARGET;
			}
			break;

		case STATE_PICKUP_TARGET:
			outputs->flag_release = 0;
			speed_regulation(cvs, 0, 0);
			if (inputs->nb_targets == strat->tmp_nb_targets + 1)
			{
				strat->targets[strat->current_target].present = false;
				if (inputs->nb_targets == 2)
				{
					strat->main_state = STATE_TWO_DISKS;
				}
				else
				{
					strat->main_state = STATE_LOOKING_CLOSEST_TARGET;
				}
			}
			break;

		case STATE_STRATEGY_FINISH:
			speed_regulation(cvs, 0.0, 0.0);
			printf("No more targets we are done\n");
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
