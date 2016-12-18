#include "strategy_gr11.h"
#include "path_planning_gr11.h"
#include "speed_regulation_gr11.h"
#include "path_regulation_gr11.h"
#include "init_pos_gr11.h"
#include "opp_pos_gr11.h"
#include "odometry_gr11.h"
#include <math.h>
#include <stdlib.h>
#include <memory.h>

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
	Cell initial_pos;
	initial_pos.x = 15;
	initial_pos.y = 24;
	Cell final_pos  ;
	final_pos.x = t.x;
	final_pos.y = t.y;

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
	strat->targets = (Target *) malloc(N_TARGETS*sizeof(Target));
	strat->tmp_targets = (Target *) malloc(N_TARGETS*sizeof(Target));

	Target ts[N_TARGETS];
	ts[0].present = true; ts[0].points = 2; ts[0].x = 10; ts[0].y = 1; ts[0].dist = 0;
	ts[1].present = true; ts[1].points = 1; ts[1].x = 15; ts[1].y = 7; ts[1].dist = 0;
	ts[2].present = true; ts[2].points = 1; ts[2].x = 4; ts[2].y = 7; ts[2].dist = 0;
	ts[3].present = true; ts[3].points = 3; ts[3].x = 0; ts[3].y = 13; ts[3].dist = 0;
	ts[4].present = true; ts[4].points = 2; ts[4].x = 9; ts[4].y = 13; ts[4].dist = 0;
	ts[5].present = true; ts[5].points = 1; ts[5].x = 4; ts[5].y = 19; ts[5].dist = 0;
	ts[6].present = true; ts[6].points = 1; ts[6].x = 15; ts[6].y = 19; ts[6].dist = 0;
	ts[7].present = true; ts[7].points = 2; ts[7].x = 10; ts[7].y = 25; ts[7].dist = 0;


	memcpy(strat->tmp_targets, ts, N_TARGETS*sizeof(Target));

	for (int i = 0; i < N_TARGETS; i++)
	{
		ts[i].dist = get_target_dist(cvs, ts[i]);
	}

	qsort(ts, N_TARGETS, sizeof(Target), dist_compare);

	memcpy(strat->targets, ts, N_TARGETS*sizeof(Target));

	for (int i = 0; i < N_TARGETS; i++)
	{
		printf("dist = %f \n", strat->targets[i].dist);
	}

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


	/*Ici j'ai une idée du tonerre. Plutôt que de prendre le noeud le plus proche,
	on peut prendre le prochain noeud de la liste. Attention tout de même à la fin de la liste,
	lorsqu'il n'y a plus de noeuds.

	Peutêtre l'idéal serait d'avoir le noeud d'avant?
	*/
	int cell_x = (int)round((cvs->rob_pos->x + 0.8) * 10.0);
	int cell_y = (int)round((cvs->rob_pos->y + 1.3) * 10.0);
	

	Cell start;
	start.x = cell_x;
	start.y = cell_y;
	Cell objective;
	objective.x = strat->targets[0].x;
	objective.y = strat->targets[0].y;



	switch (strat->main_state)
	{
		case STATE_INIT:
			init_strategy(cvs);
			strat->main_state = STATE_EMPTY;
			break;

		case STATE_EMPTY:
			// printf("x = %f ; y = %f\n", cvs->rob_pos->x, cvs->rob_pos->y);
			//printf("x_start = %d ; y_start = %d \n", start.x, start.y);
			//printf("x_obj = %d ; y_obj = %d \n", objective.x, objective.y);
			
			//free_path(cvs->path->next);
			cvs->path = NULL;
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
