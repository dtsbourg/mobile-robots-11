/*!
 * \author Group 11
 * \file strategy_gr11.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR11_H_
#define _STRATEGY_GR11_H_

#define N_TARGETS 8

#include "CtrlStruct_gr11.h"

NAMESPACE_INIT(ctrlGr11);

/// 'main_state' states
enum {
	   STATE_INIT,
	   STATE_TWO_DISKS,
	   STATE_MOVING_TO_TARGET,
	   STATE_MOVING_HOME,
	   STATE_LOOKING_CLOSEST_TARGET,
	   STATE_PICKUP_TARGET,
	   STATE_STRATEGY_FINISH
     };

enum ScoringMethod {
	SCORE_DISTANCE,
	SCORE_WEIGHTED_DISTANCE,
	SCORE_POINTS
};

/// Target Struct
typedef struct {
	bool present; // Is the target still available
	int points;   // How much is it worth
	int x;		  // position x
	int y;		  // position x
	float dist;   // How far is the target
} Target;

/// strategy main structure
typedef struct Strategy
{
	int main_state; 	///< main state of the strategy
	int tmp_nb_targets;
	int current_target;
	ScoringMethod method;
	Target targets[N_TARGETS];
} Strategy;


void init_strategy(CtrlStruct *cvs);
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
