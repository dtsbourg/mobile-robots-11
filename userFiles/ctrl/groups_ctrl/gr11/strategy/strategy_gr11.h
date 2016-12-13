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

typedef struct {
	bool present;
	int points;
	int x;
	int y;
	float dist;
} Target;

/// strategy main structure
typedef struct Strategy
{
	int main_state; ///< main state of the strategy
	Target * targets;
} Strategy;

/// 'main_state' states (adapt with your own states)
enum {
	   STATE_INIT,
	   STATE_EMPTY,
	   STATE_ONE_DISK,
	   STATE_TWO_DISKS
     };

Strategy* init_strategy(CtrlStruct *cvs);
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
