/*!
 * \author Group 11
 * \file CtrlStruct_gr11.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR11_H_
#define _CTRL_STRUCT_GR11_H_

#define OBSTACLE_NODE 1
#define FREE_NODE 0

#include "ctrl_io.h"
#include "namespace_ctrl.h"
#include "user_realtime.h"
#include "set_output.h"
#include <stdlib.h>
#include <stdio.h>

NAMESPACE_INIT(ctrlGr11);

/// main states
enum {CALIB_STATE, WAIT_INIT_STATE, RUN_STATE, STOP_END_STATE, NB_MAIN_STATES};

/// robot IDs
enum {ROBOT_B, ROBOT_R, ROBOT_Y, ROBOT_W, NB_ROBOTS};

/// teams
enum {TEAM_A, TEAM_B, NB_TEAMS};

// forward declaration
typedef struct RobotPosition RobotPosition;
typedef struct SpeedRegulation SpeedRegulation;
typedef struct RobotCalibration RobotCalibration;
typedef struct OpponentsPosition OpponentsPosition;
typedef struct Path Path;
typedef struct Node Node;
typedef struct Strategy Strategy;


/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs

	RobotPosition *rob_pos;     ///< robot position coming from the robot odometry
	RobotPosition *triang_pos;  ///< robot position coming from the triangulation
	RobotPosition *kal_pos;     ///< robot position coming from the Kalman filter
	OpponentsPosition *opp_pos; ///< opponents position
	SpeedRegulation *sp_reg;    ///< speed regulation
	RobotCalibration *calib;    ///< calibration
	Path *path;                 ///< path-planning
	Strategy *strat;            ///< strategy

	int main_state; 	///< main state
	int robot_id;  		///< ID of the robot
	int team_id;    	///< ID of the team
	int nb_opp;     	///< number of opponents
	bool map[17][27];	///< map
} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
