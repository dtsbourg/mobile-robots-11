/*! 
 * \author Group 11
 * \file init_pos_gr11.h
 * \brief initial position of the robot
 */

#ifndef _INIT_POS_GR11_H_
#define _INIT_POS_GR11_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr11.h"
#include "config_file.h"
#include "useful_gr11.h"

NAMESPACE_INIT(ctrlGr11);

/// robot position
typedef struct RobotPosition
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]

	double last_t; ///< last time position was updated

	int grid_x; ///< x position on the grid from top left
	int grid_y; ///< y position on the grid from top left

} RobotPosition;

void set_init_position(int robot_id, RobotPosition *rob_pos);
void update_grid_pos(RobotPosition *rob_pos);

NAMESPACE_CLOSE();

#endif
