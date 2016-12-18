#include "init_pos_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

/*! \brief set the initial robot position
 *
 * \param[in] robot_id robot ID
 * \param[out] rob_pos robot position structure
 *
 * Adapt these initial positions, depending on the game map.
 */
void set_init_position(int robot_id, RobotPosition *rob_pos)
{
	switch (robot_id)
	{
		case ROBOT_B: // blue robot
			rob_pos->x = BLUE_T1;
			rob_pos->y = BLUE_T2;
			rob_pos->theta = BLUE_R3 * DEG_TO_RAD / 2.0;
			break;

		case ROBOT_R: // red robot
			rob_pos->x = RED_T1;
			rob_pos->y = RED_T2;
			rob_pos->theta = RED_R3 * DEG_TO_RAD / 2.0;
			break;

		case ROBOT_Y: // yellow robot
			rob_pos->x = YELLOW_T1;
			rob_pos->y = YELLOW_T2;
			rob_pos->theta = YELLOW_R3 * DEG_TO_RAD / 2.0;
			break;

		case ROBOT_W: //  white robot
			rob_pos->x = WHITE_T1;
			rob_pos->y = WHITE_T2;
			rob_pos->theta = WHITE_R3 * DEG_TO_RAD / 2.0;
			break;

		default:
			printf("Error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}

	// set init robot position:
	update_grid_pos(rob_pos);

}

/*! \brief update the grid position from the rob_pos
 *
 * \param[in/out] rob_pos robot position structure
 *
 */
 void update_grid_pos(RobotPosition *rob_pos)
 {
 	rob_pos->grid_x = round((rob_pos->x + 0.80) * 10.0);
	rob_pos->grid_y = round((rob_pos->y + 1.30) * 10.0);
	// printf("position: (%d,%d)\n", rob_pos->grid_x, rob_pos->grid_y);
 }

NAMESPACE_CLOSE();
