#include "opp_pos_gr11.h"
#include "init_pos_gr11.h"
#include "useful_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

/*! \brief compute the opponents position using the tower
 *
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp;
	int rise_index_1, rise_index_2, fall_index_1, fall_index_2;

	double delta_t;
	double rise_1, rise_2, fall_1, fall_2;

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;

	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	// low pass filter time increment ('delta_t' is the last argument of the 'first_order_filter' function)
	delta_t = inputs->t - opp_pos->last_t;
	opp_pos->last_t = inputs->t;

	// indexes
	rise_index_1 = inputs->rising_index;
	fall_index_1 = inputs->falling_index;

	// rise and fall angles of the first opponent
	rise_1 = inputs->last_rising[rise_index_1];
	fall_1 = inputs->last_falling[fall_index_1];

	// rise and fall angles of the second opponent
	if (nb_opp == 2)
	{
		rise_index_2 = (rise_index_1-1 < 0) ? NB_STORE_EDGE-1 : rise_index_1-1;
		fall_index_2 = (fall_index_1-1 < 0) ? NB_STORE_EDGE-1 : fall_index_1-1;

		rise_2 = inputs->last_rising[rise_index_2];
		fall_2 = inputs->last_falling[fall_index_2];
	}

	// ----- opponents position computation start ----- //

	double beacon_diam = 0.040; // [m]

	// Compute the angle from rising and falling edges
	double mid_angle_1 = (rise_1 - fall_1) * 0.5;
	double mid_angle_2 = (rise_2 - fall_2) * 0.5;

	// Pre-compute trigonometry values for efficiency
	double c_1 = cos(mid_angle_1); double s_1 = sin(mid_angle_1);
	double c_2 = cos(mid_angle_2); double s_2 = sin(mid_angle_2);

	// Compute opponent positions
	opp_pos->x[0] = (0.5 * beacon_diam * c_1 * c_1) / s_1;
	opp_pos->y[0] = 0.5 * beacon_diam * c_1;

	opp_pos->x[1] = (0.5 * beacon_diam * c_2 * c_2) / s_2;
	opp_pos->y[1] = 0.5 * beacon_diam * c_2;

	// ----- opponents position computation end ----- //
}

/*! \brief compute a single opponent position
 *
 * \param[in] last_rise last rise relative angle [rad]
 * \param[in] last_fall last fall relative angle [rad]
 * \param[in] rob_x robot x position [m]
 * \param[in] rob_y robot y position [m]
 * \param[in] rob_theta robot orientation [rad]
 * \param[out] new_x_opp new known x opponent position
 * \param[out] new_y_opp new known y opponent position
 * \return 1 if computation successful, 0 otherwise
 */
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{
	*new_x_opp = 0.0;
	*new_y_opp = 0.0;

	return 1;
}

/*! \brief check if there is an opponent in front of the robot
 *
 * \param[in] cvs controller main structure
 * \return 1 if opponent robot in front of the current robot
 */
int check_opp_front(CtrlStruct *cvs)
{
	// variables declaration
	int i, nb_opp;

	OpponentsPosition *opp_pos;
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return 0;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	for(i=0; i<nb_opp; i++)
	{
		// ----- opponents check computation start ----- //

		// ----- opponents check computation end ----- //
	}

	return 0;
}

NAMESPACE_CLOSE();
