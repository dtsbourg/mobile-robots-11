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

	double beacon_radius = 0.020; // [m]
	double beacon_offset = 0.083; // [m]

	// Compute the beacon's arc from rising and falling edges
	double arc_1 = (fall_1 - rise_1) * 0.5;
	double arc_2 = (fall_2 - rise_2) * 0.5;

	// Compute the beacon's angle relative to the robot from rising and falling edges
	double beacon_angle_1 = 0.5 * (fall_1 + rise_1);
	double beacon_angle_2 = 0.5 * (fall_2 + rise_2);

	// Compute relative distance between robot and beacons
	// TODO : Fix tan range
	double dist_1 = beacon_radius / limit_range(tan(arc_1), -10, 10);
	double dist_2 = beacon_radius / limit_range(tan(arc_2), -10, 10);

	// Compute relative positions from sensors
	int beacon_offset_sign_1 = 1; // is -1 if the beacon is behind the robot
	if (fabs(rise_1) <= M_PI * 0.5 && fabs(fall_1) <= M_PI * 0.5) {
		beacon_offset_sign_1 = -1;
	}
	double x_1 = dist_1 * cos(beacon_angle_1) + beacon_offset_sign_1 * beacon_offset;
	double y_1 = dist_1 * sin(beacon_angle_1);

	int beacon_offset_sign_2 = 1; // is -1 if the beacon is behind the robot
	if (fabs(rise_2) <= M_PI * 0.5 && fabs(fall_2) <= M_PI * 0.5) {
		beacon_offset_sign_2 = -1;
	}
	double x_2 = dist_2 * cos(beacon_angle_2) + beacon_offset_sign_2 * beacon_offset;
	double y_2 = dist_2 * sin(beacon_angle_2);

	// Compute opponent positions
	opp_pos->x[0] = first_order_filter(opp_pos->x[0], x_1+rob_pos->x, 0.25, delta_t);
	opp_pos->y[0] = first_order_filter(opp_pos->y[0], y_1+rob_pos->y, 0.25, delta_t) ;

	opp_pos->x[1] = first_order_filter(opp_pos->x[1], x_2+rob_pos->x, 0.25, delta_t);
	opp_pos->y[1] = first_order_filter(opp_pos->y[1], y_2+rob_pos->y, 0.25, delta_t);

	set_plot(opp_pos->x[0], "Expected Yellow x [m] ");
	set_plot(opp_pos->y[0], "Expected Yellow y [m] ");

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
