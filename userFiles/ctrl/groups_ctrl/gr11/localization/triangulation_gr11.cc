#include "triangulation_gr11.h"
#include "useful_gr11.h"
#include "init_pos_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

// Filter time constant
#define ALPHA 0.05
// Tower is shifted from center of robot
#define TOWER_SHIFT 0.083 // [m]

/*! \brief set the fixed beacons positions, depending on the team
 *
 * \param[in] team_id ID of the team ('TEAM_A' or 'TEAM_B')
 * \param[out] x_beac_1 first beacon x position [m]
 * \param[out] y_beac_1 first beacon y position [m]
 * \param[out] x_beac_2 second beacon x position [m]
 * \param[out] y_beac_2 second beacon y position [m]
 * \param[out] x_beac_3 third beacon x position [m]
 * \param[out] y_beac_3 third beacon y position [m]
 *
 * This function can be adapted, depending on the map.
 */
void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
										 double *x_beac_2, double *y_beac_2,
										 double *x_beac_3, double *y_beac_3)
{
	switch (team_id)
	{
		case TEAM_A:
			*x_beac_1 = 1.062;
			*y_beac_1 = 1.562;

			*x_beac_2 = -1.062;
			*y_beac_2 =  1.562;

			*x_beac_3 =  0.000;
			*y_beac_3 = -1.562;
			break;

		case TEAM_B:
			*x_beac_1 =  1.062;
			*y_beac_1 = -1.562;

			*x_beac_2 = -1.062;
			*y_beac_2 = -1.562;

			*x_beac_3 = 0.000;
			*y_beac_3 = 1.562;
			break;

		default:
			printf("Error unknown team ID (%d) !\n", team_id);
			exit(EXIT_FAILURE);
	}
}

/*! \brief get the index of the best angle prediction
 *
 * \param[in] alpha_predicted angle to reach [rad]
 * \param[in] alpha_a angle computed for A [rad]
 * \param[in] alpha_b angle computed for B [rad]
 * \param[in] alpha_c angle computed for C [rad]
 * \return best index (0, 1, or 2)
 */
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c)
{
	double pred_err_a, pred_err_b, pred_err_c;

	pred_err_a = fabs(limit_angle(alpha_a - alpha_predicted));
	pred_err_b = fabs(limit_angle(alpha_b - alpha_predicted));
	pred_err_c = fabs(limit_angle(alpha_c - alpha_predicted));

	return (pred_err_a < pred_err_b) ? ((pred_err_a < pred_err_c) ? 0 : 2) : ((pred_err_b < pred_err_c) ? 1 : 2);
}

/*! \brief triangulation main algorithm
 *
 * \param[in] cvs controller main structure
 */
void triangulation(CtrlStruct *cvs)
{
	// variables declaration
	RobotPosition *pos_tri, *rob_pos;
	CtrlIn *inputs;

	int alpha_1_index, alpha_2_index, alpha_3_index;
	int rise_index_1, rise_index_2, rise_index_3;
	int fall_index_1, fall_index_2, fall_index_3;

	double dt;

	double alpha_a, alpha_b, alpha_c;
	double alpha_1, alpha_2, alpha_3;
	double alpha_1_predicted, alpha_2_predicted, alpha_3_predicted;
	double x_beac_1, y_beac_1, x_beac_2, y_beac_2, x_beac_3, y_beac_3;

	// variables initialization
	pos_tri = cvs->triang_pos;
	rob_pos = cvs->rob_pos;
	inputs  = cvs->inputs;

	dt = inputs->t - pos_tri->last_t;

	// safety
	if ((inputs->rising_index_fixed < 0) || (inputs->falling_index_fixed < 0))
	{
		return;
	}

	// known positions of the beacons
	fixed_beacon_positions(cvs->team_id, &x_beac_1, &y_beac_1, &x_beac_2, &y_beac_2, &x_beac_3, &y_beac_3);

	// indexes fot the angles detection
	rise_index_1 = inputs->rising_index_fixed;
	rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_1 - 1;
	rise_index_3 = (rise_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_2 - 1;

	fall_index_1 = inputs->falling_index_fixed;
	fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_1 - 1;
	fall_index_3 = (fall_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_2 - 1;

	// beacons angles measured with the laser (to compute)
	alpha_a = avg(inputs->last_rising_fixed[rise_index_1], inputs->last_falling_fixed[fall_index_1]);
	alpha_b = avg(inputs->last_rising_fixed[rise_index_2], inputs->last_falling_fixed[fall_index_2]);
	alpha_c = avg(inputs->last_rising_fixed[rise_index_3], inputs->last_falling_fixed[fall_index_3]);

	// beacons angles predicted thanks to odometry measurements (to compute)
	alpha_1_predicted = limit_angle(atan((y_beac_1 - rob_pos->y)/(x_beac_1 - rob_pos->x)) - rob_pos->theta);
	alpha_2_predicted = limit_angle(M_PI - atan((y_beac_2 - rob_pos->y)/(x_beac_2 - rob_pos->x)) - rob_pos->theta);
	alpha_3_predicted = limit_angle(atan((y_beac_3 - rob_pos->y)/(x_beac_3 - rob_pos->x)) - rob_pos->theta - M_PI);

	// indexes of each beacon
	alpha_1_index = index_predicted(alpha_1_predicted, alpha_a, alpha_b, alpha_c);
	alpha_2_index = index_predicted(alpha_2_predicted, alpha_a, alpha_b, alpha_c);
	alpha_3_index = index_predicted(alpha_3_predicted, alpha_a, alpha_b, alpha_c);

	// safety
	if ((alpha_1_index == alpha_2_index) || (alpha_1_index == alpha_3_index) || (alpha_2_index == alpha_3_index))
	{
		printf("Failed index sanity check\n");
		return;
	}

	// angle of the first beacon
	switch (alpha_1_index)
	{
		case 0: alpha_1 = alpha_a; break;
		case 1: alpha_1 = alpha_b; break;
		case 2: alpha_1 = alpha_c; break;

		default:
			printf("Error: unknown index %d !\n", alpha_1_index);
			exit(EXIT_FAILURE);
	}

	// angle of the second beacon
	switch (alpha_2_index)
	{
		case 0: alpha_2 = alpha_a; break;
		case 1: alpha_2 = alpha_b; break;
		case 2: alpha_2 = alpha_c; break;

		default:
			printf("Error: unknown index %d !\n", alpha_2_index);
			exit(EXIT_FAILURE);
	}

	// angle of the third beacon
	switch (alpha_3_index)
	{
		case 0: alpha_3 = alpha_a; break;
		case 1: alpha_3 = alpha_b; break;
		case 2: alpha_3 = alpha_c; break;

		default:
			printf("Error: unknown index %d !\n", alpha_3_index);
			exit(EXIT_FAILURE);
	}


	// ----- triangulation computation start ----- //

	/*
	 *	We will use the ToTal algorithm proposed by
	 *  Vincent Pierlot and Marc Van Droogenbroeck,
	 *	(INTELSIG, University of Liège, Belgium)
	 *	http://www.telecom.ulg.ac.be/triangulation/
	 */

	// 1. Compute the modified beacon coordinates
	double dx_1 = x_beac_1 - x_beac_2;
	double dy_1 = y_beac_1 - y_beac_2;

	double dx_3 = x_beac_3 - x_beac_2;
	double dy_3 = y_beac_3 - y_beac_2;

	// 2. Compute the three cotangents
	double T_12 = limit_range(cot(alpha_2 - alpha_1), -COT_MAX, COT_MAX);
	double T_23 = limit_range(cot(alpha_3 - alpha_2), -COT_MAX, COT_MAX);
	double T_31 = limit_range((1 - (T_12 * T_23)) / (T_12 + T_23), -COT_MAX, COT_MAX);

	// 3. compute the modified circle center coordinates
	double x_12 = dx_1 + T_12 * dy_1;
	double y_12 = dy_1 - T_12 * dx_1;

	double x_23 = dx_3 - T_23 * dy_3;
	double y_23 = dy_3 + T_23 * dx_3;

	double x_31 = (dx_3 + dx_1) + T_31 * (dy_3 - dy_1);
	double y_31 = (dy_3 + dy_1) - T_31 * (dx_3 - dx_1);

	// 4. Compute k_31
	double k_31 = dx_1 * dx_3 + dy_1 * dy_3 + T_31 * (dx_1 * dy_3 - dx_3 * dy_1);

	// 5. Compute D
	double D = (x_12 - x_23) * (y_23 - y_31) - (y_12 - y_23) * (x_23 - x_31);

	if (fabs(D) < 1e-16) {
		printf("[ERROR] D = 0 in triangulation_gr11.c \n");
		return;
	}

	// 6. Compute the robot position
	double invD = 1. / D;
	double K = k_31 * invD;
	double tower_shift = -TOWER_SHIFT;

	double new_x = x_beac_2 + tower_shift * cos(pos_tri->theta) + K * (y_12 - y_23);
	double new_y = y_beac_2 + tower_shift * sin(pos_tri->theta) + K * (x_23 - x_12);

	pos_tri->x = first_order_filter(pos_tri->x, new_x, ALPHA, dt);
	pos_tri->y = first_order_filter(pos_tri->y, new_y, ALPHA, dt);
	pos_tri->theta = rob_pos->theta;
	pos_tri->last_t = inputs->t;

	// ----- triangulation computation end ----- //
}

NAMESPACE_CLOSE();
