#include "odometry_gr11.h"
#include "useful_gr11.h"
#include "init_pos_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

/*! \brief update the robot odometry
 *
 * \param[in,out] cvs controller main structure
 *
 *
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	double r_sp, l_sp;
	double dt;

	RobotPosition *rob_pos;
	CtrlIn *inputs;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	r_sp = inputs->r_wheel_speed; // right wheel speed
	l_sp = inputs->l_wheel_speed; // left wheel speed

	// time
	dt = inputs->t - rob_pos->last_t; // time increment since last call

	// safety
	if (dt <= 0.0)
	{
		return;
	}


	// ----- odometry computation start ----- //
   /*
	*  Robot state -- (x,y,theta)
	*  Distances   -- delta_s_r, delta_s_l
	*  Pos update  -- delta_s, delta_theta
	*  Wheel dist  -- b
	*/
	// TODO : Find real constant !!
	const double wheel_rad = 0.030; // [m]
	double delta_s_r = r_sp * dt * wheel_rad;
	double delta_s_l = l_sp * dt * wheel_rad;

   /*
    *  Compute delta state
	*/
	// TODO : Find real constant !!
	const double b = 0.225; // [m]
	double delta_s = (delta_s_r + delta_s_l) / 2.0;

   /*
	*  Compute position deltas
	*/

	double delta_theta  = (delta_s_r - delta_s_l) / b;
	double delta_x 		= delta_s * cos(rob_pos->theta + delta_theta / 2.0);
	double delta_y 		= delta_s * sin(rob_pos->theta + delta_theta / 2.0);

	// ----- odometry computation end ----- //

	// Update position estimation
	rob_pos->x 	  = rob_pos->x + delta_x;
	rob_pos->y 	  = rob_pos->y + delta_y;
	rob_pos->theta = rob_pos->theta + delta_theta;

	// Plot value
	//set_plot(rob_pos->x, "Blue sensor x [m]");
	//set_plot(rob_pos->y, "Odometry Y pos [m]");
	//set_plot(rob_pos->theta, "Odometry theta [rad]");

	// last update time
	rob_pos->last_t = inputs->t;
}

NAMESPACE_CLOSE();
