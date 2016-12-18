#include "speed_regulation_gr11.h"
#include "useful_gr11.h"

NAMESPACE_INIT(ctrlGr11);

/*! \brief wheel speed regulation
 *
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	double r_sp, l_sp;
	double dt;
	double Kp = 80.0;
	double Ki = 0.7;
	double threshold = 0.1;		// Threshold for no movement (static position)

	// variables declaration
	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg  = cvs->sp_reg;

	// wheel speeds
	r_sp = inputs->r_wheel_speed;
	l_sp = inputs->l_wheel_speed;

	// time
	dt = inputs->t - sp_reg->last_t; // time interval since last call

	// ----- Wheels regulation computation start ----- //

	double r_err = r_sp_ref-r_sp;
	sp_reg->int_error_r += limit_range(r_err * dt, -5.0, 5.0);

	double l_err = l_sp_ref-l_sp;
	sp_reg->int_error_l += limit_range(l_err * dt, -5.0, 5.0);

	// wheel commands
	outputs->wheel_commands[R_ID] = limit_range(Kp * r_err + Ki * sp_reg->int_error_r, -100.0, 100.0);
	outputs->wheel_commands[L_ID] = limit_range(Kp * l_err + Ki * sp_reg->int_error_l, -100.0, 100.0);

	// Threshold for static position
	if (r_sp_ref == 0 && r_sp <= threshold)
		outputs->wheel_commands[R_ID] = 0;

	if (l_sp_ref == 0 && l_sp <= threshold)
		outputs->wheel_commands[L_ID] = 0;


	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
