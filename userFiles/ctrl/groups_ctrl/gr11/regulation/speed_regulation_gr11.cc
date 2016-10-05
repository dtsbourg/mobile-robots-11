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
	double Kp = 10.0;
	double Ki = 2.0;

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
	sp_reg->int_error_r += r_err;

	set_plot(sp_reg->int_error_r, "err");
	double l_err = l_sp_ref-l_sp;
	sp_reg->int_error_l += l_err;

	// wheel commands
	outputs->wheel_commands[R_ID] = limit_range(Kp * r_err + Ki * sp_reg->int_error_r * dt, -100.0, 100.0);
	outputs->wheel_commands[L_ID] = limit_range(Kp * l_err + Ki * sp_reg->int_error_l * dt, -100.0, 100.0);

	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
