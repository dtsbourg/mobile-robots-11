#include "speed_regulation_gr11.h"
#include "useful_gr11.h"

NAMESPACE_INIT(ctrlGr11);

// Tuned PID Parameters
#define Kp 5.0
#define Ki 0.1

// Utility
// threhsold for static position hold
#define STATIC_THRESH 0.1
// Max possible speed
#define MAX_SPEED     100.0
// Max integrator error
#define MAX_PID_ERR       5.0

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
	sp_reg->int_error_r += limit_range(r_err * dt, -MAX_PID_ERR, MAX_PID_ERR);

	double l_err = l_sp_ref-l_sp;
	sp_reg->int_error_l += limit_range(l_err * dt, -MAX_PID_ERR, MAX_PID_ERR);

	// wheel commands
	outputs->wheel_commands[R_ID] = limit_range(Kp * r_err + Ki * sp_reg->int_error_r, -MAX_SPEED, MAX_SPEED);
	outputs->wheel_commands[L_ID] = limit_range(Kp * l_err + Ki * sp_reg->int_error_l, -MAX_SPEED, MAX_SPEED);

	// Threshold for static position
	if (r_sp_ref == 0 && r_sp <= STATIC_THRESH)
		outputs->wheel_commands[R_ID] = 0;

	if (l_sp_ref == 0 && l_sp <= STATIC_THRESH)
		outputs->wheel_commands[L_ID] = 0;


	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
