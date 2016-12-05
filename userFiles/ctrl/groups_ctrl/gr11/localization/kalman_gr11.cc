#include "kalman_gr11.h"
#include "odometry_gr11.h"
#include "triangulation_gr11.h"
#include "useful_gr11.h"

#define X  0
#define Y  1
#define Vx 2
#define Vy 3

#define cov_noise 0.001
#define cov_obs   0.001

NAMESPACE_INIT(ctrlGr11);

void kalman_step(KalmanStruc * ekf, CtrlStruct * cvs);

void unpack_state(double * x, CtrlStruct * cvs)
{
	x[0] = cvs->rob_pos->x;
	x[1] = cvs->rob_pos->y;
	x[2] = cvs->inputs->r_wheel_speed;
	x[3] = cvs->inputs->l_wheel_speed;
}

void unpack_input(double * u, double * wheel_commands)
{
	u[0] = wheel_commands[0];
	u[1] = wheel_commands[1];
}

void unpack_observation(double * y, CtrlStruct * cvs)
{
	y[0] = cvs->triang_pos->x;
	y[1] = cvs->triang_pos->y;
}

/*! \brief follow a given path
 *
 * \param[in,out] cvs controller main structure
 */
void kalman(CtrlStruct *cvs)
{
	static int init = 0;
	static KalmanStruc ekf;

	// variable declaration
	RobotPosition *rob_pos;
	RobotPosition *triang_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	triang_pos = cvs->triang_pos;

	kalman_step(&ekf, cvs);
}

void kalman_step(KalmanStruc * ekf, CtrlStruct * cvs)
{
	// printf("%f -- Starting Kalman filter step ...\n", cvs->inputs->t);
	ekf->dt = 0.001;

	// printf("[EKF] Unpacking state ...\n");
	unpack_state(ekf->x, cvs);
	// printf("[EKF] Unpacking input ...\n");
	unpack_input(ekf->u, cvs->outputs->wheel_commands);

	// 1. Predict
	// printf("[EKF] Starting state prediction ...\n");
	/***
	 * x+ = f(x, u, n) = F_x . x + F_u . u + F_n . n
	 ***/
	// F_x . x
	ekf->x_[X]  = ekf->x[X]  + ekf->x[Vx] * ekf->dt;
	ekf->x_[Y]  = ekf->x[Y]  + ekf->x[Vy] * ekf->dt;
	ekf->x_[Vx] = ekf->x[Vx] + ekf->u[X]  * ekf->dt;
	ekf->x_[Vy] = ekf->x[Vy] + ekf->u[Y]  * ekf->dt;

	// printf("[EKF] Starting covariance prediction ...\n");
	/***
	 * P+ = F_x . P . F_x' + F_n . Q . F_n'
	 ***/
	 /*      cov_p_1     0       cov_p_2       0
	  *		     0    cov_p_1       0       cov_p_2
	  *  P =     0       0       cov_p_3       0
	  *		     0       0          0       cov_p_3
	  */
	  double cov_p_1 = ekf->cov_pred + ekf->cov_pred * ekf->dt * ekf->dt;
	  double cov_p_2 = ekf->cov_pred * ekf->dt;
	  double cov_p_3 = ekf->cov_pred + cov_noise;

	//printf("[EKF] Ended covariance prediction ...\n");

	// 2. Update
	//printf("[EKF] Starting Update ...\n");
	/***
	 * e = h(x+) = H . x+
	 ***/
	 //printf("[EKF] Expected ...\n");
	 unpack_observation(ekf->y, cvs);
	 ekf->z[X] = ekf->y[X] - ekf->x_[X];
	 ekf->z[Y] = ekf->y[Y] - ekf->x_[Y];

	/***
	 * Z = R + H . P . H'
	 ***/
	 //printf("[EKF] Innovation Covariance ...\n");
	 /*     cov_obs+cov_p_1          0             0       0
	  *		       0 		 cov_obs + cov_p_1     0       0
	  * Z =        0                 0          cov_obs    0
	  *            0                 0             0    cov_obs
	  */
	 double cov_z_1 = cov_obs + cov_p_1;

	/***
	 * K = P H' Z^-1
	 ***/
	 //printf("[EKF] Kalman gain ...\n");
	double k = cov_p_1 / cov_z_1;

	/***
	 * x+ = x + K . z
	 ***/
	 //printf("[EKF] Updating state ...\n");
	 ekf->x_[X]  = ekf->x_[X]  + k * ekf->z[X];
	 ekf->x_[Y]  = ekf->x_[Y]  + k * ekf->z[Y];

	/***
	 * P+ = P - K . H . P
	 ***/
	 //printf("[EKF] Updating covariances ...\n");
	 ekf->cov_pred = cov_p_1 * (1 - k);
	//  printf("[EKF] End filter step ...\n");

	set_plot(ekf->x_[0], "X EKF [m]");
	set_output(ekf->x_[0], "x_");
}

NAMESPACE_CLOSE();
