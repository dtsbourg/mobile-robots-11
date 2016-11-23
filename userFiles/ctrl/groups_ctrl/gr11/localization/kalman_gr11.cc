#include "kalman_gr11.h"
#include "odometry_gr11.h"
#include "triangulation_gr11.h"
#include "useful_gr11.h"
#include "matrix_gr11.h"

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

void kalman_init(KalmanStruc * ekf, CtrlStruct * cvs)
{
	printf("Initializing Kalman filter ...\n");
	double dt = cvs->inputs->t - ekf->last_t;
	ekf->dt = dt;

	// Pointers for x, u
	unpack_state(ekf->x, cvs);
	unpack_input(ekf->u, cvs->outputs->wheel_commands);

	// Jacobians const
	double H[N*N] = {
		 1, 0, 0, 0 , \
		 0, 1, 0, 0 , \
		 0, 0, 0, 0 , \
		 0, 0, 0, 0   \
	};

	memcpy(ekf->H, H, N*N*sizeof(double));

	double Ht[N*N] = {
		 1, 0, 0, 0 , \
		 0, 1, 0, 0 , \
		 0, 0, 0, 0 , \
		 0, 0, 0, 0   \
	};

	memcpy(ekf->Ht, Ht, N*N*sizeof(double));

	double F_x[N*N] = {
		 1, 0, dt,  0 , \
		 0, 1,  0, dt , \
		 0, 0,  1,  0 , \
		 0, 0,  0,  1   \
	};

	memcpy(ekf->F_x, F_x, N*N*sizeof(double));

	double F_xt[N*N] = {
		 1,  0,  0,  0 , \
		 0,  1,  0,  0 , \
		 dt, 0,  1,  0 , \
		 0, dt,  0,  1   \
	};

	memcpy(ekf->F_xt, F_xt, N*N*sizeof(double));

	double F_u[N*M] = {
		  0,  0 , \
		  0,  0 , \
		 dt,  0 , \
		  0,  dt  \
	};

	memcpy(ekf->F_u, F_u, M*N*sizeof(double));

	double F_n[N*M] = {
		0, 0 , \
		0, 0 , \
		1, 0 , \
		0, 1   \
	};

	memcpy(ekf->F_n, F_n, M*N*sizeof(double));

	double F_nt[N*M] = {
		0, 0 , 1, 0 , \
		0, 0 , 0, 1   \
	};

	memcpy(ekf->F_nt, F_nt, M*N*sizeof(double));

	mat_addeye(ekf->P, N);
	mat_addeye(ekf->Z, N);
	mat_addeye(ekf->E, N);
	mat_addeye(ekf->Q, N);
	mat_addeye(ekf->R, N);

	printf("Initialized Kalman filter ...\n");

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

	if (init == 0) {
		kalman_init(&ekf, cvs);
		init = 1;
	} else {
		kalman_step(&ekf, cvs);
	}

	set_plot(ekf.x[0], "X ekf");
}

void kalman_step(KalmanStruc * ekf, CtrlStruct * cvs)
{
	printf("%f -- Starting Kalman filter step ...\n", cvs->inputs->t);

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
	mulvec(ekf->F_x, ekf->x, ekf->x_, N, N);
	// F_u . u
	mulvecaccum(ekf->F_u, ekf->u, ekf->x_, M, N);
	//F_n . n , n = 0

	// printf("[EKF] Starting covariance prediction ...\n");
	/***
	 * P+ = F_x . P . F_x' + F_n . Q . F_n'
	 ***/
	// F_x . P
	mulmat(ekf->F_x, ekf->P, ekf->tmp1, N, N, N);
	// F_x . P . F_x'
	mulmat(ekf->tmp1, ekf->F_xt, ekf->P_, N, N, N);
	// F_n . Q
	mulmat(ekf->F_n, ekf->Q, ekf->tmp2, M, N, N);
	// F_n . Q . F_n'
	mulmat(ekf->tmp2, ekf->F_nt, ekf->Q, N, M, N);
	// P+ = F_x . P . F_x' + F_n . Q . F_n'
	accum(ekf->P_, ekf->Q, N);
	//for (int i=0; i<N; i++) { for (int j=0; j<N; j++) {  printf("%f ", ekf->P_[i*N+j]); } printf("\n"); }

	//printf("[EKF] Ended covariance prediction ...\n");

	// 2. Update
	//printf("[EKF] Starting Update ...\n");
	/***
	 * e = h(x+) = H . x+
	 ***/
	 //printf("[EKF] Expected ...\n");
	 mulvec(ekf->H, ekf->x_, ekf->e, N, N);

	/***
	 * E = H . P . H'
	 ***/
	 //printf("[EKF] Expected Covariance ...\n");
	 // H . P
	 mulmat(ekf->H, ekf->P, ekf->tmp1, N, N, N);
	 // H . P . H'
	 mulmat(ekf->tmp1, ekf->Ht, ekf->E, N, N, N);

	/***
	 * z = y - e
	 ***/
	 //printf("[EKF] Innovation ...\n");
	 sub(ekf->y, ekf->e, ekf->z, M);

	/***
	 * Z = R + E
	 ***/
	 //printf("[EKF] Innovation Covariance ...\n");
	 matsum(ekf->R, ekf->E, ekf->Z, N, N);

	/***
	 * K = P H' Z^-1
	 ***/
	 //printf("[EKF] Kalman gain ...\n");
	 mulmat(ekf->P, ekf->Ht, ekf->tmp3, N, N, N);
	 if (cholsl(ekf->Z, ekf->tmp4, ekf->tmp5, N)) {
		 printf("[EKF] ERROR : Could not compute innovation covariance inverse ...\n");
		 return;
	 }
	 mulmat(ekf->tmp3, ekf->tmp4, ekf->K, N, N, N);
	 //printf("[EKF] Kalman gain done ...\n");

	/***
	 * x+ = x + K . z
	 ***/
	 //printf("[EKF] Updating state ...\n");
	 mulvec(ekf->K, ekf->z, ekf->x_, N, N);
	 accum(ekf->x_, ekf->x, N);

	/***
	 * P+ = P - K . H . P
	 ***/
	 //printf("[EKF] Updating covariances ...\n");
	 mulmat(ekf->K, ekf->H, ekf->tmp1, N, N, N);
	 mulmat(ekf->tmp1, ekf->P, ekf->tmp2, N, N, N);
	 matsub(ekf->P, ekf->tmp2, ekf->P_, N, N);
	 printf("[EKF] End filter step ...\n");

	// for (int i=0; i<N; i++) { printf("[EKF] ekf.x_[%d] = %f\n", i, ekf->x_[i]);}

}

NAMESPACE_CLOSE();
