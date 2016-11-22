/*!
 * \author Group 11
 * \file kalman_gr11.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR11_H_
#define _KALMAN_GR11_H_

#include "CtrlStruct_gr11.h"
#include "init_pos_gr11.h"

NAMESPACE_INIT(ctrlGr11);

#define N 4
#define M 2

/// Kalman main structure
typedef struct
{
	int n; //< number of state values
   	int m; //< number of observables

	double dt;		//< Delta time
	double last_t;  //< Last call time

	/// State vectors
	double x[N];  //< State
	double x_[N]; //< Predicted state
	double e[M];  //< State expectation
	double u[M];  //< State Input
	double y[M];  //< State Observed
	double z[N];  //< Innovation

	/// Covariances
	double P[N*N]; //< Covariance of prediction error
	double P_[N*N]; //< Predicted ovariance of prediction error
    double Q[M*M]; //< Covariance of process noise
	double E[M*M]; //< Covariance of expectation
    double R[M*M]; //< Covariance of measurement noise
	double Z[N*N]; //< Covariance of innovation


	/// Jacobians
	double H[M*N];		//< Jacobian of measurement model
	double F_x[N*N];	//< Jacobian of state model
	double F_u[N*M];	//< Jacobian of input model
	double F_n[N*M];	//< Jacobian of noise model

	/// Kalman
	double K[N*M]; //< Kalman gain

	/// Utility
	double tmp1[N*N];
	double tmp2[N*N];
	double tmp3[N*N];
	double tmp4[N*N];
	double tmp5[N*N];

	double F_xt[N*N]; //< Jacobian of state model transposed
	double F_nt[N*N]; //< Jacobian of noise model transposed
	double Ht[M*N];   //< Jacobian of measurement model transposed
} KalmanStruc;

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
