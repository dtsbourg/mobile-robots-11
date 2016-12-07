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

	/// State vectors
	double x[N];  //< State (odom)
	double x_[N]; //< Predicted state
	double u[M];  //< State Input
	double y[M];  //< State Observed (tri)
	double z[N];  //< Innovation

	double cov_pred;

} KalmanStruc;

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
