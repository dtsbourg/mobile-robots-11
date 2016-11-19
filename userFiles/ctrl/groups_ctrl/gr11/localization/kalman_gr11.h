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

typedef struct {
	double ** matrix;
	int rows;
	int cols;
} matrix_t;

/// Kalman main structure
struct KalmanStruc
{
	/// State vectors
	double * x; //< State
	double * u; //< State Input
	double * y; //< State Observed
	double * z; //< Innovation

	/// Covariances
	double * P; //< Covariance of prediction error
    double * Q; //< Covariance of process noise
	double * E; //< Covariance of expectation
    double * R; //< Covariance of measurement noise
	double * Z; //< Covariance of innovation


	/// Jacobians
	double * H;		//< Jacobian of measurement model
	double * F_x;	//< Jacobian of state model
	double * F_u;	//< Jacobian of input model
	double * F_n;	//< Jacobian of noise model

	/// Kalman
	double * K; //< Kalman gain

	/// Utility
	double * tmp;
};

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
