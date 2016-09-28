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

/// Kalman main structure
struct KalmanStruc
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
