/*!
 * \author Group 11
 * \file calibration_gr11.h
 * \brief calibration of the robot
 */

#ifndef _CALIBRATION_GR11_H_
#define _CALIBRATION_GR11_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr11.h"

NAMESPACE_INIT(ctrlGr11);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time for flag

	int flag;      ///< flag for calibration

} RobotCalibration;

void calibration(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
