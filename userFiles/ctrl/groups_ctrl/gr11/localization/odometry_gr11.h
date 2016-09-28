/*! 
 * \author Group 11
 * \file odometry_gr11.h
 * \brief odometry of the robot
 */

#ifndef _ODOMETRY_GR11_H_
#define _ODOMETRY_GR11_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr11.h"

NAMESPACE_INIT(ctrlGr11);

void update_odometry(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
