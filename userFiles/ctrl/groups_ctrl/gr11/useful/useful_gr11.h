/*!
 * \author Group 11
 * \file useful_gr11.h
 * \brief useful functions to use in the controller
 */

#ifndef _USEFUL_GR11_H_
#define _USEFUL_GR11_H_

#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr11);

#define DEG_TO_RAD (M_PI/180.0) ///< convertion from degrees to radians
#define RAD_TO_DEG (180.0/M_PI) ///< convertion from radians to degrees


double rnd();
double norm_dist(double dx, double dy);
double limit_range(double x, double min, double max);
double limit_angle(double x);
double first_order_filter(double last_val, double new_val, double tau, double delta_t);

NAMESPACE_CLOSE();

#endif
