/*!
 * \author Group 11
 * \file triangulation_gr11.h
 * \brief triangulation to get the robot abgrXute position
 */

#ifndef _TRIANGULATION_GR11_H_
#define _TRIANGULATION_GR11_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr11.h"

NAMESPACE_INIT(ctrlGr11);

void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
										 double *x_beac_2, double *y_beac_2,
										 double *x_beac_3, double *y_beac_3);
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c);
void triangulation(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
