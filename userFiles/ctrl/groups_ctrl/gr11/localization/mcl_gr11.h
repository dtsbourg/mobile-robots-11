/*! 
 * \author Group 11
 * \file mcl_gr11.h
 * \brief localization using Monte Carlo technic
 */

#ifndef _MCL_GR11_H_
#define _MCL_GR11_H_

#include "CtrlStruct_gr11.h"
#include "init_pos_gr11.h"

NAMESPACE_INIT(ctrlGr11);

/// Markov main structure
struct MCLStruct
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void mcl_update(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
