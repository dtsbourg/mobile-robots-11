/*!
 * \author Group 11
 * \file matrix_gr11.h
 * \brief Matrix calculus library
 */

#ifndef _MATRIX_GR11_H_
#define _MATRIX_GR11_H_

#include "namespace_ctrl.h"
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr11);

void matcopy(double * res, double * src, int m, int n);
void mulmat(double * a, double * b, double * c, int arows, int acols, int bcols);
void mulvec(double * a, double * x, double * y, int m, int n);
void transpose(double * a, double * at, int m, int n);
void add(double * a, double * b, double * c, int n);
void sub(double * a, double * b, double * c, int n);
void accum(double * a, double * b, int n);
void mulmataccum(double * a, double * b, double * c, int arows, int acols, int bcols);
void mulvecaccum(double * a, double * b, double * c, int m, int n);
void matsum(double * a, double * b, double * c, int m, int n);
void matsub(double * a, double * b, double * c, int m, int n);
void zeros(double * a, int m, int n);
void mat_addeye(double * a, int n);
int choldc1(double * a, double * p, int n);
int choldcsl(double * A, double * a, double * p, int n);
int cholsl(double * A, double * a, double * p, int n);

NAMESPACE_CLOSE();

#endif
