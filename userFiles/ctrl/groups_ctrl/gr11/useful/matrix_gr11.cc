#include "matrix_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

void matcopy(double * res, double * src, int m, int n)
{
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n ; j++ )
            res[i*n+j] = src[i*j+n];
}

void mulmat(double * a, double * b, double * c, int arows, int acols, int bcols)
{
    int i, j,l;

    for(i=0; i<arows; i++)
        for(j=0; j<bcols; j++) {
            c[i*bcols+j] = 0;
            for(l=0; l<acols; ++l)
                c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j];
        }
}

void mulvec(double * a, double * x, double * y, int m, int n)
{
    int i, j;

    for(i=0; i<m; i++) {
        y[i] = 0;
        for(j=0; j<n; j++)
            y[i] += x[j] * a[i*n+j];
    }
}


/* C <- A + B */
void add(double * a, double * b, double * c, int n)
{
    int j;

    for(j=0; j<n; j++)
        c[j] = a[j] + b[j];
}

/* C <- A - B */
void sub(double * a, double * b, double * c, int n)
{
    int j;

    for(j=0; j<n; j++)
        c[j] = a[j] - b[j];
}

void accum(double * a, double * b, int n)
{
    int i;

    for(i=0; i<n; i++)
        a[i] += b[i];
}

void mulmataccum(double * a, double * b, double * c, int arows, int acols, int bcols)
{
	double * tmp = (double *) malloc(arows * bcols * sizeof(double));

	mulmat(a, b, tmp, arows, acols, bcols);
	accum(c, tmp, bcols);
}

void mulvecaccum(double * a, double * b, double * c, int m, int n)
{
	double * tmp = (double *) malloc(m * n * sizeof(double));

	mulvec(a, b, tmp, m, n);
	accum(c, tmp, n);
}


void matsum(double * a, double * b, double * c, int m, int n)
{
    int i,j;

    for(i=0; i<m; i++)
        for(j=0; j<n; j++)
            c[i*n+j] = a[i*n+j] + b[i*n+j];
}

void matsub(double * a, double * b, double * c, int m, int n)
{
    int i,j;

    for(i=0; i<m; i++)
        for(j=0; j<n; j++)
            c[i*n+j] = a[i*n+j] - b[i*n+j];
}

void zeros(double * a, int m, int n)
{
    int j;
    for (j=0; j<m*n; j++)
        a[j] = 0;
}

void mat_addeye(double * a, int n)
{
    int i;
    for (i=0; i<n; i++)
        a[i*n+i] += 1;
}

void mat_addeye(double * a, int n, double scale)
{
    int i;
    for (i=0; i<n; i++)
        a[i*n+i] += scale;
}

 int choldc1(double * a, double * p, int n) {
    int i,j,k;
    double sum;

    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            sum = a[i*n+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= a[i*n+k] * a[j*n+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    return 1; /* error */
                }
                p[i] = sqrt(sum);
            }
            else {
                a[j*n+i] = sum / p[i];
            }
        }
    }

    return 0; /* success */
}

 int choldcsl(double * A, double * a, double * p, int n)
{
    int i,j,k; double sum;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            a[i*n+j] = A[i*n+j];
    if (choldc1(a, p, n)) return 1;
    for (i = 0; i < n; i++) {
        a[i*n+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*n+k] * a[k*n+i];
            }
            a[j*n+i] = sum / p[j];
        }
    }

    return 0; /* success */
}


 int cholsl(double * A, double * a, double * p, int n)
{
    int i,j,k;
    if (choldcsl(A,a,p,n)) return 1;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            a[i*n+j] = 0.0;
        }
    }
    for (i = 0; i < n; i++) {
        a[i*n+i] *= a[i*n+i];
        for (k = i + 1; k < n; k++) {
            a[i*n+i] += a[k*n+i] * a[k*n+i];
        }
        for (j = i + 1; j < n; j++) {
            for (k = j; k < n; k++) {
                a[i*n+j] += a[k*n+i] * a[k*n+j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            a[i*n+j] = a[j*n+i];
        }
    }

    return 0; /* success */
}

NAMESPACE_CLOSE();
