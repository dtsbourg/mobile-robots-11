#include "useful_gr11.h"
#include <math.h>
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr11);

/*! \brief Compute the average of 2 doubles
 *
 * \return The average of a and b
 */
double avg(double a, double b)
{
	return (a+b)/2.0;
}

/*! \brief generate random number in [0,1]
 *
 * \return random number in the [0,1] range
 */
double rnd()
{
	return ((double)rand())/((double)RAND_MAX);
}

/*! \brief get the normal distance, given two vector components
 *
 * \param[in] dx x vector component
 * \param[in] dy y vector component
 * \return normal distance
 */
double norm_dist(double dx, double dy)
{
	return sqrt(dx * dx + dy * dy);
}

/*! \brief limit a function in a given range
 *
 * \param[in] x value to limit
 * \param[in] min minimal bound of the range
 * \param[in] max maximal bound of the range
 * \return value limited in the given range
 */
double limit_range(double x, double min, double max)
{
	return (x < min) ? min : (x > max) ? max : x;
}

/*! \brief set an angle in the in ]-pi;pi] range
 *
 * \param[in] x angle to limit
 * \return angle limited in ]-pi;pi]
 */
double limit_angle(double x)
{
	while (x <= -M_PI)
	{
		x += 2.0*M_PI;
	}
	while (x > M_PI)
	{
		x -= 2.0*M_PI;
	}

	return x;
}

/*! \brief first-order low-pass filter
 *
 * \param[in] last_val last value
 * \param[in] new_val new value
 * \param[in] tau time constant [s]
 * \param[in] delta_t time increment since last call [s]
 * \return output of the low-pass filter
 */
double first_order_filter(double last_val, double new_val, double tau, double delta_t)
{
	double f = delta_t / tau;
	double frac = 1.0 / (1.0 + f);

	return f * frac * new_val + frac * last_val;
}

/*! \brief Check if 2 floats are equal with an EPSILON value
 *
 * \param[in] a first float to check
 * \param[in] b second float to check
 * \return bool true if equal false otherwise
 */
bool equal2float(float a, float b)
{
	return fabs(a-b) < EPSILON;
}

/*! \brief Converts a world X coordinate to a map X coordinate
 *
 * \param[in] world_x global frame coordinate
 * \return int map cell X coordinate
 */
int world_to_map_x(double world_x)
{
	int map_pos_x = (int)(((world_x + 0.8) * 10.0));
	return map_pos_x;
}

/*! \brief Converts a world Y coordinate to a map Y coordinate
 *
 * \param[in] world_y global frame coordinate
 * \return int map cell Y coordinate
 */
int world_to_map_y(double world_y)
{
	int map_pos_y = (int)((world_y + 1.3) * 10.0);
	return map_pos_y;
}

/*! \brief Converts a map X coordinate to a world X coordinate
 *
 * \param[in] map_x map cell coordinate
 * \return double world frame X coordinate
 */
double map_to_world_x(int map_x)
{
	double world_x = ((map_x / 10.0) - 0.8);
	return world_x;
}

/*! \brief Converts a map Y coordinate to a world Y coordinate
 *
 * \param[in] map_y map cell coordinate
 * \return double world frame Y coordinate
 */
double map_to_world_y(int map_y)
{
	double world_y = ((map_y / 10.0) - 1.3);
	return world_y;
}

int convert_pos_to_map(double world_pos)
{
	int map_pos = (int)round((world_pos + 0.8)*10.0);
	return map_pos;
}

NAMESPACE_CLOSE();
