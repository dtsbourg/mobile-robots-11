#include "path_regulation_gr11.h"
#include "useful_gr11.h"
#include "speed_regulation_gr11.h"
#include "path_planning_gr11.h"
#include "init_pos_gr11.h"
#include "CtrlStruct_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

#define CST_INTER_CASE 0.1
#define CST_ANGLE 50
#define CST_PASSED_NODE 0.05
#define MAX_SPEED 10.0

/*! \brief follow a given path
 *
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{
	//trois possibilité: pas de chemin, un dernier noeud, plusieurs noeuds.

	RobotPosition *rob_pos;
	Path *path;

	// variables initialization
	rob_pos = cvs->rob_pos;
	path = cvs->path;

	if (cvs->path == NULL)
	{
		speed_regulation(cvs, 0, 0);
	}
	else
	{

		// Distance du robot jusqu'au prochain noeud, en m
		double dist_x = (double)path->cell.x / 10.0 - 0.8 - rob_pos->x;
		double dist_y = (double)path->cell.y / 10.0 - 1.3 - rob_pos->y;

		// Calcul de la distance
		double distance = sqrt(dist_x*dist_x + dist_y*dist_y);

		set_plot(distance, "distance");

		// Calcul de l'angle avec rob_pos qui est absolu!
		double angle_cible = atan2(dist_y, dist_x);
		double angle_error = angle_cible - rob_pos->theta;

		// Performing modulo
		while (angle_error > M_PI)
			angle_error -= 2*M_PI;
		while (angle_error < -M_PI)
			angle_error += 2 * M_PI;

		// Sqrd angle is better for correcting trajectory
		double angle_sqrd = angle_error * angle_error;

		// declaration of the two final speed
		double speed_right;
		double speed_left;

		// let's begin with max speed
		speed_right = MAX_SPEED;
		speed_left = MAX_SPEED;

		// However, if the roboto is too far, not so speed
		if (distance > CST_INTER_CASE*3)
		{
			speed_right = MAX_SPEED/2;
			speed_left = MAX_SPEED/2;
		}

		// Has to turn left, decrease of speed left
		if (angle_error > 0)
			speed_left -= CST_ANGLE * angle_sqrd;
		else
		// Has to turn right, decrease of speed right
			speed_right -= CST_ANGLE * angle_sqrd;

		// Here is the trick if we have to turn the robot
		if (angle_error > M_PI / 2 || angle_error < -M_PI / 2)
		{
			if (angle_error > 0)
				speed_right += CST_ANGLE * angle_sqrd;
			else
				speed_left += CST_ANGLE * angle_sqrd;
		}

		// There's in only one node left in the path: has to stop
		if (path->next == NULL)
		{
			// Calcul percentage of dist
			double percentage_dist;
			if (distance != 0)
				percentage_dist = distance / CST_INTER_CASE;
			else
				percentage_dist = 0;

			speed_left = percentage_dist * speed_left;
			speed_right = percentage_dist * speed_right;
		} 

		speed_regulation(cvs, speed_right, speed_left);

		// delete node in path with the robot is in the neiborghood
		if (distance < CST_PASSED_NODE)
		{
			Path* toDelete = path;
			cvs->path = path->next;
			free(toDelete);
		}
	}
}


NAMESPACE_CLOSE();
