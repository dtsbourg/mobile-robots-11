#include "path_regulation_gr11.h"
#include "useful_gr11.h"
#include "speed_regulation_gr11.h"
#include "path_planning_gr11.h"
#include "init_pos_gr11.h"
#include "CtrlStruct_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

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
		exit(EXIT_FAILURE);
	}
	else
	{
		rob_pos->x = rob_pos->x;
		rob_pos->y = rob_pos->y;

		double dist_x = (double)path->cell.x / 10.0 - 0.9 - rob_pos->x;
		double dist_y = (double)path->cell.y / 10.0 - 1.4 - rob_pos->y;
		//printf("%d  celle.x y \n", path->cell.y);
		//printf("%f  distance rob y \n", rob_pos->y);
		//printf("%f  distance y \n", dist_y);
		double dist = sqrt(dist_x*dist_x + dist_y*dist_y);
		//printf("%f  distance \n", dist);
		double angle = rob_pos->theta - atan(dist_y / dist_x);
		//printf("%f  distance \n", dist);

		if (cvs->path->next == NULL)
		{// Dernière case: vitesse est proportionnelle à la distance qui reste
		 //vitesse = distance restante 		 //rotation = angle
			//printf("pas de chemin mec");

			if(angle>0)
				speed_regulation(cvs, dist * 100, dist*100-angle*10/ M_PI);
			else
				speed_regulation(cvs, dist * 100 + angle * 10 / M_PI, dist*100 );
		}

		if (cvs->path->next != NULL)
		{// il reste des noeuds
			//printf("fgdjfzhg");
			//speed_regulation(cvs, 10.0, 10.0);
			// rotation = angle

			if (angle>0)
				speed_regulation(cvs, 10, 10 - angle * 10 /M_PI);
			else
				speed_regulation(cvs, 10 + angle * 10 / M_PI, 10);
			// si très prêt, enlever première case
		}

		if (dist<0.05)
		{
			speed_regulation(cvs,0.0 , 0.0);
			Path* toDelete = path;
			path = path->next;
			free(toDelete);
		}
	}
}


NAMESPACE_CLOSE();
