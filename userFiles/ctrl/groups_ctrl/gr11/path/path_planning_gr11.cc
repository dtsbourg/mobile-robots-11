#include "path_planning_gr11.h"
#include "init_pos_gr11.h"
#include "opp_pos_gr11.h"
#include "useful_gr11.h"
#include <math.h>

#define CELL_SIZE 36

NAMESPACE_INIT(ctrlGr11);


/*! \brief Evaluate distance between 2 cells on the map
 * 
 * \param[in] x1,y1 map coordinates of the first cell
 * \param[in] x2,y2 map coordinates of the second cell
 * \return distance between the 2 cells 
 */
int evaluate_distance(int x1, int y1, int x2, int y2)
{
	// diag movement
    int distance = 0;
    while (x1 != x2 && y1 != y2)
    {
        (x1<x2) ? x1++ : x1--;
        (y1<y2) ? y1++ : y1--;
        distance += sqrt(CELL_SIZE);
    }
    // line movement
    return distance + (abs(y1-y2) + abs(x1-x2)) * CELL_SIZE;
}

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */
PathPlanning* init_path_planning()
{
	PathPlanning *path;

	// memory allocation
	path = (PathPlanning*) malloc(sizeof(PathPlanning));

	// ----- path-planning initialization start ----- //


	// ----- path-planning initialization end ----- //

	// return structure initialized
	return path;
}

/*! \brief close the path-planning algorithm (memory released)
 * 
 * \param[in,out] path path-planning main structure
 */
void free_path_planning(PathPlanning *path)
{
	// ----- path-planning memory release start ----- //


	// ----- path-planning memory release end ----- //

	free(path);
}

NAMESPACE_CLOSE();
