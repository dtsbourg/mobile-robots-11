#include "path_planning_gr11.h"
#include "init_pos_gr11.h"
#include "opp_pos_gr11.h"
#include "useful_gr11.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr11);

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
