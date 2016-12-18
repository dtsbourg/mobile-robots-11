#include "path_planning_gr11.h"
#include "init_pos_gr11.h"
#include "opp_pos_gr11.h"
#include "useful_gr11.h"
#include <math.h>

#define CELL_SIZE 1.0

NAMESPACE_INIT(ctrlGr11);


/*! \brief Evaluate distance between 2 cells on the map
 *
 * \param[in] x1,y1 map coordinates of the first cell
 * \param[in] x2,y2 map coordinates of the second cell
 * \return distance between the 2 cells
 */
float evaluate_distance(Cell cell1, Cell cell2)
{
	// Diagonal movement
    float distance = 0;
    while (cell1.x != cell2.x && cell1.y != cell2.y)
    {
        (cell1.x<cell2.x) ? cell1.x++ : cell1.x--;
        (cell1.y<cell2.y) ? cell1.y++ : cell1.y--;
        distance += CELL_SIZE * sqrt(2);
    }
    // Straight movement
    return distance + (abs(cell1.y-cell2.y) + abs(cell1.x-cell2.x)) * CELL_SIZE;
}

/*! \brief Get the 8 cells arround cell
 *
 * \param[in] cell is the cell to check arround
 * \return Cell* cell_arround a pointer on 8 cells table
 */
void get_cell_neighbourhood(Cell* cell_arround, Cell cell)
{
	cell_arround[0].x = cell.x-1;
	cell_arround[0].y = cell.y-1;
	cell_arround[1].x = cell.x;
	cell_arround[1].y = cell.y-1;
	cell_arround[2].x = cell.x+1;
	cell_arround[2].y = cell.y-1;
	cell_arround[3].x = cell.x+1;
	cell_arround[3].y = cell.y;
	cell_arround[4].x = cell.x+1;
	cell_arround[4].y = cell.y+1;
	cell_arround[5].x = cell.x;
	cell_arround[5].y = cell.y+1;
	cell_arround[6].x = cell.x-1;
	cell_arround[6].y = cell.y+1;
	cell_arround[7].x = cell.x-1;
	cell_arround[7].y = cell.y;
}

/*! \brief check if a cell is viable aka it's in the map and not obstacle
 *
 * \[in] Cell cell with the cell to check in the list
 * \[in] map[4][4] pointer on the array of the map
 * \return bool true if the cell is viable
 */
bool cell_is_viable(Cell cell, bool map[MAP_SIZE_X][MAP_SIZE_Y])
{
	if ((cell.x < MAP_SIZE_X && cell.x >= 0) && (cell.y < MAP_SIZE_Y && cell.y >= 0))
	{  // Cell is inside the map <=> is valid
		if (map[cell.x][cell.y] != 1) // check if it's not an obstacle
		{
			return true;
		}
	}
	return false;
}

/*! \brief check if a cell is viable aka it's in the map and not obstacle
 *
 * \[in] Node current node
 * \return bool true if the cell is viable
 */
void display_nodes(Node* current)
{
	Node* tracker = current;
	int count = 0;
	while(current != NULL && count < 50)
	{
		printf("(%d,%d) -> ", current->cell.x, current->cell.y);
		current = current->next;
		count++;
	}
	printf("NULL\n");
	count = 0;
	while(tracker != NULL && count < 50)
	{
		printf("   %f   -> ", tracker->f);
		tracker = tracker->next;
		count++;
	}
	printf("NULL\n");

}

/*! \brief Add a node to an ordered chained list (f lowest to highest)
 *
 * \[in] Node** list the chained list where you want to add the node
 * \[in] Node* element pointer on the element you want to add
 */
void add_to_list(Node** list, Node* element)
{
	Node* tracker = (*list);
	element->next = NULL;

	if (tracker == NULL) {
		(*list) = element;
	} else {
		Node* before = NULL;

		while (tracker != NULL)
		{
			if (element->f < tracker->f)
			{
				// put element before
				element->next = tracker;
				if (before == NULL)
					(*list) = element;
				else
					before->next = element;
				break;
			}
			else if (equal2float(element->f,tracker->f))
			{
				if (element->h < tracker->h)
				{
					// put element before
					element->next = tracker;
					if (before == NULL)
						(*list) = element;
					else
						before->next = element;
					break;
				}
				// else we keep going on the loop to put it right after.
			}

			// if we reach the end just
			if (tracker->next == NULL)
			{
				tracker->next = element;
				break;
			}
			before = tracker;
			tracker = tracker->next;
		}
	}
}

/*! \brief generate the path from the reversed node parent hierarchy
 *
 * \[in] Node* goal_node pointer on the final node
 */
Path* generate_path(Node* goal_node)
{
	Path* path = NULL;
	Node* tracker = goal_node;
	Node* before = NULL;

	tracker = before;
	Path* path_tracker = NULL;

	while (tracker != NULL)
	{
		Path* temp = (Path*)malloc(sizeof(Path));
		temp->cell = tracker->cell;
		temp->next = NULL;

		if (path_tracker == NULL)
			path = temp;
		else
			path_tracker->next = temp;

		tracker = tracker->parent;
		path_tracker = temp;
	}
	return path;
}

/*! \brief Check if a node is already in a list with an f inferior
 *
 * \[in] Node* list pointer on the list you want to check
 * \[in] Node* element pointer on the element you want to check
 * \return bool true if it's in the list
 */
bool is_in_list(Node* list, Node* element)
{
	Node* tracker = list;
	while (tracker != NULL) // if a node with lower f and same position in the list skip it
	{
		if (tracker->cell.x == element->cell.x && tracker->cell.y == element->cell.y)
			return true;

		tracker = tracker->next;
	}
	return false;
}

/*! \brief Free the memory used by a node chained list
 *
 * \[in] Node* list the Node chained list you want to free
 */
void free_nodes(Node* list)
{
	while(list != NULL)
	{
		Node* next = list->next;
		free(list);
		list = next;
	}
}

/*! \brief Free the memory used by a path chained list
 *
 * \[in] Path* path the Path chained list you want to free
 */
void free_path(Path* path)
{
	while(path != NULL)
	{
		Path* next = path->next;
		free(path);
		path = next;
	}
}

/*! \brief Main algorithm function for path_planning: compute a new path.
 *
 * \[in] Cell start the cell where you want to start the path planning
 * \[in] Cell goal the cell where your path planning aim
 * \[in] bool map[4][4] the map with the obstacle you want to dodge
 * \[in] Path* old_path the old_path you want to free from memory before path_planning again
 * \return Void* the new path calculated or the G distance from the initial to goal
 */
void* path_planning(Cell start, Cell goal, bool map[17][27], Path* old_path, bool return_distance)
{
	// -- Initialization ---
	Node* open_list = (Node*) malloc(sizeof(Node));
	Node* closed_list = NULL;
	Node* tracker = NULL;

	if (old_path != NULL)
	{
		free_path(old_path);
	}

	Path* path = NULL;

	open_list->cell = start;
	open_list->g    = 0.0;
	open_list->h    = evaluate_distance(start, goal);
	open_list->f    = 0.0;
	open_list->parent = NULL;
	open_list->next   = NULL;

	Cell cell_arround[8];

	while (open_list != NULL)
	{
		Node* current = open_list; // pop the current node with lowest f

		open_list = open_list->next;
		get_cell_neighbourhood(cell_arround, current->cell); // generate the 8 successors

		for(int i =0;i<8;i++)
		{
			if(cell_is_viable(cell_arround[i], map))
			{
				// create a successor node
				Node* successor = (Node*)malloc(sizeof(Node));
				successor->parent = current;
				successor->next = NULL;
				successor->cell = cell_arround[i];

				if (successor->cell.x == goal.x && successor->cell.y == goal.y)
				{
					if (return_distance)
					{
						float* distance = (float*)malloc(sizeof(float));
						*distance = successor->g = successor->parent->g + evaluate_distance(successor->parent->cell, successor->cell);
						free_nodes(open_list);
						free_nodes(closed_list);
						return distance;
					}
					else
					{
						path = generate_path(successor);
						free_nodes(open_list);
						free_nodes(closed_list);
						return path;
					}
					break;
				}
				successor->g = successor->parent->g + evaluate_distance(successor->parent->cell, successor->cell);
				successor->h = evaluate_distance(goal,successor->cell);
				successor->f = successor->g + successor->h;

				if(is_in_list(open_list, successor) || is_in_list(closed_list, successor))
					free(successor);
				else
					add_to_list(&open_list, successor);
			}
		}
		add_to_list(&closed_list, current);
	}

	printf("Warning: NO PATH FOUND.. \n");
	return path;
}

NAMESPACE_CLOSE();
