#include "path_planning_gr11.h"
#include "init_pos_gr11.h"
#include "opp_pos_gr11.h"
#include "useful_gr11.h"
#include <math.h>

#define CELL_SIZE 1

NAMESPACE_INIT(ctrlGr11);


/*! \brief Evaluate distance between 2 cells on the map
 * 
 * \param[in] x1,y1 map coordinates of the first cell
 * \param[in] x2,y2 map coordinates of the second cell
 * \return distance between the 2 cells 
 */
float evaluate_distance(Cell cell1, Cell cell2)
{
	// diag movement
    float distance = 0;
    while (cell1.x != cell2.x && cell1.y != cell2.y)
    {
        (cell1.x<cell2.x) ? cell1.x++ : cell1.x--;
        (cell1.y<cell2.y) ? cell1.y++ : cell1.y--;
        distance += CELL_SIZE * sqrt(2);
    }
    // line movement
    return distance + (abs(cell1.y-cell2.y) + abs(cell1.x-cell2.x)) * CELL_SIZE;
}

/*! \brief Get the 8 cells arround cell
 * 
 * \param[in] cell is the cell to check arround
 * \return Cell* cell_arround a pointer on 8 cells table
 */
Cell * get_cells_arround(Cell cell)
{
	Cell * cell_arround = (Cell*)malloc(8 * sizeof(Cell));
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
	return cell_arround;
}

/*! \brief add and order by f (score) a new element to the list storing the checked cells
 * 
 * \[in,out] CheckedCell* list pointer on the first element of the list
 * \[in,out] CheckedCell* element pointer on the element to add to the list
 */
void add_element(CheckedCell** list, CheckedCell* element)
{
	if ((*list) == NULL || element == NULL )
	{
		exit(EXIT_FAILURE);
	}

	CheckedCell* current = (*list);   // pointer to the current element we are checking
	CheckedCell* before = current; // pointer to the element just before current
	int i = 0;

	while (current != NULL)
	{
		if (element->f < current->f)
		{
			if (current == (*list))
			{
				// store element at the beginning
				(*list) = element;
				element->next = current;
			}
			else
			{
				// store element in the middle
				element->next = current;
				before->next = element;
			}
			break;
		}
		else
		{
			if (current->next == NULL)
			{
				//store element at the end
				current->next = element;
				break;
			}
			// increment current and before
			current = current->next;
			if (i > 0)
			{
				before = before->next;
			}
			i++;
		}
	}
}

/*! \brief update the list with a new checked cell
 * 
 * \[in,out] CheckedCell* list pointer on the first element of the list
 * \[in,out] CheckedCell* element pointer on the element to update on the list
 */
void add_cell_to_list(CheckedCell** list, Cell cell, Cell initial_pos, Cell final_pos, int step)
{
	// create element
	CheckedCell* element = (CheckedCell*)malloc(sizeof(*element));
	element->cell.x = cell.x;
	element->cell.y = cell.y;
	element->f = evaluate_distance(cell,initial_pos) + evaluate_distance(cell,final_pos);
	element->step = step;
	element->stuck = 0;
	element->next = NULL;
	add_element(list, element);
}
/*! \brief check if a cell is already in the list
 * 
 * \[in] CheckedCell* list pointer on the first element of the list or NULL if no list yet
 * \[in] Cell cell with the cell to check in the list
 * \return bool true if the cell is viable to go in the list
 */
bool cell_is_viable(CheckedCell* list, Cell cell)
{
	// check if the cell is on the map
	if ((cell.x < 4 && cell.x >= 0) && (cell.y < 4 && cell.y >= 0)) // TO CHANGE 4 AND 0
	{			
		// fake map
		bool map[4][4] = {0};
		map[1][0] = 1;
		map[1][1] = 1;
		map[2][1] = 1;

		// check if it's not an obstacle
		if (map[cell.x][cell.y] != 1)
		{
			// check if already in the list
			if (is_in_list(list, cell))
			{
				return false;
			}
			else
			{
				return true;
			}	
		}
	}
	return false;
}

/*! \brief check if a cell is already in the list
 * 
 * \[in] CheckedCell* list pointer on the first element of the list
 * \[in] Cell cell with the cell to check in the list
 * \return bool true if already in the list false otherwise
 */
bool is_in_list(CheckedCell* list, Cell cell)
{
	if (list == NULL)
	{
		exit(EXIT_FAILURE);
	}

	CheckedCell* current = list;
	while (current != NULL)
	{
		if (current->cell.x == cell.x)
		{
			if (current->cell.y == cell.y)
			{
				return true;
			}
		}
		current = current->next;
	}
	return false;
}

CheckedCell* get_best_cell(CheckedCell* first)
{
	if (first == NULL)
	{
		exit(EXIT_FAILURE);
	}

	while (first != NULL)
	{
		if(first->step != 0 && first->stuck == 0)
		{
			return first;
		}
		first = first->next;
	}
	return NULL;
}

// test function display list:
void display_list(CheckedCell* current)
{
	if (current == NULL)
	{
		exit(EXIT_FAILURE);
	}

	while (current != NULL)
	{
		printf("(%d,%d) | f:%f | Step: %d -> ", current->cell.x, current->cell.y, current->f, current->step);
		current = current->next;
	}
	printf("NULL \n");
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
