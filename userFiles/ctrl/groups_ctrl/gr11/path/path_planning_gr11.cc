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
		if (element->f <= current->f)
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

// TODO utiliser pointer2D sur la map pour check à la place de fake MAP
bool cell_is_viable(CheckedCell* list, Cell cell, bool map[17][27])
{
	// check if the cell is on the map
	if ((cell.x < 17 && cell.x >= 0) && (cell.y < 27 && cell.y >= 0)) // TO CHANGE 4 AND 0
	{			
		// fake map
		/*
		bool map[4][4] = {0};
		map[1][0] = 1;
		map[1][1] = 1;
		map[2][1] = 1;
		*/

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

void add_to_path(Path** path, Cell cell, int step)
{
	if (path == NULL)
	{
		exit(EXIT_FAILURE);
	}
	int i = 0;
	Path* current = *path;
	while (i < (step-1))
	{
		current = current->next;
		i++;
	}
	if (current->next == NULL)
	{
		Path* new_path = (Path*)malloc(sizeof(*new_path));
		new_path->cell = cell;
		new_path->next = NULL;
		current->next = new_path;
	}
	else
	{
		current->cell = cell;
	}
}
// test function display list:
void display_list(CheckedCell* current)
{
	if (current == NULL)
	{
		exit(EXIT_FAILURE);
	}

	float fake_map[17][27];
	for (int i=0; i<17;i++)
	{
		for(int j=0; j<27; j++)
		{
			fake_map[i][j] = 0;
		}
	}

	while (current != NULL)
	{
		//printf("(%d,%d) | f:%f | Step: %d -> ", current->cell.x, current->cell.y, current->f, current->step);
		fake_map[current->cell.x][current->cell.y] = current->f;
		current = current->next;
	}
	//printf("NULL \n");
	// print fake map
	printf("--- Score Map Begin ---\n");
	for (int j=0; j < 27; j++)
	{
		if (j < 10)
			printf("%d  | ", j);
		else
			printf("%d | ", j);
		for(int i =0; i < 17; i++)
		{
			if(fake_map[i][j] == 0)
				printf(" 0  ");
			else
				printf("%d ", (int)(100*fake_map[i][j]));
		}
		printf("| \n");
	}
	printf("--- Score Map End ---\n");
}
void display_path(Path* path)
{
	if (path == NULL)
	{
		exit(EXIT_FAILURE);
	}

	while (path != NULL)
	{
		printf("(%d,%d) -> ", path->cell.x, path->cell.y);
		path = path->next;
	}
	printf("NULL \n");
}

void free_CheckedCell(CheckedCell* list)
{
	while(list != NULL)
	{
		CheckedCell* toDelete = list;
		list = list->next;
		free(toDelete);
	}
}
void free_path(Path* path)
{
	while(path != NULL)
	{
		Path* toDelete = path;
		path = path->next;
		free(toDelete);
	}
}

// New function / New algo
Node* init_first_node(Cell start)
{
	Node* first_node = (Node*)malloc(sizeof(Node));
	first_node->cell = start;
	first_node->f = 0;
	first_node->stuck = 0;
	first_node->list = NULL;
	first_node->before = NULL;
	first_node->next = NULL;
}

Path* path_planning(Cell start, Cell goal, bool map[17][27])
{

	// create a function to initialize everything
	/*
	CheckedCell* list = (CheckedCell*)malloc(sizeof(*list)); // pointer to the first element of the list
	list->cell = start;
	list->f = evaluate_distance(start, goal);
	list->step = 0;
	list->stuck = 0;
	list->next = NULL;
	CheckedCell* selected_cell = list; // currently selected cell for the algorithm
	*/
	Node* first_node = init_first_node(start);

	Path* path = (Path*)malloc(sizeof(*path)); // store the current optimal path
	path->cell = start;
	path->next = NULL;
	//test
	/*
	
	bool path_found = 0;
	while (!path_found)
	{
		// recupere les 8 cells autour de la position intial:
		Cell* cell_arround = get_cells_arround(selected_cell->cell);
		for (int i=0; i < 8; i++)
		{
			int viable_cell = 0;

			if (cell_arround[i].x == goal.x &&
				cell_arround[i].y == goal.y)
			{
				// si c'est la goal ajoute la cell au path et finish
				path_found = true;
				printf("path found!\n");
			}
			if (cell_is_viable(list, cell_arround[i], map))
			{
				add_cell_to_list(&list, cell_arround[i], start, goal, selected_cell->step + 1);
				//display_list(list);
				viable_cell++;
			}
			if (viable_cell == 0)
			{
				selected_cell->stuck = true;
			}
		}
		// choose a new cell
		selected_cell = get_best_cell(list);
		if( selected_cell == NULL)
		{
			path_found = true;
			printf("unable to find a path..");
		}
		else
		{
			display_list(list);
			printf("Next Cell to check arround: (%d,%d)\n", selected_cell->cell.x, selected_cell->cell.y);
			add_to_path(&path, selected_cell->cell, selected_cell->step);
		}
	}
	free_CheckedCell(list);
	*/
	return path;
}

/*! \brief close the path-planning algorithm (memory released)
 * 
 * \param[in,out] path path-planning main structure
 */

NAMESPACE_CLOSE();
