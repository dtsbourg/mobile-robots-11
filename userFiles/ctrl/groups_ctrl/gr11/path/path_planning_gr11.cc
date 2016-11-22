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
void get_cells_arround(Cell* cell_arround, Cell cell)
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

bool cell_is_viable(Cell cell, bool map[17][27], Node* selected_node)
{
	if ((cell.x < 17 && cell.x >= 0) && (cell.y < 27 && cell.y >= 0)) // if cell is on the map
	{		
		if (map[cell.x][cell.y] != 1) // if not an obstacle
		{
			// check if the cell is already in the nodes before:
			selected_node = selected_node->before;
			while (selected_node != NULL)
			{
				Node* tracker = selected_node->list;
				while (tracker != NULL)
				{
					if (tracker->cell.x == cell.x && tracker->cell.y == cell.y)
					{
						return false;
					}
					tracker = tracker->next;
				}
				if (selected_node->before == NULL)
				{
					if (selected_node->cell.x == cell.x && selected_node->cell.y == cell.y)
					{
						return false; // for the first cell
					}
				}
				selected_node = selected_node->before;
			}
			return true;
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
	return first_node;
}

Node* create_new_node(Cell cell, Cell start, Cell goal)
{
	Node* new_node = (Node*)malloc(sizeof(Node));
	new_node->cell = cell;
	new_node->f = evaluate_distance(cell, start) + evaluate_distance(cell, goal);
	new_node->stuck = 0;
	new_node->list = NULL;
	new_node->before = NULL;
	new_node->next = NULL;
	return new_node;
}

void insert_new_node(Node* selected_node, Node* new_node)
{
	Node* current = selected_node->list;
	if (current == NULL)
	{
		selected_node->list = new_node; // first new node in the list
	}
	else
	{
		Node* before = NULL;
		while(current != NULL)
		{
			if (new_node->f <= current->f)
			{
				new_node->next = current;
				if (before == NULL)
				{
					selected_node->list = new_node; // become first node of the list
				}
				else
				{
					before->next = new_node; // inserted in the middle
				}
				break;
			}
			before = current;
			current = current->next;
			if (current == NULL)
			{
				before->next = new_node; // inserted at the end
			}
		}
	}
	new_node->before = selected_node;
}

Node* new_selected_node(Node* old_node)
{
	if (old_node == NULL)
	{
		exit(EXIT_FAILURE);
	}
	Node* tracker = old_node->list;
	while (tracker != NULL)
	{
		if (!tracker->stuck)
		{
			return tracker;
		}
		if (tracker->next == NULL) 		// End of the list all node are stucked => stuck node before
		{
			tracker = tracker->before;
			if (tracker == NULL)
			{
				return NULL;
			}
			else
			{
				tracker->stuck = 1;
			}
		}
		else
			tracker = tracker->next;
	}
	printf("stucked \n");
	return NULL;
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
	printf("Pathplanning from (%d,%d) to (%d,%d)\n", start.x, start.y, goal.x, goal.y);
	Node* first_node = init_first_node(start); // TODO LIBERER LA MEMOIRE A LA FIN !!
	Cell cell_arround[8]; 
	// printf("First node: (%d,%d) \n", first_node->cell.x, first_node->cell.y);
	Node* selected_node = first_node;
	int algo_step = 0;
	while (!algo_step)
	{
		get_cells_arround(cell_arround, selected_node->cell);
		int viable_cell = 0;
		for (int i=0; i < 8; i++)
		{
			if (cell_arround[i].x == goal.x &&
				cell_arround[i].y == goal.y)
			{
				Node* new_node = create_new_node(cell_arround[i], start, goal);
				insert_new_node(selected_node, new_node);
				selected_node = new_node;
				//Path* path = create_path(selected_node);
				Node* tracker = selected_node;
				printf("\npath found!\n");
				while (tracker != NULL)
				{
					printf(" (%d,%d) ->", tracker->cell.x, tracker->cell.y);
					tracker = tracker->before;
				}
				printf("NULL \n\n");
				// selected_node = new node
				// path = create_path(selected_node)
				// return path;
				//return;
				algo_step = 1;
			}
			if (cell_is_viable(cell_arround[i], map, selected_node))
			{
				Node* new_node = create_new_node(cell_arround[i], start, goal);
				insert_new_node(selected_node, new_node);
				// printf("New node: (%d,%d) \n", new_node->cell.x, new_node->cell.y);
				Node* tracks = selected_node->list;
				while(tracks != NULL)
				{
					// printf(" %f ->", tracks->f);
					printf(" (%d, %d) ->", tracks->cell.x, tracks->cell.y);
					tracks = tracks->next;
				}
				printf(" NULL \n");
				viable_cell++;
			}
		}
		if (viable_cell == 0)
		{
			printf("no viable cell\n");
			selected_node->stuck = 1;
		}
		Node* old_node = selected_node;
		selected_node = new_selected_node(old_node);
		if (selected_node != NULL)
			printf("Le node choisi est: (%d,%d)\n", selected_node->cell.x, selected_node->cell.y);
		if (selected_node == NULL)
		{
			printf("Warning: No path Found \n");
			algo_step = 1;
			//return;
		}
		//algo_step++;
	}

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
