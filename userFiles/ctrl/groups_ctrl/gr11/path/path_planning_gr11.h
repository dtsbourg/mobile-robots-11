/*! 
 * \author Group 11
 * \file path_planning_gr11.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR11_H_
#define _PATH_PLANNING_GR11_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr11.h"

NAMESPACE_INIT(ctrlGr11);

/// path-planning main structure
struct PathPlanning
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

/// cell struct to store a cell
struct Cell
{
	int x;	// x coordinate
	int y;	// y coordinate
};

/// checked cell struct stored when a cell is check
struct CheckedCell
{
	Cell cell;  	   // cell coordinates
	float f; 			   // store the score computed (f(n) = g(n) + h(n))
	int step;   	   // store at which step of the path is the cell
	bool stuck; 	   // is the cell stuck or not
	CheckedCell* next; // pointer to the next element of the list
};
/// store the path
struct Path
{
	Cell cell;
	Path* next;
};

// main algo function
Path* path_planning(Cell start, Cell goal, bool map[17][27]);

// Evaluate distance between 2 cells on the map
float evaluate_distance(Cell cell1, Cell cell2);
// Return the 8 cells arround cell
Cell * get_cells_arround(Cell cell);
// add a cell to the list
void add_cell_to_list(CheckedCell** list, Cell cell, Cell initial_pos, Cell final_pos, int step);
// initialize the list of checked cells
void add_element(CheckedCell** list, CheckedCell* element);

// check if a cell is viable to go in the list
bool cell_is_viable(CheckedCell* list, Cell cell, bool map[17][27]);
// test if a cell is in the list
bool is_in_list(CheckedCell* list, Cell cell);
// get best cell
CheckedCell* get_best_cell(CheckedCell* first);
// create a new dynamic array holding the current path
void add_to_path(Path** path, Cell cell, int step);

// test function to display the list
void display_list(CheckedCell* current);
void display_path(Path* path);

void free_CheckedCell(CheckedCell* list);
void free_path(Path* path);

NAMESPACE_CLOSE();

#endif
