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

/// cell struct to store a cell
struct Cell
{
	int x;	// x coordinate
	int y;	// y coordinate
};
/// store the path
struct Path
{
	Cell cell;
	Path* next;
};

struct Node{
	Node* parent;
	Cell cell;
	float f,g,h;
	Node* next;
};

// main algo function
Path* path_planning(Cell start, Cell goal, bool map[17][27], Path* old_path);
// Evaluate distance between 2 cells on the map
float evaluate_distance(Cell cell1, Cell cell2);
// Return the 8 cells arround a cell
void get_cells_arround(Cell* cell_arround, Cell cell);
// Generate path from the goal_node
Path* generate_path(Node* goal_node);
// check if a cell is viable to go in the list
bool cell_is_viable(Cell cell, bool map[17][27]);
// check if the node is already in a list
bool is_in_list(Node* list, Node* element);
// free nodes list:
void free_nodes(Node* list);
// free path list:
void free_path(Path* path);

// test function to display list of nodes:
void display_nodes(Node* current);


NAMESPACE_CLOSE();

#endif
