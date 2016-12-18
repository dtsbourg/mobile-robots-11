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

#define MAP_SIZE_X 17
#define MAP_SIZE_Y 27

/// Cell struct
/*
 * This struct wraps a map's cell
 */
struct Cell
{
	int x;	///< x coordinate
	int y;	///< y coordinate
};

/// Path struct
/*
 * A Path throught the map is a linked list
 * of Cells.
 */
struct Path
{
	Cell cell;
	Path* next;
};

/// Node struct
/*
 * The path planning algorithm builds a linked list
 * of Nodes.
 */
struct Node{
	Node* parent;   // The parent node in the graph
	Node* next;     // The next node in the graph
	Cell cell;      // The corresponding cell
	float f, g, h;  // The scores for the A* algorithm
};

/// Main path planning
void* path_planning(Cell start, Cell goal, bool map[MAP_SIZE_X][MAP_SIZE_Y], Path* old_path, bool return_distance);

/// Returns the distance between 2 cells on the map
float evaluate_distance(Cell cell1, Cell cell2);

/// Return the 8 cells neighborhood arround a cell
void get_cell_neighbourhood(Cell* cell_arround, Cell cell);

/// Generate path from the goal_node
Path* generate_path(Node* goal_node);

/// Check if a cell is viable to go in the list
bool cell_is_viable(Cell cell, bool map[17][27]);

/// Check if the node is already in the list
bool is_in_list(Node* list, Node* element);

/// free nodes list
void free_nodes(Node* list);

/// free path list
void free_path(Path* path);

/// Debug function to display list of nodes
void display_nodes(Node* current);


NAMESPACE_CLOSE();

#endif
