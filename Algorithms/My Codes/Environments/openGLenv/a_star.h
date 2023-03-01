#ifndef A_STAR_H
#define A_STAR_H

#include <vector>

#include "global_variables.h"
#include "mcc.h"
#include "constants.h"


class a_star_algo {
public:

	void init(std::vector<Node>* list, 
		Node* algo_graph,
		bool keep_solving,
		int start_cell_index,
		int goal_cell_index);

	double sub_path_cost(std::vector<Node>* list, int start_cell_index, int goal_cell_index);

private:
	std::vector<Node>* nodes_list;
	//algo specific variables
	Node* algo_graph;
	bool keep_solving = false;
	int start_cell_index, goal_cell_index;

	float heuristic_cost_a_star(Node* node);

	Node* cost_update(Node* me, Node* neighbour);

	void solver(Node* node);

	bool cost_resolver(Node* node, std::vector<Node>* list, int start_cell_index);
};

#endif