#ifndef A_STAR_CPP
#define A_STAR_CPP

#include "../../Header Files/Algorithms/a_star.h"


void a_star_algo::init(std::vector<Node>* list,	Node* algo_graph, bool keep_solving, int start_cell_index, int goal_cell_index) {
	this->nodes_list = list;
	this->algo_graph = algo_graph;
	this->keep_solving = keep_solving;
	this->start_cell_index = start_cell_index;
	this->goal_cell_index = goal_cell_index;

	nodes_list->at(start_cell_index).cost = 0 + heuristic_cost_a_star(algo_graph);
	this->keep_solving = true;
	solver(algo_graph);
}

float a_star_algo::heuristic_cost_a_star(Node* node) {
	float x_node = node->x;
	float y_node = node->y;
	float x_goal = nodes_list->at(goal_cell_index).x;
	float y_goal = nodes_list->at(goal_cell_index).y;

	float dist = sqrt((x_node - x_goal) * (x_node - x_goal) + (y_node - y_goal) * (y_node - y_goal));
	return dist;
}

Node* a_star_algo::cost_update(Node* me, Node* neighbour) {
	if (neighbour == nullptr)
		return nullptr;
	if (neighbour->type == BASE_TAKEN)
		return nullptr;
	float expected_cost = me->cost + 1 + heuristic_cost_a_star(neighbour);
	if (neighbour->cost > expected_cost) {
		neighbour->cost = expected_cost;
		neighbour->best = me;
		//solver(neighbour);
		return neighbour;
	}
	return nullptr;
}

void a_star_algo::solver(Node* node) {
	if (keep_solving) {
		// return conditions
		if (node == nullptr)
			return;
		if (node->type == GOAL) {
			//keep_solving = false;
			//extract_path(node);
			return;
		}

		// updating costs of reaching neighbouring cells; 
		// adding approachable cells to the list
		std::vector<Node*> approachable_cells;

		if (cost_update(node, node->left) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->left);

		if (cost_update(node, node->right) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->right);

		if (cost_update(node, node->top) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->top);

		if (cost_update(node, node->bottom) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->bottom);

		// neighbour with least cost of approach to explore first
		while (!approachable_cells.empty()) {
			int least_cost_cell_index = 0;

			// find position of least cost neighbour
			for (int i = 0; i < approachable_cells.size(); i++)
				if (approachable_cells.at(i)->cost < approachable_cells.at(least_cost_cell_index)->cost)
					least_cost_cell_index = i;

			// trigger a search towards that neighbour
			solver(approachable_cells.at(least_cost_cell_index));

			// remove that neighbour from the list of "unexplored" approachable_cells
			approachable_cells.erase(approachable_cells.begin() + least_cost_cell_index);
		}
	}
}

double a_star_algo::sub_path_cost(std::vector<Node>* list, int start_cell_index, int goal_cell_index) {
	this->nodes_list = list;
	this->algo_graph = &(list->at(start_cell_index));
	this->start_cell_index = start_cell_index;
	this->goal_cell_index = goal_cell_index;

	nodes_list->at(start_cell_index).cost = 0 + heuristic_cost_a_star(algo_graph);
	this->keep_solving = true;
	cost_resolver(algo_graph, list, start_cell_index);
	return list->at(goal_cell_index).cost;
}

bool a_star_algo::cost_resolver(Node* node, std::vector<Node>* list, int start_cell_index) {
	if (keep_solving) {
		// return conditions
		if (node == nullptr)
			return false;
		if (node->x == list->at(goal_cell_index).x && node->y == list->at(goal_cell_index).y) {
			keep_solving = false;
			//extract_path(node);
			return true;
		}

		// updating costs of reaching neighbouring cells; 
		// adding approachable cells to the list
		std::vector<Node*> approachable_cells;

		if (cost_update(node, node->left) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->left);

		if (cost_update(node, node->right) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->right);

		if (cost_update(node, node->top) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->top);

		if (cost_update(node, node->bottom) != nullptr)
			approachable_cells.insert(approachable_cells.end(), node->bottom);

		// neighbour with least cost of approach to explore first
		while (!approachable_cells.empty()) {
			int least_cost_cell_index = 0;

			// find position of least cost neighbour
			for (int i = 0; i < approachable_cells.size(); i++)
				if (approachable_cells.at(i)->cost < approachable_cells.at(least_cost_cell_index)->cost)
					least_cost_cell_index = i;

			// trigger a search towards that neighbour
			solver(approachable_cells.at(least_cost_cell_index));

			// remove that neighbour from the list of "unexplored" approachable_cells
			approachable_cells.erase(approachable_cells.begin() + least_cost_cell_index);
		}
	}
}

#endif