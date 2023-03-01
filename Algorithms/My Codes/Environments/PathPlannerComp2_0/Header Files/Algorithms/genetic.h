#ifndef GENETIC_ALGO_H
#define GENETIC_ALGO_H

#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>

#include "../mcc.h"
#include "a_star.h"

class genetic_algo {
public:
	void init(int grid_height, int grid_width,
			int start_cell_index, int goal_cell_index, 
			std::vector<Node>* graph);

	void solver(void);

	void extract_path(std::vector<Path>* path, int start_cell_index, int goal_cell_index);
	
private:
	std::vector<Node>* node_list;

	int start_x;
	int start_y;
	int goal_x;
	int goal_y;

	int start_cell_index;
	int goal_cell_index;

	int GRID_SPACE_MIN_X;
	int GRID_SPACE_MAX_X;
	int GRID_SPACE_MIN_Y;
	int GRID_SPACE_MAX_Y;

	static const int CONTROL_POINTS = 8;

	const int SAMPLE_SIZE = 100000;
	const int SOLUTION_SPACE_SIZE = 200;
	const int GENERATIONS = 500;
	const float LOWER_MUTATION_FACTOR = 0.5;
	const float UPPER_MUTATION_FACTOR = 2;

	struct Solution {
		double rank;
		int pts[2*CONTROL_POINTS];
		int GRID_SPACE_MIN_X;
		int GRID_SPACE_MAX_X;

		Solution(int GRID_SPACE_MIN_X, int GRID_SPACE_MAX_X, int GRID_SPACE_MIN_Y, int GRID_SPACE_MAX_Y) {
			this->GRID_SPACE_MIN_X = GRID_SPACE_MIN_X;
			this->GRID_SPACE_MAX_X = GRID_SPACE_MAX_X;
			std::random_device device;
			std::uniform_int_distribution<int> unif_x(GRID_SPACE_MIN_X, GRID_SPACE_MAX_X);
			std::uniform_int_distribution<int> unif_y(GRID_SPACE_MIN_Y, GRID_SPACE_MAX_Y);
			rank = 0;
			for (int i = 0; i < CONTROL_POINTS; i++) {
				this->pts[2 * i] = unif_x(device);
				this->pts[2 * i + 1] = unif_y(device);
			}
		}

		void fitness(int start_x, int start_y, int goal_x, int goal_y, std::vector<Node>* node_list, int start_cell_index, int goal_cell_index) {
			double dist=0;
			std::vector<double> distances;
			for (int i = 0; i < CONTROL_POINTS; i++) {
				if (i == CONTROL_POINTS - 1) {
					//distances.push_back(std::sqrt(std::pow(pts[2 * i] - goal_x, 2) + std::pow(pts[2 * i + 1] - goal_y, 2)));
					distances.push_back(a_star.sub_path_cost(node_list, getIndex(i), goal_cell_index));
					break;
				}
				if (i == 0) {
					//distances.push_back(std::sqrt(std::pow(pts[2 * i] - start_x, 2) + std::pow(pts[2 * i + 1] - start_y, 2)));
					distances.push_back(a_star.sub_path_cost(node_list, getIndex(i), start_cell_index));
				}
				//distances.push_back(std::sqrt(std::pow(pts[2 * i] - pts[2 * (i + 1)], 2) + std::pow(pts[2 * i + 1] - pts[2 * (i + 1) + 1], 2)));
				distances.push_back(a_star.sub_path_cost(node_list, getIndex(i), getIndex(i+1)));
			}
			//a_star.sub_path_cost(node_list, start_cell_index, goal_cell_index);
			//Sort these distances by magnitude
			std::sort(distances.begin(), distances.end(),
				[](const auto& lhs, const auto& rhs) {
					return lhs > rhs;
				});
			rank = std::abs(1 / std::pow(distances.at(0), 3));
		}

	private:
		a_star_algo a_star;

		double dist(int i) {
			return std::sqrt(std::pow(pts[2 * i] - pts[2 * (i + 1)], 2) + std::pow(pts[2 * i + 1] - pts[2 * (i + 1) + 1], 2));
		}

		int getIndex(int i) {
			int x = pts[2 * i];
			int y = pts[2 * i + 1];
			return x * (GRID_SPACE_MAX_X - GRID_SPACE_MIN_X + 1) + y;
		}

		void clear_costs(std::vector<Node>* list) {

		}
	};

	std::vector<Solution> sample, solutions;
	Solution best_solution = Solution{GRID_SPACE_MIN_X, GRID_SPACE_MAX_X, GRID_SPACE_MAX_X, GRID_SPACE_MAX_Y};

	void print_best_10(std::vector<Solution>* list, char separator);

	void sort_solutions_by_rank(std::vector<Solution>* list);

	void cross_over(std::vector<Solution>* list);

	void mutate(std::vector<Solution>* list);

	int getIndex(int i);
};

#endif