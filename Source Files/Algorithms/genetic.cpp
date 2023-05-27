#ifndef GENETIC_ALGO_CPP
#define GENETIC_ALGO_CPP
#include "../../Header Files/Algorithms/genetic.h"

void genetic_algo::init(int grid_height, int grid_width,
	int start_cell_index, int goal_cell_index, std::vector<Node>* graph){
	this->start_x = start_cell_index / grid_width;
	this->start_y = start_cell_index % grid_width;
	this->goal_x = goal_cell_index / grid_width;
	this->goal_y = goal_cell_index % grid_width;

	std::cout << "start_x : " << start_x << std::endl;
	std::cout << "start_y : " << start_y << std::endl;
	std::cout << "goal_x : " << goal_x << std::endl;
	std::cout << "goal_y : " << goal_y << std::endl;

	this->GRID_SPACE_MIN_X = 0;
	this->GRID_SPACE_MAX_X = grid_width - 1;
	this->GRID_SPACE_MIN_Y = 0;
	this->GRID_SPACE_MAX_Y = grid_height - 1;

	std::cout << "GRID_SPACE_MIN_X : " << GRID_SPACE_MIN_X << std::endl;
	std::cout << "GRID_SPACE_MAX_X : " << GRID_SPACE_MAX_X << std::endl;
	std::cout << "GRID_SPACE_MIN_Y : " << GRID_SPACE_MIN_Y << std::endl;
	std::cout << "GRID_SPACE_MAX_Y : " << GRID_SPACE_MAX_Y << std::endl;

	this->node_list = graph;

	this->start_cell_index = start_cell_index;
	this->goal_cell_index = goal_cell_index;
}

void genetic_algo::solver(void) {
	if (!solutions.empty())
		solutions.clear();
	if (!sample.empty())
		sample.clear();
	std::cout << "solutions sizes : " << solutions.size() << std::endl;
	std::cout << "sample sizes : " << sample.size() << std::endl;

	// Create initial random solutions
	for (int i = 0; i < SAMPLE_SIZE; i++)
		sample.push_back(Solution{GRID_SPACE_MIN_X, GRID_SPACE_MAX_X, GRID_SPACE_MIN_Y, GRID_SPACE_MAX_Y});

	sort_solutions_by_rank(&sample);
	print_best_10(&sample, '-');

	//Take best samples as starting solutions
	for (int i = 0; i < SOLUTION_SPACE_SIZE; i++)
		solutions.push_back(sample.at(i));

	for (int k = 0; k < GENERATIONS; k++){
		sort_solutions_by_rank(&solutions);
		if (k % 100 == 0)
			print_best_10(&solutions, '*');
		cross_over(&solutions);
		mutate(&solutions);

		//store best solution
		if (solutions.at(0).rank > best_solution.rank)
			best_solution = solutions.at(0);
	}

	sort_solutions_by_rank(&solutions);
	print_best_10(&solutions, '=');

	//store best solution
	if (solutions.at(0).rank > best_solution.rank)
		best_solution = solutions.at(0);

	std::cout << "BEST SOLUTION : Dist = " << 1 / best_solution.rank << std::endl;;
}

void genetic_algo::extract_path(std::vector<Path>* path, int start_cell_index, int goal_cell_index) {
	std::cout << "X : " << start_x << " , Y : " << start_y << std::endl;
	for (int i = 0; i < CONTROL_POINTS; i++) {
		if (i == CONTROL_POINTS - 1) {
			path->insert(path->end(), Path(getIndex(i), goal_cell_index));
			break;
		}
		if (i == 0) {
			path->insert(path->end(), Path(start_cell_index, getIndex(i)));
		}
		path->insert(path->end(), Path(getIndex(i), getIndex(i + 1)));
		std::cout << "X : " << best_solution.pts[2 * i] << " , Y : " << best_solution.pts[2 * i + 1] << std::endl;
	}
	std::cout << "X : " << goal_x << " , Y : " << goal_y << std::endl;
}

void genetic_algo::print_best_10(std::vector<Solution>* list, char separator) {
	for (int i = 0; i < 60; i++)
		std::cout << separator;
	std::cout << std::endl;

	for (int i = 0; i < 10; i++) {
		const auto s = (*list).at(i);
		/*std::cout << "[";
		for (int in = 0; in < 40; in++)
			std::cout << s.pts[in] << ", ";
		std::cout << std::endl;
		*/
		std::cout << "rank : " << s.rank << " | distance : " << 1 / s.rank << std::endl;
	}
}

void genetic_algo::sort_solutions_by_rank(std::vector<Solution>* list){
	//Run fitness function
	for (auto& s : *list) { s.fitness(start_x, start_y, goal_x, goal_y, node_list, start_cell_index, goal_cell_index); }

	//Sort our solutions by rank
	std::sort(list->begin(), list->end(),
		[](const auto& lhs, const auto& rhs) {
			return lhs.rank > rhs.rank;
		});
}

void genetic_algo::cross_over(std::vector<Solution>* list) {
	//Cross-over
	std::random_device device;
	std::uniform_int_distribution<int> cross(0, 2*CONTROL_POINTS);
	for (int i = 10; i < list->size() / 2; i++) {
		int temp;
		for (int index = cross(device); index < 2*CONTROL_POINTS; index++) {
			temp = list->at(2 * i).pts[index];
			list->at(2 * i).pts[index] = list->at(2 * i + 1).pts[index];
			list->at(2 * i + 1).pts[index] = temp;
		}
	}
}

void genetic_algo::mutate(std::vector<Solution>* list){
	//Mutate the top solutions by %
	std::random_device device;
	std::uniform_int_distribution<int> cross(0, 2*CONTROL_POINTS);
	std::uniform_real_distribution<double> m(LOWER_MUTATION_FACTOR, UPPER_MUTATION_FACTOR);
	for (auto& s : *list) {
		int index = cross(device);
		if (index > 0 && index < 2 * CONTROL_POINTS) {
			if (index % 2 == 0) {
				int temp = (int)(s.pts[cross(device)] * m(device));
				if (temp >= GRID_SPACE_MIN_X && temp <= GRID_SPACE_MAX_X)
					s.pts[index] = temp;
			}
			else {
				int temp = (int)(s.pts[cross(device)] * m(device));
				if (temp >= GRID_SPACE_MIN_Y && temp <= GRID_SPACE_MAX_Y)
					s.pts[index] = temp;
			}
		}
	}
}

int genetic_algo::getIndex(int i) {
	int x = best_solution.pts[2 * i];
	int y = best_solution.pts[2 * i + 1];
	return x * (GRID_SPACE_MAX_X - GRID_SPACE_MIN_X + 1) + y;
}

#endif