#include "../Header Files/elliptical_approx.h"

bool ellipse::isInside(float x, float y, int i) {
	//std::cout << "(x, y) -> (" << x << ", " << y << ") | (center_x, center_y) -> (" <<
	//	center_x << ", " << center_y << ")" << std::endl;
	float x_rel = x - center_x;
	float y_rel = y - center_y;

	float x_tilt_frame = x_rel * cos(tilt) + y_rel * sin(tilt);
	float y_tilt_frame = -x_rel * sin(tilt) + y_rel * cos(tilt);

	float query_angle = atan2(a*y_rel, b*x_rel);
	if (false) {
		std::cout << "atan2(y/x) = " << query_angle * 180 / 3.141592 << " degrees" << std::endl;
		std::cout << "(query_angle, tilt) = " << query_angle << " , " << tilt << std::endl;
		std::cout << "(x_tilt, limit) -> " << x_tilt_frame << " , " << (a * cos(query_angle - tilt)) << "." << std::endl;
		std::cout << "(y_tilt, limit) -> " << y_tilt_frame << " , " << (b * sin(query_angle - tilt)) << "." << std::endl;
		std::cout << "(a, b) -> " << a << " , " << b << "." << std::endl;
	}

	float x_factor = x_tilt_frame / (a * cos(query_angle - tilt));
	if (x_factor > 1 || x_factor < 0)
		return false;
		
	return true;
}

local_visualizer::local_visualizer(void) {
	GRID_WIDTH = 0;
	GRID_HEIGHT = 0;
	paths = nullptr;
	renderer = nullptr;
	render_callback = nullptr;
	scribbled_paths = 0;
}

local_visualizer::local_visualizer(int GRID_WIDTH, int GRID_HEIGHT, RenderClass* renderer, std::vector<Path>* paths,
	void (*func_updater)(std::vector<float>*)) {
	this->GRID_WIDTH = GRID_WIDTH;
	this->GRID_HEIGHT = GRID_HEIGHT;
	this->paths = paths; // for visualization
	this->renderer = renderer;
	this->render_callback = func_updater;
	scribbled_paths = 0;
}

void local_visualizer::clear_paths(void) {
	paths->clear();
	render_callback(nullptr);
	renderer->render();
}

void local_visualizer::clear_paths(int count) {
	for (int i = 0; i < count; i++)
		paths->pop_back();
	invalidate();
}

void local_visualizer::clear_path(void) {
	clear_paths(scribbled_paths);
	scribbled_paths = 0;
}

void local_visualizer::add_path(Path path) {
	paths->push_back(path);
}

void local_visualizer::draw_ellipse(ellipse ellipse) {
	int DIVISIONS = 36;
	std::cout << "Ellipse info\n";
	std::cout << "center_x : " << ellipse.center_x << std::endl;
	std::cout << "center_y : " << ellipse.center_y << std::endl;
	std::cout << "a : " << ellipse.a << std::endl;
	std::cout << "b : " << ellipse.b << std::endl;
	std::cout << "tilt : " << ellipse.tilt << std::endl;
	for (int i = 0; i < DIVISIONS; i++) {
		float x_i1_no_tilt = ellipse.a * cos(2 * i * PI / DIVISIONS);
		float y_i1_no_tilt = ellipse.b * sin(2 * i * PI / DIVISIONS);
		float x_i2_no_tilt = ellipse.a * cos(2 * (i+1) * PI / DIVISIONS);
		float y_i2_no_tilt = ellipse.b * sin(2 * (i+1) * PI / DIVISIONS);

		paths->push_back(Path(
			ellipse.center_x +
			x_i1_no_tilt * sin(ellipse.tilt) + y_i1_no_tilt * cos(ellipse.tilt),
			-(-ellipse.center_y -
				x_i1_no_tilt * cos(ellipse.tilt) + y_i1_no_tilt * sin(ellipse.tilt)),
			ellipse.center_x +
			x_i2_no_tilt * sin(ellipse.tilt) + y_i2_no_tilt * cos(ellipse.tilt),
			-(-ellipse.center_y -
				x_i2_no_tilt * cos(ellipse.tilt) + y_i2_no_tilt * sin(ellipse.tilt))
		));
	}
	invalidate();
	return;
}

void local_visualizer::show_edges(std::vector<Line>* edge_list) {
	if (true) {
		Color color = Color(0.5f, 1.0f, 0.5f);

		for (auto& edge : *edge_list) {
			paths->push_back(Path(edge.x1, edge.y1, edge.x2, edge.y2, color));
		}
		invalidate();

		/*std::cin.ignore();
		for (auto& edge : *edge_list) {
			paths->pop_back();
		}
		invalidate();*/
	}
}

void local_visualizer::draw_ears(int data_x[], int data_y[], std::vector<std::vector<int>>* poly_indice) {
	if (true) {
		Color color[] = { Color(0.0f, 0.0f, 0.0f), Color(1.0f, 1.0f, 1.0f), Color(1.0f, 0.0f, 0.0f),
		Color(0.0f, 1.0f, 1.0f), Color(0.0f, 1.0f, 0.0f), Color(1.0f, 0.0f, 1.0f), Color(1.0f, 1.0f, 0.0f) };

		for (int k = 0; k < poly_indice->size(); k++) {
			std::vector<int> ear = poly_indice->at(k);
			for (int i = 0; i < ear.size(); i++) {
				if (i + 1 == ear.size()) {
					paths->push_back(Path(data_x[ear[i]], data_y[ear[i]], data_x[ear[0]], data_y[ear[0]], color[k % 7]));
					break;
				}
				paths->push_back(Path(data_x[ear[i]], data_y[ear[i]], data_x[ear[i + 1]], data_y[ear[i + 1]], color[k % 7]));
			}
		}
		invalidate();
	}
}

void local_visualizer::visualize_ear(int data_x[], int data_y[], std::vector<int>* ear, bool clear_again) {
	if (true) {
		Color color = Color(0.0f, 0.0f, 0.0f);

		for (int i = 0; i < ear->size(); i++) {
			if (i + 1 == ear->size()) {
				paths->push_back(Path(data_x[ear->at(i)], data_y[ear->at(i)], data_x[ear->at(0)], data_y[ear->at(0)], color));
				break;
			}
			paths->push_back(Path(data_x[ear->at(i)], data_y[ear->at(i)], data_x[ear->at(i + 1)], data_y[ear->at(i + 1)], color));
		}
		invalidate();


		std::cin.ignore();
		if (clear_again) {
			for (int i = 0; i < ear->size(); i++) {
				paths->pop_back();
			}
			invalidate();
		}
		else {
			scribbled_paths += ear->size();
		}
	}
}

void local_visualizer::invalidate(void) {
	render_callback(nullptr);
	renderer->render();
}

void elliptical_approx::init(std::vector<Node>* node_list, int width, int height, RenderClass* renderer,
	std::vector<Path>* paths, void (*func_updater)(std::vector<float>*)) {
	this->node_list = node_list;
	this->GRID_WIDTH = width;
	this->GRID_HEIGHT = height;
	this->render_agent = local_visualizer(GRID_WIDTH, GRID_HEIGHT, renderer, paths, func_updater);
	convex_clustering clusterer = convex_clustering(&render_agent);
	this->cluster = &clusterer;

	contour_extractor2();
	//locate_obstacles();

	//ellipse ellipse1 = ellipse(77.04f, 25.04f, 15.09f, 21.54f, 46.69 * PI / 180);
	//cluster->clustering_2(node_list, GRID_WIDTH);
}

void elliptical_approx::merge_strips(int row, int index2, polygon2D* obstacle, std::vector<std::vector<coord>>* strips_list) {
	obstacle->vertices.push_back(strips_list->at(row).at(index2).a);
	if (obstacle->looseBounds.bottom < row)
		obstacle->looseBounds.bottom = row;
	if (obstacle->looseBounds.left > strips_list->at(row).at(index2).a % GRID_WIDTH)
		obstacle->looseBounds.left = strips_list->at(row).at(index2).a % GRID_WIDTH;

	if (row + 1 < GRID_HEIGHT) {
		if (!strips_list->at(row + 1).empty()) {
			for (int i = 0; i < strips_list->at(row + 1).size(); i++) {
				if ((strips_list->at(row + 1).at(i).a % GRID_WIDTH <= strips_list->at(row).at(index2).b % GRID_WIDTH)
					&& (strips_list->at(row + 1).at(i).b % GRID_WIDTH >= strips_list->at(row).at(index2).a % GRID_WIDTH)) {
					merge_strips(row + 1, i, obstacle, strips_list);
					break;
				}
			}
		}
	}

	obstacle->vertices.push_back(strips_list->at(row).at(index2).b);
	if (obstacle->looseBounds.top > row)
		obstacle->looseBounds.top = row;
	if (obstacle->looseBounds.right < strips_list->at(row).at(index2).b % GRID_WIDTH)
		obstacle->looseBounds.right = strips_list->at(row).at(index2).b % GRID_WIDTH;
	strips_list->at(row).erase(strips_list->at(row).begin() + index2);
}

void elliptical_approx::locate_obstacles(void) {
	bool* checklist = (bool*)malloc(node_list->size() * sizeof(bool));

	std::vector<std::vector<coord>> strips(GRID_HEIGHT);

	int change_over_point = 0;
	bool obstacle_flag = false;
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			int index = i * GRID_WIDTH + j;
			if (node_list->at(index).type == BASE_TAKEN) {
				if (!obstacle_flag) {
					change_over_point = index;
					obstacle_flag = true;
				}
			}
			else {
				if (obstacle_flag) {
					strips[i].push_back(coord(change_over_point, index - 1));
					change_over_point = index;
					obstacle_flag = false;
				}
			}
		}
		if (node_list->at(change_over_point).type == BASE_TAKEN)
			strips[i].push_back(coord(change_over_point, (i + 1) * GRID_WIDTH - 1));
		change_over_point = 0;
		obstacle_flag = false;
	}
	//std::cin.ignore();

	render_agent.clear_paths();
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < strips[i].size(); j++) {
			render_agent.add_path(Path(strips[i].at(j).a, strips[i].at(j).b));
		}
		std::cout << std::endl;
	}
	render_agent.invalidate();

	polygon2D obstacle = polygon2D(node_list, GRID_WIDTH);
	for (int i = 0; i < strips.size(); i++) {
		int count = 0;
		while (!strips[i].empty()) {
			obstacle.looseBounds = rectangularBound(strips.at(i).at(0).a % GRID_WIDTH, strips.at(i).at(0).b % GRID_WIDTH, i, i);
			merge_strips(i, 0, &obstacle, &strips);
			if (obstacle.vertices.size() < 3)
				continue;
			obstacles.push_back(obstacle);
			obstacles.at(obstacles.size() - 1).uniqueID = obstacles.size();
			obstacle = polygon2D(node_list, GRID_WIDTH);
		}
	}

	std::cout << "Obstacle count = " << obstacles.size() << std::endl;
	//std::cin.ignore();
	render_agent.clear_paths();
	std::cout << "obstacle count : " << obstacles.size() << std::endl;
	for (auto& obstacle : obstacles) {
		obstacle.reconstruct_edges();
		std::cout << "obstacle edge count : " << obstacle.vertices.size() << std::endl;
		for (int i = 0; i < obstacle.vertices.size(); i++) {
			if (i == obstacle.vertices.size() - 1)
				render_agent.add_path(Path(obstacle.vertices.at(i), obstacle.vertices.at(0)));
			else
				render_agent.add_path(Path(obstacle.vertices.at(i), obstacle.vertices.at(i + 1)));
		}
	}
	render_agent.invalidate();
}

void elliptical_approx::find_in(ellipse ellipse1) {
	std::cout << "environment variables | (WIDTH, HEIGHT) -> (" << GRID_WIDTH << ", " << GRID_HEIGHT << ")\n";
	for (int i = 0; i < node_list->size(); i++) {
		int x = i % GRID_WIDTH;
		int y = i / GRID_WIDTH;
		if (ellipse1.isInside(node_list->at(i).x*GRID_WIDTH/2, node_list->at(i).y*GRID_HEIGHT/2, i))
			node_list->at(i).type = GOAL;
		else
			node_list->at(i).type = START;

		if (false) {
			std::cin.ignore();
			render_agent.invalidate();
		}
	}
}

void elliptical_approx::find_circle1(void) {
	int deviation = 100000000;
	float radii = 0.0;
	for (int r = 1; r < GRID_WIDTH / 2 && r < GRID_HEIGHT / 2; r++) {
		int mis_classified_count = 0;
		for (int i = 0; i < node_list->size(); i++) {
			ellipse circle = ellipse(0.0f, 0.0f, r, r, 0.0 * PI / 180);
			bool verdict = circle.isInside(node_list->at(i).x * GRID_WIDTH / 2, node_list->at(i).y * GRID_HEIGHT / 2, i);
			if (verdict && node_list->at(i).type == BASE_EMPTY)
				mis_classified_count++;
			if (!verdict && node_list->at(i).type == BASE_TAKEN)
				mis_classified_count++;
		}
		if (mis_classified_count < deviation) {
			deviation = mis_classified_count;
			radii = r;
		}
	}
	std::cout << "radii == " << radii << std::endl;
	std::cout << "deviation == " << deviation << std::endl;
	this->render_agent.draw_ellipse(ellipse(0.0f, 0.0f, radii, radii, 0.0 * PI / 180));
}

void elliptical_approx::find_circle2(void) {
	int deviation = 100000000;
	float radii = 0.0;
	float major_a = 0.0;
	float minor_b = 0.0;
	float center_x = 0.0;
	float center_y = 0.0;
	float tilt_ellip = 0;
	for (int tilt = 0; tilt < 360; tilt++) {
		std::cout << "tilt : " << tilt << std::endl;
		for (int x = 1; x < GRID_WIDTH / 2; x++) {
			for (int y = 1; y < GRID_HEIGHT / 2; y++) {
				for (int a = -GRID_WIDTH / 2; a < GRID_WIDTH / 2; a++) {
					for (int b = -GRID_HEIGHT / 2; b < GRID_HEIGHT / 2; b++) {
						int mis_classified_count = 0;
						for (int i = 0; i < node_list->size(); i++) {
							ellipse circle = ellipse(x, y, a, b, tilt * PI / 180);
							bool verdict = circle.isInside(node_list->at(i).x * GRID_WIDTH / 2, node_list->at(i).y * GRID_HEIGHT / 2, i);
							if (verdict && node_list->at(i).type == BASE_EMPTY)
								mis_classified_count++;
							if (!verdict && node_list->at(i).type == BASE_TAKEN)
								mis_classified_count++;
						}
						if (mis_classified_count < deviation) {
							deviation = mis_classified_count;
							major_a = a;
							minor_b = b;
							center_x = x;
							center_y = y;
							tilt_ellip = tilt;
						}
					}
				}
			}
		}
	}
	std::cout << "center_x == " << center_x << std::endl;
	std::cout << "center_y == " << center_y << std::endl;
	std::cout << "major_a == " << major_a << std::endl;
	std::cout << "minor_b == " << minor_b << std::endl;
	std::cout << "tilt_ellip == " << tilt_ellip << std::endl;
	std::cout << "deviation == " << deviation << std::endl;
	this->render_agent.draw_ellipse(ellipse(center_x, center_y, major_a, minor_b, tilt_ellip * PI / 180));
}

void elliptical_approx::find_ellipse1(void) {
	std::cout << GRID_WIDTH << std::endl;
	int** map = new int* [GRID_HEIGHT];
	int left_bound = GRID_WIDTH - 1, right_bound = 0, top_bound = GRID_HEIGHT - 1, bottom_bound = 0;

	for (int i = 0; i < GRID_HEIGHT; i++) {
		map[i] = new int[GRID_WIDTH];
	}
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			map[i][j] = node_list->at(i * GRID_WIDTH + j).type;
		}
	}
	for (int j = 0; j < GRID_HEIGHT; j++) {
		for (int i = 0; i < GRID_WIDTH; i++) {
			if (j > 0 && i > 0 && j < GRID_HEIGHT - 1 && i < GRID_WIDTH - 1) {
				int up = node_list->at((j-1) * GRID_WIDTH + i).type;
				int down = node_list->at((j+1) * GRID_WIDTH + i).type;
				int left = node_list->at(j * GRID_WIDTH + (i-1)).type;
				int right = node_list->at(j * GRID_WIDTH + (i+1)).type;
				//if (up != down || left != right)
				if (map[j - 1][i] != map[j + 1][i] || map[j][i - 1] != map[j][i + 1]) {
					node_list->at(j * GRID_WIDTH + i).type = START;
					points.push_back(Point(i, j));
					if (left_bound > i)
						left_bound = i;
					if (right_bound < i)
						right_bound = i;
					if (top_bound > j)
						top_bound = j;
					if (bottom_bound < j)
						bottom_bound = j;
				}
				else {
					node_list->at(j * GRID_WIDTH + i).type = GOAL;
				}
			}
		}
	}

	column_vector params(5);
	//params = (left_bound + right_bound) / 2, (top_bound + bottom_bound) / 2, std::abs(left_bound - right_bound) / 2, std::abs(top_bound - right_bound) / 2, 45;
	params = 77.980, 23.878, 19.59261, 15.90431, 56.754;

	params(0, 0) /= GRID_WIDTH;
	params(1, 0) /= GRID_HEIGHT;
	params(2, 0) /= GRID_WIDTH;
	params(3, 0) /= GRID_HEIGHT;
	params(4, 0) /= 360;

	std::cout << "Starting params : \n";
	std::cout << "left : " << left_bound << "\n";
	std::cout << "right : " << right_bound << "\n";
	std::cout << "top : " << top_bound << "\n";
	std::cout << "bottom : " << bottom_bound << "\n";
	printf("x0 = %f;\n", params(0, 0));
	printf("y0 = %f;\n", params(1, 0));
	printf("a = %f;\n", params(2, 0));
	printf("b = %f;\n", params(3, 0));
	printf("alpha = %f;\n", params(4, 0));

	/*dlib::find_min_using_approximate_derivatives(
		dlib::cg_search_strategy(),
		dlib::objective_delta_stop_strategy(1).be_verbose(),
		[&](const column_vector& a) {
			return this->rateCurveElliptical(a);
		},
		params,
			-1);*/
	column_vector lb = {0,0,0,0, 0};
	column_vector ub = { 1, 1, 1, 1, 1 };

	dlib::find_min_box_constrained(dlib::bfgs_search_strategy(),
		dlib::objective_delta_stop_strategy(1).be_verbose(),
		[&](const column_vector& a) {
			return this->rateCurveElliptical(a);
		}, dlib::derivative([&](const column_vector& a) {
			return this->rateCurveElliptical(a);
			}),
			params, lb, ub);

	params(0, 0) *= GRID_WIDTH;
	params(1, 0) *= GRID_HEIGHT;
	params(2, 0) *= GRID_WIDTH;
	params(3, 0) *= GRID_HEIGHT;
	params(4, 0) *= 360;

	std::cout << "Optimized params : \n";
	printf("x0 = %f;\n", params(0, 0));
	printf("y0 = %f;\n", params(1, 0));
	printf("a = %f;\n", params(2, 0));
	printf("b = %f;\n", params(3, 0));
	printf("alpha = %f;\n", params(4, 0));

	this->render_agent.draw_ellipse(ellipse(params(0, 0), params(1, 0), params(2, 0), params(3, 0), params(4, 0) * PI / 180));
}

void elliptical_approx::call_next_left(Node* boundary_cell, int dir, Node* stopping_node) {
	Node* next_node = nullptr;
	switch (dir) {
	case 0:		next_node = boundary_cell->left;		break;
	case 1:		next_node = boundary_cell->bottom;		break;
	case 2:		next_node = boundary_cell->right;		break;
	case 3:		next_node = boundary_cell->top;			break;
	default:	next_node = nullptr;					break;
	}

	boundary_cell->boundary_left = next_node;
	next_node->boundary_right = boundary_cell;
	if (next_node == stopping_node)
		return;
	contour_builder(next_node, true, false, dir, stopping_node);
}

void elliptical_approx::call_next_right(Node* boundary_cell, int dir, Node* stopping_node) {
	render_agent.invalidate();
	std::cin.ignore();
	Node* next_node = nullptr;
	switch (dir) {
	case 0:		next_node = boundary_cell->left;		break;
	case 1:		next_node = boundary_cell->bottom;		break;
	case 2:		next_node = boundary_cell->right;		break;
	case 3:		next_node = boundary_cell->top;			break;
	default:	next_node = nullptr;					break;
	}

	boundary_cell->boundary_right = next_node;
	next_node->boundary_left = boundary_cell;
	render_agent.add_path(Path(boundary_cell, next_node));
	if (next_node == stopping_node)
		return;
	contour_builder(next_node, false, true, dir, stopping_node);
}

void elliptical_approx::contour_builder(Node* boundary_cell, bool search_left, bool search_right, int last_operation, Node* stopping_node) {
	// for a given end point; locate its left and right neighbour.
	bool bounds[4] = { false, false, false, false };
	bounds[0] = (boundary_cell->left->type == BASE_EMPTY);
	bounds[1] = (boundary_cell->bottom->type == BASE_EMPTY);
	bounds[2] = (boundary_cell->right->type == BASE_EMPTY);
	bounds[3] = (boundary_cell->top->type == BASE_EMPTY);

	std::cout << " [CELL] : x = " << boundary_cell->x << ", y = " << boundary_cell->y << 
		". Boundaries : {" << bounds[0] << ", " << bounds[1] << ", " << bounds[2] << ", " << bounds[3] << "}\n";
	//std::cin.ignore();

	// -- termination step --> if all neighbours are BASE_TAKEN. STOP!!!
	int first_boundary = -1;
	for (int i = 0; i < 4; i++) {
		if (bounds[i]){
			first_boundary = i;
			break;
		}
		if (i == 3){
			// checkout for corner boundary (plus-center case) condition
			// modify the last call to skip this call and forward it to next point
			if (search_left) {
				//call_next_left(boundary_cell, (last_operation + 3) % 4, stopping_node);
			}
			if (search_right) {
				call_next_right(boundary_cell, (last_operation + 1) % 4, stopping_node);
			}
			// terminate function here
			return;
		}
	}

	// -- First find the boundary direction (left/right/top/bottom).
	// first_boundary holds that info;
	
	// -- move clockwise/anti-clockwise from there to spot neighbour
	// ** left neighbour : 
	/*if (search_left) {
		for (int i = 1; i < 4; i++) {
			if (bounds[(first_boundary + i) % 4] == false) {
				call_next_left(boundary_cell, (first_boundary + i) % 4, stopping_node);
				break;
			}
		}
	}*/
	// ** right neighbour : 
	if (search_right) {
		for (int i = 1; i < 4; i++) {
			if (bounds[(4 + first_boundary - i) % 4] == false) {
				call_next_right(boundary_cell, (4 + first_boundary - i) % 4, stopping_node);
				break;
			}
		}
	}
}

void elliptical_approx::contour_analyzer(void) {

}

void elliptical_approx::contour_extractor2(void) {
	int first_index = -1000;
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			if (node_list->at(i * GRID_HEIGHT + j).type == BASE_TAKEN)
				first_index = i * GRID_HEIGHT + j;
		}
	}

	if (first_index < 0 || first_index >= node_list->size())
		return;

	render_agent.invalidate();
	contour_builder(&node_list->at(first_index), true, true, 0, &node_list->at(first_index));

	/*Node* start_node = &node_list->at(first_index);
	Node* next_node = node_list->at(first_index).boundary_left;
	render_agent.add_path(Path(start_node, next_node));
	render_agent.invalidate();
	std::cout << " [PATH] : {(" << start_node->x << ", " << start_node->y << ") ; (" << next_node->x << ", " << next_node->y << ")}\n";

	while (next_node != start_node) {
		start_node = next_node;
		next_node = start_node->boundary_left;
		/*render_agent.add_path(Path(start_node, next_node));
		render_agent.invalidate();*
		std::cout << " [PATH] : {(" << start_node->x << ", " << start_node->y << ") ; (" << next_node->x << ", " << next_node->y << ")}\n";
	}*/
}

void elliptical_approx::contour_extractor(void) {
	std::cout << GRID_WIDTH << std::endl;
	int** map = new int* [GRID_HEIGHT];
	int left_bound = GRID_WIDTH - 1, right_bound = 0, top_bound = GRID_HEIGHT - 1, bottom_bound = 0;
	int first_index = -5000;

	for (int i = 0; i < GRID_HEIGHT; i++) {
		map[i] = new int[GRID_WIDTH];
	}
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			map[i][j] = node_list->at(i * GRID_WIDTH + j).type;
		}
	}
	for (int j = 0; j < GRID_HEIGHT; j++) {
		for (int i = 0; i < GRID_WIDTH; i++) {
			if (j > 0 && i > 0 && j < GRID_HEIGHT - 1 && i < GRID_WIDTH - 1) {
				int up = node_list->at((j - 1) * GRID_WIDTH + i).type;
				int down = node_list->at((j + 1) * GRID_WIDTH + i).type;
				int left = node_list->at(j * GRID_WIDTH + (i - 1)).type;
				int right = node_list->at(j * GRID_WIDTH + (i + 1)).type;
				//if (up != down || left != right)
				if (map[j - 1][i] != map[j + 1][i] || map[j][i - 1] != map[j][i + 1]) {
					if (first_index == -5000)
						first_index = j * GRID_WIDTH + i;
					node_list->at(j * GRID_WIDTH + i).type = START;
					points.push_back(Point(i, j));
					std::cout << " [" << i << ", " << j << "], ";
					if (left_bound > i)
						left_bound = i;
					if (right_bound < i)
						right_bound = i;
					if (top_bound > j)
						top_bound = j;
					if (bottom_bound < j)
						bottom_bound = j;
				}
				else {
					node_list->at(j * GRID_WIDTH + i).type = GOAL;
				}
			}
		}
	}

	bool* travel_list = new bool[node_list->size()];
	for (int i = 0; i < node_list->size(); i++)
		travel_list[i] = false;
	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_HEIGHT; j++) {
			Node* left = nullptr, * bottom = nullptr, * right = nullptr, * top = nullptr;
			if (i - 1 > 0) {
				left = &node_list->at(j * GRID_WIDTH + i - 1);
			}
			if (j + 1 < GRID_HEIGHT) {
				bottom = &node_list->at((j + 1) * GRID_WIDTH + i);
			}
			if (i + 1 < GRID_WIDTH) {
				right = &node_list->at(j * GRID_WIDTH + i + 1);
			}
			if (j - 1 > 0) {
				top = &node_list->at((j - 1) * GRID_WIDTH + i);
			}
			node_list->at(j * GRID_WIDTH + i).update_neighbours(left, right, top, bottom);
		}
	}
	build_contour(&node_list->at(first_index), travel_list, first_index, points.size(), 1);
	render_agent.invalidate();
}

int elliptical_approx::contour_explorer(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int start, int pass) {
	if (pass > 4)
		return 5;
	// left -> 1
	// bottom -> 2
	// right -> 3
	// top -> 4

	switch (start) {
	case 1:
		if (node->left->type == START) {
			render_agent.add_path(Path(node, node->left));
			std::cout << "New vertice : (" << (this_node_index - 1) % GRID_WIDTH << ", " << (this_node_index - 1) / GRID_WIDTH << ")\n";
			if (build_contour(node->left, travel_list, this_node_index - 1, remaining_nodes, start-1))
				return 1;
		}
		break;
	case 2:
		if (node->bottom->type == START) {
			render_agent.add_path(Path(node, node->bottom));
			std::cout << "New vertice : (" << (this_node_index + GRID_WIDTH) % GRID_WIDTH << ", " << (this_node_index + GRID_WIDTH) / GRID_WIDTH << ")\n";
			if (build_contour(node->bottom, travel_list, this_node_index + GRID_WIDTH, remaining_nodes, start-1))
				return 2;
		}
		break;
	case 3:
		if (node->right->type == START) {
			render_agent.add_path(Path(node, node->right));
			std::cout << "New vertice : (" << (this_node_index + 1) % GRID_WIDTH << ", " << (this_node_index + 1) / GRID_WIDTH << ")\n";
			if (build_contour(node->right, travel_list, this_node_index + 1, remaining_nodes, start-1))
				return 3;
		}
		break;
	case 4:
		if (node->top->type == START) {
			render_agent.add_path(Path(node, node->top));
			std::cout << "New vertice : (" << (this_node_index - GRID_WIDTH) % GRID_WIDTH << ", " << (this_node_index - GRID_WIDTH) / GRID_WIDTH << ")\n";
			if (build_contour(node->top, travel_list, this_node_index - GRID_WIDTH, remaining_nodes, start-1))
				return 4;
		}
		break;
	default:
		break;
	}
	contour_explorer(node, travel_list, this_node_index, remaining_nodes, (start + 1) % 4, pass + 1);
	return 0;
}

bool elliptical_approx::build_contour(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int starting_param) {
	if (remaining_nodes <= 0) {
		std::cout << "Target achieved hence exiting \n";
		return false;
	}
	if (node == nullptr) {
		std::cout << "Nullptr. Hence terminating \n";
		return false;
	}
	if (this_node_index < 0 || this_node_index >= node_list->size()) {
		std::cout << "Out of Bounds. Hence preventing \n";
		return false;
	}
	if (travel_list[this_node_index]) {
		std::cout << "Already explored. Stopping\n";
		return false;
	}
	travel_list[this_node_index] = true;
	remaining_nodes -= 1;

	std::cout << " self.type : " << node->type << " left : " << node->left->type << 
		" bottom : " << node->bottom->type << " right : " << node->right->type << " top : " << node->top->type << "\n";

	render_agent.invalidate();
	std::cin.ignore();

	/*if (contour_explorer(node, travel_list, this_node_index, remaining_nodes, 1, 1))
		return true;*/
	contour_explorer(node, travel_list, this_node_index, remaining_nodes, starting_param, 1);
	return true;
}

double elliptical_approx::rateCurveElliptical(const column_vector& params)
{
	double x0 = params(0, 0) * GRID_WIDTH;
	double y0 = params(1, 0) * GRID_HEIGHT;
	double a = params(2, 0) * GRID_WIDTH;
	double b = params(3, 0) * GRID_HEIGHT;
	double alpha = params(4, 0) * 360;

	double distances = 0;

	for (Point target : points)
	{
		double distance = _DMAX;

		for (int t = 0; t <= 360; t += 5)
		{
			Point pt = ellipseParametric(x0, y0, a, b, alpha, t);

			double dCandidate = dist(pt.x, pt.y, target.x, target.y);

			distance = std::min(distance, dCandidate);
		}

		distances += distance;
	}
	printf("x0 = %f;\ty0 = %f;\ta = %f;\tb = %f;\talpha = %f;\n", x0, y0, a, b, alpha);

	// Thats the curve-fitting done.  Now incentivise slightly smoother curves.
	/*double p1c1 = dist(p1x, p1y, c1x, c1y);
	double p2c2 = dist(p2x, p2y, c2x, c2y);
	p1c1 = 0.0;
	p2c2 = 0.0;

	return distances + pow(p1c1, 0.6) + pow(p2c2, 0.6);*/
	if (a < 0 || b < 0)
		return 10000 * distances;
	return distances;
}

double elliptical_approx::dist(double x1, double y1, double x2, double y2)
{
	double x = x2 - x1;
	double y = y2 - y1;

	return (x * x) + (y * y);
}

Point elliptical_approx::ellipseParametric(double x0, double y0, double a, double b, double alpha, double t) {
	Point pt;
	pt.x = x0 + (a * cos(t * PI / 180) * cos(alpha * PI / 180) - b * sin(t * PI / 180) * sin(alpha * PI / 180));
	pt.y = y0 + (a * cos(t * PI / 180) * sin(alpha * PI / 180) + b * sin(t * PI / 180) * cos(alpha * PI / 180));
	return pt;
}

bool convex_clustering::leftOnly(float line_x1, float line_y1,
	float line_x2, float line_y2, float pt_x0, float pt_y0) {
	/*std::cout << "[LEFT ONLY] : [" << line_x1 << ", " << line_y1 << ", " << line_x2 << ", " <<
		line_y2 << ", " << pt_x0 << ", " << pt_y0 << "] result = " << ((line_x1 * (line_y2 - pt_y0)
			+ line_x2 * (pt_y0 - line_y1) + pt_x0 * (line_y1 - line_y2)) > 0) << "\n";*/
	return ((line_x1 * (line_y2 - pt_y0) + line_x2 * (pt_y0 - line_y1) + pt_x0 * (line_y1 - line_y2)) > 0);
}

bool convex_clustering::leftOn(float line_x1, float line_y1,
	float line_x2, float line_y2, float pt_x0, float pt_y0) {
	return ((line_x1 * (line_y2 - pt_y0) + line_x2 * (pt_y0 - line_y1) + pt_x0 * (line_y1 - line_y2)) >= 0);
}

float area_triangle(int x1, int y1, int x2, int y2, int x3, int y3) {
	float area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
	//printf("x1 :  %d. y1 = %d. x2 = %d. y2 = %d. x3 = %d. y3 = %d. Area = %f\n", x1, y1, x2, y2, x3, y3, area);
	return area;
}

float length(int x1, int y1, int x2, int y2) {
	float len = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	//std::cout << len << "\n";
	return len;
}

float ear_area(std::vector<int>* ear, int data_x[], int data_y[], local_visualizer* render_agent, std::vector<result>* filter, bool special) {
	float area = 0;
	bool pure = false;
	for (int i = 2; i < ear->size(); i++) {
		float del_area = area_triangle(data_x[ear->at(0)], data_y[ear->at(0)], data_x[ear->at(i - 1)], data_y[ear->at(i - 1)], data_x[ear->at(i)], data_y[ear->at(i)]);
		area += del_area;
		if (area / del_area > 0)
			pure = true;
		else
			pure = false;
	}
	float bridge_width = length(data_x[ear->at(0)], data_y[ear->at(0)], data_x[ear->at(ear->size() - 1)], data_y[ear->at(ear->size() - 1)]);
	float independency_factor = area / bridge_width;
	// print(independency_factor
	float skewness = independency_factor / bridge_width;
	//printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n", 
		//area, independency_factor, ear->size(), bridge_width, skewness);

	/*if (abs(independency_factor) > 7.0)
		render_agent->visualize_ear(data_x, data_y, ear, false);
	else
		render_agent->visualize_ear(data_x, data_y, ear, true);*/
	/*if (area > 0) {
		render_agent->visualize_ear(data_x, data_y, ear, false);
		printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n",
			area, independency_factor, ear->size(), bridge_width, skewness);
		filter->push_back(ear->at(0));
	}*/
	if (independency_factor > 0.1 || special) {
		render_agent->visualize_ear(data_x, data_y, ear, false);
		printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n",
			area, independency_factor, ear->size(), bridge_width, skewness);
		filter->push_back(result(ear->at(0), area, independency_factor, ear->size(), bridge_width, skewness));
	}
	else
		render_agent->visualize_ear(data_x, data_y, ear, true);

	//int label;
	//scanf("%d", &label);
	//results.push_back(result(area, independency_factor, ear->size(), bridge_width, skewness, pure, label));

	return independency_factor;
}

void convex_clustering::ear_handler(int edge1, int edge2, int pt, std::vector<std::vector<int>>* poly_indice) {
	std::cout << "Concave vertex identified : node " << poly_indice->at(0).at(edge2) << ". edge1 = " << poly_indice->at(0).at(edge1) << ", pt = " << poly_indice->at(0).at(pt) << "\n";
	// ear clipping logic here
	int start = edge2;
	int end = edge2;
	for (int i = 1; i < poly_indice->at(0).size(); i++) { // for a selected line segment, iterate through next coming vertices
		bool convex_test = leftOnly(
			data_x[poly_indice->at(0).at((edge1 + i) % poly_indice->at(0).size())],
			data_y[poly_indice->at(0).at((edge1 + i) % poly_indice->at(0).size())],
			data_x[poly_indice->at(0).at((edge2 + i) % poly_indice->at(0).size())],
			data_y[poly_indice->at(0).at((edge2 + i) % poly_indice->at(0).size())],
			data_x[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())],
			data_y[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())]);
		if (!convex_test) {
			std::cout<<"[EAR] : convex vertex spotted : node "<<poly_indice->at(0).at((edge2+i)% poly_indice->at(0).size())<<"\n";
			bool concave_ear = leftOnly(
				data_x[poly_indice->at(0).at(edge2)], data_y[poly_indice->at(0).at(edge2)],
				data_x[poly_indice->at(0).at(pt)], data_y[poly_indice->at(0).at(pt)],
				data_x[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())],
				data_y[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())]);
			if (concave_ear) {
				//if ((edge2+i-1)%len(poly_indice[0]) > start):
				//   end = (edge2 + i - 1) % len(poly_indice[0])
				end = (pt + i - 1) % poly_indice->at(0).size();
				std::cout<<"preventing ear from getting concave. start = "<<start<<". end = "<<end<<"\n";
				break;
			}
		}
		else {
			printf("[EAR] : concave vertex indicates termination of ear here.\n");
			end = (edge2 + i) % poly_indice->at(0).size();
			break;
		}
	}

	if (start != end) { // separate out identified ear from the raw polygon
		std::cout<<"start : "<<start<<".end : "<<end<<"\n";
		std::vector<int> ear;
		if (end < start)
			end += poly_indice->at(0).size();
		for (int i = start; i < end + 1; i++) // copy the ear points to a new list
			ear.push_back(poly_indice->at(0).at(i % poly_indice->at(0).size()));
		if (ear.size() < 3)
			return; // that is either a point or line. and not ear
		/*float factor = ear_area(&ear, data_x, data_y, render_agent);
		if (factor < 2.0) {
			return;
		}*/
		/*a, b, c, d = ear_area(ear, data_x, data_y)
		print(f"area = {a:.2f}. factor = {b:.2f}. vertices = {c}. bridge_width = {d:.2f}")
		if (b < 1.0 or (d / b > 15)) :
			return*/
		poly_indice->push_back(ear);          // append these ear coordinates to polygon dataset
		for (int i = start + 1; i < end; i++) {
			if (start + 1 >= poly_indice->at(0).size()) // remove the excess ear coordinates from the raw polygon points set
				poly_indice->at(0).erase(poly_indice->at(0).begin() + 0);
			else
				poly_indice->at(0).erase(poly_indice->at(0).begin() + start + 1); // remove the excess ear coordinates from the raw polygon points set
		}
	}
}

int convex_clustering::convex_completion_handler(int pt, int edge2, std::vector<std::vector<int>>* poly_indice) {
	bool convex_completion_test = leftOnly(data_x[poly_indice->at(0).at(1)], data_y[poly_indice->at(0).at(1)],
		data_x[poly_indice->at(0).at(0)], data_y[poly_indice->at(0).at(0)],
		data_x[poly_indice->at(0).at(pt)], data_y[poly_indice->at(0).at(pt)]);

	if (!convex_completion_test) {
		// if (pt == 0):        // break loop on success
		//   print("Computation finished == :)")
		//   return 0 # terminate any computation ahead
		if ((edge2 == poly_indice->at(0).size()) || (poly_indice->at(0).at(edge2) == 1))
			return 1;
		std::cout<<"[CONVEX COMPLETION] : triggered. node 0 will be joined with node "<<poly_indice->at(0)[edge2]<<"\n";
		std::vector<int> ear;
		for (int i = 0; i < pt; i++) // copy the ear points to a new list
			ear.push_back(poly_indice->at(0).at(i % poly_indice->at(0).size()));
		poly_indice->push_back(ear);               // append these ear coordinates to polygon dataset
		for (int i = 1; i < pt - 1; i++)
			poly_indice->at(0).erase(poly_indice->at(0).begin() + 1);       // remove the excess ear coordinates from the raw polygon points set
			// circulation_start = edge1
			return 1; // continue with the coming steps
  }
}

void print_poly_indice(std::vector<std::vector<int>>* poly_indice) {
		//-----------
	std::cout << "[\n";
	for (int i = 0; i < poly_indice->size(); i++) {
		std::cout << "[";
		for (int j = 0; j < poly_indice->at(i).size(); j++)
			std::cout << poly_indice->at(i).at(j) << " , ";
		std::cout << "\b\b]\n";
	}
	std::cout << "]\n";
	//-----------
}

void convex_clustering::clustering(void) {
	std::vector<std::vector<int>> poly_indice = { {} };
	for (int i = 0; i < 107; i++) {
		poly_indice.at(0).push_back(i);
	}
	std::cout << "Enters here \n";
	int index = 0;
	int circulation_start = poly_indice.at(0).size() - 1;
	//while (true) { // iterate continously to pick vertices on polygon one-by-one
	for (int k=0; k<1000; k++){ // iterate continously to pick vertices on polygon one-by-one
		//std::cout << "length of raw data : " << poly_indice.at(0).size() << ".Index : " << index << " \n";
		int edge1 = (index) % poly_indice.at(0).size();
		int edge2 = (index + 1) % poly_indice.at(0).size();
		int pt = (index + 2) % poly_indice.at(0).size();

		//print_poly_indice(&poly_indice);

		if (edge1 == circulation_start) { // break loop on success
			printf("Computation finished :)\n");
			break;
		}

		bool convex_completion = convex_completion_handler(pt, edge2, &poly_indice);
		if (convex_completion == 0) {
			break;
			if (convex_completion == 1) {
				index = 0;
				continue;
			}
		}

		bool concave_test = leftOnly(data_x[poly_indice[0][edge1]], data_y[poly_indice[0][edge1]],
			data_x[poly_indice[0][edge2]], data_y[poly_indice[0][edge2]],
			data_x[poly_indice[0][pt]], data_y[poly_indice[0][pt]]);
			if (concave_test) {
				ear_handler(edge1, edge2, pt, &poly_indice);
				circulation_start = edge1;
			}
			else {
				std::cout<<"convex vertex : node "<<poly_indice[0][edge2]<<".edge1 = "<<poly_indice[0][edge1]<<", pt = "<<poly_indice[0][pt]<<"\n";
			}
				index = (index + 1) % poly_indice.at(0).size();
			// plot_current_distribution()
	}

	print_poly_indice(&poly_indice);
	render_agent->draw_ears(data_x, data_y, &poly_indice);
}

void convex_clustering::filter_dimples(std::vector<result>* traversal, std::vector<result>* filter, int filter_size) {
	std::cout << " length = " << data_size << "\n";
	for (int i = 0; i < traversal->size(); i++) {
		std::vector<int> ear;
		for (int k = 0; k < filter_size; k++) {
			//std::cout << ((traversal->at(i).i + k) % data_size) << "\n";
			ear.push_back((traversal->at(i).i + k) % data_size);
		}
		ear_area(&ear, data_x, data_y, render_agent, filter, false);
	}
	std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	std::cin.ignore();
}

void convex_clustering::cleaning_dimples(std::vector<result>* dimples) {
	int PARAM_VICINITY_RESOLUTION = 3;
	for (int i = 0; i < dimples->size(); i++) {
		int val1 = dimples->at(i % dimples->size()).i;
		int val2 = dimples->at((i + 1) % dimples->size()).i;
		if (val2 < val1)
			val2 += data_size;
		for (int j = 1; j < dimples->size(); j++) {
			int val2 = dimples->at((i + j) % dimples->size()).i;
			if (val2 < val1)
				val2 += data_size;
			if (val2 - val1 > PARAM_VICINITY_RESOLUTION)
				break;
			else {
				if (dimples->at(i % dimples->size()).independence < dimples->at((i + j) % dimples->size()).independence) {
					dimples->erase(dimples->begin() + i % dimples->size());
				}
				else {
					dimples->erase(dimples->begin() + (i + j) % dimples->size());
				}
				j--;
			}
		}
	}
}

void convex_clustering::clustrify(std::vector<result>* dimples, std::vector<std::vector<int>>* clustered_data) {
	int PARAM_CLUSTER_FACTOR_THRESHOLD = 10;
	std::vector<result> results;
	

	std::cout << "Following pairs will be joinined \n";
	for (int i = 0; i < dimples->size(); i++) {
		std::vector<int> ear;
		int start = dimples->at(i).i + 1;
		int end = dimples->at((i + 1) % dimples->size()).i + 1;
		if (end < start)
			end += data_size;
		for (int k = start; k <= end; k++) {
			//std::cout << ((k) % data_size) << "\n";
			ear.push_back((k) % data_size);
		}
		ear_area(&ear, data_x, data_y, render_agent, &results, true);
		if (abs(results.at(results.size() - 1).independence) > PARAM_CLUSTER_FACTOR_THRESHOLD) {
			std::cout << start % data_size << " will be joinined with " << end % data_size << "\n";
			// add new poly-loop to poly-indices
			clustered_data->push_back({});
			int index = clustered_data->size() - 1;
			for (int i = start; i <= end; i++) {
					clustered_data->at(index).push_back(i % data_size);
			}
			print_poly_indice(clustered_data);
			// cut that part from original poly
			for (int k = 0; k < clustered_data->at(0).size(); k++) {
				if (clustered_data->at(0).at(k) == start + 1) {
					for (int i = start + 1; i < end; i++) {
						if (k < clustered_data->at(0).size())
							clustered_data->at(0).erase(clustered_data->at(0).begin() + k);
						else
							clustered_data->at(0).erase(clustered_data->at(0).begin());
					}
					break;
				}
			}
		}
	}
}

void convex_clustering::clustering_2(std::vector<Node>* node_list, int GRID_WIDTH) {
	std::vector<result> initial_list;
	for (int i = 0; i < data_size; i++) {
		initial_list.push_back(result(i));
	}

	//locating dimples
	std::vector<result> filtered;
	filter_dimples(&initial_list, &filtered, 7);

	std::vector<result> second_filter;
	filter_dimples(&filtered, &second_filter, 5);

	std::vector<result> third_filter;
	filter_dimples(&second_filter, &third_filter, 4);

	std::vector<result> fourth_filter;
	filter_dimples(&third_filter, &fourth_filter, 3);

	/*std::vector<result> fourth_filter;
	filter_dimples(&second_filter, &fourth_filter, 3);*/
	
	// extracting good dimples
	cleaning_dimples(&fourth_filter);
	
	//highlights extracted dimples
	for (int i = 0; i < fourth_filter.size(); i++) {
		std::cout << fourth_filter[i].i + 1 << "\n";
		node_list->at(data_y[fourth_filter[i].i + 1] * GRID_WIDTH + data_x[fourth_filter[i].i + 1]).type = START;
	}

	// pair dimples
	std::vector<std::vector<int>> clustered_data = { {} };
	for (int i = 0; i < data_size; i++) {
		clustered_data.at(0).push_back(i);
	}
	clustrify(&fourth_filter, &clustered_data);

	std::cout << "final outcome\n";
	print_poly_indice(&clustered_data);


	std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	std::cin.ignore();

	render_agent->draw_ears(data_x, data_y, &clustered_data);
}
