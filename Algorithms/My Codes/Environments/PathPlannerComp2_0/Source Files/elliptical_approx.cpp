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

void local_visualizer::draw_ears_v2(std::vector<Point>* points, std::vector<std::vector<int>>* poly_indice) {
	if (true) {
		Color color[] = { Color(0.0f, 0.0f, 0.0f), Color(1.0f, 1.0f, 1.0f), Color(1.0f, 0.0f, 0.0f),
		Color(0.0f, 1.0f, 1.0f), Color(0.0f, 1.0f, 0.0f), Color(1.0f, 0.0f, 1.0f), Color(1.0f, 1.0f, 0.0f) };

		for (int k = 0; k < poly_indice->size(); k++) {
			std::vector<int> ear = poly_indice->at(k);
			for (int i = 0; i < ear.size(); i++) {
				if (i + 1 == ear.size()) {
					paths->push_back(Path(points->at(ear[i]).x, points->at(ear[i]).y, points->at(ear[0]).x, points->at(ear[0]).y, color[k % 7]));
					break;
				}
				paths->push_back(Path(points->at(ear[i]).x, points->at(ear[i]).y, points->at(ear[i + 1]).x, points->at(ear[i + 1]).y, color[k % 7]));
			}
		}
		invalidate();
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

void local_visualizer::visualize_ear_2(std::vector<Point>*points, std::vector<int>* ear, bool clear_again) {
	if (true) {
		Color color = Color(0.0f, 0.0f, 0.0f);

		for (int i = 0; i < ear->size(); i++) {
			if (i + 1 == ear->size()) {
				paths->push_back(Path(points->at(ear->at(i)).x, points->at(ear->at(i)).y, 
					points->at(ear->at(0)).x, points->at(ear->at(0)).y, color));
				break;
			}
			paths->push_back(Path(points->at(ear->at(i)).x, points->at(ear->at(i)).y,
				points->at(ear->at(i+1)).x, points->at(ear->at(i+1)).y, color));
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
	//cluster->clustering_3(node_list, GRID_WIDTH, points);
}
/*
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
					points.at(points.size() - 1).push_back(int_point(i, j));
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
			-1);*
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
*/
void elliptical_approx::call_next_counter_clockwise(Node* boundary_cell, int dir, Node* stopping_node, bool forward) {
	Node* next_node = nullptr;
	switch (dir) {
	case 0:		next_node = boundary_cell->left;				break;
	case 1:		next_node = boundary_cell->left->bottom;		break;
	case 2:		next_node = boundary_cell->bottom;				break;
	case 3:		next_node = boundary_cell->bottom->right;		break;
	case 4:		next_node = boundary_cell->right;				break;
	case 5:		next_node = boundary_cell->right->top;			break;
	case 6:		next_node = boundary_cell->top;					break;
	case 7:		next_node = boundary_cell->top->left;			break;
	default:	next_node = nullptr;							break;
	}

	boundary_cell->boundary_left = next_node;
	next_node->boundary_right = boundary_cell;

	if (next_node->x == stopping_node->x && next_node->y == stopping_node->y) {
		//if (next_node == stopping_node) {
		std::cout << "breaking out...\n";
		return;
	}
	contour_builder_v2(next_node, true, false, dir, stopping_node);
}

void GL_to_image_coords_temp(int GRID_WIDTH, int GRID_HEIGHT, float x_GL, float y_GL, int* x_GL_int, int* y_GL_int) {
	float x_off = (float)GRID_WIDTH / 2;
	float y_off = (float)GRID_HEIGHT / 2;
	int scale = ((GRID_WIDTH) > (GRID_HEIGHT) ? (GRID_WIDTH) : (GRID_HEIGHT)) / 2;

	float x_image = x_GL * scale + x_off;
	float y_image = y_off - y_GL * scale;
	*x_GL_int = round(x_image * 100) / 100;
	*y_GL_int = round(y_image * 100) / 100;
}

void elliptical_approx::contour_builder_v2(Node* boundary_cell, bool search_left, bool search_right, int last_operation, Node* stopping_node) {
	// for a given end point; locate its left and right neighbour.
	bool bounds[8] = { false, false, false, false, false, false, false, false };
	if (boundary_cell->left == nullptr) {
		bounds[0] = true;
		bounds[1] = true;
	}
	else {
		bounds[0] = (boundary_cell->left->type == BASE_EMPTY);
		if (boundary_cell->left->bottom == nullptr)
			bounds[1] = true;
		else
			bounds[1] = (boundary_cell->left->bottom->type == BASE_EMPTY);
	}
	if (boundary_cell->bottom == nullptr) {
		bounds[2] = true;
		bounds[3] = true;
	}
	else {
		bounds[2] = (boundary_cell->bottom->type == BASE_EMPTY);
		if (boundary_cell->bottom->right == nullptr)
			bounds[3] = true;
		else
			bounds[3] = (boundary_cell->bottom->right->type == BASE_EMPTY);
	}
	if (boundary_cell->right == nullptr) {
		bounds[4] = true;
		bounds[5] = true;
	}
	else {
		bounds[4] = (boundary_cell->right->type == BASE_EMPTY);
		if (boundary_cell->right->top == nullptr)
			bounds[5] = true;
		else
			bounds[5] = (boundary_cell->right->top->type == BASE_EMPTY);
	}
	if (boundary_cell->top == nullptr) {
		bounds[6] = true;
		bounds[7] = true;
	}
	else {
		bounds[6] = (boundary_cell->top->type == BASE_EMPTY);
		if (boundary_cell->top->left == nullptr)
			bounds[7] = true;
		else
			bounds[7] = (boundary_cell->top->left->type == BASE_EMPTY);
	}

	// -- termination step --> if all neighbours are BASE_TAKEN. STOP!!!
	int first_boundary = -1;
	for (int i = 0; i < 8; i++) {
		if (bounds[i]) {
			first_boundary = i;
			break;
		}
		if (i == 7)
			return;
	}

	// -- First find the boundary direction (left/bottom-left/bottom/bottom-right/right/top-right/top/top-left).
	// first_boundary holds that info;

	// -- move anti-clockwise from there to spot neighbour
	// ** left neighbour : 
	if (search_left) {
		for (int i = 1; i < 8; i++) {
			if (bounds[(first_boundary + i) % 8] == false) {
				call_next_counter_clockwise(boundary_cell, (first_boundary + i) % 8, stopping_node, false);
				break;
			}
		}
	}
}

void elliptical_approx::boundary_pointers_network_to_points_list(Node* start_node, Node* next_node,int first_index, int poly_index) {
	//render_agent.add_path(Path(start_node, next_node));
	int ptx, pty;
	GL_to_image_coords_temp(GRID_WIDTH, GRID_HEIGHT, start_node->x, start_node->y, &ptx, &pty);
	points.at(poly_index).push_back(int_point(ptx, pty));

	while (true) {
		start_node = next_node;
		next_node = start_node->boundary_left;
		int ptx, pty;
		GL_to_image_coords_temp(GRID_WIDTH, GRID_HEIGHT, start_node->x, start_node->y, &ptx, &pty);
		points.at(poly_index).push_back(int_point(ptx, pty));
		if (next_node->x == node_list->at(first_index).x && next_node->y == node_list->at(first_index).y) {
			std::cout << "extraction_complete\n";
			break;
		}
	}
	render_agent.invalidate();

	// add these boundary paths to render queue
	for (int i = 0; i < points.at(poly_index).size(); i++) {
		render_agent.add_path(
			Path(
				points.at(poly_index).at(i % points.at(poly_index).size()).x,
				points.at(poly_index).at(i % points.at(poly_index).size()).y,
				points.at(poly_index).at((i + 1) % points.at(poly_index).size()).x,
				points.at(poly_index).at((i + 1) % points.at(poly_index).size()).y,
				Color(1.0, 1.0, 1.0)
			)
		);
	}
	render_agent.invalidate();
}

void elliptical_approx::contour_analyzer(int first_index, std::vector<std::vector<strip>>* strips) {
	render_agent.invalidate();
	// build boundary pointers network
	contour_builder_v2(&node_list->at(first_index), true, false, 0, &node_list->at(first_index));

	// add a new entry for this polygon
	points.push_back(std::vector<int_point>());
	int poly_index = points.size() - 1;

	Node* start_node = &node_list->at(first_index);
	Node* next_node = node_list->at(first_index).boundary_left;
	boundary_pointers_network_to_points_list(start_node, next_node, first_index, poly_index);

	// store this boundary point data with "y" as key and "x" as field
	std::vector<std::vector<int>> edges;
	for (int i = 0; i < GRID_HEIGHT; i++) {
		edges.push_back(std::vector<int>());
	}
	for (int i = 0; i < points.at(poly_index).size(); i++) {
		int_point cell_point = points.at(poly_index).at(i);
		int cell_x = cell_point.x;
		int cell_y = cell_point.y;
		
		edges.at(cell_y).push_back(cell_x);
	}

	// for each "y" key in edges. sort "x" values and trim out unnecessary ones
	for (int i = 0; i<edges.size(); i++) {
		if (edges.at(i).empty()) {
			std::cout << "x x x x x\n";
			continue;
		}
		for (int j = 0; j < edges.at(i).size(); j++) {
			int biggest = edges.at(i).at(j);
			int index = j;
			for (int k = j; k < edges.at(i).size(); k++) {
				if (edges.at(i).at(k) > biggest) {
					biggest = edges.at(i).at(k);
					index = k;
				}
			}
			//std::cout << biggest << " ";
			edges.at(i).erase(edges.at(i).begin() + index);
			edges.at(i).insert(edges.at(i).begin(), biggest);
		}
		for (int j = 0; j < edges.at(i).size(); j++) {
			Node node = node_list->at(i * GRID_WIDTH + edges.at(i).at(j));
			//std::cout << node.left->type << "   " << node.right->type << "\n";
			bool flag_left_taken = false;
			bool flag_right_taken = false;
			if (node.left != nullptr)
				flag_left_taken = (node.left->type == BASE_TAKEN);
			if (node.right != nullptr)
				flag_right_taken = (node.right->type == BASE_TAKEN);

			if (flag_left_taken && flag_right_taken) {
				edges.at(i).erase(edges.at(i).begin() + j);
				j--;
			}
		}
		for (int j = 0; j < edges.at(i).size(); j++) {
			std::cout << edges.at(i).at(j) << " ";
		}
		std::cout << "\n";
	}

	// append these edge info to strips list
	for (int i = 0; i < edges.size(); i++) {
		for (int j = 0; j < edges.at(i).size(); j++) {
			if (edges.at(i).empty())
				continue;
			int start = edges.at(i).at(0);
			int end = edges.at(i).at(1);
			for (int k = 0; k < strips->at(i).size(); k++) {
				if (end < strips->at(i).at(k).start) {
					strips->at(i).insert(strips->at(i).begin() + k, strip(start, end));
					edges.at(i).erase(edges.at(i).begin());
					edges.at(i).erase(edges.at(i).begin());
					break;
				}
			}
			if (!edges.at(i).empty()) {
				if (edges.at(i).at(0) == start) {
					strips->at(i).push_back(strip(start, end));
					edges.at(i).erase(edges.at(i).begin());
					edges.at(i).erase(edges.at(i).begin());
					continue;
				}
			}
		}
	}
}

void elliptical_approx::contour_extractor2(void) {
	int first_index = -1000;
	std::vector<std::vector<strip>> strips;
	for (int i = 0; i < GRID_HEIGHT; i++) {
		strips.push_back({});
	}
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			bool continue_flag = false;
			for (int k = 0; k < strips.at(i).size(); k++) {
				if (strips.at(i).at(k).start <= j && strips.at(i).at(k).end>=j) {
					j = strips.at(i).at(k).end;
					continue_flag = true;
					break;
				}
			}
			if (continue_flag) {
				continue;
			}

			if (node_list->at(i * GRID_WIDTH + j).type == BASE_TAKEN) {
				first_index = i * GRID_WIDTH + j;
				contour_analyzer(first_index, &strips);
			}
		}
	}
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

float convex_clustering::ear_area_2(std::vector<int>* ear, std::vector<result>* filter, bool special) {
	float area = 0;
	bool pure = false;
	for (int i = 2; i < ear->size(); i++) {
		float del_area = area_triangle(points.at(ear->at(0)).x, points.at(ear->at(0)).y,
			points.at(ear->at(i - 1)).x, points.at(ear->at(i - 1)).y,
			points.at(ear->at(i)).x, points.at(ear->at(i)).y);
		area += del_area;
		if (area / del_area > 0)
			pure = true;
		else
			pure = false;
	}

	float bridge_width = length(points.at(ear->at(0)).x, points.at(ear->at(0)).y,
		points.at(ear->at(ear->size() - 1)).x, points.at(ear->at(ear->size() - 1)).y);
	float independency_factor = area / bridge_width;
	float skewness = independency_factor / bridge_width;

	if (independency_factor > 0.3 || special) {
		printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n",
			area, independency_factor, ear->size(), bridge_width, skewness);
		filter->push_back(result(ear->at(0), area, independency_factor, ear->size(), bridge_width, skewness));
		render_agent->visualize_ear_2(&points, ear, false);
	}
	else
		render_agent->visualize_ear_2(&points, ear, true);

	return independency_factor;
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

void convex_clustering::filter_dimples_2(std::vector<result>* traversal, std::vector<result>* filter, int filter_size) {
	std::cout << " length = " << data_size << "\n";
	for (int i = 0; i < traversal->size(); i++) {
		std::vector<int> ear;
		for (int k = 0; k < filter_size; k++) {
			//std::cout << ((traversal->at(i).i + k) % data_size) << "\n";
			ear.push_back((traversal->at(i).i + k) % data_size);
		}
		ear_area_2(&ear, filter, false);
	}
	std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	std::cin.ignore();
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

void convex_clustering::clustrify_v2(std::vector<result>* dimples, std::vector<std::vector<int>>* clustered_data) {
	int PARAM_CLUSTER_FACTOR_THRESHOLD = 10;
	int PARAM_FILTER_SIZE_CENTER_OFFSET_VAL = 2;
	std::vector<result> results;

	std::cout << "Following pairs will be joinined \n";
	for (int i = 0; i < dimples->size(); i++) {
		std::vector<int> ear;
		int start = (dimples->at((i) % dimples->size()).i + PARAM_FILTER_SIZE_CENTER_OFFSET_VAL)%data_size;
		int end = (dimples->at((i + 1) % dimples->size()).i + PARAM_FILTER_SIZE_CENTER_OFFSET_VAL) % data_size;
		if (end < start)
			end += data_size;
		for (int k = start; k <= end; k++) {
			//std::cout << ((k) % data_size) << "\n";
			ear.push_back((k) % data_size);
		}
		ear_area_2(&ear, &results, true);
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

void convex_clustering::clustering_3(std::vector<Node>* node_list, int GRID_WIDTH, std::vector<Point> points) {
	this->points = points;
	std::vector<result> initial_list;
	data_size = points.size();
	for (int i = 0; i < data_size; i++) {
		initial_list.push_back(result(i));
	}

	//locating dimples
	std::vector<result> filtered;

	// setting up iterators for vertices list
	std::vector<result>* old_vertices_set, * updated_vertices_set;
	old_vertices_set = &initial_list;
	updated_vertices_set = &filtered;

	int filters[4] = {8, 5, 3 };

	for (int i = 0; i < 2; i++) {
		filter_dimples_2(old_vertices_set, updated_vertices_set, filters[i]);
		
		// transfer the new content to old list
		std::vector<result>* temp = old_vertices_set;
		old_vertices_set = updated_vertices_set;
		updated_vertices_set = temp;
		updated_vertices_set->clear();
	}

	for (int i = 0; i < old_vertices_set->size(); i++) {
		std::cout << old_vertices_set->at(i).i << "     " << old_vertices_set->at(i).area << "     " << old_vertices_set->at(i).independence << "\n";
	}

	// extracting good dimples
	for (int i = 0; i < old_vertices_set->size(); i++) {
		if (old_vertices_set->at((i + 1) % old_vertices_set->size()).i - old_vertices_set->at(i).i == 1) {
			// new set of interest zone
			std::cout << "interest zone starts at : " << i;
			float max_factor = old_vertices_set->at(i).independence;
			int best_index = i;
			for (int j = 1; j < old_vertices_set->size(); j++) {
				int i_th_index = old_vertices_set->at((i + j) % old_vertices_set->size()).i;
				int last_index = old_vertices_set->at((i + j-1) % old_vertices_set->size()).i;
				if (i_th_index < last_index)
					i_th_index += data_size;

				if (i_th_index - last_index > 1) {
					// end of that set
					updated_vertices_set->push_back(old_vertices_set->at(best_index));
					std::cout << " best performer was : " << old_vertices_set->at(best_index).i << ". factor = " << max_factor << "\n";
					i = (i + j) - 1;
					break;
				}
				if (old_vertices_set->at((i + j) % old_vertices_set->size()).independence > max_factor) {
					max_factor = old_vertices_set->at((i + j) % old_vertices_set->size()).independence;
					best_index = (i + j) % old_vertices_set->size();
				}
			}
		}
	}
	std::vector<result>* temp = old_vertices_set;
	old_vertices_set = updated_vertices_set;
	updated_vertices_set = temp;
	updated_vertices_set->clear();
	
	//highlights extracted dimples
	for (int i = 0; i < old_vertices_set->size(); i++) {
		std::cout << old_vertices_set->at(i).i + 2 << "\n";
		node_list->at(points[old_vertices_set->at(i).i + 2].y * GRID_WIDTH + points[old_vertices_set->at(i).i + 2].x).type = START;
	}

	// pair dimples
	std::vector<std::vector<int>> clustered_data = { {} };
	for (int i = 0; i < data_size; i++) {
		clustered_data.at(0).push_back(i);
	}
	clustrify_v2(old_vertices_set, &clustered_data);

	std::cout << "final outcome\n";
	print_poly_indice(&clustered_data);


	std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	std::cin.ignore();

	render_agent->draw_ears_v2(&points, &clustered_data);
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