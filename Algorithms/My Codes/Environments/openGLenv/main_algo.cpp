#ifndef ALGO_A_STAR_H
#define ALGO_A_STAR_H

#include "global_variables.h"
#include "RenderClass.h"
#include "Map2D.h"
#include "mcc.h"
#include "constants.h"
#include "a_star.h"
#include "genetic.h"
#include "voronoi.h"
#include "elliptical_approx.h"
#include <vector>

// initializations
RenderClass renderer;

Map2D map_obj_1;

std::vector<Node> nodes;
std::vector<Path> paths;

a_star_algo a_star;
genetic_algo genetic;
voronoi_algo voronoi;
elliptical_approx approximater;

int algorithm_mode = VORONOI;

//constants
float current_color[] = { 0.7f, 0.15f, 0.5f };
Color color_list[] = { Color(1.0f, 0.0f, 0.0f), Color(0.7f, 0.15f, 0.5f), Color(0.2f, 0.2f, 1.0f),
							Color(0.0f, 1.0f, 0.0f), Color(0.3f, 0.85f, 0.5f) };

float half_node_size = 0.04f;
float half_thickness_path = 0.005f;
int grid_height, grid_width;


//algo specific variables
Node* algo_graph;
bool keep_solving = false;
int start_cell_index, goal_cell_index;

//algo specific functions

void extract_path(Node* node) {
	if (node == nullptr || node->best == nullptr)
		return;
	paths.insert(paths.end(), Path(node, node->best));
	if (node->best->type != START) {
		extract_path(node->best);
	}
}

void update_starting_cell(int updated_index) {
	nodes.at(start_cell_index).cost = 10000.0f;
	nodes.at(start_cell_index).type = BASE_EMPTY;
	nodes.at(start_cell_index).empty = true;
	start_cell_index = updated_index;
	nodes.at(start_cell_index).cost = 0.0f;
	nodes.at(start_cell_index).type = START;
	nodes.at(start_cell_index).empty = true;
	algo_graph = &(nodes.at(start_cell_index));
}

void update_target_cell(int updated_index) {
	nodes.at(goal_cell_index).type = BASE_EMPTY;
	nodes.at(goal_cell_index).empty = true;
	goal_cell_index = updated_index;
	nodes.at(goal_cell_index).type = GOAL;
	nodes.at(goal_cell_index).empty = true;
}

void array_updater(std::vector<float>* vertices);

void find_path() {
	paths.clear();
	for (int i = 0; i < nodes.size(); i++) {
		nodes.at(i).cost = 10000.0f;
		nodes.at(i).best = nullptr;
	}
	// ------- A-Star
	if (algorithm_mode == A_STAR) {
		a_star.init(&nodes, algo_graph, keep_solving, start_cell_index, goal_cell_index);
		extract_path(&(nodes.at(goal_cell_index)));
	}
	
	// ------- Genetic 
	if (algorithm_mode == GENETIC) {
		genetic.init(grid_width, grid_height, start_cell_index, goal_cell_index, &nodes);
		genetic.solver();
		genetic.extract_path(&paths, start_cell_index, goal_cell_index);
	}

	// ------- Voronoi
	if (algorithm_mode == VORONOI) {
		voronoi.finder(start_cell_index, goal_cell_index);
		//voronoi.visualize_obstacles(&paths, array_updater);
	}
}

//general purpose helper functions
void sample_node_insert() {
	nodes.insert(nodes.end(), Node(1.0f, 2.0f));
	nodes.insert(nodes.end(), Node(-1.0f, 1.0f));
	nodes.insert(nodes.end(), Node(-1.0f, -1.0f));
	nodes.insert(nodes.end(), Node(1.0f, -1.0f));
	nodes.insert(nodes.end(), Node(1.0f, 1.0f));
	nodes.insert(nodes.end(), Node(-1.0f, 0.0f));
	nodes.insert(nodes.end(), Node(0.0f, -1.0f));
	nodes.insert(nodes.end(), Node(1.0f, 0.0f));
	nodes.insert(nodes.end(), Node(0.0f, 1.0f));
}

void sample_path_insert() {
	paths.insert(paths.end(), Path(2, 3));
	paths.insert(paths.end(), Path(4, 13));
	paths.insert(paths.end(), Path(23, 14));
	paths.insert(paths.end(), Path(98, 7));
	paths.insert(paths.end(), Path(69, 41));
	paths.insert(paths.end(), Path(78, 12));
}

void insert_nodes_grid() {
	for (int i = -5; i < 5; i++)
		for (int j = -5; j < 5; j++)
			nodes.insert(nodes.end(), Node(i / 10.0f, j / 10.0f));
}

void sample_graph_manipulations() {
	renderer.vertices =
	{ //               COORDINATES                  /     COLORS           //
		-0.5f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 1.0f,  0.0f, // Lower left corner
		 0.5f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 0.0f,  1.0f, // Lower right corner
		 0.0f,  0.5f * float(sqrt(3)) * 2 / 3, 0.0f,     1.0f, 0.0f,  0.0f, // Upper corner
	};
	renderer.vertices.insert(renderer.vertices.end(), {
		-0.25f, 0.5f * float(sqrt(3)) * 1 / 6, 0.0f, 0.7f, 0.7f, 0.0f, // Inner left
		0.25f, 0.5f * float(sqrt(3)) * 1 / 6, 0.0f, 0.7f, 0.0f, 0.7f, // Inner right
		0.0f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f, 0.0f, 0.7f, 0.7f  // Inner down
		});
	renderer.indices = { 0, 3, 5 };
	renderer.indices.insert(renderer.indices.end(), {
	3, 2, 4, // Lower right triangle
	5, 4, 1 // Upper triangle
		});
}

void gl_to_image_coords(float* x, float* y) {
	*x = (*x - 400) / 400;
	*y = -(*y - 400) / 400;
}

void image_coords_to_GL(float* x, float* y) {
	float temp_x = *x;
	float temp_y = *y;
	float x_off = (float)grid_width / 2;
	float y_off = (float)grid_height / 2;
	int scale = ((grid_width) > (grid_height) ? (grid_width) : (grid_height)) / 2;

	float x_GL = (temp_x - x_off) / scale;
	float y_GL = -(temp_y - y_off) / scale;
	*x = x_GL;
	*y = y_GL;
}

void add_nodes_to_render_queue(std::vector<Node> node_list) {
	for (int i = 0; i < node_list.size(); i++) {
		unsigned int index = (int) (renderer.vertices.size() / 6);
		Color color = Color();
		switch (node_list.at(i).type)
		{
		case START: color = color_list[START]; break;
		case GOAL: color = color_list[GOAL]; break;
		default:
			if (node_list.at(i).empty)
				color = color_list[BASE_EMPTY];
			else
				color = color_list[BASE_TAKEN];
		}

		renderer.vertices.insert(renderer.vertices.end(), { node_list.at(i).x - half_node_size, node_list.at(i).y - half_node_size, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { node_list.at(i).x + half_node_size, node_list.at(i).y - half_node_size, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { node_list.at(i).x + half_node_size, node_list.at(i).y + half_node_size, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { node_list.at(i).x - half_node_size, node_list.at(i).y + half_node_size, 0.0f,
			color.r, color.g, color.b });
		renderer.indices.insert(renderer.indices.end(), {
			index, index + 1, index + 2,
			index, index + 2, index + 3
			});
	}
}

void add_paths_to_render_queue(std::vector<Path> path_list) {
	float x1, y1, x2, y2;
	for (int i = 0; i < path_list.size(); i++) {
		unsigned int index = (int)(renderer.vertices.size() / 6);

		switch (path_list.at(i).mode) {
		case 1: // index method of addressing -- Index
			x1 = nodes.at(path_list.at(i).a).x;
			x2 = nodes.at(path_list.at(i).b).x;
			y1 = nodes.at(path_list.at(i).a).y;
			y2 = nodes.at(path_list.at(i).b).y;
			break;
		case 2: // reference method of addressing -- Pointer
			x1 = path_list.at(i).end_a->x;
			x2 = path_list.at(i).end_b->x;
			y1 = path_list.at(i).end_a->y;
			y2 = path_list.at(i).end_b->y;
			break;
		case 3: // coordinate methods of data storage -- Coords
			x1 = path_list.at(i).x1;
			y1 = path_list.at(i).y1;
			x2 = path_list.at(i).x2;
			y2 = path_list.at(i).y2;
			image_coords_to_GL(&x1, &y1);
			image_coords_to_GL(&x2, &y2);
			break;
		default: // coordinate methods of data storage -- Coords
			x1 = path_list.at(i).x1;
			y1 = path_list.at(i).y1;
			x2 = path_list.at(i).x2;
			y2 = path_list.at(i).y2;
			break;
		}
		float increment_x = ((y2 - y1) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;
		float increment_y = ((x1 - x2) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;

		Color color = path_list.at(i).color;

		renderer.vertices.insert(renderer.vertices.end(), { x1 - increment_x, y1 - increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { x2 - increment_x, y2 - increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { x2 + increment_x, y2 + increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { x1 + increment_x, y1 + increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.indices.insert(renderer.indices.end(), {
			index, index + 1, index + 2,
			index, index + 2, index + 3
			});
	}
}

void add_quads_to_render_queue(std::vector<float>* vertices) {
	if (vertices != nullptr) {
		unsigned int index = (int)(renderer.vertices.size() / 6);
		Color color = Color(0.1, 0.1, 0.1);

		float x, y;

		for (int i = 0; i < vertices->size() / 8; i++) {
			unsigned int index = (int)(renderer.vertices.size() / 6);

			for (int j = 0; j < 4; j++) {
				x = vertices->at(8 * i + 2 * j);
				y = vertices->at(8 * i + 2 * j + 1);
				image_coords_to_GL(&x, &y);
				renderer.vertices.insert(renderer.vertices.end(), { x, y, 0.0f,
				color.r, color.g, color.b });
			}

			renderer.indices.insert(renderer.indices.end(), {
				index, index + 1, index + 2,
				index, index + 2, index + 3
				});
		}
	}
}

void add_paths_to_render_queue_pointer_type(std::vector<Path> path_list) {
	for (int i = 0; i < path_list.size(); i++) {
		unsigned int index = (int) (renderer.vertices.size() / 6);

		float x1 = path_list.at(i).end_a->x;
		float x2 = path_list.at(i).end_b->x;
		float y1 = path_list.at(i).end_a->y;
		float y2 = path_list.at(i).end_b->y;
		float increment_x = ((y2 - y1) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;
		float increment_y = ((x1 - x2) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;

		Color color = path_list.at(i).color;

		renderer.vertices.insert(renderer.vertices.end(), { x1 - increment_x, y1 - increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { x2 - increment_x, y2 - increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { x2 + increment_x, y2 + increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.vertices.insert(renderer.vertices.end(), { x1 + increment_x, y1 + increment_y, 0.0f,
			color.r, color.g, color.b });
		renderer.indices.insert(renderer.indices.end(), {
			index, index + 1, index + 2,
			index, index + 2, index + 3
			});
	}
}

void array_updater(std::vector<float>* vertices) {
	renderer.vertices.clear();
	renderer.indices.clear();

	add_nodes_to_render_queue(nodes);
	add_paths_to_render_queue(paths);
	add_quads_to_render_queue(vertices);
	add_quads_to_render_queue(nullptr);

	renderer.update_buffers();
	renderer.render();
}

void node_clicked(float x, float y) {
	int index = 0;
	x = (x - 400) / 400;
	y = -(y - 400) / 400;
	for (int i = 0; i < nodes.size(); i++) {
		Node node = nodes.at(i);
		if (x < node.x + half_node_size)
			if (x > node.x - half_node_size)
				if (y < node.y + half_node_size)
					if (y > node.y - half_node_size) {
						if (node.type != START && node.type != GOAL) {
							index = i;
							nodes.at(index).check();

							find_path();

							renderer.vertices.clear();
							renderer.indices.clear();

							if (algorithm_mode == A_STAR) {
								add_nodes_to_render_queue(nodes);
								add_paths_to_render_queue_pointer_type(paths);
							}

							if (algorithm_mode == VORONOI) {
								add_nodes_to_render_queue(nodes);
								add_paths_to_render_queue(paths);
							}
							break;
						}
					}
	}
	return;
}

void voronoi_utility(float x, float y) {
	int index = 0;
	x = (x - 400) / 400;
	y = -(y - 400) / 400;
	for (int i = 0; i < nodes.size(); i++) {
		Node node = nodes.at(i);
		if (x < node.x + half_node_size)
			if (x > node.x - half_node_size)
				if (y < node.y + half_node_size)
					if (y > node.y - half_node_size) {
						index = i;
					}
	}
	voronoi.checkInside(index);
}

void mouse_triggers(float x, float y) {
	voronoi_utility(x, y);
	//node_clicked(x, y);
}

void keyboard_triggers(int key) {
	if (key == GLFW_KEY_A) {
		if (start_cell_index % grid_width - 1 >= 0) {
			update_starting_cell(start_cell_index - 1);
		}
	}
	if (key == GLFW_KEY_S) {
		if (start_cell_index + grid_width < grid_width * grid_height) {
			update_starting_cell(start_cell_index + grid_width);
		}
	}
	if (key == GLFW_KEY_D) {
		if (start_cell_index % grid_width + 1 < grid_width) {
			update_starting_cell(start_cell_index + 1);
		}
	}
	if (key == GLFW_KEY_W) {
		if (start_cell_index - grid_width >= 0) {
			update_starting_cell(start_cell_index - grid_width);
		}
	}

	if (key == GLFW_KEY_LEFT) {
		if (goal_cell_index % grid_width - 1 >= 0)
			update_target_cell(goal_cell_index - 1);
	}
	if (key == GLFW_KEY_RIGHT) {
		if (goal_cell_index % grid_width + 1 < grid_width)
			update_target_cell(goal_cell_index + 1);
	}
	if (key == GLFW_KEY_UP) {
		if (goal_cell_index - grid_width >= 0)
			update_target_cell(goal_cell_index - grid_width);
	}
	if (key == GLFW_KEY_DOWN) {
		if (goal_cell_index + grid_width < grid_width * grid_height)
			update_target_cell(goal_cell_index + grid_width);
	}


	find_path();

	renderer.vertices.clear();
	renderer.indices.clear();

	if (algorithm_mode == A_STAR) {
		add_nodes_to_render_queue(nodes);
		add_paths_to_render_queue_pointer_type(paths);
	}

	if (algorithm_mode == VORONOI) {
		add_nodes_to_render_queue(nodes);
		add_paths_to_render_queue(paths);
	}
}

int main() {
	renderer.init(mouse_triggers, keyboard_triggers, "Visualizer @ME18B074");
	renderer.render();
	nodes = *map_obj_1.getGraph(&grid_height, &grid_width, &half_node_size);

	if (false) {//bypassing all the algo
		//insert_nodes_grid();
		//start_cell_index = 30300;
		start_cell_index = 15;
		nodes.at(start_cell_index).type = START;
		nodes.at(start_cell_index).cost = 0;
		algo_graph = &(nodes.at(start_cell_index));
		goal_cell_index = 262;
		//goal_cell_index = 30302;
		nodes.at(goal_cell_index).type = GOAL;

		if (algorithm_mode == VORONOI)
			voronoi.init(&nodes, grid_width, grid_height, &renderer, &paths, array_updater);
		find_path();
	}

	approximater.init(&nodes, grid_width, grid_height, &renderer, &paths, array_updater);

	renderer.vertices.clear();
	renderer.indices.clear();

	if (algorithm_mode == A_STAR) {
		add_nodes_to_render_queue(nodes);
		add_paths_to_render_queue_pointer_type(paths);
	}

	if (algorithm_mode == VORONOI) {
		add_nodes_to_render_queue(nodes);
		add_paths_to_render_queue(paths);
	}


	renderer.rendering_thread();
	//renderer.clean_up();
	return 0;
}
//voronoi -- martin held

#endif