/*#pragma once
#include<vector>

#include"RenderClass.h"

//Graph Elements
class Node
{
public:
	Node(float x, float y) {
		this->x = x;
		this->y = y;
		empty = true;
		type = 1;

		this->left = nullptr;
		this->right = nullptr;
		this->top = nullptr;
		this->bottom = nullptr;
		this->best = nullptr;
	}

	float x, y;
	bool empty;
	int type;


	Node* left, * right, * bottom, * top, * best;
	float cost = 10000.0f;

	void check() {
		empty = not empty;
		if (empty)
			type = 1;
		else
			type = 2;
	}

	void update_neighbours(Node* left, Node* right, Node* top, Node* bottom) {
		this->left = left;
		this->right = right;
		this->top = top;
		this->bottom = bottom;
	}

	void print_info() {
		std::cout << "--------------------------------" << std::endl;
		std::cout << "NODE" << std::endl;
		std::cout << "(x,y) : {" << x << "," << y << ")" << std::endl;
		if (empty)
			std::cout << "empty : yes" << std::endl;
		else
			std::cout << "empty : no" << std::endl;
		switch (type) {
		case 0: std::cout << "type : START" << std::endl; break;
		case 1: std::cout << "type : BASE_EMPTY" << std::endl; break;
		case 2: std::cout << "type : BASE_TAKEN" << std::endl; break;
		case 3: std::cout << "type : GOAL" << std::endl; break;
		}
		std::cout << "cost : " << cost << std::endl;
	}
};

class Path
{
public:
	Path(int id_a, int id_b) {
		a = id_a;
		b = id_b;
		end_a = nullptr;
		end_b = nullptr;
	}
	Path(Node* end_a, Node* end_b) {
		this->end_a = end_a;
		this->end_b = end_b;
		a = 0;
		b = 0;
	}

	int a, b;
	Node* end_a, * end_b;
};

class Color
{
public:
	Color() {}

	Color(float r, float g, float b) {
		this->r = r;
		this->g = g;
		this->b = b;
	}

	float r, g, b;
};

// initializations
std::vector<Node> nodes;
std::vector<Path> paths;
RenderClass renderer;

//constants
float current_color[] = { 0.7f, 0.15f, 0.5f };
Color color_list[] = { Color(1.0f, 0.0f, 0.0f), Color(0.7f, 0.15f, 0.5f), Color(0.2f, 0.2f, 1.0f),
							Color(0.0f, 1.0f, 0.0f), Color(0.3f, 0.85f, 0.5f) };
const int START = 0;
const int BASE_EMPTY = 1;
const int BASE_TAKEN = 2;
const int GOAL = 3;
const int PATH_1 = 4;

float half_node_size = 0.04f;
float half_thickness_path = 0.005f;

//algo specific variables
Node* algo_graph;
bool keep_solving = false;
int start_cell_index, goal_cell_index;

//algo specific functions

void create_graph_from_nodes() {
	Node* left = nullptr;
	Node* right = nullptr;
	Node* top = nullptr;
	Node* bottom = nullptr;

	for (int i = 0; i < nodes.size(); i++) {
		if (i - 10 >= 0)
			left = &(nodes.at(i - 10));
		if (i + 10 < 100)
			right = &(nodes.at(i + 10));
		if (i % 10 - 1 >= 0)
			bottom = &(nodes.at(i - 1));
		if (i % 10 + 1 < 10)
			top = &(nodes.at(i + 1));
		nodes.at(i).update_neighbours(left, right, top, bottom);

		left = nullptr;
		right = nullptr;
		top = nullptr;
		bottom = nullptr;
	}
}

void extract_path(Node* node) {
	if (node == nullptr || node->best == nullptr)
		return;
	paths.insert(paths.end(), Path(node, node->best));
	if (node->best->type != START)
		extract_path(node->best);
}

void solver(Node* node);

void handle_neighbour(Node* me, Node* neighbour) {
	if (neighbour == nullptr)
		return;
	if (neighbour->type == BASE_TAKEN)
		return;
	if (neighbour->cost > me->cost + 1) {
		neighbour->cost = me->cost + 1;
		neighbour->best = me;
		solver(neighbour);
	}
}

void solver(Node* node) {
	if (keep_solving) {
		if (node == nullptr)
			return;
		if (node->type == GOAL) {
			keep_solving = false;
			extract_path(node);
			return;
		}
		handle_neighbour(node, node->left);
		handle_neighbour(node, node->right);
		handle_neighbour(node, node->top);
		handle_neighbour(node, node->bottom);
	}
}

void update_starting_cell(float updated_index) {
	nodes.at(start_cell_index).cost = 10000.0f;
	nodes.at(start_cell_index).type = BASE_EMPTY;
	nodes.at(start_cell_index).empty = true;
	start_cell_index = updated_index;
	nodes.at(start_cell_index).cost = 0.0f;
	nodes.at(start_cell_index).type = START;
	nodes.at(start_cell_index).empty = true;
	algo_graph = &(nodes.at(start_cell_index));
}

void update_target_cell(float updated_index) {
	nodes.at(goal_cell_index).type = BASE_EMPTY;
	nodes.at(goal_cell_index).empty = true;
	goal_cell_index = updated_index;
	nodes.at(goal_cell_index).type = GOAL;
	nodes.at(goal_cell_index).empty = true;
}

void find_path() {
	paths.clear();
	for (int i = 0; i < nodes.size(); i++) {
		nodes.at(i).cost = 10000.0f;
		nodes.at(i).best = nullptr;
	}
	nodes.at(start_cell_index).cost = 0;
	keep_solving = true;
	solver(algo_graph);
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

void add_nodes_to_render_queue(std::vector<Node> node_list) {
	for (int i = 0; i < node_list.size(); i++) {
		unsigned int index = renderer.vertices.size() / 6;
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

void add_paths_to_render_queue_index_type(std::vector<Path> path_list, float color[]) {
	for (int i = 0; i < path_list.size(); i++) {
		unsigned int index = renderer.vertices.size() / 6;

		float x1 = nodes.at(path_list.at(i).a).x;
		float x2 = nodes.at(path_list.at(i).b).x;
		float y1 = nodes.at(path_list.at(i).a).y;
		float y2 = nodes.at(path_list.at(i).b).y;
		float increment_x = ((y2 - y1) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;
		float increment_y = ((x1 - x2) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;

		renderer.vertices.insert(renderer.vertices.end(), { x1 - increment_x, y1 - increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.vertices.insert(renderer.vertices.end(), { x2 - increment_x, y2 - increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.vertices.insert(renderer.vertices.end(), { x2 + increment_x, y2 + increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.vertices.insert(renderer.vertices.end(), { x1 + increment_x, y1 + increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.indices.insert(renderer.indices.end(), {
			index, index + 1, index + 2,
			index, index + 2, index + 3
			});
	}
}

void add_paths_to_render_queue_pointer_type(std::vector<Path> path_list, float color[]) {
	for (int i = 0; i < path_list.size(); i++) {
		unsigned int index = renderer.vertices.size() / 6;

		float x1 = path_list.at(i).end_a->x;
		float x2 = path_list.at(i).end_b->x;
		float y1 = path_list.at(i).end_a->y;
		float y2 = path_list.at(i).end_b->y;
		float increment_x = ((y2 - y1) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;
		float increment_y = ((x1 - x2) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))) * half_thickness_path;

		renderer.vertices.insert(renderer.vertices.end(), { x1 - increment_x, y1 - increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.vertices.insert(renderer.vertices.end(), { x2 - increment_x, y2 - increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.vertices.insert(renderer.vertices.end(), { x2 + increment_x, y2 + increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.vertices.insert(renderer.vertices.end(), { x1 + increment_x, y1 + increment_y, 0.0f,
			color[0], color[1], color[2] });
		renderer.indices.insert(renderer.indices.end(), {
			index, index + 1, index + 2,
			index, index + 2, index + 3
			});
	}
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

							add_nodes_to_render_queue(nodes);
							add_paths_to_render_queue_pointer_type(paths, new float[3] {0.3f, 0.85f, 0.5f});
						}
					}
	}
	return;
}

void mouse_triggers(float x, float y) {
	node_clicked(x, y);
}

void keyboard_triggers(int key) {
	if (key == GLFW_KEY_A) {
		if (start_cell_index - 10 >= 0) {
			update_starting_cell(start_cell_index - 10);
		}
	}
	if (key == GLFW_KEY_S) {
		if (start_cell_index % 10 - 1 >= 0)	{
			update_starting_cell(start_cell_index - 1);
		}
	}
	if (key == GLFW_KEY_D) {
		if (start_cell_index + 10 < 100)	{
			update_starting_cell(start_cell_index + 10);
		}
	}
	if (key == GLFW_KEY_W) {
		if (start_cell_index % 10 + 1 < 10)	{
			update_starting_cell(start_cell_index + 1);
		}
	}

	if (key == GLFW_KEY_LEFT) {
		if (goal_cell_index - 10 >= 0)
			update_target_cell(goal_cell_index - 10);
	}
	if (key == GLFW_KEY_RIGHT) {
		if (goal_cell_index + 10 < 100)
			update_target_cell(goal_cell_index + 10);
	}
	if (key == GLFW_KEY_UP) {
		if (goal_cell_index % 10 + 1 < 10)
			update_target_cell(goal_cell_index + 1);
	}
	if (key == GLFW_KEY_DOWN) {
		if (goal_cell_index % 10 - 1 >= 0)
			update_target_cell(goal_cell_index - 1);
	}


	find_path();

	renderer.vertices.clear();
	renderer.indices.clear();

	add_nodes_to_render_queue(nodes);
	add_paths_to_render_queue_pointer_type(paths, new float[3] {0.3f, 0.85f, 0.5f});
}

int main() {

	insert_nodes_grid();
	start_cell_index = 15;
	nodes.at(start_cell_index).type = START;
	nodes.at(start_cell_index).cost = 0;
	algo_graph = &(nodes.at(start_cell_index));
	goal_cell_index = 99;
	nodes.at(goal_cell_index).type = GOAL;

	create_graph_from_nodes();
	find_path();

	renderer.vertices.clear();
	renderer.indices.clear();

	add_nodes_to_render_queue(nodes);
	add_paths_to_render_queue_pointer_type(paths, new float[3] {0.3f, 0.85f, 0.5f});

	return renderer.handler(mouse_triggers, keyboard_triggers);
}
*/