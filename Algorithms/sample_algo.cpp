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
	}

	float x, y;
	bool empty;

	void check() { empty = not empty; }
};

class Path
{
public:
	Path(int id_a, int id_b) {
		a = id_a;
		b = id_b;
	}

	int a, b;
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

float half_node_size = 0.02f;
float half_thickness_path = 0.005f;

// helper functions
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
		if (node_list.at(i).empty)
			color = color_list[BASE_EMPTY];
		else
			color = color_list[BASE_TAKEN];

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

void add_paths_to_render_queue(std::vector<Path> path_list, float color[]) {
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

void node_clicked(float x, float y) {
	int index = 0;
	x = (x - 400) / 400;
	y = -(y - 400) / 400;
	std::cout << "GL Space : " << x << " | " << y << std::endl;
	for (int i = 0; i < nodes.size(); i++) {
		Node node = nodes.at(i);
		if (x < node.x + half_node_size)
			if (x > node.x - half_node_size)
				if (y < node.y + half_node_size)
					if (y > node.y - half_node_size) {
						index = i;
						nodes.at(index).check();

						renderer.vertices.clear();
						renderer.indices.clear();

						add_nodes_to_render_queue(nodes);
						add_paths_to_render_queue(paths, new float[3] {0.3f, 0.85f, 0.5f});
					}
	}
	return;
}

void mouse_triggers(float x, float y) {
	std::cout << "Roger that!" << std::endl;
	std::cout << "I heard : " << x << " | " << y << std::endl;

	node_clicked(x, y);
}

void keyboard_triggers(int key) {
	std::cout << "I heard something... ..." << std::endl;
	if (key == GLFW_KEY_A)
		std::cout << "Key pressed... Detected as 'A'" << std::endl;
	if (key == GLFW_KEY_S)
		std::cout << "Key pressed... Detected as 'S'" << std::endl;
	if (key == GLFW_KEY_D)
		std::cout << "Key pressed... Detected as 'D'" << std::endl;
	if (key == GLFW_KEY_W)
		std::cout << "Key pressed... Detected as 'W'" << std::endl;

	if (key == GLFW_KEY_LEFT)
		std::cout << "Key pressed... Detected as 'LEFT'" << std::endl;
	if (key == GLFW_KEY_RIGHT)
		std::cout << "Key pressed... Detected as 'RIGHT'" << std::endl;
	if (key == GLFW_KEY_UP)
		std::cout << "Key pressed... Detected as 'UP'" << std::endl;
	if (key == GLFW_KEY_DOWN)
		std::cout << "Key pressed... Detected as 'DOWN'" << std::endl;
	std::cout << std::endl;
}


int main() {
	sample_node_insert();

	insert_nodes_grid();
	sample_path_insert();
	sample_graph_manipulations();
	renderer.vertices.clear();
	renderer.indices.clear();

	add_nodes_to_render_queue(nodes);
	add_paths_to_render_queue(paths, new float[3] {0.3f, 0.85f, 0.5f});

	return renderer.handler(mouse_triggers, keyboard_triggers);
}*/