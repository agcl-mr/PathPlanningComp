#ifndef MAP2D_CLASS_H
#define MAP2D_CLASS_H
#define _CRT_SECURE_NO_DEPRECATE
#include "glad/glad.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
//#include <stb_image.h>
#include "RenderClass.h"
#include "mcc.h"
#include "constants.h"

class Map2D {
public:

	Map2D();

	// initializations
	std::vector<Node> nodes;
	std::vector<Path> paths;
	RenderClass renderer;

	//constants
	float current_color[3] = {0.7f, 0.15f, 0.5f};
	Color color_list[5] = { Color(1.0f, 0.0f, 0.0f), Color(0.7f, 0.15f, 0.5f), Color(0.2f, 0.2f, 1.0f),
							Color(0.0f, 1.0f, 0.0f), Color(0.3f, 0.85f, 0.5f) };

	float half_node_size = 0.05f;
	float half_thickness_path = 0.005f;

	//algo specific functions
	void create_graph_from_nodes(int height, int width);

	void create_nodes(unsigned char* img, int height, int width, int channels);

	//general purpose helper functions
	void add_nodes_to_render_queue();

	//entrypoint to class
	int init(void);

	std::vector<Node>* getGraph(int* Height, int* Width, float* node_size);
};

#endif