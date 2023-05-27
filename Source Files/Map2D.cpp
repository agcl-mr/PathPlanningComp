
#include "../Header Files/Map2D.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int width, height, channels;

Map2D::Map2D(){
	init();
}

//algo specific functions

void Map2D::create_graph_from_nodes(int height, int width) {
	Node* left = nullptr;
	Node* right = nullptr;
	Node* top = nullptr;
	Node* bottom = nullptr;

	for (int i = 0; i < nodes.size(); i++) {
		if (i % width - 1 >= 0)
			left = &(nodes.at(i - 1));
		if (i % width + 1 < width)
			right = &(nodes.at(i + 1));
		if (i - width >= 0)
			top = &(nodes.at(i - width));
		if (i + width < width*height)
			bottom = &(nodes.at(i + width));
		nodes.at(i).update_neighbours(left, right, top, bottom);

		left = nullptr;
		right = nullptr;
		top = nullptr;
		bottom = nullptr;
	}
}

void Map2D::create_nodes(unsigned char* img, int height, int width, int channels) {
	float x_off = (float) width / 2;
	float y_off = (float) height / 2;
	int scale = ((width) > (height) ? (width) : (height)) / 2;

	half_node_size = 0.5 / scale;

	for (int channel = 0; channel < 1; channel++) {
		if (show_logs)
			std::cout << "CHANNEL " << channel << std::endl;
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				unsigned char* pixelOffset = img + (j + width * i) * channels;
				nodes.insert(nodes.end(), Node((j-x_off)/scale, -(i-y_off)/scale));
				if (static_cast<unsigned int>(pixelOffset[channel]) == 255) {

				}
				else {
					nodes.at(nodes.size()-1).check();
				}
			}
		}
	}
}

//general purpose helper functions
void Map2D::add_nodes_to_render_queue() {
	unsigned int index = (unsigned int)(renderer.vertices.size() / 6);
	for (int i = 0; i < nodes.size(); i++) {
		if (show_logs)
			if (i % 1000 == 0)
				std::cout << i << "\n";
		//unsigned int index = preview.vertices.size() / 6;
		Color color = Color();
		switch (nodes.at(i).type)
		{
		case START: color = color_list[START]; break;
		case GOAL: color = color_list[GOAL]; break;
		default:
			if (nodes.at(i).empty)
				color = color_list[BASE_EMPTY];
			else
				color = color_list[BASE_TAKEN];
		}
		renderer.vertices.insert(renderer.vertices.begin(), {
			nodes.at(i).x - half_node_size, nodes.at(i).y - half_node_size, 0.0f, color.r, color.g,  color.b,
			nodes.at(i).x + half_node_size, nodes.at(i).y - half_node_size, 0.0f, color.r, color.g,  color.b,
			nodes.at(i).x + half_node_size, nodes.at(i).y + half_node_size, 0.0f, color.r, color.g,  color.b,
			nodes.at(i).x - half_node_size, nodes.at(i).y + half_node_size, 0.0f, color.r, color.g,  color.b,
			});

		renderer.indices.insert(renderer.indices.begin(),
			{ index, index + 1, index + 2, index, index + 2,  index + 3 });
		index += 4;
	}
}

std::vector<Node>* Map2D::getGraph(int* Height, int* Width, float* node_size) {
	*Height = height; 
	*Width = width;
	*node_size = half_node_size;
	return &nodes;
}

int Map2D::init() {
	//renderer.init(mouse_triggers_null, keyboard_triggers_null, "Map Preview @ME18B074");

    //unsigned char* img = stbi_load("C:/Users/HP/Downloads/pat1.bmp", &width, &height, &channels, 0);
	//unsigned char* img = stbi_load("E:/2022/PathPlanning/Data/Maps/map12.bmp", &width, &height, &channels, 0);
	//unsigned char* img = stbi_load("E:/2022/PathPlanning/Data/Maps/map11.bmp", &width, &height, &channels, 0);
	unsigned char* img = stbi_load("E:/2022/PathPlanning/Data/Images/exp6.bmp", &width, &height, &channels, 0);
	//unsigned char* img = stbi_load("E:/2022/PathPlanning/Data/Maps/map8.bmp", &width, &height, &channels, 0);
	if (img == NULL) {
        printf("Error in loading the image\n");
        exit(1);
    }
    
    printf("Loaded image with a width of %dpx, a height of %dpx and %d channels\n", width, height, channels);

	create_nodes(img, height, width, channels);

	create_graph_from_nodes(height, width);
	if (show_logs)
		std::cout << " nodes size : " << nodes.size() << std::endl;

	//renderer.vertices.clear();
	//renderer.indices.clear();
	//sample_graph_manipulations2(&renderer);

	//add_nodes_to_render_queue();
	//renderer.render();
	//Sleep(1000);
	//renderer.rendering_thread();
	return 0;
}
