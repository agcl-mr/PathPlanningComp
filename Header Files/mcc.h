#ifndef MY_CUSTOM_CLASSES_H
#define MY_CUSTOM_CLASSES_H

#include <iostream>
#include<glad/glad.h>
#include<vector>
#include <string>

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

	Node* boundary_left, * boundary_right;

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

class Path
{
public:
	Path(int id_a, int id_b) {
		mode = 1;
		a = id_a;
		b = id_b;
		end_a = nullptr;
		end_b = nullptr;
		x1 = 0;
		y1 = 0;
		x2 = 0;
		y2 = 0;
	}
	Path(int id_a, int id_b, Color color) {
		mode = 1;
		a = id_a;
		b = id_b;
		end_a = nullptr;
		end_b = nullptr;
		x1 = 0;
		y1 = 0;
		x2 = 0;
		y2 = 0;
		this->color = color;
	}
	Path(Node* end_a, Node* end_b) {
		mode = 2;
		this->end_a = end_a;
		this->end_b = end_b;
		a = 0;
		b = 0;
		x1 = 0;
		y1 = 0;
		x2 = 0;
		y2 = 0;
	}
	Path(float x1, float y1, float x2, float y2) {
		mode = 3;
		this->x1 = x1;
		this->y1 = y1;
		this->x2 = x2;
		this->y2 = y2;
		end_a = nullptr;
		end_b = nullptr;
		a = 0;
		b = 0;
	}
	Path(float x1, float y1, float x2, float y2, Color color) {
		mode = 3;
		this->x1 = x1;
		this->y1 = y1;
		this->x2 = x2;
		this->y2 = y2;
		end_a = nullptr;
		end_b = nullptr;
		a = 0;
		b = 0;
		this->color = color;
	}

	int a, b;
	Node* end_a, * end_b;
	float x1, y1, x2, y2;
	int mode = 0;
	Color color = Color(0.937f, 0.749f, 0.2196f);
	//Color color = Color(0.918f, 0.416f, 0.278f);
	//Color color = Color(0.99f, 0.729f, 0.1294f);
	//Color color = Color(0.953f, 0.42f, 0.11f);
	//Color color = Color(0.3f, 0.85f, 0.5f);;
	/*
	* mode = 1 : node id method
	* mode = 2 : node reference method
	* mode = 3 : point coordinates method
	*/
};

class VectorWrapperGLuint {
private:

public:
	std::vector<GLuint>* indices;
	VectorWrapperGLuint();

	std::vector<GLuint>* getList(void);

	void push_back(GLuint indice);

	//void insert(GLuint list[], int size);
	void insert(GLuint* list, int size);

	void clear(void);
};

class VectorWrapperGLfloat {
private:

public:
	std::vector<GLfloat>* vertices;
	VectorWrapperGLfloat();

	std::vector<GLfloat>* getList(void);

	void push_back(GLfloat vertice);

	//void insert(GLfloat list[], int size);
	void insert(GLfloat* list, int size);

	void clear(void);
};

class algo_result {
public:
	float path_length;
	float computation_time;
	std::string path;

	algo_result(void) {
		this->path_length = 0.0f;
		this->computation_time = 0.0f;
		this->path = "";
	}

	algo_result(float length, float time, std::string path) {
		this->path_length = length;
		this->computation_time = time;
		this->path = path;
	}

	std::string stringify(void) {
		std::string res;
		res = "{'length' : " + std::to_string(path_length) + ", 'time' : " + std::to_string(computation_time) + ", path : '" + path + "'}";
		return res;
	}
};

class consolidated_result {
public:
	std::string map_id;
	int start_x, start_y, goal_x, goal_y;
	algo_result me_ea, rrtStar, prm, fmt, est, rlrt, sst, stride;

	consolidated_result(std::string map_id, int start_index, int goal_index, int GRID_WIDTH) {
		this->map_id = map_id;
		start_x = start_index % GRID_WIDTH;
		start_y = start_index / GRID_WIDTH;
		goal_x = goal_index % GRID_WIDTH;
		goal_y = goal_index / GRID_WIDTH;
		me_ea = algo_result();
		rrtStar = algo_result();
		prm = algo_result();
		fmt = algo_result();
		est = algo_result();
		rlrt = algo_result();
		sst = algo_result();
		stride = algo_result();
	}

	std::string stringify(void) {
		std::string res;
		res = "{'map_id' : " + map_id + ", 'states' : {'start' : {'x' : " + std::to_string(start_x) + ", 'y' : " 
			+ std::to_string(start_y) +	"}, 'goal' : {'x' : " + std::to_string(goal_x) + ", 'y' : " + 
			std::to_string(goal_y) + "}}, 'results' : {'me_ea' : '"+me_ea.stringify() + "', 'rrtStar' : '" +
			rrtStar.stringify() + "', 'prm' : '" + prm.stringify() + "', 'fmt' : '" + fmt.stringify() +
			"', 'est' : '" + est.stringify() + "', 'rlrt' : '" + rlrt.stringify() + "', 'sst' : '" +
			sst.stringify() + "', 'stride' : '" + stride.stringify() + "'}}";
		return res;
	}
};

#endif