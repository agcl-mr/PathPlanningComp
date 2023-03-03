#ifndef MY_CUSTOM_CLASSES_H
#define MY_CUSTOM_CLASSES_H

#include <iostream>
#include<glad/glad.h>
#include<vector>

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
	Color color = Color(0.3f, 0.85f, 0.5f);;
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

#endif