#include "mcc.h"

VectorWrapperGLuint::VectorWrapperGLuint() {
	indices = new std::vector<GLuint>;
	*indices =	{
		0, 3, 5, // Lower left triangle
		3, 2, 4, // Lower right triangle
		5, 4, 1 // Upper triangle
	};
}

std::vector<GLuint>* VectorWrapperGLuint::getList(void) {
	return indices;
}

void VectorWrapperGLuint::push_back(GLuint indice) {
	indices->push_back(indice);
}

void VectorWrapperGLuint::insert(GLuint* list, int size) {
	indices->insert(indices->end(), list, list+size);
	delete(list);
}

void VectorWrapperGLuint::clear() {
	indices->clear();
}

VectorWrapperGLfloat::VectorWrapperGLfloat() {
	vertices = new std::vector<GLfloat>;
	*vertices = { //               COORDINATES                  /     COLORS           //
			-0.5f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 1.0f,  0.0f, // Lower left corner
			 0.5f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 0.0f,  1.0f, // Lower right corner
			 0.0f,  0.5f * float(sqrt(3)) * 2 / 3, 0.0f,     1.0f, 0.0f,  0.0f, // Upper corner
			-0.25f, 0.5f * float(sqrt(3)) * 1 / 6, 0.0f,     0.7f, 0.7f,  0.0f, // Inner left
			 0.25f, 0.5f * float(sqrt(3)) * 1 / 6, 0.0f,     0.7f, 0.0f,  0.7f, // Inner right
			 0.0f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 0.7f,  0.7f  // Inner down
	};
}

std::vector<GLfloat>* VectorWrapperGLfloat::getList(void) {
	return vertices;
}

void VectorWrapperGLfloat::push_back(GLfloat vertice) {
	vertices->push_back(vertice);
}

void VectorWrapperGLfloat::insert(GLfloat* list, int size) {
	vertices->insert(vertices->begin(), list, list + size);
	delete(list);
}

void VectorWrapperGLfloat::clear() {
	vertices->clear();
}