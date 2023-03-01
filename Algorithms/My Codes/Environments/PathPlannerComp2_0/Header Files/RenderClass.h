#ifndef RENDER_CLASS_H
#define RENDER_CLASS_H

#include<iostream>
#include<vector>
#include<Windows.h>
#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<stb_image.h>
#include<string>

#include"shaderClass.h"
#include"VAO.h"
#include"VBO.h"
#include"EBO.h"
#include"mcc.h"

class RenderClass
{
public:

	RenderClass();

	std::vector<GLuint> indices;
	std::vector<GLfloat> vertices;

	void init(void (*func_mouse)(float, float), void(*func_key)(int), std::string window_name);

	void rendering_thread(void);

	void clean_up(void);

	void update_buffers(void);

	void render(void);

private:
	GLFWwindow* window;
	GLuint uniID;
	// Generates Vertex Array Object and binds it
	VAO vao;
	VBO vbo;
	EBO ebo;
	// ID reference for the Vertex Array Object
	GLuint vao_id;
	// Generates Shader object using shaders defualt.vert and default.frag
	Shader shaderProgram;


	//handling mouse inputs
	typedef void (*external_call_mouse)(float, float);
	external_call_mouse mouse_callback;

	//handling keyboard inputs
	typedef void (*external_call_keyboard)(int);
	external_call_keyboard keyboard_callback;

	int create_window(void (*func_mouse)(float, float), void(*func_key)(int), std::string window_name);

	void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

	void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
};

#endif