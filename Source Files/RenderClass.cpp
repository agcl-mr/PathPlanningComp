#include "../Header Files/RenderClass.h"

RenderClass::RenderClass(void) {
	window = nullptr;
	uniID = 0;
	vao = VAO();
	vbo = VBO();
	ebo = EBO();
	vao_id = 0;
	shaderProgram = Shader();

	keyboard_callback = nullptr;
	mouse_callback = nullptr;

	indices = {
		0, 3, 5, // Lower left triangle
		3, 2, 4, // Lower right triangle
		5, 4, 1 // Upper triangle
	};

	vertices = { //               COORDINATES                  /     COLORS           //
			-0.5f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 1.0f,  0.0f, // Lower left corner
			 0.5f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 0.0f,  1.0f, // Lower right corner
			 0.0f,  0.5f * float(sqrt(3)) * 2 / 3, 0.0f,     1.0f, 0.0f,  0.0f, // Upper corner
			-0.25f, 0.5f * float(sqrt(3)) * 1 / 6, 0.0f,     0.7f, 0.7f,  0.0f, // Inner left
			 0.25f, 0.5f * float(sqrt(3)) * 1 / 6, 0.0f,     0.7f, 0.0f,  0.7f, // Inner right
			 0.0f, -0.5f * float(sqrt(3)) * 1 / 3, 0.0f,     0.0f, 0.7f,  0.7f  // Inner down
	};
}

static void error_callback(int error, const char* description)
{
	puts(description);
}

void RenderClass::update_buffers(void) {
	// Generates Vertex Array Object and binds it
	vao_id = vao.ID;
	vao.Bind();

	// Generates Vertex Buffer Object and links it to vertices
	vbo = VBO(vertices.data(), sizeof(GLfloat) * vertices.size());
	// Generates Element Buffer Object and links it to indices
	ebo = EBO(indices.data(), sizeof(GLfloat) * indices.size());

	// Links VBO attributes such as coordinates and colors to VAO
	vao.LinkAttrib(vbo, 0, 3, GL_FLOAT, 6 * sizeof(float), (void*)0);
	vao.LinkAttrib(vbo, 1, 3, GL_FLOAT, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	// Unbind all to prevent accidentally modifying them
	vao.Unbind();
	vbo.Unbind();
	ebo.Unbind();
}

void RenderClass::render(void) {
	// Specify the color of the background
	glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
	// Clean the back buffer and assign the new color to it
	glClear(GL_COLOR_BUFFER_BIT);
	// Tell OpenGL which Shader Program we want to use
	shaderProgram.Activate();
	// Assigns a value to the uniform; NOTE: Must always be done after activating the Shader Program
	glUniform1f(uniID, 0.001f);
	// Bind the VAO so OpenGL knows to use it
	vao.Bind();
	// Draw primitives, number of indices, datatype of indices, index of indices
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	// Swap the back buffer with the front buffer
	glfwSwapBuffers(window);
	// Take care of all GLFW events
	glfwPollEvents();
}

void RenderClass::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		double x;
		double y;
		glfwGetCursorPos(window, &x, &y);
		mouse_callback((float)x, (float)y);
		update_buffers();
	}
}

void RenderClass::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS) {

		keyboard_callback(key);
		update_buffers();
	}
}

int RenderClass::create_window(void (*func_mouse)(float, float), void (*func_key)(int), std::string window_name) {
	mouse_callback = func_mouse;
	keyboard_callback = func_key;

	// Initialize GLFW
	glfwInit();

	// Tell GLFW what version of OpenGL we are using 
	// In this case we are using OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// Tell GLFW we are using the CORE profile
	// So that means we only have the modern functions
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Create a GLFWwindow object of 800 by 800 pixels, naming it "Visualizer @ME18B074"
	//GLFWwindow* window = glfwCreateWindow(800, 800, "Visualizer @ME18B074", NULL, NULL);
	window = glfwCreateWindow(800, 800, window_name.c_str() , NULL, NULL);
	// Error check if the window fails to create
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	// Introduce the window into the current context
	glfwMakeContextCurrent(window);

	/* ... Initialize everything here ... */

	glfwSetWindowUserPointer(window, this);

	auto mouse_callback_pointer = [](GLFWwindow* window, int button, int action, int mods)
	{
		static_cast<RenderClass*>(glfwGetWindowUserPointer(window))->mouse_button_callback(window, button, action, mods);
	};

	auto key_callback_pointer = [](GLFWwindow* window, int key, int scancode, int action, int mods)
	{
		static_cast<RenderClass*>(glfwGetWindowUserPointer(window))->key_callback(window, key, scancode, action, mods);
	};

	glfwSetMouseButtonCallback(window, mouse_callback_pointer);
	glfwSetKeyCallback(window, key_callback_pointer);

	return 1;
}

void RenderClass::init(void (*func_mouse)(float, float), void (*func_key)(int), std::string window_name) {
	//create a GLFW window
	create_window(func_mouse, func_key, window_name);

	//Load GLAD so it configures OpenGL
	gladLoadGL();
	// Specify the viewport of OpenGL in the Window
	// In this case the viewport goes from x = 0, y = 0, to x = 800, y = 800
	glViewport(0, 0, 800, 800);

	// Generates Shader object using shaders defualt.vert and default.frag
	shaderProgram = Shader("E:/2022/DDP_ME18B074/Algorithms/My Codes/Environments/PathPlannerComp2_0/default.vert",
		"E:/2022/DDP_ME18B074/Algorithms/My Codes/Environments/PathPlannerComp2_0/default.frag");

	// Generates Vertex Array Object and binds it
	vao = VAO(true);
	update_buffers();

	// Gets ID of uniform called "scale"
	uniID = glGetUniformLocation(shaderProgram.ID, "scale");
}

void RenderClass::rendering_thread(void) {
	update_buffers();
	
	// Main while loop
	while (!glfwWindowShouldClose(window))
	{
		render();
		//Sleep(100);
		// Take care of all GLFW events
		//glfwPollEvents();
	}
}

void RenderClass::clean_up(void) {
	// Delete all the objects we've created
	vao.Delete();
	vbo.Delete();
	ebo.Delete();
	shaderProgram.Delete();
	// Delete window before ending the program
	glfwDestroyWindow(window);
	// Terminate GLFW before ending the program
	glfwTerminate();
}
