#ifndef ELLIPTICAL_APPROX_H
#define ELLIPTICAL_APPROX_H

#include <vector>
//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
#include <iostream>
#include "mcc.h"
#include "RenderClass.h"
#include "constants.h"

class ellipse {
public:
	float center_x, center_y, a, b, tilt;
	ellipse() {
		center_x = 0.0f;
		center_y = 0.0f;
		a = 0.0f;
		b = 0.0f;
		tilt = 0.0f;
	}
	ellipse(float center_x, float center_y, float a, float b, float tilt) {
		this->center_x = center_x;
		this->center_y = center_y;
		this->a = a;
		this->b = b;
		this->tilt = tilt;
	}

	bool isInside(float x, float y, int i);

private:
};

class local_visualizer {
public:
	std::vector<Path>* paths;
	local_visualizer(void);

	local_visualizer(int GRID_WIDTH, int GRID_HEIGHT, RenderClass* renderer, std::vector<Path>* paths,
		void (*func_updater)(std::vector<float>*));

	void draw_ellipse(ellipse ellipse);

	void invalidate(void);

private:
	int GRID_WIDTH, GRID_HEIGHT;
	RenderClass* renderer;
	//handling callbacks render-counter part function
	typedef void (*external_render_counter_part)(std::vector<float>*);
	external_render_counter_part render_callback;
};

class elliptical_approx {
public:
	void init(
		std::vector<Node>* node_list,
		int width,
		int height,
		RenderClass* renderer,
		std::vector<Path>* paths,
		void (*func_updater)(std::vector<float>*)
	);

private:
	int GRID_WIDTH, GRID_HEIGHT;
	std::vector<Node>* node_list;
	local_visualizer render_agent;

	void find_in(ellipse ellipse1);

	void find_circle1(void);

	void find_circle2(void);

	void find_ellipse1(void);
};

#endif