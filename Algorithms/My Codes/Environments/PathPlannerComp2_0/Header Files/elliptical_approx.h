#ifndef ELLIPTICAL_APPROX_H
#define ELLIPTICAL_APPROX_H

#include <vector>
#include <iostream>
#include <dlib/optimization.h>
#include "mcc.h"
#include "RenderClass.h"
#include "constants.h"
#include "Algorithms/voronoi.h"

struct Point {
	double x, y;
};

struct Line {
	float x1, y1, x2, y2;
};

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

struct result {
	int i = 0;
	float area = 0.0f, independence = 0.0f, vertices = 0.0f, bridge_width = 0.0f, skewness = 0.0f;

	result(int i, float area, float independence, float vertices, float bridge_width, float skewness) {
		this->i = i;
		this->area = area;
		this->independence = independence;
		this->vertices = vertices;
		this->bridge_width = bridge_width;
		this->skewness = skewness;
	}

	result(int i) {
		this->i = i;
		this->area = 0;
		this->independence = 0;
		this->vertices = 0;
		this->bridge_width = 0;
		this->skewness = 0;
	}
};

class local_visualizer {
public:
	std::vector<Path>* paths;
	int scribbled_paths;
	local_visualizer(void);

	local_visualizer(int GRID_WIDTH, int GRID_HEIGHT, RenderClass* renderer, std::vector<Path>* paths,
		void (*func_updater)(std::vector<float>*));

	void clear_path(void);

	void clear_paths(void);

	void clear_paths(int count);

	void add_path(Path path);

	void draw_ellipse(ellipse ellipse);

	void draw_ears(int data_x[], int data_y[], std::vector<std::vector<int>>* poly_indice);

	void visualize_ear(int data_x[], int data_y[], std::vector<int>* ear, bool clear_again);

	void show_edges(std::vector<Line>* edge_list);

	void invalidate(void);

private:
	int GRID_WIDTH, GRID_HEIGHT;
	RenderClass* renderer;
	//handling callbacks render-counter part function
	typedef void (*external_render_counter_part)(std::vector<float>*);
	external_render_counter_part render_callback;
};

class convex_clustering {
public:

	void filter_dimples(std::vector<result>* traversal, std::vector<result>* filter, int filter_size);

	void cleaning_dimples(std::vector<result>* dimples);

	void clustrify(std::vector<result>* dimples, std::vector<std::vector<int>>* clustered_data);

	void clustering_2(std::vector<Node>* node_list, int GRID_WIDTH);

	void clustering(void);

	int convex_completion_handler(int pt, int edge2, std::vector<std::vector<int>>* poly_indice);

	bool leftOnly(float line_x1, float line_y1, float line_x2, float line_y2, float pt_x0, float pt_y0);

	bool leftOn(float line_x1, float line_y1, float line_x2, float line_y2, float pt_x0, float pt_y0);

	void ear_handler(int edge1, int edge2, int pt, std::vector<std::vector<int>>* poly_indice);

	convex_clustering(local_visualizer* render_agent) {
		this->render_agent = render_agent;
		data_size = 107;
	}
private:
	local_visualizer* render_agent;
	int data_size = 0;
	int data_x[107] = { 23, 20, 17, 17, 15, 15, 14, 14, 13, 13, 12, 12, 13, 13, 14, 14, 15, 15, 17, 17, 19, 19, 23, 26, 28, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 55, 55, 56, 56, 57, 57, 58, 58, 60, 60, 62, 62, 66, 69, 73, 73, 75, 75, 77, 77, 78, 78, 79, 79, 80, 80, 79, 79, 78, 78, 77, 77, 75, 75, 73, 73, 72, 70, 65, 64, 64, 60, 59, 57, 56, 54, 53, 51, 50, 48, 46, 45, 41, 40, 38, 36, 36, 37, 37, 36, 36, 35, 35, 34, 34, 32, 32, 30, 30, 29, 27 };
	int data_y[107] = { 12, 13, 16, 17, 19, 20, 21, 23, 24, 29, 30, 38, 39, 44, 45, 47, 48, 49, 51, 52, 53, 54, 56, 56, 55, 55, 56, 57, 59, 60, 62, 63, 64, 65, 67, 68, 69, 76, 77, 82, 83, 85, 86, 87, 89, 90, 91, 92, 94, 94, 92, 91, 90, 89, 87, 86, 85, 83, 82, 77, 76, 67, 66, 62, 61, 59, 58, 57, 55, 54, 53, 52, 51, 50, 49, 48, 47, 45, 44, 43, 42, 41, 40, 39, 38, 37, 37, 38, 40, 41, 42, 42, 39, 38, 29, 28, 24, 23, 21, 20, 19, 17, 16, 15, 14, 13, 12 };
};

class elliptical_approx {
public:
	typedef dlib::matrix<double, 0, 1> column_vector;
	void init(
		std::vector<Node>* node_list,
		int width,
		int height,
		RenderClass* renderer,
		std::vector<Path>* paths,
		void (*func_updater)(std::vector<float>*)
	);

	double rateCurveElliptical(const column_vector& params);

	elliptical_approx() {}

private:
	int GRID_WIDTH, GRID_HEIGHT;
	std::vector<Node>* node_list;
	std::vector<polygon2D> obstacles;
	local_visualizer render_agent;
	std::vector<Point> points;
	convex_clustering* cluster;

	class coord {
	public:
		int a, b;
		coord(int a, int b) {
			this->a = a;
			this->b = b;
		}
	};

	void find_in(ellipse ellipse1);

	void find_circle1(void);

	void find_circle2(void);

	void find_ellipse1(void);

	void contour_extractor(void);

	int contour_explorer(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int start, int pass);

	bool build_contour(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int starting_param);

	double dist(double x1, double y1, double x2, double y2);

	Point ellipseParametric(double x0, double y0, double a, double b, double alpha, double t);

	void merge_strips(int row, int index2, polygon2D* obstacle, std::vector<std::vector<coord>>* strips_list);

	void locate_obstacles(void);
};

#endif