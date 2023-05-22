#ifndef ELLIPTICAL_APPROX_H
#define ELLIPTICAL_APPROX_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <dlib/optimization.h>
#include <Eigen/Dense>
//#include <cmath>
#include "mcc.h"
#include "RenderClass.h"
#include "constants.h"
#include "Algorithms/voronoi.h"

struct Point {
	double x, y;

	bool equals(Point point) {
		float del_x = std::abs(point.x - x);
		float del_y = std::abs(point.y - y);
		const float PRECISION = 0.01;

		if (del_x < PRECISION && del_y < PRECISION)
			return true;
		return false;
	}
};

struct int_point {
	int x, y;
	int_point(int x, int y) {
		this->x = x;
		this->y = y;
	}
};

struct strip {
	int start, end;
	strip(int start, int end) {
		this->start = start;
		this->end = end;
	}
};

struct Line {
	float x1, y1, x2, y2;
};

class ellipse;

class vector_segment {
public:
	float x1, y1, x2, y2;

	vector_segment() {
		this->x1 = 0;
		this->x2 = 0;
		this->y1 = 0;
		this->y2 = 0;
	}

	vector_segment(float x1, float y1, float x2, float y2) {
		this->x1 = x1;
		this->y1 = y1;
		this->x2 = x2;
		this->y2 = y2;
	}

	int checkLeft(float x3, float y3);

	bool leftOn(float x3, float y3);

	float leftPredicate(float x3, float y3);

	bool rightOn(float x3, float y3);

	float subtended_angle_measure(float x3, float y3);

	bool isIntersecting(vector_segment* edge);

	bool isExtendedIntersecting(vector_segment* edge);

	bool isVectorExtendedIntersecting(vector_segment* edge);

	bool isStrictlyVectorExtendedIntersecting(vector_segment* edge);

	float length(void);

	bool isNull(void) { return (x1 == 0 && x2 == 0 && y1 == 0 && y2 == 0); }

	vector_segment invert(void);

	float min_distance_from_line_segment(float x, float y);

	float dot_product(vector_segment* edge);

	float normalized_dot_product(vector_segment* edge);

	float perpendicular_distance(float x, float y);

	void intersection_points(ellipse* ellipse, vector_segment* result);

	Point intersection_points(vector_segment* edge);
};

class local_visualizer;

class quad {
public:
	class sibling {
	public:
		quad* path;
		float area_measure; // common area between two pair of quads
		float length_measure; // extent till which common boundaries run wrt. common ellipse center
		// -- max distance of a point on common poly to ellipse center
		float length_measure2; // length of common junction between 2 quads
		Point ellip_center, farthest_point;

		sibling(quad* neighbour, float area_measure, float length_measure, 
			float length_measure2, Point ellip_center, Point farthest_point);

		float importance_function1(void);
	};
		
	vector_segment free1, free2, blocked1, blocked2; // all these wrt. A
	ellipse* me_A, * me_B;
	std::vector<sibling> neighbours_A, neighbours_B;
	std::vector<int> order_A, order_B;
	int id = 0;
	float mfa = 0.0, mpl = 0.0, mpw = 0.0;
	//mfa : mean free area, mpl : mean path length, mpw : mean path width

	quad(ellipse* me_A, ellipse* me_B, vector_segment external_tangent_1, vector_segment external_tangent_2, int id) {
		this->me_A = me_A;
		this->me_B = me_B;
		free1 = external_tangent_1;
		free2 = external_tangent_2;
		blocked1 = vector_segment(external_tangent_1.x1, external_tangent_1.y1,
			external_tangent_2.x1, external_tangent_2.y1);
		blocked2 = vector_segment(external_tangent_1.x2, external_tangent_1.y2,
			external_tangent_2.x2, external_tangent_2.y2);
		this->id = id;
		compute_params();
	}

	bool isIntersecting(quad* quad, local_visualizer* render_agent);

	void map_expander(quad* quad, std::vector<bool>* checklist, local_visualizer* render_agent);

	void estimate_ellipse_blockage(ellipse* ellip, vector_segment chord, float* area_reduction,
		float* length_reduction);

	void compute_params(void);

	float common_area(quad* quad, local_visualizer* render_agent);

	void common_area_v2(quad* other, ellipse* common_ellipse, local_visualizer* render_agent);

	float poly_bounds(quad* quad, local_visualizer* render_agent);

	bool clockwise_intersection_locator(std::vector<Point>* set_2,
		vector_segment* edge1, int index_start_2, int* index_set2, Point* intersection_pt,
		bool propogation);

	float importance_function1(void);

	float distance(float x, float y);

	bool isInside(float x, float y);
};

static class CompGeomFuncEllipseApprox {
public:
	static bool leftOn(float x1, float y1, float x2, float y2, float x0, float y0);

	static float leftPredicate(float x1, float y1, float x2, float y2, float x0, float y0);

	static bool rightOn(float x1, float y1, float x2, float y2, float x0, float y0);

	static Point intersection_point(vector_segment* slicee_edge, vector_segment* slicer_edge);

	static bool isIntersecting(vector_segment* edge1, vector_segment* edge2);

	static bool isExtendedIntersecting(vector_segment* fixed_edge, vector_segment* extendable_edge);

	static bool isVectorExtendedIntersecting(vector_segment* fixed_edge, vector_segment* extendable_edge);

	static float distance(float x1, float y1, float x2, float y2);

	static int getSign(float x);

	static float triangle_area(float x1, float y1, float x2, float y2, float x3, float y3);
};

class ellipse {
public:
	int uniqueID = 0;
	int origin_signature = 0;
	class neighbourSweep;
	std::vector<neighbourSweep> neighbours;

	float center_x, center_y, a, b, tilt;
	ellipse() {
		center_x = 0.0f;
		center_y = 0.0f;
		a = 0.0f;
		b = 0.0f;
		tilt = 0.0f;
	}

	ellipse(float center_x, float center_y, float a, float b, float tilt/*tilt has to be in radians*/) {
		this->center_x = center_x;
		this->center_y = center_y;
		this->a = a;
		this->b = b;
		this->tilt = tilt;
	}

	bool isInside(float x, float y, int i);

	class neighbourSweep {
	public:
		vector_segment internal_tangent1, internal_tangent2, internal_range,
			external_tangent1, external_tangent2, external_range;
		vector_segment visibility_limit_left, visibility_limit_right, visibility_range;
		vector_segment* temp_left_pointer = nullptr, * temp_right_pointer = nullptr;
		ellipse* pointer;
		bool visible = false;
		bool direct_neighbour = true;
		std::string id;
		int importance = 0;
		int rank = 0;

		neighbourSweep(ellipse* neighbour, int me_id) {
			this->internal_tangent1 = vector_segment();
			this->internal_tangent2 = vector_segment();
			this->internal_range = vector_segment();
			this->external_tangent1 = vector_segment();
			this->external_tangent2 = vector_segment();
			this->external_range = vector_segment();
			this->pointer = neighbour;
			//this->id = ("GC" + std::to_string(me_id)) + ("X" + std::to_string(neighbour->uniqueID));
			//std::cout << "NEW quad created : " << this->id << std::endl;
		}

		void update_tangents_info(vector_segment internal_tangent1, vector_segment internal_tangent2,
			vector_segment external_tangent1, vector_segment external_tangent2);

		void evaluate_visibility_range(void);

		int compute_importance(void);

		bool is_degenerate(void);
	};

	class boundingChords {
	public:
		vector_segment bound1, bound2;
		bool bound1_empty = true, bound2_empty = true, empty = true;

		boundingChords() {
			bound1 = vector_segment();
			bound2 = vector_segment();
			bound1_empty = true;
			bound2_empty = true;
			empty = true;
		}
		
		boundingChords(vector_segment* bound1, vector_segment* bound2);

		vector_segment* update_bound(vector_segment* original, vector_segment* update);

		void intersection_update(boundingChords bound);
	};

	std::vector<int> create_priority_list(std::vector<ellipse>* obstacles);

	bool checkShielded_reloaded(ellipse* neighbour, local_visualizer* render_agent);

	bool checkShielded_reloaded_v2(ellipse* neighbour, local_visualizer* render_agent);

	void create_neighbour_map(std::vector<ellipse>* obstacles, local_visualizer* render_agent);

	ellipse::neighbourSweep compute_approximate_tangent(ellipse* neighbour, local_visualizer* render_agent);

	void approximate_tangent(ellipse* neighbour, local_visualizer* render_agent);

	void tangents_handler(ellipse* neighbour, local_visualizer* render_agent);

	void visibility_handler_reloaded(local_visualizer* render_agent);

	bool modify_visibility_zones_reloaded(ellipse::neighbourSweep* to_be_blocked,
		ellipse::neighbourSweep* blocker, bool right, local_visualizer* render_agent);

	bool isIntersecting(ellipse* ellipse);

private:
};

struct local_path_node {
	Point coords;
	quad* freeCell;

	local_path_node(float x, float y, quad* freeCell) {
		this->coords = Point(x, y);
		this->freeCell = freeCell;
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

	void add_path(vector_segment vector_segment);

	void add_path(vector_segment vector_segment, Color color);

	void draw_ellipse(ellipse ellipse);

	int draw_ellipse(ellipse ellipse, Color color);

	void draw_ears_v2(std::vector<int_point>* points, std::vector<std::vector<int>>* poly_indice);

	void draw_ears(int data_x[], int data_y[], std::vector<std::vector<int>>* poly_indice);

	void draw_quad(quad* quad);

	void draw_quad(quad* quad, Color color);

	void visualize_ear_2(std::vector<int_point>* points, std::vector<int>* ear, bool clear_again);

	void visualize_ear(int data_x[], int data_y[], std::vector<int>* ear, bool clear_again);

	void visualize_nearest_neighbour_ellipses(ellipse* obstacle);

	void shielding_visualizer(ellipse* obstacle, ellipse* neighbour);

	void visualize_tangent_approximations(ellipse* object1, ellipse* object2);

	void visualize_quads(quad* quad, bool ellipses, bool clear_back);

	void visualize_quads_intersection(quad* quad1, quad* quad2);

	void visualize_quads_intersection(quad* quad1, quad* quad2, std::vector<Point>* poly);

	void visualize_petals(ellipse* obstacle);

	void visualize_path(std::vector<quad*>* path);

	void visualize_path2(std::vector<quad*>* path, vector_segment start_goal);

	void visualize_path3(std::vector<local_path_node> path);

	void visualize_graph_exploration_options(quad* node);

	void show_edges(std::vector<Line>* edge_list);

	void invalidate(void);

private:
	int GRID_WIDTH, GRID_HEIGHT;
	RenderClass* renderer;
	//handling callbacks render-counter part function
	typedef void (*external_render_counter_part)(std::vector<float>*);
	external_render_counter_part render_callback;
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

class convex_clustering {
public:
	std::vector<ellipse> ellipse_list;

	float ear_area_2(std::vector<int>* ear, std::vector<result>* filter, bool special);

	void filter_dimples_2(std::vector<result>* traversal, std::vector<result>* filter, int filter_size);

	void filter_dimples(std::vector<result>* traversal, std::vector<result>* filter, int filter_size);

	void cleaning_dimples(std::vector<result>* dimples);

	void clustrify_v2(std::vector<result>* dimples, std::vector<std::vector<int>>* clustered_data);

	void clustrify(std::vector<result>* dimples, std::vector<std::vector<int>>* clustered_data);

	void clustering_3(std::vector<Node>* node_list, int GRID_WIDTH, std::vector<int_point> points);

	void clustering_2(std::vector<Node>* node_list, int GRID_WIDTH);

	void clustering(void);

	void ellipse_fitter(int poly_index);

	int convex_completion_handler(int pt, int edge2, std::vector<std::vector<int>>* poly_indice);

	bool leftOnly(float line_x1, float line_y1, float line_x2, float line_y2, float pt_x0, float pt_y0);

	bool leftOn(float line_x1, float line_y1, float line_x2, float line_y2, float pt_x0, float pt_y0);

	void ear_handler(int edge1, int edge2, int pt, std::vector<std::vector<int>>* poly_indice);

	convex_clustering(local_visualizer* render_agent) {
		this->render_agent = render_agent;
		data_size = 107;
	}

	convex_clustering(){}
private:
	local_visualizer* render_agent;
	int data_size = 0;
	int data_x[107] = { 23, 20, 17, 17, 15, 15, 14, 14, 13, 13, 12, 12, 13, 13, 14, 14, 15, 15, 17, 17, 19, 19, 23, 26, 28, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 55, 55, 56, 56, 57, 57, 58, 58, 60, 60, 62, 62, 66, 69, 73, 73, 75, 75, 77, 77, 78, 78, 79, 79, 80, 80, 79, 79, 78, 78, 77, 77, 75, 75, 73, 73, 72, 70, 65, 64, 64, 60, 59, 57, 56, 54, 53, 51, 50, 48, 46, 45, 41, 40, 38, 36, 36, 37, 37, 36, 36, 35, 35, 34, 34, 32, 32, 30, 30, 29, 27 };
	int data_y[107] = { 12, 13, 16, 17, 19, 20, 21, 23, 24, 29, 30, 38, 39, 44, 45, 47, 48, 49, 51, 52, 53, 54, 56, 56, 55, 55, 56, 57, 59, 60, 62, 63, 64, 65, 67, 68, 69, 76, 77, 82, 83, 85, 86, 87, 89, 90, 91, 92, 94, 94, 92, 91, 90, 89, 87, 86, 85, 83, 82, 77, 76, 67, 66, 62, 61, 59, 58, 57, 55, 54, 53, 52, 51, 50, 49, 48, 47, 45, 44, 43, 42, 41, 40, 39, 38, 37, 37, 38, 40, 41, 42, 42, 39, 38, 29, 28, 24, 23, 21, 20, 19, 17, 16, 15, 14, 13, 12 };
	std::vector<int_point> points;
	std::vector<std::vector<int_point>> clustered_points;
};

class quad_builder {
public:
	std::vector<quad> quad_list;
	std::vector<int> order;
	quad_builder(std::vector<ellipse> list, local_visualizer* render_agent);
	quad_builder() {}

private:
	std::vector<ellipse> obstacles;
	local_visualizer* render_agent;
	quad* quad_map;

	void stich_quads(void);
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

	void finder(int start_cell_index, int goal_cell_index, 
		std::chrono::time_point<std::chrono::high_resolution_clock> start,
		consolidated_result* result);

	double rateCurveElliptical(const column_vector& params);

	elliptical_approx() {}

private:
	int GRID_WIDTH, GRID_HEIGHT;
	std::vector<Node>* node_list;
	std::vector<polygon2D> obstacles;
	local_visualizer render_agent;
	std::vector<std::vector<int_point>> points;;
	convex_clustering cluster;
	quad_builder map_builder;

	int start_cell_index, goal_cell_index;
	int start_x, start_y, goal_x, goal_y;
	const int ROBOT_SIZE = 25;

	std::vector<long long>run_time;

	class coord {
	public:
		int a, b;
		coord(int a, int b) {
			this->a = a;
			this->b = b;
		}
	};

	void contour_extractor2(void);

	void boundary_pointers_network_to_points_list(Node* start_node, Node* next_node, int first_index, int poly_index);

	void contour_builder_v2(Node* boundary_cell, bool search_left, bool search_right, int last_operation, Node* stopping_node);

	//void contour_builder(Node* boundary_cell, bool search_left, bool search_right, int last_operation, Node* stopping_node);
	
	void call_next_counter_clockwise(Node* boundary_cell, int dir, Node* stopping_node, bool forward);
	
	void contour_analyzer(int first_index, std::vector<std::vector<strip>>* strips);

	int contour_explorer(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int start, int pass);

	bool build_contour(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int starting_param);

	double dist(double x1, double y1, double x2, double y2);

	Point ellipseParametric(double x0, double y0, double a, double b, double alpha, double t);

	quad* nearest_quad(float x, float y);

	float heuristic_function(quad::sibling* node, ellipse* pivot_point);

	bool search_path(quad* start, quad* goal, ellipse* pivot, std::vector<quad*>* path);

	void compute_search_order(std::vector<quad::sibling>* list, std::vector<int>* order, ellipse* common_ellip);

	bool scrape_common_area_measure(quad* quad1, quad* quad2, float* metrics);

	void path_cleanup(std::vector<quad*>* feasible_path);

	void path_cleanup(std::vector<quad*>* feasible_path, std::vector<local_path_node>* local_path);

	int smoothen_paths(int index, std::vector<quad*>* path_list);

	bool check_contains(std::vector<quad*>* quads, std::vector<vector_segment>* edges);

	void local_path_planning(std::vector<quad*>* feasible_path, std::vector<local_path_node>* local_path);

	float path_length(std::vector<local_path_node>* path);

	std::string stringify(std::vector<local_path_node>* path);
};

#endif