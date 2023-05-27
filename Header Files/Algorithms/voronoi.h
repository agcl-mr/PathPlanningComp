#ifndef VORONOI_ALGO_H
#define VORONOI_ALGO_H

#include <vector>
#include<iostream>
#include <algorithm>
#include <math.h>
#include<Windows.h>
#include<map>

#include "../constants.h"
#include "../mcc.h"
#include "../RenderClass.h"

class point {
public:
	float x, y;

	point() {
		x = 0;
		y = 0;
	}

	point(float x, float y) {
		this->x = x;
		this->y = y;
	}

	void add(point pt);

	void scale(float scale);
};

class polygonEdge {
public:
	float x1, y1, x2, y2;

	polygonEdge() {
		this->x1 = 0;
		this->x2 = 0;
		this->y1 = 0;
		this->y2 = 0;
	}

	polygonEdge(float x1, float y1, float x2, float y2) {
		this->x1 = x1;
		this->y1 = y1;
		this->x2 = x2;
		this->y2 = y2;
	}

	int checkLeft(float x3, float y3);

	bool leftOn(float x3, float y3);

	bool rightOn(float x3, float y3);

	bool isIntersecting(polygonEdge* edge);

	bool isExtendedIntersecting(polygonEdge* edge);

	bool isVectorExtendedIntersecting(polygonEdge* edge);

	bool isStrictlyVectorExtendedIntersecting(polygonEdge* edge);

	float length(void);

	bool isNull(void) {return (x1 == 0 && x2 == 0 && y1 == 0 && y2 == 0);}

	polygonEdge invert(void);

	float min_distance_from_line_segment(float x, float y);

	float dot_product(polygonEdge* edge);
};

static class CompGeomFunc {
public:
	static bool leftOn(float x1, float y1, float x2, float y2, float x0, float y0);

	static bool rightOn(float x1, float y1, float x2, float y2, float x0, float y0);

	static point intersection_point(polygonEdge* slicee_edge, polygonEdge* slicer_edge);

	static bool isIntersecting(polygonEdge* edge1, polygonEdge* edge2);

	static bool isExtendedIntersecting(polygonEdge* fixed_edge, polygonEdge* extendable_edge);

	static bool isVectorExtendedIntersecting(polygonEdge* fixed_edge, polygonEdge* extendable_edge);

	static float distance(float x1, float y1, float x2, float y2);

	static int getSign(float x);
};

class rectangularBound {
public:
	int left, right, top, bottom;

	rectangularBound(void) {
		left = 0;
		right = 0;
		top = 0;
		bottom = 0;
	}

	rectangularBound(int left, int right, int top, int bottom) {
		this->left = left;
		this->right = right;
		this->top = top;
		this->bottom = bottom;
	}
};

class renderAgent;

class polygon2D {
private:
public:
	class neighbourSweep {
	public:
		polygonEdge internal_tangent1, internal_tangent2, internal_range,
			external_tangent1, external_tangent2, external_range;
		polygonEdge visibility_limit_left, visibility_limit_right, visibility_range;
		polygonEdge *temp_left_pointer = nullptr, *temp_right_pointer = nullptr;
		polygon2D* pointer;
		bool visible = false;
		bool direct_neighbour = true;
		std::string id;
		int importance = 0;
		int rank = 0;

		neighbourSweep(polygon2D* neighbour, int me_id) {
			this->internal_tangent1 = polygonEdge();
			this->internal_tangent2 = polygonEdge();
			this->internal_range = polygonEdge();
			this->external_tangent1 = polygonEdge();
			this->external_tangent2 = polygonEdge();
			this->external_range = polygonEdge();
			this->pointer = neighbour;
			this->id = ("GC" + std::to_string(me_id)) + ("X" + std::to_string(neighbour->uniqueID));
			//std::cout << "NEW quad created : " << this->id << std::endl;
		}

		void update_tangents_info(polygonEdge internal_tangent1, polygonEdge internal_tangent2, polygonEdge internal_range,
			polygonEdge external_tangent1, polygonEdge external_tangent2, polygonEdge external_range);

		void evaluate_visibility_range(void);

		int compute_importance(void);
	};

	std::vector<int> vertices;
	std::vector<polygonEdge> edges;
	std::vector<Node>* node_list;
	int GRID_WIDTH = 0;
	rectangularBound looseBounds;
	std::vector<neighbourSweep> neighbours;
	int uniqueID = 0;

	polygon2D(std::vector<Node>* node_list, int GRID_WIDTH) {
		this->node_list = node_list;
		this->GRID_WIDTH = GRID_WIDTH;
	}

	int addVertice(int index, int GRID_WIDTH);

	bool checkInside(int index);

	void reconstruct_edges(void);

	bool isInside(int point_x, int point_y);

	void create_neighbour_map(std::vector<polygon2D>* obstacles, renderAgent* render_agent);

	void tangents_handler(polygon2D* neighbour, renderAgent* render_agent);

	void visibility_handler(polygon2D* neighbour, renderAgent* render_agent);

	void visibility_handler_reloaded(renderAgent* render_agent);

private:

	void findPos(int index, int* supporting_points);

	bool leftOn(int x1, int y1, int x2, int y2, int x0, int y0);

	bool rightOn(int x1, int y1, int x2, int y2, int x0, int y0);

	bool checkShielded(polygon2D* neighbour, renderAgent* render_agent);

	bool checkShielded_reloaded(polygon2D* neighbour, renderAgent* render_agent);

	point intersection_point(polygonEdge* slicee_edge, polygonEdge* slicer_edge);

	void find_tangent(
		bool (polygon2D::* discriminator1)(int, int, int, int, int, int),
		bool (polygon2D::* discriminator2)(int, int, int, int, int, int),
		std::vector<int>* set_1, std::vector<int>* set_2, polygonEdge* tangent);

	std::vector<int> create_priority_list(std::vector<polygon2D>* obstacles);

	bool modify_visibility_zones(polygon2D::neighbourSweep* to_be_blocked, polygon2D::neighbourSweep* blocker,
		bool right, bool* flag_direct_neighbour, renderAgent* render_agent);

	bool modify_visibility_zones_reloaded(polygon2D::neighbourSweep* to_be_blocked, polygon2D::neighbourSweep* blocker,
		bool right, renderAgent* render_agent);
};

class freeCellsGraph {
public:
	//point clk_ws_1, clk_ws_2, clk_ws_3, clk_ws_4;
	polygonEdge bound_left, bound_right, bound_range;
	std::vector<freeCellsGraph*> diagraph_sibs, diagraph_sibs_forward, diagraph_sibs_backward;
	std::string identifier;
	int importance;
	int rank;
	freeCellsGraph* potential_path, *last_visited_cell;
	point center;

	freeCellsGraph(polygon2D::neighbourSweep* sweep);

	bool checkIntersecting(freeCellsGraph* cell, renderAgent* render_agent);

	void add_member(freeCellsGraph* cell, std::vector<freeCellsGraph*>* tally, renderAgent* render_agent);

	void add_member_new(freeCellsGraph* cell, std::vector<freeCellsGraph*>* tally, renderAgent* render_agent);

	void traverser1(std::vector<freeCellsGraph*>* tally_list, renderAgent* render_agent);

	bool surrounds(int x, int y);

	float how_far_is(int x, int y);

	void search_path(freeCellsGraph* goal);

	void find_center(void);

private:
	void evaluate_bounding_range(void);

	bool* leftOn(float x, float y);
};

class renderAgent {
private:
	int GRID_WIDTH;
	std::vector<Path>* paths;
	RenderClass* renderer;
	//handling callbacks render-counter part function
	typedef void (*external_render_counter_part)(std::vector<float>*);
	external_render_counter_part render_callback;

public:
	renderAgent() {
		GRID_WIDTH = 0;
		paths = nullptr;
		renderer = nullptr;
		render_callback = nullptr;
	}

	renderAgent(int GRID_WIDTH, RenderClass* renderer, std::vector<Path>* paths, void (*func_updater)(std::vector<float>*));

	void clear_paths(void);

	void clear_paths(int count);

	void add_path(Path path);

	void add_path(polygonEdge* edge);

	void add_path(polygonEdge* edge, bool floating_point);

	void add_path(polygonEdge* edge, bool floating_point, Color color);

	void show_edges(std::vector<polygonEdge>* edge_list);

	void invalidate(void);

	void visualising_helper1(polygonEdge edge1, polygonEdge edge2, polygonEdge edge3,
		polygonEdge edge4, polygonEdge edge5, polygonEdge edge6);

	void visualizing_helper2(polygon2D* polygonal_obstacle);

	void visualizing_helper3(polygon2D* polygonal_obstacle, std::vector<float>* vertices);

	void visualizing_helper4(freeCellsGraph *freeCell);

	void visualising_helper5(polygonEdge edge1, polygonEdge edge2, polygonEdge edge3,
		polygonEdge edge4, polygonEdge edge5, polygonEdge edge6);

	void visualizing_helper6(freeCellsGraph* start, freeCellsGraph* goal,
		int start_x, int start_y, int goal_x, int goal_y);
};

class voronoi_algo {
public:
	void init(
		std::vector<Node>* node_list, 
		int width, 
		int height, 
		RenderClass* renderer, 
		std::vector<Path>* paths, 
		void (*func_updater)(std::vector<float>*)
	);

	void finder(int start_cell_index, int goal_cell_index);

	void checkInside(int index);

private:
	std::vector<Node>* node_list;
	std::vector<polygon2D> obstacles;
	std::map<int, freeCellsGraph*> good_quads;


	bool *checklist;
	int GRID_WIDTH, GRID_HEIGHT;
	int start_cell_index, goal_cell_index;

	renderAgent render_agent;

	class coord {
	public:
		int a, b;
		coord(int a, int b) {
			this->a = a;
			this->b = b;
		}
	};

	void locate_obstacles(void);

	void merge_strips_up(int index1, int index2, polygon2D* obstacle, std::vector<std::vector<coord>>* strips_list, std::string id);

	void merge_strips(int index1, int index2, polygon2D* obstacle, std::vector<std::vector<coord>>* strips_list, std::string id);

	void build_quad_graph(freeCellsGraph** graphPointer, std::map<int, freeCellsGraph*>* priority_quads);
};

#endif