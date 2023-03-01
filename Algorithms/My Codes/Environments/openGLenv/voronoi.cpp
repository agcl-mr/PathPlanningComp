#ifndef VORONOI_ALGO_CPP
#define VORONOI_ALGO_CPP
#include "voronoi.h"

bool CompGeomFunc::leftOn(float x1, float y1, float x2, float y2, float x0, float y0) {
	return ((x1 * (y2 - y0) + x2 * (y0 - y1) + x0 * (y1 - y2)) <= 0);
}

bool CompGeomFunc::rightOn(float x1, float y1, float x2, float y2, float x0, float y0) {
	return ((x1 * (y2 - y0) + x2 * (y0 - y1) + x0 * (y1 - y2)) >= 0);
}

point CompGeomFunc::intersection_point(polygonEdge* slicee_edge, polygonEdge* slicer_edge) {
	float x1, y1, x2, y2, a1, b1, a2, b2;
	x1 = slicee_edge->x1;
	y1 = slicee_edge->y1;
	x2 = slicee_edge->x2;
	y2 = slicee_edge->y2;
	a1 = slicer_edge->x1;
	b1 = slicer_edge->y1;
	a2 = slicer_edge->x2;
	b2 = slicer_edge->y2;

	point result;
	float denom1 = float((x2 - x1) * (b2 - b1) - (a2 - a1) * (y2 - y1));
	if (denom1 == 0){
		std::cout << "[WARNING] : Parallel lines\n";
		result.x = (x1 + x2) / 2;
		result.y = (y1 + y2) / 2;
	}
	else {
		result.x = float((a2 - a1) * (y1 * x2 - y2 * x1) - (x2 - x1) * (b1 * a2 - b2 * a1))
			/ denom1;

		if (x2 == x1)
			result.y = float((b2 - b1) * (result.x) + (b1 * a2 - b2 * a1)) / float(a2 - a1);
		else
			result.y = float((y2 - y1) * (result.x) + (y1 * x2 - y2 * x1)) / float(x2 - x1);
	}
	return result;
}

bool CompGeomFunc::isIntersecting(polygonEdge* edge1, polygonEdge* edge2) {
	bool test1 = edge1->leftOn(edge2->x1, edge2->y1);
	bool test2 = edge1->leftOn(edge2->x2, edge2->y2);

	if (test1 == test2)
		return false;

	bool test3 = edge2->leftOn(edge1->x1, edge1->y1);
	bool test4 = edge2->leftOn(edge1->x2, edge1->y2);

	if (test3 == test4)
		return false;

	return true;
}

bool CompGeomFunc::isVectorExtendedIntersecting(polygonEdge* fixed_edge, polygonEdge* extendable_edge) {
	bool test1 = extendable_edge->leftOn(fixed_edge->x1, fixed_edge->y1);
	bool test2 = extendable_edge->leftOn(fixed_edge->x2, fixed_edge->y2);

	if (test1 == test2)
		return false;

	//checks if edge has to be extended from end2 or end1 side
	float dist1 = abs(fixed_edge->x1 * (fixed_edge->y2 - extendable_edge->y1) + fixed_edge->x2 *
		(extendable_edge->y1 - fixed_edge->y1) + extendable_edge->x1 * (fixed_edge->y1 - fixed_edge->y2));
	float dist2 = abs(fixed_edge->x1 * (fixed_edge->y2 - extendable_edge->y2) + fixed_edge->x2 *
		(extendable_edge->y2 - fixed_edge->y1) + extendable_edge->x2 * (fixed_edge->y1 - fixed_edge->y2));
	if (dist2 > dist1) // end2 is farther from fixed line; so needs to be extended from end1
		return false;

	return true;
}

bool CompGeomFunc::isExtendedIntersecting(polygonEdge* fixed_edge, polygonEdge* extendable_edge) {
	bool test1 = extendable_edge->leftOn(fixed_edge->x1, fixed_edge->y1);
	bool test2 = extendable_edge->leftOn(fixed_edge->x2, fixed_edge->y2);

	if (test1 == test2)
		return false;

	return true;
}

float CompGeomFunc::distance(float x1, float y1, float x2, float y2) {
	return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
}

int CompGeomFunc::getSign(float x) {
	if (x >= 0)
		return 1;
	else
		return 0;
}

void point::add(point pt) {
	x += pt.x;
	y += pt.y;
}

void point::scale(float scale) {
	x *= scale;
	y *= scale;
}

int polygonEdge::checkLeft(float x3, float y3) {
	float expression = (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
	//if (expression != 0)
		if (expression > 0)
			return 1;
		else
			return -1;
	return 0;
	//return ((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) >= 0);
}

bool polygonEdge::leftOn(float x3, float y3) {
	return CompGeomFunc::leftOn(x1, y1, x2, y2, x3, y3);
}

bool polygonEdge::rightOn(float x3, float y3) {
	return CompGeomFunc::rightOn(x1, y1, x2, y2, x3, y3);
}

bool polygonEdge::isIntersecting(polygonEdge* edge) {
	return CompGeomFunc::isIntersecting(this, edge);
}

bool polygonEdge::isExtendedIntersecting(polygonEdge* edge) {
	return CompGeomFunc::isExtendedIntersecting(edge, this);
}

bool polygonEdge::isVectorExtendedIntersecting(polygonEdge* edge) {
	return CompGeomFunc::isVectorExtendedIntersecting(edge, this);
}

bool polygonEdge::isStrictlyVectorExtendedIntersecting(polygonEdge* edge) {
	if (CompGeomFunc::isIntersecting(edge, this))
		return false;
	return CompGeomFunc::isVectorExtendedIntersecting(edge, this);
}

float polygonEdge::length(void) {
	return CompGeomFunc::distance(x1, y1, x2, y2);
}

polygonEdge polygonEdge::invert(void) {
	return polygonEdge(x2, y2, x1, y1);
}

float polygonEdge::min_distance_from_line_segment(float x, float y) {
	float distance;
	if (CompGeomFunc::getSign(polygonEdge(x, y, this->x1, this->y1).dot_product(this))
				!= CompGeomFunc::getSign(polygonEdge(x, y, this->x2, this->y2).dot_product(this))) {
		// projection of point on line segment falls between the end points;
		// So minimum distance is the perpendicular distance
		distance = std::abs(((this->x1-x)*(this->y2-this->y1)-(this->y1-y)*(this->x2-this->x1))/this->length());
	}
	else {
		// projection of point on line segment falls outside the bounds of line segment
		// So minimum distance is the min of distance from this point to line segment ends
		float dist1 = polygonEdge(x, y, x1, y1).length();
		float dist2 = polygonEdge(x, y, x2, y2).length();
		distance = (((dist1) < (dist2)) ? (dist1) : (dist2));
	}
	return distance;
}

float polygonEdge::dot_product(polygonEdge* edge) {
	return ((this->x2 - this->x1) * (edge->x2 - edge->x1) + (this->y2 - this->y1) * (edge->y2 - edge->y1));
}

void polygon2D::neighbourSweep::update_tangents_info(polygonEdge internal_tangent1, polygonEdge internal_tangent2, polygonEdge internal_range,
	polygonEdge external_tangent1, polygonEdge external_tangent2, polygonEdge external_range) {
	this->internal_tangent1 = internal_tangent1;
	this->internal_tangent2 = internal_tangent2;
	this->internal_range = internal_range;
	this->external_tangent1 = external_tangent1;
	this->external_tangent2 = external_tangent2;
	this->external_range = external_range;

	this->visibility_limit_left = external_tangent1;
	this->visibility_limit_right = external_tangent2;
	evaluate_visibility_range();
}

void polygon2D::neighbourSweep::evaluate_visibility_range(void) {
	if (visibility_range.isNull()) {
		visibility_range = polygonEdge(visibility_limit_left.x2, visibility_limit_left.y2, 
			visibility_limit_right.x2, visibility_limit_right.y2);
	}
	else {
		visibility_range.x1 = visibility_limit_left.x2;
		visibility_range.y1 = visibility_limit_left.y2;
		visibility_range.x2 = visibility_limit_right.x2;
		visibility_range.y2 = visibility_limit_right.y2;
	}
}

int polygon2D::neighbourSweep::compute_importance(void) {
	float x1, x2, x3, x4, y1, y2, y3, y4;
	x1 = this->visibility_limit_left.x1;
	x2 = this->visibility_limit_left.x2;
	x3 = this->visibility_limit_right.x2;
	x4 = this->visibility_limit_right.x1;
	y1 = this->visibility_limit_left.y1;
	y2 = this->visibility_limit_left.y2;
	y3 = this->visibility_limit_right.y2;
	y4 = this->visibility_limit_right.y1;
	float param1 = 0.5 * ((x1 * y2 + x2 * y3 + x3 * y4 + x4 * y1) - (x2 * y1 + x3 * y2 + x4 * y3 + x1 * y4));
	float len1 = visibility_limit_left.length();
	float len2 = visibility_limit_right.length();
	float len3 = visibility_range.length();
	float len4 = polygonEdge(visibility_limit_left.x1, visibility_limit_left.y1,
		visibility_limit_right.x1, visibility_limit_right.y1).length();
	float param2 = ((len1 > len2) ? (len1) : (len2));
	float param3 =  1 /(((len3 > len4) ? (len3) : (len4)));
	float importance = param1*param2*param3;
	std::cout << importance << std::endl;
	this->importance = importance;
	return importance;
}

int polygon2D::addVertice(int index, int GRID_WIDTH) {
	float x1, y1, x2, y2, x3, y3;
	
	switch (vertices.size()) {
	case 0:
		vertices.push_back(index);
		return 0;
	case 1:
		x1 = node_list->at(vertices.at(0)).x;
		y1 = node_list->at(vertices.at(0)).y;
		x3 = node_list->at(index).x;
		y3 = node_list->at(index).y;
		if ((std::max)(std::abs(x1 - x3), std::abs(y1 - y3)) <= 1)
			vertices.push_back(index);
		return 1;
	case 2:
		x1 = vertices.at(0) % GRID_WIDTH;
		y1 = vertices.at(0) / GRID_WIDTH;
		x2 = vertices.at(1) % GRID_WIDTH;
		y2 = vertices.at(1) / GRID_WIDTH;
		x3 = index % GRID_WIDTH;
		y3 = index / GRID_WIDTH;

		std::cout << "3 results : " << (std::max)(std::abs(x1 - x3), std::abs(y1 - y3)) << " | " << (std::max)(std::abs(x2 - x3), std::abs(y2 - y3)) << " | " << (std::min)(
			(std::max)(std::abs(x1 - x3), std::abs(y1 - y3)),
			(std::max)(std::abs(x2 - x3), std::abs(y2 - y3))
			) << std::endl;

		if ((std::min)(
			(std::max)(std::abs(x1 - x3), std::abs(y1 - y3)),
			(std::max)(std::abs(x2 - x3), std::abs(y2 - y3))
			) <= 1)
			vertices.push_back(index);
		std::cout << "size_update : " << vertices.size() << std::endl;

		return 2;
	default:
		int supporting_points[2] = { 0, 0 };
		findPos(index, supporting_points);

		if (supporting_points[0] == 0 && supporting_points[1] == 0)
			return 3;

		std::cout << "point addition approved!\n";
		std::cout << "Before update : supporting_points[0] : " << supporting_points[0] << " , supporting_points[1] : " << supporting_points[1] << std::endl;

		if (supporting_points[1] < supporting_points[0])
			supporting_points[1] += (int) vertices.size();
		std::cout << "After update : supporting_points[0] : " << supporting_points[0] << " , supporting_points[1] : " << supporting_points[1] << std::endl;

		for (int i = 0; i < supporting_points[1] - supporting_points[0] - 1; i++) {
			if (supporting_points[0] + 1 < vertices.size())
				vertices.erase(vertices.begin() + supporting_points[0] + 1);
			else
				vertices.erase(vertices.begin());
		}

		vertices.insert(vertices.begin() + supporting_points[0] + 1, index);

		return  4;
	}
}

bool polygon2D::checkInside(int index) {
	int supporting_vecs[2] = {0, 0};
	findPos(index, supporting_vecs);

	if (supporting_vecs[0] == 0 && supporting_vecs[1] == 0)
		return false;
	return true;
}

void polygon2D::reconstruct_edges(void) {
	if (vertices.size() < 2)
		return;

	int x1, y1, x2, y2, x3, y3;

	for (int i = 0; i < vertices.size(); i++) {
		if (vertices.size() < 2)
			return;
		//identifying 3 consecutive points
		if (i == 0) {
			x1 = vertices.at(vertices.size() - 1) % GRID_WIDTH;
			y1 = vertices.at(vertices.size() - 1) / GRID_WIDTH;
			x2 = vertices.at(i) % GRID_WIDTH;
			y2 = vertices.at(i) / GRID_WIDTH;
			x3 = vertices.at(i + 1) % GRID_WIDTH;
			y3 = vertices.at(i + 1) / GRID_WIDTH;
		}
		else if (i == (int)(vertices.size()) - 1) {
			x1 = vertices.at(i - 1) % GRID_WIDTH;
			y1 = vertices.at(i - 1) / GRID_WIDTH;
			x2 = vertices.at(i) % GRID_WIDTH;
			y2 = vertices.at(i) / GRID_WIDTH;
			x3 = vertices.at(0) % GRID_WIDTH;
			y3 = vertices.at(0) / GRID_WIDTH;
		}
		else {
			x1 = vertices.at(i-1) % GRID_WIDTH;
			y1 = vertices.at(i-1) / GRID_WIDTH;
			x2 = vertices.at(i) % GRID_WIDTH;
			y2 = vertices.at(i) / GRID_WIDTH;
			x3 = vertices.at(i+1) % GRID_WIDTH;
			y3 = vertices.at(i+1) / GRID_WIDTH;
		}

		// query for slope on the 3 points identified; if slopes are equal --> delete the second point
		if (x2 == x1) {
			if (x3 == x2) {
				vertices.erase(vertices.begin() + i);
				i--;
			}
		}
		else {
			if (x3 != x2) {
				float slope1 = float((y2 - y1)) / float((x2 - x1));
				float slope2 = float((y3 - y2)) / float((x3 - x2));
				if (slope1 == slope2) {
					vertices.erase(vertices.begin() + i);
					i--;
				}
			}
		}
	}

	if (vertices.size() < 2)
		return;

	if (!edges.empty())
		edges.clear();

	// convert to edge format
	for (int i = 0; i < vertices.size(); i++) {
		if (i == (int)(vertices.size()) - 1)
			edges.push_back(polygonEdge(
				node_list->at(vertices.at(i)).x,
				node_list->at(vertices.at(i)).y,
				node_list->at(vertices.at(0)).x,
				node_list->at(vertices.at(0)).y
			));
		else
			edges.push_back(polygonEdge(
				node_list->at(vertices.at(i)).x,
				node_list->at(vertices.at(i)).y,
				node_list->at(vertices.at(i + 1)).x,
				node_list->at(vertices.at(i + 1)).y
			));
	}
}

bool polygon2D::isInside(int point_x, int point_y) {
	if (point_y < looseBounds.top || point_y > looseBounds.bottom)
		return false;
	if (point_x < looseBounds.left || point_x > looseBounds.right)
		return false;

	float left_lim = 0.0f, right_lim = 0.0f;
	float flag_left = false, flag_right = false;

	int SIZE = vertices.size();
	for (int i = 0; i < SIZE; i++) {
		if (flag_left && flag_right)
			break;
		if (i == 0) {
			if (vertices.at(i) / GRID_WIDTH >= point_y && vertices.at(SIZE - 1) / GRID_WIDTH <= point_y) {
				int x1 = vertices.at(SIZE - 1) % GRID_WIDTH;
				int y1 = vertices.at(SIZE - 1) / GRID_WIDTH;
				int x2 = vertices.at(i) % GRID_WIDTH;
				int y2 = vertices.at(i) / GRID_WIDTH;

				if (y1 == y2)
					left_lim = (x1 < x2) ? x1 : x2; //min() function
				else
					left_lim = float((y2 - point_y) * (x1)+(point_y - y1) * (x2)) / float(y2 - y1);
				flag_left = true;
			}
			if (vertices.at(SIZE - i - 1) / GRID_WIDTH >= point_y && vertices.at(0) / GRID_WIDTH <= point_y) {
				int x1 = vertices.at(0) % GRID_WIDTH;
				int y1 = vertices.at(0) / GRID_WIDTH;
				int x2 = vertices.at(SIZE - i - 1) % GRID_WIDTH;
				int y2 = vertices.at(SIZE - i - 1) / GRID_WIDTH;

				if (y1 == y2)
					right_lim = (x1 > x2) ? x1 : x2; //max() function
				else
					right_lim = float((y2 - point_y) * (x1)+(point_y - y1) * (x2)) / float(y2 - y1);
				flag_right = true;
			}

		}
		else {
			if (vertices.at(i) / GRID_WIDTH >= point_y && vertices.at(i - 1) / GRID_WIDTH <= point_y) {
				int x1 = vertices.at(i - 1) % GRID_WIDTH;
				int y1 = vertices.at(i - 1) / GRID_WIDTH;
				int x2 = vertices.at(i) % GRID_WIDTH;
				int y2 = vertices.at(i) / GRID_WIDTH;

				if (y1 == y2)
					left_lim = (x1 < x2) ? x1 : x2; //min() function
				else
					left_lim = float((y2 - point_y) * (x1)+(point_y - y1) * (x2)) / float(y2 - y1);
				flag_left = true;
			}
			if (vertices.at(SIZE - i - 1) / GRID_WIDTH >= point_y && vertices.at(SIZE - i) / GRID_WIDTH <= point_y) {
				int x1 = vertices.at(SIZE - i) % GRID_WIDTH;
				int y1 = vertices.at(SIZE - i) / GRID_WIDTH;
				int x2 = vertices.at(SIZE - i - 1) % GRID_WIDTH;
				int y2 = vertices.at(SIZE - i - 1) / GRID_WIDTH;

				if (y1 == y2)
					right_lim = (x1 > x2) ? x1 : x2; //max() function
				else
					right_lim = float((y2 - point_y) * (x1)+(point_y - y1) * (x2)) / float(y2 - y1);
				flag_right = true;
			}
		}
	}

	if (point_x < left_lim || point_x > right_lim)
		return false;

	return true;
}

void polygon2D::create_neighbour_map(std::vector<polygon2D>* obstacles, renderAgent* render_agent) {

	int clear_count = 0;

	// sort list to iterate through nearest objects first; will reduce unnecessary computations
	std::vector<int> traversal_order = create_priority_list(obstacles);

	for (int i = 0; i < obstacles->size(); i++) {
		// iterate in a systematic way by this traversal
		//polygon2D* obstacle = &obstacles->at(i);
		// greedy approach; iterating through nearest obstacle first
		polygon2D* obstacle = &obstacles->at(traversal_order.at(i));
		if (this == obstacle) {
			continue;
		}

		// check if this obstacle is already shielded by existing neighbour "petals"
		if (checkShielded_reloaded(obstacle, render_agent)) {
			continue;
		}
		else {// if not shielded; add this obstacle to neighbour list
			// compute tangents
			tangents_handler(obstacle, render_agent);
			// refine boundaries to minimise overlap and get more accurate insights
			//visibility_handler(obstacle, render_agent);
		}


		//visualizer section :
		if (false) {
			neighbourSweep* neighbour = &neighbours.at(neighbours.size() - 1);
			for (int index = 0; index < neighbours.size(); index++) {
				if (obstacle->uniqueID == neighbours.at(index).pointer->uniqueID) {
					neighbour = &neighbours.at(index);
				}
			}

			render_agent->add_path(&neighbour->internal_tangent1, true);
			render_agent->add_path(&neighbour->internal_tangent2, true);
			render_agent->add_path(&neighbour->internal_range, true);
			render_agent->add_path(&neighbour->external_tangent1, true);
			render_agent->add_path(&neighbour->external_tangent2, true);
			render_agent->add_path(&neighbour->external_range, true);
			polygonEdge temp_edge = polygonEdge(neighbour->pointer->looseBounds.left, neighbour->pointer->looseBounds.top,
				neighbour->pointer->looseBounds.left, neighbour->pointer->looseBounds.bottom);
			render_agent->add_path(&temp_edge);
			temp_edge = polygonEdge(neighbour->pointer->looseBounds.left, neighbour->pointer->looseBounds.bottom,
				neighbour->pointer->looseBounds.right, neighbour->pointer->looseBounds.bottom);
			render_agent->add_path(&temp_edge);
			temp_edge = polygonEdge(neighbour->pointer->looseBounds.right, neighbour->pointer->looseBounds.bottom,
				neighbour->pointer->looseBounds.right, neighbour->pointer->looseBounds.top);
			render_agent->add_path(&temp_edge);
			temp_edge = polygonEdge(neighbour->pointer->looseBounds.right, neighbour->pointer->looseBounds.top,
				neighbour->pointer->looseBounds.left, neighbour->pointer->looseBounds.top);
			render_agent->add_path(&temp_edge);

			clear_count += 10;

			render_agent->invalidate();
			std::cin.ignore();

			render_agent->clear_paths(clear_count);
			clear_count = 0;
		}
	}
	visibility_handler_reloaded(render_agent);
}

void polygon2D::findPos(int index, int* supporting_points) {

	reconstruct_edges();
	float x3 = node_list->at(index).x;
	float y3 = node_list->at(index).y;
	int sign = 0;
	int temp_sign = 0;

	for (int i = 0; i < edges.size(); i++) {
		if (i == 0) {
			sign = edges.at(i).checkLeft(x3, y3);
			if (sign == 0) {
				if ((std::max)(std::abs(x3 - edges.at(i).x1), std::abs(x3 - edges.at(i).x2)) < std::abs(edges.at(i).x1 - edges.at(i).x2)) //(x3, y3) is inside point
					break;
			}
			continue;
		}

		temp_sign = edges.at(i).checkLeft(x3, y3);
		if (temp_sign == 0) {
			if ((std::max)(std::abs(x3 - edges.at(i).x1), std::abs(x3 - edges.at(i).x2)) < std::abs(edges.at(i).x1 - edges.at(i).x2)) //(x3, y3) is inside point
				break;
		}
		if (temp_sign != sign) {
			if (supporting_points[0] == 0)
				supporting_points[0] = i;
			else
				supporting_points[1] = i;
			sign = -sign;
		}

		if (i == edges.size() - 1) {
			temp_sign = edges.at(0).checkLeft(x3, y3);
			if (temp_sign == 0) {
				if ((std::max)(std::abs(x3 - edges.at(0).x1), std::abs(x3 - edges.at(0).x2)) < std::abs(edges.at(0).x1 - edges.at(0).x2)) //(x3, y3) is inside point
					break;
			}
			if (temp_sign != sign) {
				if (supporting_points[0] == 0)
					supporting_points[0] = 0;
				else
					supporting_points[1] = 0;
				sign = -sign;
			}
		}
	}

	return;
}

void polygon2D::tangents_handler(polygon2D* neighbour, renderAgent* render_agent) {
	std::vector<int>* set_1 = &vertices;
	std::vector<int>* set_2 = &neighbour->vertices;

	polygonEdge internal_tangent1, internal_tangent2, internal_range,
		external_tangent1, external_tangent2, external_range;

	int index = 0;
	for (int i = 0; i < neighbours.size(); i++) {
		if (neighbours.at(i).pointer->uniqueID == neighbour->uniqueID) {
			index = i;
			break;
		}
	}
	if (neighbours.at(index).internal_tangent1.isNull()) {
		find_tangent(&polygon2D::leftOn, &polygon2D::rightOn, set_1, set_2, &internal_tangent1);
		find_tangent(&polygon2D::rightOn, &polygon2D::leftOn, set_1, set_2, &internal_tangent2);
		find_tangent(&polygon2D::rightOn, &polygon2D::rightOn, set_1, set_2, &external_tangent1);
		find_tangent(&polygon2D::leftOn, &polygon2D::leftOn, set_1, set_2, &external_tangent2);

		internal_range = polygonEdge(internal_tangent1.x2, internal_tangent1.y2, internal_tangent2.x2, internal_tangent2.y2);
		external_range = polygonEdge(external_tangent1.x2, external_tangent1.y2, external_tangent2.x2, external_tangent2.y2);

		neighbours.at(index).update_tangents_info(internal_tangent1, internal_tangent2, internal_range,
			external_tangent1, external_tangent2, external_range);
		neighbours.at(index).visible = true;

		for (int i = 0; i < neighbour->neighbours.size(); i++) {
			if (neighbour->neighbours.at(i).pointer->uniqueID == uniqueID) {
				neighbour->neighbours.at(i).update_tangents_info(internal_tangent1.invert(), internal_tangent2.invert(),
					polygonEdge(internal_tangent1.x1, internal_tangent1.y1, internal_tangent2.x1, internal_tangent2.y1),
					external_tangent2.invert(), external_tangent1.invert(),
					polygonEdge(external_tangent2.x1, external_tangent2.y1, external_tangent1.x1, external_tangent1.y1));
			}
		}
	}
	else {
		external_tangent1 = neighbours.at(index).external_tangent1;
		external_tangent2 = neighbours.at(index).external_tangent2;
		external_range = neighbours.at(index).external_range;
		internal_tangent1 = neighbours.at(index).internal_tangent1;
		internal_tangent2 = neighbours.at(index).internal_tangent2;
		internal_range = neighbours.at(index).internal_range;

		neighbours.at(index).visibility_limit_left = external_tangent1;
		neighbours.at(index).visibility_limit_right = external_tangent2;
		neighbours.at(index).visibility_range = external_range;
		neighbours.at(index).visible = true;
	}
}

void polygon2D::visibility_handler(polygon2D* neighbour, renderAgent* render_agent) {
	int index = 0;
	for (int i = 0; i < neighbours.size(); i++) {
		if (neighbours.at(i).pointer->uniqueID == neighbour->uniqueID) {
			index = i;
			break;
		}
	}

	for (int i = 0; i < neighbours.size(); i++) {
		if (i == index) {
			continue;
		}

		polygon2D::neighbourSweep* latest = &neighbours.at(index);
		polygon2D::neighbourSweep* member = &neighbours.at(i);

		if (!member->visible)
			continue;

		bool flag_direct_neighbour = true;
		if (latest->visibility_limit_left.isIntersecting(&member->visibility_range)
			&& latest->visibility_limit_right.isIntersecting(&member->visibility_range)) { // this edge is completely blocked by ith range
			latest->visible = false;
			render_agent->visualising_helper1(latest->visibility_limit_left, latest->visibility_limit_right, latest->visibility_range,
				member->visibility_limit_left, member->visibility_limit_right, member->visibility_range);
			return;
		}

		if (member->visibility_limit_left.isIntersecting(&latest->visibility_range)
			&& member->visibility_limit_right.isIntersecting(&latest->visibility_range)) { // ith edge is completely blocked by this edge
			// mark ith neighbour invisible in the list
			member->visible = false;
			render_agent->visualising_helper1(latest->visibility_limit_left, latest->visibility_limit_right, latest->visibility_range,
				member->visibility_limit_left, member->visibility_limit_right, member->visibility_range);
			continue;
		}

		if (modify_visibility_zones(latest, member, false, &flag_direct_neighbour, render_agent)) {
			if (!flag_direct_neighbour)
				latest->direct_neighbour = false;
			continue;
		}
		if (modify_visibility_zones(latest, member, true, &flag_direct_neighbour, render_agent)) {
			if (!flag_direct_neighbour)
				latest->direct_neighbour = false;
			continue;
		}
		if (modify_visibility_zones(member, latest, false, &flag_direct_neighbour, render_agent)) {
			if (!flag_direct_neighbour)
				member->direct_neighbour = false;
			continue;
		}
		if (modify_visibility_zones(member, latest, true, &flag_direct_neighbour, render_agent)) {
			if (!flag_direct_neighbour)
				member->direct_neighbour = false;
			continue;
		}
	}
	if (neighbours.at(index).visibility_range.length() < 0.5f) {
		neighbours.at(index).visible = false;
		return;
	}
}

void polygon2D::visibility_handler_reloaded(renderAgent* render_agent) {

	for (int i = 0; i < neighbours.size(); i++) {

		polygon2D::neighbourSweep* petal1 = &neighbours.at(i);
		if (!petal1->visible)
			continue;

		for (int j = 0; j < neighbours.size(); j++) {

			if (i == j)
				continue;
			polygon2D::neighbourSweep* petal2 = &neighbours.at(j);
			if (!petal2->visible)
				continue;

			if (petal2->visibility_limit_left.isIntersecting(&petal1->visibility_range)
				&& petal2->visibility_limit_right.isIntersecting(&petal1->visibility_range)) { // ith edge is completely blocked by this edge
				// mark ith neighbour invisible in the list
				petal2->visible = false;
				std::cout << j << "th marked invisible\n";
				continue;
			}

			modify_visibility_zones_reloaded(petal2, petal1, false, render_agent);
			modify_visibility_zones_reloaded(petal2, petal1, true, render_agent);
		}
	}
}

bool polygon2D::leftOn(int x1, int y1, int x2, int y2, int x0, int y0) {
	return CompGeomFunc::leftOn(x1, y1, x2, y2, x0, y0);
}

bool polygon2D::rightOn(int x1, int y1, int x2, int y2, int x0, int y0) {
	return CompGeomFunc::rightOn(x1, y1, x2, y2, x0, y0);
}

bool polygon2D::checkShielded(polygon2D* neighbour, renderAgent* render_agent) {

	std::vector<polygonEdge> edges_check_list;
	edges_check_list.push_back(polygonEdge(
		neighbour->looseBounds.left,
		neighbour->looseBounds.top,
		neighbour->looseBounds.right,
		neighbour->looseBounds.bottom
	));
	edges_check_list.push_back(polygonEdge(
		neighbour->looseBounds.right,
		neighbour->looseBounds.top,
		neighbour->looseBounds.left,
		neighbour->looseBounds.bottom
	));

	int rejection_count = 0;
	for (int i = 0; i < neighbours.size(); i++) {// iterating over "petals" like bounds of the complete "flower" like polygon
		neighbourSweep* bounds = &neighbours.at(i);
		if (!bounds->visible) {
			rejection_count++;
			continue;
		}
		if (edges_check_list.empty())
			return true;
		for (int j = 0; j < edges_check_list.size(); j++) {// iterating over checklist
			polygonEdge* edge = &edges_check_list.at(j);

			bool prelim_test1 = bounds->external_range.leftOn(edge->x1, edge->y1);
			bool prelim_test2 = bounds->external_range.leftOn(edge->x2, edge->y2);
			bool prelim_test3 = bounds->external_range.leftOn((looseBounds.left + looseBounds.right) / 2,
				(looseBounds.top + looseBounds.bottom) / 2);

			if (prelim_test1 && prelim_test2 && !prelim_test3) {
				// if shielding object is actually between source and the concerned edge
				bool test1 = bounds->external_tangent1.leftOn(edge->x1, edge->y1);
				bool test2 = bounds->external_tangent2.leftOn(edge->x1, edge->y1);
				bool test3 = bounds->external_tangent1.leftOn(edge->x2, edge->y2);
				bool test4 = bounds->external_tangent2.leftOn(edge->x2, edge->y2);

				if (!test1 && test2 && !test3 && test4) { // both the ends are inside the region of interest
					edges_check_list.erase(edges_check_list.begin() + j);
					j--;
					continue;
				}
				if (!test1 && test2 && !(!test3 && test4)) { // end 1 is blocked by obstacle; end 2 is exposed
					if (test3 && test4) { // end 2 is to the left of angular limit
						// redefine end1 as intersection of edge and tangent 1
						point intersection = intersection_point(edge, &bounds->external_tangent1);
						edge->x1 = intersection.x;
						edge->y1 = intersection.y;
					}
					if (!test3 && !test4) { // end 2 is to the right of angular limit
						// redefine end1 as intersection of edge and tangent 2
						point intersection = intersection_point(edge, &bounds->external_tangent2);
						edge->x1 = intersection.x;
						edge->y1 = intersection.y;
					}
				}
				if (!(!test1 && test2) && !test3 && test4) { // end 2 is blocked by obstacle; end 1 is exposed
					if (test1 && test2) { // end 1 is to the left of angular limit
						// redefine end2 as intersection of edge and tangent 1
						point intersection = intersection_point(edge, &bounds->external_tangent1);
						edge->x2 = intersection.x;
						edge->y2 = intersection.y;
					}
					if (!test1 && !test2) { // end 1 is to the right of angular limit
						// redefine end2 as intersection of edge and tangent 2
						point intersection = intersection_point(edge, &bounds->external_tangent2);
						edge->x2 = intersection.x;
						edge->y2 = intersection.y;
					}

				}

			}
		}
	}

	if (edges_check_list.empty())
		return true;

	render_agent->show_edges(&edges_check_list);

	return false;
}

bool polygon2D::checkShielded_reloaded(polygon2D* neighbour, renderAgent* render_agent) {

	std::vector<polygonEdge> edges_check_list;
	edges_check_list.push_back(polygonEdge(
		neighbour->looseBounds.left,
		neighbour->looseBounds.top,
		neighbour->looseBounds.right,
		neighbour->looseBounds.bottom
	));
	edges_check_list.push_back(polygonEdge(
		neighbour->looseBounds.right,
		neighbour->looseBounds.top,
		neighbour->looseBounds.left,
		neighbour->looseBounds.bottom
	));

	for (int i = 0; i < neighbours.size(); i++) {// iterating over "petals" like bounds of the complete "flower" like polygon
		neighbourSweep* bounds = &neighbours.at(i);
		if (!bounds->visible) {
			continue;
		}

		if (edges_check_list.empty())
			return true;

		for (int j = 0; j < edges_check_list.size(); j++) {// iterating over checklist
			polygonEdge* edge = &edges_check_list.at(j);

			bool prelim_test1 = bounds->internal_range.leftOn(edge->x1, edge->y1);
			bool prelim_test2 = bounds->internal_range.leftOn(edge->x2, edge->y2);
			bool prelim_test3 = bounds->internal_range.leftOn((looseBounds.left + looseBounds.right) / 2.0,
				(looseBounds.top + looseBounds.bottom) / 2.0);

			if (prelim_test1 && prelim_test2 && !prelim_test3) {
				// if shielding object is actually between source and the concerned edge

				bool test1 = bounds->internal_tangent1.leftOn(edge->x1, edge->y1);
				bool test2 = bounds->internal_tangent1.leftOn(edge->x2, edge->y2);
				bool test3 = bounds->internal_tangent2.leftOn(edge->x1, edge->y1);
				bool test4 = bounds->internal_tangent2.leftOn(edge->x2, edge->y2);

				if (!test1 && test3) {// end 1 of edge is shielded
					if (!test2 && test4) {// end 2 of edge is shielded
						// complete edge is shielded, delete it
						edges_check_list.erase(edges_check_list.begin() + j);
						j--;
						continue;
					}
					else {// end2 of edge is exposed
						// partially shielded from end 1 so trim from end 1 side

						// end 1 is blocked by obstacle; end 2 is exposed
						if (test2 && test4) { // end 2 is to the left of angular limit
							// redefine end1 as intersection of edge and tangent 1
							point intersection = intersection_point(edge, &bounds->internal_tangent1);
							edge->x1 = intersection.x;
							edge->y1 = intersection.y;
							continue;
						}
						if (!test2 && !test4) { // end 2 is to the right of angular limit
							// redefine end1 as intersection of edge and tangent 2
							point intersection = intersection_point(edge, &bounds->internal_tangent2);
							edge->x1 = intersection.x;
							edge->y1 = intersection.y;
							continue;
						}
					}
				}
				else {// end 1 of edge is exposed
					if (!test2 && test4) {// end 2 of edge is shielded
						// partially shielded from end 2 so trim from end 2 side

						// end 2 is blocked by obstacle; end 1 is exposed
						if (test1 && test3) { // end 1 is to the left of angular limit
							// redefine end2 as intersection of edge and tangent 1
							point intersection = intersection_point(edge, &bounds->internal_tangent1);
							edge->x2 = intersection.x;
							edge->y2 = intersection.y;
						}
						if (!test1 && !test3) { // end 1 is to the right of angular limit
							// redefine end2 as intersection of edge and tangent 2
							point intersection = intersection_point(edge, &bounds->internal_tangent2);
							edge->x2 = intersection.x;
							edge->y2 = intersection.y;
						}
					}
					else {// end2 of edge is exposed
						// no shielding, continue with checks
						continue;
					}
				}
			}
		}

	}

	if (edges_check_list.empty())
		return true;

	render_agent->show_edges(&edges_check_list);

	return false;
}

point polygon2D::intersection_point(polygonEdge* slicee_edge, polygonEdge* slicer_edge) {
	return CompGeomFunc::intersection_point(slicee_edge, slicer_edge);
}

void polygon2D::find_tangent(
	bool (polygon2D::* discriminator1)(int, int, int, int, int, int),
	bool (polygon2D::* discriminator2)(int, int, int, int, int, int),
	std::vector<int>* set_1, std::vector<int>* set_2, polygonEdge* tangent) {

	int x1 = 0, y1 = 0, x2 = 0, y2 = 0, x0a = 0, y0a = 0, x0b = 0, y0b = 0;
	bool is_spotted = false;
	int counter = 0;

	for (int i = 0; i < set_1->size(); i++) {
		if (is_spotted)
			break;
		for (int j = 0; j < set_2->size(); j++) {
			x1 = set_1->at(i) % GRID_WIDTH;
			y1 = set_1->at(i) / GRID_WIDTH;
			x2 = set_2->at(j) % GRID_WIDTH;
			y2 = set_2->at(j) / GRID_WIDTH;

			for (int a = 0; a < set_1->size(); a++) {
				x0a = set_1->at(a) % GRID_WIDTH;
				y0a = set_1->at(a) / GRID_WIDTH;
				if (!is_spotted) {
					if (!(this->*discriminator1)(x1, y1, x2, y2, x0a, y0a))
						break;
				}
				for (int b = 0; b < set_2->size(); b++) {
					x0b = set_2->at(b) % GRID_WIDTH;
					y0b = set_2->at(b) / GRID_WIDTH;

					if (!is_spotted) {
						if (!(this->*discriminator2)(x1, y1, x2, y2, x0b, y0b))
							break;
						counter++;
					}
				}
			}
			if (!is_spotted) {
				if (counter == set_1->size() * set_2->size()) {
					*tangent = polygonEdge(x1, y1, x2, y2);
					is_spotted = true;
					break;
				}
				else
					counter = 0;
			}
		}
	}

	if (!is_spotted)
		*tangent = polygonEdge(x1, y1, x2, y2);
}

std::vector<int> polygon2D::create_priority_list(std::vector<polygon2D>* obstacles) {
	std::vector<int> priority_queue;

	float current_min = 100000.0f;
	int current_index = 0;
	float me_center_x = (looseBounds.left + looseBounds.right) / 2.0f;
	float me_center_y = (looseBounds.top + looseBounds.bottom) / 2.0f;
	bool find_val = false;

	for (auto& obstacle : *obstacles) {
		current_min = 100000.0f;
		for (int i = 0; i < obstacles->size(); i++) {
			for (auto& id : priority_queue) {
				if (id == i) {
					find_val = true;
					break;
				}
			}
			if (find_val) {
				find_val = false;
				continue;
			}

			float you_center_x = (obstacles->at(i).looseBounds.left + obstacles->at(i).looseBounds.right) / 2.0f;
			float you_center_y = (obstacles->at(i).looseBounds.top + obstacles->at(i).looseBounds.bottom) / 2.0f;
			float dist = sqrt(pow(me_center_x - you_center_x, 2.0) + pow(me_center_y - you_center_y, 2.0));
			if (dist < current_min) {
				current_min = dist;
				current_index = i;
			}
		}
		priority_queue.push_back(current_index);
	}

	return priority_queue;
}

bool polygon2D::modify_visibility_zones(polygon2D::neighbourSweep* to_be_blocked,
	polygon2D::neighbourSweep* blocker, bool right, bool* flag_direct_neighbour, renderAgent* render_agent) {
	polygonEdge* tangent_member1, * tangent_member2, * edgePointer;
	if (right) {
		// requested operation for right tangents
		tangent_member1 = &to_be_blocked->visibility_limit_right;
		tangent_member2 = &blocker->visibility_limit_left;
		edgePointer = to_be_blocked->temp_right_pointer;
	}
	else {
		// requested operation for left tangents
		tangent_member1 = &to_be_blocked->visibility_limit_left;
		tangent_member2 = &blocker->visibility_limit_right;
		edgePointer = to_be_blocked->temp_left_pointer;
	}

	if (blocker->visibility_range.isIntersecting(tangent_member1)) { // blockee's tangent is intersecting blocker's range
		if (tangent_member2->isExtendedIntersecting(&to_be_blocked->visibility_range)) {
			// blocker's extended tangent will intersect blockee's range; case of truncation
			point intersection = intersection_point(&to_be_blocked->visibility_range, tangent_member2);
			*tangent_member1 = *tangent_member2;
			tangent_member1->x2 = intersection.x;
			tangent_member1->y2 = intersection.y;
			to_be_blocked->evaluate_visibility_range();
			render_agent->visualising_helper1(blocker->visibility_limit_left, blocker->visibility_limit_right, blocker->visibility_range,
				to_be_blocked->visibility_limit_left, to_be_blocked->visibility_limit_right, to_be_blocked->visibility_range);
			*flag_direct_neighbour = false;
			return true;
		}
		else {
			// blocker's extended tangent is not intersecting blockee's range; case of "internal tangent handling"
			for (int k = 0; k < blocker->pointer->neighbours.size(); k++) {
				if (blocker->pointer->neighbours.at(k).pointer->uniqueID == to_be_blocked->pointer->uniqueID) {
					if (blocker->pointer->neighbours.at(k).internal_tangent1.isNull()) {
						// find tangents between member2 and member1
						blocker->pointer->tangents_handler(to_be_blocked->pointer, render_agent);
					}
					// ensured tangents are initialized now!

					if (right) {
						*tangent_member1 = blocker->pointer->neighbours.at(k).internal_tangent2;
					}
					else {
						*tangent_member1 = blocker->pointer->neighbours.at(k).internal_tangent1;
					}
					render_agent->visualising_helper1(blocker->visibility_limit_left, blocker->visibility_limit_right, blocker->visibility_range,
						to_be_blocked->visibility_limit_left, to_be_blocked->visibility_limit_right, to_be_blocked->visibility_range);
					*flag_direct_neighbour = false;
					return true;
				}
			}
		}
	}

	// blockee's tangent does not intersects blocker's range
	render_agent->visualising_helper1(blocker->visibility_limit_left, blocker->visibility_limit_right, blocker->visibility_range,
		to_be_blocked->visibility_limit_left, to_be_blocked->visibility_limit_right, to_be_blocked->visibility_range);
	return false;
}

bool polygon2D::modify_visibility_zones_reloaded(polygon2D::neighbourSweep* to_be_blocked,
	polygon2D::neighbourSweep* blocker, bool right, renderAgent* render_agent) {
	polygonEdge* tangent_member_to_be_blocked, * tangent_member_blocker, * new_limit, * the_other_tangent;
	if (right) {
		// requested operation for right tangents
		tangent_member_to_be_blocked = &to_be_blocked->visibility_limit_right;
		tangent_member_blocker = &blocker->visibility_limit_left;
		the_other_tangent = &to_be_blocked->visibility_limit_left;
	}
	else {
		// requested operation for left tangents
		tangent_member_to_be_blocked = &to_be_blocked->visibility_limit_left;
		tangent_member_blocker = &blocker->visibility_limit_right;
		the_other_tangent = &to_be_blocked->visibility_limit_right;
	}
	new_limit = nullptr;
	for (int i = 0; i < neighbours.size(); i++) {
		if (neighbours.at(i).pointer->uniqueID == blocker->pointer->uniqueID) {
			if (right) {
				new_limit = &neighbours.at(i).internal_tangent1;
				break;
			}
			else {
				new_limit = &neighbours.at(i).internal_tangent2;
				break;
			}
		}
	}

	if (new_limit->isStrictlyVectorExtendedIntersecting(&to_be_blocked->visibility_range)) {
		// blocker's extended tangent will intersect blockee's range; case of truncation
		point intersection = intersection_point(&to_be_blocked->visibility_range, new_limit);
		*tangent_member_to_be_blocked = *tangent_member_blocker;
		tangent_member_to_be_blocked->x1 = new_limit->x1;
		tangent_member_to_be_blocked->y1 = new_limit->y1;
		tangent_member_to_be_blocked->x2 = intersection.x;
		tangent_member_to_be_blocked->y2 = intersection.y;
		to_be_blocked->evaluate_visibility_range();
		if (to_be_blocked->visibility_range.length() < 0.0)
			to_be_blocked->visible = false;
		return true;
	}

	if (new_limit->isStrictlyVectorExtendedIntersecting(the_other_tangent)) {
		std::cout << "USING VETO :P\n";
		std::cout << "You should keep a check on veto...\n";
		std::cout << "new_limit : (" << new_limit->x1 << ", " << new_limit->y1 << ") ; (" << new_limit->x2 << ", " << new_limit->y2 << ") " << std::endl;
		std::cout << "the_other_tangent : (" << the_other_tangent->x1 << ", " << the_other_tangent->y1 << ") ; (" << the_other_tangent->x2 << ", " << the_other_tangent->y2 << ") " << std::endl;
		to_be_blocked->visible = false;
	}

	return false;
}

void voronoi_algo::init(std::vector<Node>* node_list, int width, int height, RenderClass* renderer, 
	std::vector<Path>* paths, void (*func_updater)(std::vector<float>*)) {
	this->node_list = node_list;
	this->GRID_WIDTH = width;
	this->GRID_HEIGHT = height;
	this->render_agent = renderAgent(GRID_WIDTH, renderer, paths, func_updater);

	locate_obstacles();

	// create empty neighbour map spaces
	for (int j = 0; j < obstacles.size(); j++) {
		for (int i = 0; i < obstacles.size(); i++) {
			if (obstacles.at(j).uniqueID == obstacles.at(i).uniqueID) {
				continue;
			}
			obstacles.at(j).neighbours.push_back(polygon2D::neighbourSweep(&obstacles.at(i), j+1));
		}
	}

	// initialise neighbour details
	std::vector<float> vertices;
	for (auto& obstacle : obstacles) {
		obstacle.create_neighbour_map(&obstacles, &render_agent);

		render_agent.visualizing_helper3(&obstacle, &vertices);
		//std::cin.ignore();
	}

	for (int j = 0; j < obstacles.size(); j++) {
		std::cout << "[SIZE] : " << obstacles.at(j).neighbours.size() << std::endl;
	}

	// importance factor
	for (int j = 0; j < obstacles.size(); j++) {
		for (int i = 0; i < obstacles.at(j).neighbours.size(); i++) {
			if (obstacles.at(j).neighbours.at(i).visible) {
				std::cout << "[VISIBLE] ";
				obstacles.at(j).neighbours.at(i).compute_importance();
			}
			else {
				std::cout << "[HIDDEN]" << std::endl;
			}
		}
		std::cout << " -------------------------------------------------------------- \n";
	}

	// sort quads by rank
	polygon2D::neighbourSweep* best = nullptr;
	int rank = 1;
	while (true) {
		best = nullptr;
		for (int j = 0; j < obstacles.size(); j++) {
			for (int i = 0; i < obstacles.at(j).neighbours.size(); i++) {
				if (obstacles.at(j).neighbours.at(i).visible) {
					if (obstacles.at(j).neighbours.at(i).rank == 0) {
						if (best != nullptr) {
							if (obstacles.at(j).neighbours.at(i).importance > best->importance)
								best = &obstacles.at(j).neighbours.at(i);
						}
						else {
							best = &obstacles.at(j).neighbours.at(i);
						}
					}
				}
			}
		}
		if (best == nullptr) {
			std::cout << " BREAKING THE LOOP....\n";
			break;
		}
		render_agent.visualising_helper1(best->visibility_limit_left, best->visibility_limit_right, best->visibility_range,
			best->visibility_limit_left, best->visibility_limit_right, best->visibility_range);
		best->rank = rank;
		std::cout << "Rank : " << best->rank << " , Importance : " << best->importance << std::endl;
		best = nullptr;
		rank++;
	}

	freeCellsGraph* stichedGraph = nullptr;
	int graph_size = 0;

	//create area-map from neighbour map and details
	build_quad_graph(&stichedGraph, &(this->good_quads));

	this->render_agent.clear_paths(6 * 44);
	this->render_agent.invalidate();

	std::vector<freeCellsGraph*> tally_list;
	//stichedGraph->traverser1(&tally_list, &(this->render_agent));

	for (int i = 1; i <= this->good_quads.size();i++) {
		for (int j = 0; j < (this->good_quads)[i]->diagraph_sibs.size(); j++) {
			if ((this->good_quads)[i]->diagraph_sibs.at(j)->identifier == (this->good_quads)[i]->identifier)
				std::cout << "Duplicate found : " << (this->good_quads)[i]->identifier << std::endl;
		}
	}

	for (int i = 1; i <= this->good_quads.size();i++) {
		(this->good_quads)[i]->find_center();
	}
}

void voronoi_algo::finder(int start_cell_index, int goal_cell_index) {
	this->start_cell_index = start_cell_index;
	this->goal_cell_index = goal_cell_index;

	int start_x = this->start_cell_index % GRID_WIDTH;
	int start_y = this->start_cell_index / GRID_WIDTH;
	int goal_x = this->goal_cell_index % GRID_WIDTH;
	int goal_y = this->goal_cell_index / GRID_WIDTH;

	freeCellsGraph* start_quad = nullptr, * goal_quad = nullptr;

	for (int i = 1;i <= this->good_quads.size(); i++) {
		if ((start_quad != nullptr) && (goal_quad != nullptr))
			break;
		if ((this->good_quads)[i]->surrounds(start_x, start_y))
			start_quad = (this->good_quads)[i];
		if ((this->good_quads)[i]->surrounds(goal_x, goal_y))
			goal_quad = (this->good_quads)[i];
	}

	bool flag_start = (start_quad == nullptr);
	bool flag_end = (goal_quad == nullptr);
	if (flag_start || flag_end) {
		float dist_start = 100000000000.0f, dist_goal = 100000000000.0f;
		float dist_temp;
		for (int i = 1; i <= this->good_quads.size(); i++) {
			if (flag_start) {
				dist_temp = (this->good_quads)[i]->how_far_is(start_x, start_y);
				if (dist_temp < dist_start) {
					dist_start = dist_temp;
					start_quad = (this->good_quads)[i];
					std::cout << start_quad->identifier << " " << dist_start << std::endl;
				}
			}
			if (flag_end) {
				dist_temp = (this->good_quads)[i]->how_far_is(goal_x, goal_y);
				if (dist_temp < dist_goal) {
					dist_goal = dist_temp;
					goal_quad = (this->good_quads)[i];
					std::cout << goal_quad->identifier << " " << dist_goal << std::endl;
				}
			}
		}
	}

	std::cout << "[SUCCESS] : Start and goal quads identified\n";
	std::cout << "Start quad : pointer = " << start_quad << " identifier = " << start_quad->identifier << std::endl;
	std::cout << "Goal quad : pointer = " << goal_quad << " identifier = " << goal_quad->identifier << std::endl;

	for (int i = 1; i <= good_quads.size(); i++) {
		(this->good_quads)[i]->potential_path = nullptr;
	}

	start_quad->search_path(goal_quad);

	this->render_agent.visualizing_helper6(start_quad, goal_quad, start_x, start_y, goal_x, goal_y);
	return;
}

void voronoi_algo::checkInside(int index) {
	int x = index % GRID_WIDTH;
	int y = index / GRID_WIDTH;
	for (auto& obstacle : obstacles) {
		if (obstacle.isInside(x, y)) {
			std::cout << "success; \n";
			for (auto& vertice : obstacle.vertices) {
				node_list->at(vertice).check();
			}
			//std::cin.ignore();
			for (auto& vertice : obstacle.vertices) {
				node_list->at(vertice).check();
			}
		}
	}
}

void voronoi_algo::merge_strips(int row, int index2, polygon2D* obstacle, std::vector<std::vector<coord>>* strips_list) {
	obstacle->vertices.push_back(strips_list->at(row).at(index2).a);
	if (obstacle->looseBounds.bottom < row)
		obstacle->looseBounds.bottom = row;
	if (obstacle->looseBounds.left > strips_list->at(row).at(index2).a % GRID_WIDTH)
		obstacle->looseBounds.left = strips_list->at(row).at(index2).a % GRID_WIDTH;

	if (row + 1 < GRID_HEIGHT) {
		if (!strips_list->at(row + 1).empty()) {
			for (int i = 0; i < strips_list->at(row + 1).size(); i++) {
				if ((strips_list->at(row + 1).at(i).a % GRID_WIDTH <= strips_list->at(row).at(index2).b % GRID_WIDTH)
					&& (strips_list->at(row + 1).at(i).b % GRID_WIDTH >= strips_list->at(row).at(index2).a % GRID_WIDTH)) {
					merge_strips(row + 1, i, obstacle, strips_list);
					break;
				}
			}
		}
	}

	obstacle->vertices.push_back(strips_list->at(row).at(index2).b);
	if (obstacle->looseBounds.top > row)
		obstacle->looseBounds.top = row;
	if (obstacle->looseBounds.right < strips_list->at(row).at(index2).b % GRID_WIDTH)
		obstacle->looseBounds.right = strips_list->at(row).at(index2).b % GRID_WIDTH;
	strips_list->at(row).erase(strips_list->at(row).begin() + index2);
}

void voronoi_algo::locate_obstacles(void) {
	bool* checklist = (bool*)malloc(node_list->size() * sizeof(bool));

	std::vector<std::vector<coord>> strips(GRID_HEIGHT);

	int change_over_point = 0;
	bool obstacle_flag = false;
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			int index = i * GRID_WIDTH + j;
			if (node_list->at(index).type == BASE_TAKEN) {
				if (!obstacle_flag) {
					change_over_point = index;
					obstacle_flag = true;
				}
			}
			else {
				if (obstacle_flag) {
					strips[i].push_back(coord(change_over_point, index - 1));
					change_over_point = index;
					obstacle_flag = false;
				}
			}
		}
		if (node_list->at(change_over_point).type == BASE_TAKEN)
			strips[i].push_back(coord(change_over_point, (i + 1) * GRID_WIDTH - 1));
		change_over_point = 0;
		obstacle_flag = false;
	}
	//std::cin.ignore();

	render_agent.clear_paths();
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < strips[i].size(); j++) {
			render_agent.add_path(Path(strips[i].at(j).a, strips[i].at(j).b));
		}
		std::cout << std::endl;
	}
	render_agent.invalidate();

	polygon2D obstacle = polygon2D(node_list, GRID_WIDTH);
	for (int i = 0; i < strips.size(); i++) {
		int count = 0;
		while (!strips[i].empty()) {
			obstacle.looseBounds = rectangularBound(strips.at(i).at(0).a % GRID_WIDTH, strips.at(i).at(0).b % GRID_WIDTH, i, i);
			merge_strips(i, 0, &obstacle, &strips);
			if (obstacle.vertices.size() < 3)
				continue;
			obstacles.push_back(obstacle);
			obstacles.at(obstacles.size() - 1).uniqueID = obstacles.size();
			obstacle = polygon2D(node_list, GRID_WIDTH);
		}
	}

	std::cout << "Obstacle count = " << obstacles.size() << std::endl;
	//std::cin.ignore();
	render_agent.clear_paths();
	std::cout << "obstacle count : " << obstacles.size() << std::endl;
	for (auto& obstacle : obstacles) {
		obstacle.reconstruct_edges();
		std::cout << "obstacle edge count : " << obstacle.vertices.size() << std::endl;
		for (int i = 0; i < obstacle.vertices.size(); i++) {
			if (i == obstacle.vertices.size() - 1)
				render_agent.add_path(Path(obstacle.vertices.at(i), obstacle.vertices.at(0)));
			else
				render_agent.add_path(Path(obstacle.vertices.at(i), obstacle.vertices.at(i + 1)));
		}
	}
	render_agent.invalidate();
}

void voronoi_algo::build_quad_graph(freeCellsGraph** graphPointer, std::map<int, freeCellsGraph*>* priority_quads) {
	freeCellsGraph* quad = nullptr;
	for (int i = 0; i < obstacles.size(); i++) {
		std::cout << "Starting iteration for obstacle with ID : " << obstacles.at(i).uniqueID << std::endl;
		for (int j = 0; j < obstacles.at(i).neighbours.size(); j++) {
			polygon2D::neighbourSweep* sweep = &obstacles.at(i).neighbours.at(j);
			//std::cout << "   -- Checking for neighbour with ID -> " << sweep->pointer->uniqueID << std::endl;
			if (!sweep->visible) {
				obstacles.at(i).neighbours.erase(obstacles.at(i).neighbours.begin() + j);
				j--;
				continue;
			}

			std::cout << " Quad id -> " << sweep->id << std::endl;
			freeCellsGraph* temp_cell = new freeCellsGraph(sweep);

			if (quad == nullptr) {
				quad = new freeCellsGraph(sweep);
				*graphPointer = quad;

				(*priority_quads)[sweep->rank] = quad;
				obstacles.at(i).neighbours.erase(obstacles.at(i).neighbours.begin() + j);
				j--;
				continue;
			}
			else {
				(*priority_quads)[sweep->rank] = temp_cell;
				std::vector<freeCellsGraph*> tally_list;
				quad->add_member_new(temp_cell, &tally_list, &(this->render_agent));
				continue;
			}
			delete temp_cell;
		}
		std::cin.ignore();
	}
}

renderAgent::renderAgent(int GRID_WIDTH, RenderClass* renderer, std::vector<Path>* paths, 
	void (*func_updater)(std::vector<float>*)){
	this->GRID_WIDTH = GRID_WIDTH;
	this->paths = paths; // for visualization
	this->renderer = renderer;
	this->render_callback = func_updater;
}

void renderAgent::clear_paths(void) {
	paths->clear();
	render_callback(nullptr);
	renderer->render();
}

void renderAgent::clear_paths(int count) {
	for (int i = 0; i < count; i++)
		paths->pop_back();
}

void renderAgent::add_path(Path path) {
	paths->push_back(path);
}

void renderAgent::add_path(polygonEdge* edge) {
	int index1 = edge->y1 * GRID_WIDTH + edge->x1;
	int index2 = edge->y2 * GRID_WIDTH + edge->x2;
	add_path(Path(index1, index2));
}

void renderAgent::add_path(polygonEdge* edge, bool floating_point) {
	if (floating_point) {
		paths->push_back(Path(edge->x1, edge->y1, edge->x2, edge->y2));
	}
	else {
		int index1 = edge->y1 * GRID_WIDTH + edge->x1;
		int index2 = edge->y2 * GRID_WIDTH + edge->x2;
		add_path(Path(index1, index2));
	}
}

void renderAgent::add_path(polygonEdge* edge, bool floating_point, Color color) {
	if (floating_point) {
		paths->push_back(Path(edge->x1, edge->y1, edge->x2, edge->y2, color));
	}
	else {
		int index1 = edge->y1 * GRID_WIDTH + edge->x1;
		int index2 = edge->y2 * GRID_WIDTH + edge->x2;
		add_path(Path(index1, index2, color));
	}
}

void renderAgent::show_edges(std::vector<polygonEdge>* edge_list) {
	if (false) {
		Color color = Color(1.0f, 0.15f, 0.15f);

		for (auto& edge : *edge_list) {
			paths->push_back(Path(edge.x1, edge.y1, edge.x2, edge.y2, color));
		}
		invalidate();

		std::cin.ignore();
		for (auto& edge : *edge_list) {
			paths->pop_back();
		}
		invalidate();
	}
}

void renderAgent::visualising_helper1(polygonEdge edge1, polygonEdge edge2, polygonEdge edge3,
		polygonEdge edge4, polygonEdge edge5, polygonEdge edge6) {
	if (true) {
		Color color = Color(1.0f, 1.0f, 1.0f);

		if (false) {
			std::cout << "[EDGE1] : (" << edge1.x1 << ", " << edge1.y1 << ") -> (" << edge1.x2 << ", " << edge1.y2 << ")\n";
			std::cout << "[EDGE2] : (" << edge2.x1 << ", " << edge2.y1 << ") -> (" << edge2.x2 << ", " << edge2.y2 << ")\n";
			std::cout << "[EDGE3] : (" << edge3.x1 << ", " << edge3.y1 << ") -> (" << edge3.x2 << ", " << edge3.y2 << ")\n";
			std::cout << "[EDGE4] : (" << edge4.x1 << ", " << edge4.y1 << ") -> (" << edge4.x2 << ", " << edge4.y2 << ")\n";
			std::cout << "[EDGE5] : (" << edge5.x1 << ", " << edge5.y1 << ") -> (" << edge5.x2 << ", " << edge5.y2 << ")\n";
			std::cout << "[EDGE6] : (" << edge6.x1 << ", " << edge6.y1 << ") -> (" << edge6.x2 << ", " << edge6.y2 << ")\n";
		}

		add_path(&edge1, true, color);
		add_path(&edge2, true, color);
		add_path(&edge3, true, color);
		color = Color(0.0f, 0.0f, 0.0f);
		add_path(&edge4, true, color);
		add_path(&edge5, true, color);
		add_path(&edge6, true, color);
		invalidate();

		std::cin.ignore();
		/*paths->pop_back();
		paths->pop_back();
		paths->pop_back();
		paths->pop_back();
		paths->pop_back();
		paths->pop_back();
		invalidate();*/
	}
}

void coordsResolver(polygonEdge tangent1, polygonEdge tangent2, std::vector<float>* vertices) {
	float k = +20.0f;
	float x1 = k * float(tangent1.x2 - tangent1.x1) + tangent1.x1;
	float y1 = k * float(tangent1.y2 - tangent1.y1) + tangent1.y1;
	float x2 = k * float(tangent2.x2 - tangent2.x1) + tangent2.x1;
	float y2 = k * float(tangent2.y2 - tangent2.y1) + tangent2.y1;

	bool test1 = tangent1.leftOn(tangent2.x2, tangent2.y2);
	bool test2 = tangent1.leftOn(x2, y2);
	bool test3 = tangent2.leftOn(tangent1.x2, tangent1.y2);
	bool test4 = tangent2.leftOn(x1, y1);

	/*if (!((test1 == test2) && (test3 == test4))) {
		// replacing x1, y1 and x2, y2 with the intersection point
		point intersection = CompGeomFunc::intersection_point(&tangent1, &tangent2);
		x1 = intersection.x;
		y1 = intersection.y;
		x2 = intersection.x;
		y2 = intersection.y;
	}*/

	vertices->push_back(tangent2.x2);
	vertices->push_back(tangent2.y2);
	vertices->push_back(x2);
	vertices->push_back(y2);
	vertices->push_back(x1);
	vertices->push_back(y1);
	vertices->push_back(tangent1.x2);
	vertices->push_back(tangent1.y2);
}

void renderAgent::visualizing_helper2(polygon2D* polygonal_obstacle) {
	if (false) {
		std::vector<float> vertices;

		for (int i = 0; i < polygonal_obstacle->neighbours.size(); i++) {
			if (polygonal_obstacle->neighbours.at(i).visible) {
				//coordsResolver(polygonal_obstacle->neighbours.at(i).visibility_limit_left,
					//polygonal_obstacle->neighbours.at(i).visibility_limit_right, &vertices);
				coordsResolver(polygonal_obstacle->neighbours.at(i).internal_tangent2,
					polygonal_obstacle->neighbours.at(i).internal_tangent1, &vertices);
			}
		}

		render_callback(&vertices);
		renderer->render();

		std::cin.ignore();
	}
}

void renderAgent::visualizing_helper3(polygon2D* polygonal_obstacle, std::vector<float>* vertices) {
	if (true) {
		int clear_count = 0;

		for (int i = 0; i < polygonal_obstacle->neighbours.size(); i++) {
			if (!(polygonal_obstacle->neighbours.at(i).visible && polygonal_obstacle->neighbours.at(i).visible))
				continue;

			add_path(&polygonal_obstacle->neighbours.at(i).visibility_limit_left, true);
			add_path(&polygonal_obstacle->neighbours.at(i).visibility_limit_right, true);
			add_path(&polygonal_obstacle->neighbours.at(i).visibility_range, true);
			/*add_path(&polygonal_obstacle->neighbours.at(i).external_tangent1, true);
			add_path(&polygonal_obstacle->neighbours.at(i).external_tangent2, true);
			add_path(&polygonal_obstacle->neighbours.at(i).external_range, true);*/

			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_right.x1);
			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_right.y1);
			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_right.x2);
			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_right.y2);
			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_left.x2);
			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_left.y2);
			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_left.x1);
			vertices->push_back(polygonal_obstacle->neighbours.at(i).visibility_limit_left.y1);

			//clear_count += 6;
			clear_count += 3;

			//render_callback(vertices);
			//renderer->render();
			//std::cin.ignore();
			//Sleep(200);
		}
		//render_callback(vertices);
		render_callback(nullptr);
		renderer->render();
		std::cin.ignore();

		visualizing_helper2(polygonal_obstacle);

		clear_paths(clear_count);
		clear_count = 0;
	}
}

void renderAgent::visualizing_helper4(freeCellsGraph *freeCell) {
	if (true) {

		Color color = Color(1.0f, 1.0f, 1.0f);
		add_path(&freeCell->bound_left, true, color);
		add_path(&freeCell->bound_right, true, color);
		add_path(&freeCell->bound_range, true, color);
		invalidate();
		std::cin.ignore();

		color = Color(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < freeCell->diagraph_sibs.size(); i++) {
			add_path(&freeCell->diagraph_sibs.at(i)->bound_left, true, color);
			add_path(&freeCell->diagraph_sibs.at(i)->bound_right, true, color);
			add_path(&freeCell->diagraph_sibs.at(i)->bound_range, true, color);
			invalidate();
			std::cout << i << " ";
			std::cin.ignore();
		}
		std::cout << std::endl;

		color = Color(1.0f, 1.0f, 1.0f);
		add_path(&freeCell->bound_left, true, color);
		add_path(&freeCell->bound_right, true, color);
		add_path(&freeCell->bound_range, true, color);
		invalidate();

		std::cin.ignore();
		clear_paths(3*freeCell->diagraph_sibs.size() + 6);
		invalidate();
		std::cin.ignore();
	}
}

void renderAgent::visualising_helper5(polygonEdge edge1, polygonEdge edge2, polygonEdge edge3,
	polygonEdge edge4, polygonEdge edge5, polygonEdge edge6) {
	if (true) {
		Color color = Color(1.0f, 1.0f, 1.0f);

		add_path(&edge1, true, color);
		add_path(&edge2, true, color);
		add_path(&edge3, true, color);
		polygonEdge edge = polygonEdge(edge1.x1, edge1.y1, edge3.x1, edge3.y1);
		add_path(&edge, true, color);


		color = Color(0.0f, 0.0f, 0.0f);
		add_path(&edge4, true, color);
		add_path(&edge5, true, color);
		add_path(&edge6, true, color);
		edge = polygonEdge(edge4.x1, edge4.y1, edge6.x1, edge6.y1);
		add_path(&edge, true, color);

		invalidate();

		std::cin.ignore();
		clear_paths(8);
		invalidate();
	}
}

void renderAgent::visualizing_helper6(freeCellsGraph* start, freeCellsGraph* goal,
	int start_x, int start_y, int goal_x, int goal_y) {
	std::cout << "called : start->"<<start->identifier<<" goal->"<<goal->identifier << "\n";
	freeCellsGraph* temp = goal;
	polygonEdge edge;
	Color color = Color(1.0f, 1.0f, 1.0f);
	Color quad_color = Color(0.2f, 0.2f, 0.2f);
	edge = polygonEdge(goal_x, goal_y, temp->center.x, temp->center.y);
	add_path(&edge, true, color);
	while (temp != start) {
		edge = polygonEdge(temp->center.x, temp->center.y,
			temp->last_visited_cell->center.x, temp->last_visited_cell->center.y);
		add_path(&edge, true, color);
		add_path(&temp->bound_left, true, quad_color);
		add_path(&temp->bound_right, true, quad_color);
		add_path(&temp->bound_range, true, quad_color);
		edge = polygonEdge(temp->bound_left.x1, temp->bound_left.y1,
			temp->bound_right.x1, temp->bound_right.y1);
		add_path(&edge, true, quad_color);
		temp = temp->last_visited_cell;
	}
	add_path(&temp->bound_left, true, quad_color);
	add_path(&temp->bound_right, true, quad_color);
	add_path(&temp->bound_range, true, quad_color);
	edge = polygonEdge(temp->bound_left.x1, temp->bound_left.y1,
		temp->bound_right.x1, temp->bound_right.y1);
	add_path(&edge, true, quad_color);
	std::cout << "start found \n";
	edge = polygonEdge(temp->center.x, temp->center.y, start_x, start_y);
	add_path(&edge, true, color);
	invalidate();
}

void renderAgent::invalidate(void) {
	render_callback(nullptr);
	renderer->render();
}

freeCellsGraph::freeCellsGraph(polygon2D::neighbourSweep* sweep) {
	bound_left = sweep->visibility_limit_left;
	bound_right = sweep->visibility_limit_right;
	evaluate_bounding_range();

	this->identifier = sweep->id;
	this->importance = sweep->importance;
	this->rank = sweep->rank;
}

void freeCellsGraph::evaluate_bounding_range(void) {
	if (bound_range.isNull()) {
		bound_range = polygonEdge(bound_left.x2, bound_left.y2,	bound_right.x2, bound_right.y2);
	}
	else {
		bound_range.x1 = bound_left.x2;
		bound_range.y1 = bound_left.y2;
		bound_range.x2 = bound_right.x2;
		bound_range.y2 = bound_right.y2;
	}
}

bool* freeCellsGraph::leftOn(float x, float y) {
	bool tests[4];
	tests[0] = CompGeomFunc::leftOn(bound_left.x1, bound_left.y1, bound_right.x1, bound_right.y1, x, y);
	tests[1] = CompGeomFunc::leftOn(bound_right.x1, bound_right.y1, bound_right.x2, bound_right.y2, x, y);
	tests[2] = CompGeomFunc::leftOn(bound_right.x2, bound_right.y2, bound_left.x2, bound_left.y2, x, y);
	tests[3] = CompGeomFunc::leftOn(bound_left.x2, bound_left.y2, bound_left.x1, bound_left.y1, x, y);
	return new bool[4] {tests[0], tests[1], tests[2], tests[3]};
}

bool freeCellsGraph::surrounds(int x, int y) {
	float x1 = this->bound_left.x1;
	float y1 = this->bound_left.y1;
	float x2 = this->bound_right.x1;
	float y2 = this->bound_right.y1;
	float x3 = this->bound_right.x2;
	float y3 = this->bound_right.y2;
	float x4 = this->bound_left.x2;
	float y4 = this->bound_left.y2;

	bool flow = polygonEdge(x1, y1, x2, y2).leftOn(x, y);
	if (polygonEdge(x2, y2, x3, y3).leftOn(x, y) != flow)
		return false;
	if (polygonEdge(x3, y3, x4, y4).leftOn(x, y) != flow)
		return false;
	if (polygonEdge(x4, y4, x1, y1).leftOn(x, y) != flow)
		return false;
	return true;
}

float freeCellsGraph::how_far_is(int x, int y) {
	float dist1 = this->bound_left.min_distance_from_line_segment(x, y);
	float dist2 = this->bound_right.min_distance_from_line_segment(x, y);
	return (((dist1) < (dist2)) ? (dist1) : (dist2));
}

void freeCellsGraph::add_member(freeCellsGraph* cell, std::vector<freeCellsGraph*>* tally, renderAgent* render_agent) {
	if (cell == nullptr)
		return;
	if (this == nullptr)
		return;
	if (this == cell)
		return;

	for (int i = 0; i < tally->size(); i++) {
		if (tally->at(i) == cell)
			return;
	}
	tally->push_back(cell);


	if (checkIntersecting(cell, render_agent)) {
		for (int i = 0; i < diagraph_sibs_forward.size(); i++) {
			diagraph_sibs_forward.at(i)->add_member(cell, tally, render_agent);
		}

		this->diagraph_sibs_forward.push_back(cell);
		cell->diagraph_sibs_backward.push_back(this);
	}

	return;
}

void freeCellsGraph::add_member_new(freeCellsGraph* cell, std::vector<freeCellsGraph*>* tally, renderAgent* render_agent) {
	if (cell == nullptr)
		return;
	if (this == nullptr)
		return;
	if (this == cell)
		return;

	for (int i = 0; i < tally->size(); i++) {
		if (tally->at(i) == this)
			return;
	}
	tally->push_back(this);


	if (checkIntersecting(cell, render_agent)) {
		this->diagraph_sibs.push_back(cell);
		cell->diagraph_sibs.push_back(this);
	}

	for (int i = 0; i < this->diagraph_sibs.size(); i++) {
		this->diagraph_sibs.at(i)->add_member_new(cell, tally, render_agent);
	}

	return;
}

bool freeCellsGraph::checkIntersecting(freeCellsGraph *cell, renderAgent* render_agent) {
	
	//std::cout << "-----------------------------------------------------------------------------\n";
	
	bool* checks[8];

	checks[0] = leftOn(cell->bound_left.x1, cell->bound_left.y1);
	checks[1] = leftOn(cell->bound_right.x1, cell->bound_right.y1);
	checks[2] = leftOn(cell->bound_right.x2, cell->bound_right.y2);
	checks[3] = leftOn(cell->bound_left.x2, cell->bound_left.y2);
	checks[4] = cell->leftOn(bound_left.x1, bound_left.y1);
	checks[5] = cell->leftOn(bound_right.x1, bound_right.y1);
	checks[6] = cell->leftOn(bound_right.x2, bound_right.y2);
	checks[7] = cell->leftOn(bound_left.x2, bound_left.y2);

	//render_agent->visualising_helper5(bound_left, bound_range, bound_right,
		//cell->bound_left, cell->bound_range, cell->bound_right);

	for (int i = 0; i < 4; i++) {
		if (*(checks[i]) && *(checks[i] + 1) && *(checks[i] + 2) && *(checks[i] + 3)) {
			//std::cout << "INTERSECTION RESULT: intersecting\n";
			for (int i = 0; i < 8; i++)
				delete checks[i];;
			return true;
		}
	}


	for (int i = 4; i < 8; i++) {
		if (*(checks[i]) && *(checks[i] + 1) && *(checks[i] + 2) && *(checks[i] + 3)) {
			//std::cout << "INTERSECTION RESULT: intersecting\n";
			for (int i = 0; i < 8; i++)
				delete checks[i];;
			return true;
		}
	}

	for (int i = 0; i < 4; i++) {
		int i_end1 = i, i_end2 = i + 1;
		if (i_end2 == 4)
			i_end2 = 0;
		for (int j = 0; j < 4; j++) {
			int j_end1 = j, j_end2 = j + 1;
			if (j_end2 == 4)
				j_end2 = 0;
			if ((checks[i_end1][j_end2] != checks[i_end1][j_end2]) && (checks[j_end1][i_end1] != checks[j_end1][i_end2])) {
				//std::cout << "INTERSECTION RESULT: intersecting\n";
				for (int i = 0; i < 8; i++)
					delete checks[i];;
				return true;
			}
		}
	}

	//std::cout << "INTERSECTION RESULT: not-intersecting\n";
	for (int i = 0; i < 8; i++)
		delete checks[i];;
	return false;
}

void freeCellsGraph::traverser1(std::vector<freeCellsGraph*>* tally, renderAgent* render_agent) {
	for (int i = 0; i < tally->size(); i++) {
		if (tally->at(i) == this)
			return;
	}
	tally->push_back(this);

	render_agent->visualizing_helper4(this);

	for (int i = 0; i < this->diagraph_sibs.size(); i++)
		this->diagraph_sibs.at(i)->traverser1(tally, render_agent);
}

int pass = 0;
void freeCellsGraph::search_path(freeCellsGraph* goal) {
	std::cout << ++pass << "\n";

	if (this == goal) {
		std::cout << "[YAY] : path found\n";
		return;
	}

	/*if (this->potential_path != nullptr) {
		std::cout << "Giving chance to others\n";
		return;
	}*/

	float current_best_heuristic = 100000000.0f;
	for (int i = 0; i < diagraph_sibs.size();i++) {
		freeCellsGraph* traverser = this->last_visited_cell;
		bool break_flag = false;
		while (traverser != nullptr) {
			if (diagraph_sibs.at(i) == traverser) {
				break_flag = true;
				break;
			}
			traverser = traverser->last_visited_cell;
		}
		if (break_flag)
			continue;

		if (potential_path == nullptr) {
			potential_path = diagraph_sibs.at(i);
			current_best_heuristic = diagraph_sibs.at(i)->rank *
				CompGeomFunc::distance(diagraph_sibs.at(i)->center.x, diagraph_sibs.at(i)->center.y,
					goal->center.x, goal->center.y);
			continue;
		}

		float this_heuristic = diagraph_sibs.at(i)->rank *
			CompGeomFunc::distance(diagraph_sibs.at(i)->center.x, diagraph_sibs.at(i)->center.y,
				goal->center.x, goal->center.y);
		if (this_heuristic < current_best_heuristic) {
			potential_path = diagraph_sibs.at(i);
			current_best_heuristic = this_heuristic;
		}
	}

	if (potential_path == nullptr) {
		potential_path = goal;
		// Failsafe Warning
		std::cout << "******************************************************\n";
		std::cout << " Failsafe warning\n";
		std::cout << "******************************************************\n";
	}
	std::cin.ignore();
	potential_path->last_visited_cell = this;
	potential_path->search_path(goal);
}

void freeCellsGraph::find_center(void) {
	this->center = point((this->bound_left.x1 + this->bound_left.x2 + this->bound_right.x1 + this->bound_right.x2) / 4,
		(this->bound_right.y1 + this->bound_right.y2 + this->bound_left.y1 + this->bound_left.y2) / 4);
}

#endif