#include "../Header Files/elliptical_approx.h"

bool CompGeomFuncEllipseApprox::leftOn(float x1, float y1, float x2, float y2, float x0, float y0) {
	return ((x1 * (y2 - y0) + x2 * (y0 - y1) + x0 * (y1 - y2)) <= 0);
}

bool CompGeomFuncEllipseApprox::rightOn(float x1, float y1, float x2, float y2, float x0, float y0) {
	return ((x1 * (y2 - y0) + x2 * (y0 - y1) + x0 * (y1 - y2)) >= 0);
}

point CompGeomFuncEllipseApprox::intersection_point(vector_segment* slicee_edge, vector_segment* slicer_edge) {
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
	if (denom1 == 0) {
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

bool CompGeomFuncEllipseApprox::isIntersecting(vector_segment* edge1, vector_segment* edge2) {
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

bool CompGeomFuncEllipseApprox::isVectorExtendedIntersecting(vector_segment* fixed_edge, vector_segment* extendable_edge) {
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

bool CompGeomFuncEllipseApprox::isExtendedIntersecting(vector_segment* fixed_edge, vector_segment* extendable_edge) {
	bool test1 = extendable_edge->leftOn(fixed_edge->x1, fixed_edge->y1);
	bool test2 = extendable_edge->leftOn(fixed_edge->x2, fixed_edge->y2);

	if (test1 == test2)
		return false;

	return true;
}

float CompGeomFuncEllipseApprox::distance(float x1, float y1, float x2, float y2) {
	return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
}

int CompGeomFuncEllipseApprox::getSign(float x) {
	if (x >= 0)
		return 1;
	else
		return 0;
}

int vector_segment::checkLeft(float x3, float y3) {
	float expression = (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
	//if (expression != 0)
	if (expression > 0)
		return 1;
	else
		return -1;
	return 0;
	//return ((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) >= 0);
}

bool vector_segment::leftOn(float x3, float y3) {
	return CompGeomFuncEllipseApprox::leftOn(x1, y1, x2, y2, x3, y3);
}

bool vector_segment::rightOn(float x3, float y3) {
	return CompGeomFuncEllipseApprox::rightOn(x1, y1, x2, y2, x3, y3);
}

bool vector_segment::isIntersecting(vector_segment* edge) {
	return CompGeomFuncEllipseApprox::isIntersecting(this, edge);
}

bool vector_segment::isExtendedIntersecting(vector_segment* edge) {
	return CompGeomFuncEllipseApprox::isExtendedIntersecting(edge, this);
}

bool vector_segment::isVectorExtendedIntersecting(vector_segment* edge) {
	return CompGeomFuncEllipseApprox::isVectorExtendedIntersecting(edge, this);
}

bool vector_segment::isStrictlyVectorExtendedIntersecting(vector_segment* edge) {
	if (CompGeomFuncEllipseApprox::isIntersecting(edge, this))
		return false;
	return CompGeomFuncEllipseApprox::isVectorExtendedIntersecting(edge, this);
}

float vector_segment::length(void) {
	return CompGeomFunc::distance(x1, y1, x2, y2);
}

vector_segment vector_segment::invert(void) {
	return vector_segment(x2, y2, x1, y1);
}

float vector_segment::min_distance_from_line_segment(float x, float y) {
	float distance;
	if (CompGeomFuncEllipseApprox::getSign(vector_segment(x, y, this->x1, this->y1).dot_product(this))
		!= CompGeomFuncEllipseApprox::getSign(vector_segment(x, y, this->x2, this->y2).dot_product(this))) {
		// projection of point on line segment falls between the end points;
		// So minimum distance is the perpendicular distance
		distance = std::abs(((this->x1 - x) * (this->y2 - this->y1) - (this->y1 - y) * (this->x2 - this->x1)) / this->length());
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

float vector_segment::dot_product(vector_segment* edge) {
	return ((this->x2 - this->x1) * (edge->x2 - edge->x1) + (this->y2 - this->y1) * (edge->y2 - edge->y1));
}

float vector_segment::perpendicular_distance(float x, float y) {
	/*
	* equation of line : (y1-y2) x + (x2-x1) y - y1x2 + y2x1 = 0
	* d = ((y1-y2)x0 + (x2-x1)y0 - y1x2 + y2x1)/((y1-y2)^2 + (x1-x2)^2)^0.5
	* d = (-(y2-y1)x0 + (x2-x1)y0 - (x2-x1)y1 + (y2-y1)x1)/len
	* d = ((x1-x0)*(y2-y1) - (y1-y0)*(x2-x1))/length
	*/
	return std::abs(((this->x1 - x) * (this->y2 - this->y1) - (this->y1 - y) * (this->x2 - this->x1)) / this->length());
}

void vector_segment::intersection_points(ellipse* ellipse, vector_segment* result) {

	// if ellipse is more than a(major axis) units away from line : no intersection
	float d = this->perpendicular_distance(ellipse->center_x, ellipse->center_y);
	if (d > ellipse->a) {
		result = nullptr;
		return;
	}

	/*
	* Slope of tangent m1 = (y2-y1)/(x2-x1) = tan(phi)
	* Find distance d from ellipse center in variable d
	* Transform it to axis aligned format;
	* new m = (m1-m2)/(1+m1*m2) ; m2 = tan(theta_ellipse)
	* equation of transformed line : y = mx + d*(1+m*m)^0.5
	*
	* For a origin-centered axis aligned ellipse ; (x/a)^2 + (y/b)^2 = 1
	* intersection points x:
	* (b^2 + a^2*m^2) x^2 + 2(a^2)dm*(1+m*m)^0.5 x + (a^2)*((d^2)*(1+m*m)-a*a*b*b) = 0
	* P x^2 + Q x + R = 0
	* If discriminant < 0 ==> Not interesecting
	* For discriminant >=0 ==> Find x1, y1, x2, y2 ==> update as expose_vector_chord
	*/

	float m1 = (y2 - y1) / (x2 - x1);
	float m2 = std::tan(ellipse->tilt);
	float m = (m1 - m2) / (1 + m1 * m2);
	float a = ellipse->a;
	float b = ellipse->b;

	float P = b*b + m*m*a*a;
	float Q = 2 * a*a * d * m * std::pow(1 + m * m, 0.5);
	float R = a * a * (d * d * (1 + m * m) - a * a * b * b);

	float discriminant = Q * Q - 4 * P * R;

	if (discriminant < 0) {
		result = nullptr;
		return;
	}

	float x1 = (-Q - std::pow(discriminant, 0.5)) / (2 * P);
	float x2 = (-Q + std::pow(discriminant, 0.5)) / (2 * P);
	float k = d * std::pow(1 + m * m, 0.5);
	float y1 = m * x1 + k;
	float y2 = m * x2 + k;

	vector_segment temp = vector_segment(x1, y1, x2, y2); // These points are relative to the given ellipse
									// coordinate frame is chosen centered at ellipse center, x-axis along major axis
	result = &temp;

	// ensure that tangents are always facing the same direction
	// convention : ellipse center should be on left to tangent

	if (result->rightOn(ellipse->center_x, ellipse->center_y)) {
		// if this convention was not followed; reverse the direction
		vector_segment temp = vector_segment(x2, y2, x1, y1);
		result = &temp;
	}
}

point vector_segment::intersection_points(vector_segment* edge) {
	return CompGeomFuncEllipseApprox::intersection_point(this, edge);
}

bool quad::isIntersecting(quad* quad) {
	if (blocked1.isIntersecting(&quad->free1) && blocked1.isIntersecting(&quad->free2))
		return false;
	if (blocked1.isIntersecting(&quad->blocked1) && blocked1.isIntersecting(&quad->blocked2))
		return false;

	if (blocked2.isIntersecting(&quad->free1) && blocked2.isIntersecting(&quad->free2))
		return false;
	if (blocked2.isIntersecting(&quad->blocked1) && blocked2.isIntersecting(&quad->blocked2))
		return false;

	if (quad->blocked1.isIntersecting(&free1) && quad->blocked1.isIntersecting(&free2))
		return false;
	if (quad->blocked1.isIntersecting(&blocked1) && quad->blocked1.isIntersecting(&blocked2))
		return false;

	if (quad->blocked2.isIntersecting(&free1) && quad->blocked2.isIntersecting(&free2))
		return false;
	if (quad->blocked2.isIntersecting(&blocked1) && quad->blocked2.isIntersecting(&blocked2))
		return false;

	return true;
}

void quad::map_expander(quad* quad, std::vector<bool>* checklist) {
	/*
	* If this is quad itself, return
	* Check if this quad has been already visited; if yes, return
	* Else check if it intersects with quad; if yes, add this quad to its neighbours list
	* Call this function for all the neighbours
	*/
	if (this == quad)
		return;
	if (checklist->at(id))
		return;

	checklist->at(id) = true;
	for (int i = 0; i < neighbours.size(); i++) {
		if (checklist->at(neighbours.at(i)->id))
			continue;
		neighbours.at(i)->map_expander(quad, checklist);
	}

	// intersection code here!!
	if (isIntersecting(quad))
		neighbours.push_back(quad);
}

void ellipse::neighbourSweep::update_tangents_info(vector_segment internal_tangent1, vector_segment internal_tangent2,
	vector_segment external_tangent1, vector_segment external_tangent2) {
	this->internal_tangent1 = internal_tangent1;
	this->internal_tangent2 = internal_tangent2;
	this->internal_range = vector_segment(internal_tangent1.x2, internal_tangent1.y2,
		internal_tangent2.x2, internal_tangent2.y2);

	this->external_tangent1 = external_tangent1;
	this->external_tangent2 = external_tangent2;
	this->external_range = vector_segment(external_tangent1.x2, external_tangent1.y2,
		external_tangent2.x2, external_tangent2.y2);

	this->visibility_limit_left = external_tangent1;
	this->visibility_limit_right = external_tangent2;
	evaluate_visibility_range();
}

void ellipse::neighbourSweep::evaluate_visibility_range(void) {
	if (visibility_range.isNull()) {
		visibility_range = vector_segment(visibility_limit_left.x2, visibility_limit_left.y2,
			visibility_limit_right.x2, visibility_limit_right.y2);
	}
	else {
		visibility_range.x1 = visibility_limit_left.x2;
		visibility_range.y1 = visibility_limit_left.y2;
		visibility_range.x2 = visibility_limit_right.x2;
		visibility_range.y2 = visibility_limit_right.y2;
	}
}

int ellipse::neighbourSweep::compute_importance(void) {
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
	float param3 = 1 / (((len3 > len4) ? (len3) : (len4)));
	float importance = param1 * param2 * param3;
	std::cout << importance << std::endl;
	this->importance = importance;
	return importance;
}

ellipse::boundingChords::boundingChords(vector_segment* bound1, vector_segment* bound2) {
	bound1_empty = (bound1 == nullptr);
	bound2_empty = (bound2 == nullptr);

	if (bound1_empty && bound2_empty)
		empty = true;
	else
		empty = false;

	if (!bound1_empty) {
		this->bound1 = *bound1;
	}
	if (!bound2_empty) {
		this->bound2 = *bound2;
	}
}

vector_segment* ellipse::boundingChords::update_bound(vector_segment* original, vector_segment* update) {
	if (!original->isIntersecting(update))
		return nullptr; // doesn't intersect each other

	// find the resultant tangent
	vector_segment resultant;
	if (original->rightOn(update->x1, update->y1)) {
		resultant = vector_segment(original->x1, original->y1, update->x2, update->y2);
	}
	else {
		resultant = vector_segment(update->x1, update->y1, original->x1, original->y1);
	}

	return &resultant;
}

void ellipse::boundingChords::intersection_update(boundingChords bound) {
	/*
	* check intersection of first original chord with the two bounds 1 by 1.
	* at each intersection if output is nullptr; don't update. else update the pointer.
	* after both checks if latest result is nulptr, mark that bound empty
	*/
	vector_segment* resultant = update_bound(&bound1, &bound.bound1);
	if (resultant != nullptr) {
		bound1 = *resultant;
		resultant = nullptr;
	}

	resultant = update_bound(&bound1, &bound.bound2);
	if (resultant == nullptr) {
		bound1_empty = true;
	}
	else {
		bound1 = *resultant;
		resultant = nullptr;
	}

	resultant = update_bound(&bound2, &bound.bound1);
	if (resultant != nullptr) {
		bound2 = *resultant;
		resultant = nullptr;
	}

	resultant = update_bound(&bound2, &bound.bound2);
	if (resultant == nullptr) {
		bound2_empty = true;
	}
	else {
		bound2 = *resultant;
		resultant = nullptr;
	}

	if (bound1_empty && bound2_empty)
		empty = true;
}

bool ellipse::isInside(float x, float y, int i) {
	//std::cout << "(x, y) -> (" << x << ", " << y << ") | (center_x, center_y) -> (" <<
	//	center_x << ", " << center_y << ")" << std::endl;
	float x_rel = x - center_x;
	float y_rel = y - center_y;

	float x_tilt_frame = x_rel * cos(tilt) + y_rel * sin(tilt);
	float y_tilt_frame = -x_rel * sin(tilt) + y_rel * cos(tilt);

	float query_angle = atan2(a*y_rel, b*x_rel);
	if (false) {
		std::cout << "atan2(y/x) = " << query_angle * 180 / 3.141592 << " degrees" << std::endl;
		std::cout << "(query_angle, tilt) = " << query_angle << " , " << tilt << std::endl;
		std::cout << "(x_tilt, limit) -> " << x_tilt_frame << " , " << (a * cos(query_angle - tilt)) << "." << std::endl;
		std::cout << "(y_tilt, limit) -> " << y_tilt_frame << " , " << (b * sin(query_angle - tilt)) << "." << std::endl;
		std::cout << "(a, b) -> " << a << " , " << b << "." << std::endl;
	}

	float x_factor = x_tilt_frame / (a * cos(query_angle - tilt));
	if (x_factor > 1 || x_factor < 0)
		return false;
		
	return true;
}

std::vector<int> ellipse::create_priority_list(std::vector<ellipse>* obstacles) {
	/*
	* For the refernce ellipse, sort the remaining elliptical obstacles
	in order of distance from reference ellipse
	*/
	std::vector<int> priority_queue;

	float current_min = 100000.0f;
	int current_index = 0;
	float me_center_x = center_x;
	float me_center_y = center_y;
	bool find_val = false;

	for (auto& obstacle : *obstacles) {
		// n iterations are needed; 1 list entry per iteration
		current_min = 100000.0f;
		for (int i = 0; i < obstacles->size(); i++) {
			// check if the elipse is already in priority queue; skip it by "continue"
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

			float you_center_x = obstacles->at(i).center_x;
			float you_center_y = obstacles->at(i).center_y;
			float dist2 = pow(me_center_x - you_center_x, 2.0) + pow(me_center_y - you_center_y, 2.0);
			if (dist2 < current_min) { // dist2 is squared distance
				current_min = dist2;
				current_index = i;
			}
		}
		priority_queue.push_back(current_index);
	}

	return priority_queue;
}

bool ellipse::checkShielded_reloaded(ellipse* neighbour, local_visualizer* render_agent) {
	boundingChords expose_vector;

	for (int i = 0; i < neighbours.size(); i++) {// iterating over "petals" like bounds of the complete "flower" like polygon
		vector_segment internal_chord = vector_segment(neighbours.at(i).internal_tangent1.x2,
			neighbours.at(i).internal_tangent1.y2, neighbours.at(i).internal_tangent2.x2,
			neighbours.at(i).internal_tangent2.y2);
		// if ellipse is already in front of the shielding ellipse
		bool check0 = internal_chord.leftOn(neighbour->center_x, neighbour->center_y);
		std::cout << "[KUNDALI] : check0 : " << check0 << std::endl;
		if (check0)
			continue;
		vector_segment* intersection_test0 = nullptr;
		internal_chord.intersection_points(neighbour, intersection_test0);
		std::cout << "[KUNDALI] : intersection_test0 : " << (intersection_test0 != nullptr) << std::endl;
		if (intersection_test0 != nullptr)
			continue;

		// check if the ellipse is outside the quadrant(formed by 2 tangents) of interest
		bool check1 = neighbours.at(i).internal_tangent1.leftOn(neighbour->center_x, neighbour->center_y);
		bool check2 = neighbours.at(i).internal_tangent2.rightOn(neighbour->center_x, neighbour->center_y);
		std::cout << "[KUNDALI] : check1 : " << check1 << std::endl;
		std::cout << "[KUNDALI] : check2 : " << check2 << std::endl;
		if (check1 || check2)
			continue;

		vector_segment* intersection1 = nullptr, * intersection2 = nullptr;
		neighbours.at(i).internal_tangent1.intersection_points(neighbour, intersection1);
		neighbours.at(i).internal_tangent2.intersection_points(neighbour, intersection2);
		std::cout << "[KUNDALI] : intersection1 : " << (intersection1 == nullptr) << std::endl;
		std::cout << "[KUNDALI] : intersection2 : " << (intersection2 == nullptr) << std::endl;

		if (intersection1 == nullptr && intersection2 == nullptr) {
			std::cout << " Far off !!\n";
			render_agent->shielding_visualizer(this, neighbour);
			return true;
		}

		std::cout << "[KUNDALI] : (expose_vector.empty) : " << (expose_vector.empty) << std::endl;
		if (expose_vector.empty) {
			expose_vector = boundingChords(intersection1, intersection2);
		}
		else {
			//check which tangent is involved
			boundingChords update = boundingChords(intersection1, intersection2);
			expose_vector.intersection_update(update);
		}
	}
	std::cout << " Nearest neighour certified\n";
	render_agent->shielding_visualizer(this, neighbour);
	return false;
}

bool ellipse::checkShielded_reloaded_v2(ellipse* neighbour, local_visualizer* render_agent) {
	std::vector<vector_segment> exposed_segments;
	exposed_segments.push_back(vector_segment(
		neighbour->center_x + neighbour->a * std::sin(neighbour->tilt),
		neighbour->center_y + neighbour->a * std::cos(neighbour->tilt), 
		neighbour->center_x - neighbour->a * std::sin(neighbour->tilt),
		neighbour->center_y - neighbour->a * std::cos(neighbour->tilt)));
	exposed_segments.push_back(vector_segment(
		neighbour->center_x + neighbour->b * std::cos(neighbour->tilt),
		neighbour->center_y - neighbour->b * std::sin(neighbour->tilt),
		neighbour->center_x - neighbour->b * std::cos(neighbour->tilt),
		neighbour->center_y + neighbour->b * std::sin(neighbour->tilt)));

	for (int i = 0; i < neighbours.size(); i++) {// iterating over "petals" like bounds of the complete "flower" like polygon
		vector_segment internal_chord = vector_segment(neighbours.at(i).internal_tangent1.x2,
			neighbours.at(i).internal_tangent1.y2, neighbours.at(i).internal_tangent2.x2,
			neighbours.at(i).internal_tangent2.y2);

		for (int j = 0; j < exposed_segments.size(); j++) {
			// if this ellipse is already in front of the shielding ellipse
			bool check;
			check = internal_chord.rightOn(exposed_segments.at(j).x1, exposed_segments.at(j).y1);
			if (check) {
				continue;
			}
			check = internal_chord.rightOn(exposed_segments.at(j).x2, exposed_segments.at(j).y2);
			if (check) {
				continue;
			}

			// check if the ellipse is outside the quadrant(formed by 2 tangents) of interest
			bool check1 = neighbours.at(i).internal_tangent1.leftOn(exposed_segments.at(j).x1, exposed_segments.at(j).y1);
			bool check2 = neighbours.at(i).internal_tangent1.leftOn(exposed_segments.at(j).x2, exposed_segments.at(j).y2);
			bool check3 = neighbours.at(i).internal_tangent2.rightOn(exposed_segments.at(j).x1, exposed_segments.at(j).y1);
			bool check4 = neighbours.at(i).internal_tangent2.rightOn(exposed_segments.at(j).x2, exposed_segments.at(j).y2);

			if ((check1 && check2) || (check3 && check4)) {
				continue; //completely exposed
			}

			// case of partial exposure
			if (!check1 || !check2) {
				// tangent 1 is cut accross by the line of concern
				point intersection_points = exposed_segments.at(j).intersection_points(
					&neighbours.at(i).internal_tangent1);
				if (check1) { // leading end of edge in left most interest zone
					if (check4) {
						// double cut
						point secondary_intersection = exposed_segments.at(j).intersection_points(
							&neighbours.at(i).internal_tangent2);
						exposed_segments.push_back(vector_segment(secondary_intersection.x,
							secondary_intersection.y, exposed_segments.at(j).x2,
							exposed_segments.at(j).y2));
						exposed_segments.at(j).x2 = intersection_points.x;
						exposed_segments.at(j).y2 = intersection_points.y;
					}
					else {
						// single cut
						std::cout << "[UPDATE] single cut. updating exposed vector : " 
							<< intersection_points.x<<", "<< intersection_points.y << std::endl;
						exposed_segments.at(j).x2 = intersection_points.x;
						exposed_segments.at(j).y2 = intersection_points.y;
					}
				}
				if (check2) { // trailing(rear) end of edge in left most interest zone
					if (check3) {
						// double cut
						point secondary_intersection = exposed_segments.at(j).intersection_points(
							&neighbours.at(i).internal_tangent2);
						exposed_segments.push_back(vector_segment(exposed_segments.at(j).x1,
							exposed_segments.at(j).y1, secondary_intersection.x,
							secondary_intersection.y));
						exposed_segments.at(j).x1 = intersection_points.x;
						exposed_segments.at(j).y1 = intersection_points.y;
					}
					else {
						// single cut
						exposed_segments.at(j).x1 = intersection_points.x;
						exposed_segments.at(j).y1 = intersection_points.y;
					}
				}
			}
			if (!check3 || !check4) {
				// tangent 2 is cut accross by the line of concern
				point intersection_points = exposed_segments.at(j).intersection_points(
					&neighbours.at(i).internal_tangent2);
				if (check3) { // leading end of edge in right most interest zone
					if (check2) {
						// double cut
						point secondary_intersection = exposed_segments.at(j).intersection_points(
							&neighbours.at(i).internal_tangent1);
						exposed_segments.push_back(vector_segment(secondary_intersection.x,
							secondary_intersection.y, exposed_segments.at(j).x2,
							exposed_segments.at(j).y2));
						exposed_segments.at(j).x2 = intersection_points.x;
						exposed_segments.at(j).y2 = intersection_points.y;
					}
					else {
						// single cut
						exposed_segments.at(j).x2 = intersection_points.x;
						exposed_segments.at(j).y2 = intersection_points.y;
					}
				}
				if (check4) { // trailing(rear) end of edge in right most interest zone
					if (check1) {
						// double cut
						point secondary_intersection = exposed_segments.at(j).intersection_points(
							&neighbours.at(i).internal_tangent1);
						exposed_segments.push_back(vector_segment(exposed_segments.at(j).x1,
							exposed_segments.at(j).y1, secondary_intersection.x,
							secondary_intersection.y));
						exposed_segments.at(j).x1 = intersection_points.x;
						exposed_segments.at(j).y1 = intersection_points.y;
					}
					else {
						// single cut
						exposed_segments.at(j).x1 = intersection_points.x;
						exposed_segments.at(j).y1 = intersection_points.y;
					}
				}
			}

			// case of complete shielding
			if (!check1 && !check2) {
				if (!check3 && !check4) {
					exposed_segments.erase(exposed_segments.begin() + j);
					j--;
				}
			}
		}
	}
	if (exposed_segments.empty()) {
		std::cout << " Shielded. OFFICIALLY\n";
		return true;
	}

	std::cout << " Nearest neighour certified\n";
	render_agent->shielding_visualizer(this, neighbour);
	return false;
}

ellipse::neighbourSweep ellipse::compute_approximate_tangent(ellipse* neighbour, local_visualizer* render_agent) {
	/*
	* compute approximated tangents between two ellipses, now approximated aa tangents between two quads
	*/
	vector_segment InT1, InT2, ExT1, ExT2;


	float x_i1_no_tilt = a;
	float y_i1_no_tilt = 0;
	float x_i2_no_tilt = 0;
	float y_i2_no_tilt = b;

	std::vector<std::vector<float>> quad1 = {
		{center_x + a * std::sin(tilt), center_y + a * std::cos(tilt)},
		{center_x + b * std::cos(tilt), center_y - b * std::sin(tilt)},
		{center_x - a * std::sin(tilt), center_y - a * std::cos(tilt)},
		{center_x - b * std::cos(tilt), center_y + b * std::sin(tilt)}
	};
	std::vector<std::vector<float>> quad2 = {
		{neighbour->center_x + neighbour->a * std::sin(neighbour->tilt), neighbour->center_y + neighbour->a * std::cos(neighbour->tilt)},
		{neighbour->center_x + neighbour->b * std::cos(neighbour->tilt), neighbour->center_y - neighbour->b * std::sin(neighbour->tilt)},
		{neighbour->center_x - neighbour->a * std::sin(neighbour->tilt), neighbour->center_y - neighbour->a * std::cos(neighbour->tilt)},
		{neighbour->center_x - neighbour->b * std::cos(neighbour->tilt), neighbour->center_y + neighbour->b * std::sin(neighbour->tilt)}
	};

	for (int i = 0; i < quad1.size(); i++) {
		for (int j = 0; j < quad2.size(); j++) {
			float x1, y1, x2, y2;
			x1 = quad1.at(i).at(0);
			y1 = quad1.at(i).at(1);
			x2 = quad2.at(j).at(0);
			y2 = quad2.at(j).at(1);
			vector_segment line = vector_segment(x1, y1, x2, y2);

			int check1 = 0, check2 = 0;
			for (int k = 0; k < quad1.size(); k++) {
				if (k == i)
					continue;
				bool check = line.leftOn(quad1.at(k).at(0), quad1.at(k).at(1));
				if (check) {
					check1++;
				}
				else {
					check1--;
				}
			}

			for (int k = 0; k < quad2.size(); k++) {
				if (k == j)
					continue;
				bool check = line.leftOn(quad2.at(k).at(0), quad2.at(k).at(1));
				if (check) {
					check2++;
				}
				else {
					check2--;
				}
			}

			if (check1 == -3) {
				if (check2 == -3) {
					// external tangent 1
					ExT1 = line;
				}
				if (check2 == 3) {
					// internal tangent 2
					InT2 = line;
				}
			}
			if (check1 == 3) {
				if (check2 == -3) {
					// internal tangent 1
					InT1 = line;
				}
				if (check2 == 3) {
					// external tangent 2
					ExT2 = line;
				}
			}
		}
	}

	return neighbourSweep(neighbour, uniqueID);
}

void ellipse::approximate_tangent(ellipse* neighbour, local_visualizer* render_agent) {
	/*
	* compute approximated tangents between two ellipses, now approximated aa tangents between two quads
	*/
	vector_segment InT1, InT2, ExT1, ExT2;

	std::vector<std::vector<float>> quad1 = { 
		{center_x + a*std::sin(tilt), center_y + a*std::cos(tilt)}, 
		{center_x + b*std::cos(tilt), center_y - b*std::sin(tilt)}, 
		{center_x - a*std::sin(tilt), center_y - a*std::cos(tilt)},
		{center_x - b*std::cos(tilt), center_y + b*std::sin(tilt)}
	};
	std::vector<std::vector<float>> quad2 = {
		{neighbour->center_x + neighbour->a * std::sin(neighbour->tilt), neighbour->center_y + neighbour->a * std::cos(neighbour->tilt)},
		{neighbour->center_x + neighbour->b * std::cos(neighbour->tilt), neighbour->center_y - neighbour->b * std::sin(neighbour->tilt)},
		{neighbour->center_x - neighbour->a * std::sin(neighbour->tilt), neighbour->center_y - neighbour->a * std::cos(neighbour->tilt)},
		{neighbour->center_x - neighbour->b * std::cos(neighbour->tilt), neighbour->center_y + neighbour->b * std::sin(neighbour->tilt)}
	};

	for (int i = 0; i < quad1.size(); i++) {
		for (int j = 0; j < quad2.size(); j++) {
			float x1, y1, x2, y2;
			x1 = quad1.at(i).at(0);
			y1 = quad1.at(i).at(1);
			x2 = quad2.at(j).at(0);
			y2 = quad2.at(j).at(1);
			vector_segment line = vector_segment(x1, y1, x2, y2);

			int check1 = 0, check2 = 0;
			for (int k = 0; k < quad1.size(); k++) {
				if (k == i)
					continue;
				bool check = line.leftOn(quad1.at(k).at(0), quad1.at(k).at(1));
				if (check) {
					check1++;
				}
				else {
					check1--;
				}
			}

			for (int k = 0; k < quad2.size(); k++) {
				if (k == j)
					continue;
				bool check = line.leftOn(quad2.at(k).at(0), quad2.at(k).at(1));
				if (check) {
					check2++;
				}
				else {
					check2--;
				}
			}

			if (check1 == -3) {
				if (check2 == -3) {
					// external tangent 1
					ExT1 = line;
				}
				if (check2 == 3) {
					// internal tangent 2
					InT2 = line;
				}
			}
			if (check1 == 3) {
				if (check2 == -3) {
					// internal tangent 1
					InT1 = line;
				}
				if (check2 == 3) {
					// external tangent 2
					ExT2 = line;
				}
			}
		}
	}

	neighbourSweep sweep = neighbourSweep(neighbour, uniqueID);
	sweep.update_tangents_info(InT1, InT2, ExT1, ExT2);
	neighbours.push_back(sweep);
	render_agent->visualize_tangent_approximations(this, neighbour);
}

void ellipse::tangents_handler(ellipse* neighbour, local_visualizer* render_agent) {
	/*
	* passed ellipse is already a neighbour of this ellipse
	* Just compute all the relevant details and add this to neighbours list
	*/

	vector_segment internal_tangent1, internal_tangent2, internal_range,
		external_tangent1, external_tangent2, external_range;

	/*std::vector<int>* set_1 = &vertices;
	std::vector<int>* set_2 = &neighbour->vertices;


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
	}*/
}

void ellipse::create_neighbour_map(std::vector<ellipse>* obstacles, local_visualizer* render_agent) {

	/*
	* NEAREST NEIGHBOURS
	* Step I : iterate through all ellipses; for each ellipse, sort remaining ones in order of distances
	* Step II : In this order -> call a function to locate nearest neighbours by filtering out far ones
	* Step III : Push located neighbours data to vector
	*/

	int clear_count = 0;

	// sort list to iterate through nearest objects first; will reduce unnecessary computations
	std::vector<int> traversal_order = create_priority_list(obstacles);

	for (int i = 0; i < obstacles->size(); i++) {
		// iterate in a systematic way by this traversal
		//ellipse* obstacle = &obstacles->at(i);
		// greedy approach; iterating through nearest obstacle first
		ellipse* obstacle = &obstacles->at(traversal_order.at(i));
		if (this == obstacle) {
			continue;
		}

		// check if this obstacle is already shielded by existing neighbour "petals"
		/*if (checkShielded_reloaded(obstacle, render_agent)) {
			continue;
		}*/
		if (checkShielded_reloaded_v2(obstacle, render_agent)) {
			continue;
		}
		else {
			// if not shielded; add this obstacle to neighbour list
			// First compute tangents and compute neighboursSweep object for that
			// compute tangents
			approximate_tangent(obstacle, render_agent);
		}

	}
	render_agent->visualize_nearest_neighbour_ellipses(this);
}

local_visualizer::local_visualizer(void) {
	GRID_WIDTH = 0;
	GRID_HEIGHT = 0;
	paths = nullptr;
	renderer = nullptr;
	render_callback = nullptr;
	scribbled_paths = 0;
}

local_visualizer::local_visualizer(int GRID_WIDTH, int GRID_HEIGHT, RenderClass* renderer, std::vector<Path>* paths,
	void (*func_updater)(std::vector<float>*)) {
	this->GRID_WIDTH = GRID_WIDTH;
	this->GRID_HEIGHT = GRID_HEIGHT;
	this->paths = paths; // for visualization
	this->renderer = renderer;
	this->render_callback = func_updater;
	scribbled_paths = 0;
}

void local_visualizer::clear_paths(void) {
	paths->clear();
	render_callback(nullptr);
	renderer->render();
}

void local_visualizer::clear_paths(int count) {
	for (int i = 0; i < count; i++)
		paths->pop_back();
	invalidate();
}

void local_visualizer::clear_path(void) {
	clear_paths(scribbled_paths);
	scribbled_paths = 0;
}

void local_visualizer::add_path(Path path) {
	paths->push_back(path);
}

void local_visualizer::add_path(vector_segment vector_segment) {
	paths->push_back(Path(vector_segment.x1, vector_segment.y1,
		vector_segment.x2, vector_segment.y2));
	invalidate();
}

void local_visualizer::add_path(vector_segment vector_segment, Color color) {
	paths->push_back(Path(vector_segment.x1, vector_segment.y1,
		vector_segment.x2, vector_segment.y2, color));
	invalidate();
}

void local_visualizer::draw_ellipse(ellipse ellipse) {
	int DIVISIONS = 36;
	std::cout << "Ellipse info\n";
	std::cout << "center_x : " << ellipse.center_x << std::endl;
	std::cout << "center_y : " << ellipse.center_y << std::endl;
	std::cout << "a : " << ellipse.a << std::endl;
	std::cout << "b : " << ellipse.b << std::endl;
	std::cout << "tilt : " << ellipse.tilt << std::endl;
	for (int i = 0; i < DIVISIONS; i++) {
		float x_i1_no_tilt = ellipse.a * cos(2 * i * PI / DIVISIONS);
		float y_i1_no_tilt = ellipse.b * sin(2 * i * PI / DIVISIONS);
		float x_i2_no_tilt = ellipse.a * cos(2 * (i + 1) * PI / DIVISIONS);
		float y_i2_no_tilt = ellipse.b * sin(2 * (i + 1) * PI / DIVISIONS);

		paths->push_back(Path(
			ellipse.center_x +
			x_i1_no_tilt * sin(ellipse.tilt) + y_i1_no_tilt * cos(ellipse.tilt),
			-(-ellipse.center_y -
				x_i1_no_tilt * cos(ellipse.tilt) + y_i1_no_tilt * sin(ellipse.tilt)),
			ellipse.center_x +
			x_i2_no_tilt * sin(ellipse.tilt) + y_i2_no_tilt * cos(ellipse.tilt),
			-(-ellipse.center_y -
				x_i2_no_tilt * cos(ellipse.tilt) + y_i2_no_tilt * sin(ellipse.tilt))
		));
	}
	invalidate();
	return;
}

int local_visualizer::draw_ellipse(ellipse ellipse, Color color) {
	int DIVISIONS = 36;
	std::cout << "Ellipse info\n";
	std::cout << "center_x : " << ellipse.center_x << std::endl;
	std::cout << "center_y : " << ellipse.center_y << std::endl;
	std::cout << "a : " << ellipse.a << std::endl;
	std::cout << "b : " << ellipse.b << std::endl;
	std::cout << "tilt : " << ellipse.tilt << std::endl;
	for (int i = 0; i < DIVISIONS; i++) {
		float x_i1_no_tilt = ellipse.a * cos(2 * i * PI / DIVISIONS);
		float y_i1_no_tilt = ellipse.b * sin(2 * i * PI / DIVISIONS);
		float x_i2_no_tilt = ellipse.a * cos(2 * (i + 1) * PI / DIVISIONS);
		float y_i2_no_tilt = ellipse.b * sin(2 * (i + 1) * PI / DIVISIONS);

		paths->push_back(Path(
			ellipse.center_x +
			x_i1_no_tilt * sin(ellipse.tilt) + y_i1_no_tilt * cos(ellipse.tilt),
			-(-ellipse.center_y -
				x_i1_no_tilt * cos(ellipse.tilt) + y_i1_no_tilt * sin(ellipse.tilt)),
			ellipse.center_x +
			x_i2_no_tilt * sin(ellipse.tilt) + y_i2_no_tilt * cos(ellipse.tilt),
			-(-ellipse.center_y -
				x_i2_no_tilt * cos(ellipse.tilt) + y_i2_no_tilt * sin(ellipse.tilt)),
			color
		));
	}
	invalidate();
	return DIVISIONS;
}

void local_visualizer::visualize_quads(quad* quad, bool ellipses, bool clear_back) {
	if (true) {
		int path_count = 0;
		add_path(quad->blocked1, Color(1.0, 0.0, 0.0));
		add_path(quad->blocked2, Color(1.0, 0.0, 0.0));
		add_path(quad->free1, Color(0.0, 0.0, 1.0));
		add_path(quad->free2, Color(0.0, 0.0, 1.0));
		path_count += 4;

		if (ellipses) {
			path_count += draw_ellipse(*quad->me_A, Color(1.0, 1.0, 1.0));
			path_count += draw_ellipse(*quad->me_B, Color(0.0, 0.0, 0.0));
		}
		invalidate();

		std::cin.ignore();
		if (clear_back)
			clear_paths(path_count);
	}
}

void local_visualizer::visualize_nearest_neighbour_ellipses(ellipse* obstacle) {
	if (false) {
		int path_count = 0;
		path_count += draw_ellipse(*obstacle, Color(0.0, 0.0, 0.0));

		invalidate();
		std::cin.ignore();

		for (int i = 0; i < obstacle->neighbours.size(); i++) {
			path_count += draw_ellipse(*obstacle->neighbours.at(i).pointer, Color(1.0, 0.0, 1.0));
		}

		invalidate();
		std::cin.ignore();

		clear_paths(path_count);
	}
}

void local_visualizer::shielding_visualizer(ellipse* obstacle, ellipse* neighbour) {
	if (false) {
		int path_count = 0;
		path_count += draw_ellipse(*obstacle, Color(0.0, 0.0, 0.0));
		path_count += draw_ellipse(*neighbour, Color(1.0, 1.0, 0.0));
		
		invalidate();
		std::cin.ignore();

		clear_paths(path_count);
	}
}

void local_visualizer::visualize_tangent_approximations(ellipse* object1, ellipse* object2) {
	if (false) {
		for (int i = 0; i < object1->neighbours.size(); i++) {
			if (object1->neighbours.at(i).pointer == object2) {
				ellipse::neighbourSweep sweep = object1->neighbours.at(i);
				Color color = Color(1.0, 0.0, 0.0);
				vector_segment InT1 = sweep.internal_tangent1;
				add_path(Path(InT1.x1, InT1.y1, InT1.x2, InT1.y2, color));
				vector_segment InT2 = sweep.internal_tangent2;
				add_path(Path(InT2.x1, InT2.y1, InT2.x2, InT2.y2, color));
				vector_segment InR = sweep.internal_range;
				add_path(Path(InR.x1, InR.y1, InR.x2, InR.y2, color));
				color = Color(0.0, 0.0, 1.0);
				vector_segment ExT1 = sweep.external_tangent1;
				add_path(Path(ExT1.x1, ExT1.y1, ExT1.x2, ExT1.y2, color));
				vector_segment ExT2 = sweep.external_tangent2;
				add_path(Path(ExT2.x1, ExT2.y1, ExT2.x2, ExT2.y2, color));
				vector_segment ExR = sweep.external_range;
				add_path(Path(ExR.x1, ExR.y1, ExR.x2, ExR.y2, color));
				break;
			}
		}
		invalidate();
		std::cin.ignore();

		if (true)
			clear_paths(6);
	}
}

void local_visualizer::show_edges(std::vector<Line>* edge_list) {
	if (false) {
		Color color = Color(0.5f, 1.0f, 0.5f);

		for (auto& edge : *edge_list) {
			paths->push_back(Path(edge.x1, edge.y1, edge.x2, edge.y2, color));
		}
		invalidate();

		/*std::cin.ignore();
		for (auto& edge : *edge_list) {
			paths->pop_back();
		}
		invalidate();*/
	}
}

void local_visualizer::draw_ears_v2(std::vector<int_point>* points, std::vector<std::vector<int>>* poly_indice) {
	if (false) {
		Color color[] = { Color(0.0f, 0.0f, 0.0f), Color(1.0f, 1.0f, 1.0f), Color(1.0f, 0.0f, 0.0f),
		Color(0.0f, 1.0f, 1.0f), Color(0.0f, 1.0f, 0.0f), Color(1.0f, 0.0f, 1.0f), Color(1.0f, 1.0f, 0.0f) };

		for (int k = 0; k < poly_indice->size(); k++) {
			std::vector<int> ear = poly_indice->at(k);
			for (int i = 0; i < ear.size(); i++) {
				if (i + 1 == ear.size()) {
					paths->push_back(Path(points->at(ear[i]).x, points->at(ear[i]).y, points->at(ear[0]).x, points->at(ear[0]).y, color[k % 7]));
					break;
				}
				paths->push_back(Path(points->at(ear[i]).x, points->at(ear[i]).y, points->at(ear[i + 1]).x, points->at(ear[i + 1]).y, color[k % 7]));
			}
		}
		invalidate();
	}
}

void local_visualizer::draw_ears(int data_x[], int data_y[], std::vector<std::vector<int>>* poly_indice) {
	if (false) {
		Color color[] = { Color(0.0f, 0.0f, 0.0f), Color(1.0f, 1.0f, 1.0f), Color(1.0f, 0.0f, 0.0f),
		Color(0.0f, 1.0f, 1.0f), Color(0.0f, 1.0f, 0.0f), Color(1.0f, 0.0f, 1.0f), Color(1.0f, 1.0f, 0.0f) };

		for (int k = 0; k < poly_indice->size(); k++) {
			std::vector<int> ear = poly_indice->at(k);
			for (int i = 0; i < ear.size(); i++) {
				if (i + 1 == ear.size()) {
					paths->push_back(Path(data_x[ear[i]], data_y[ear[i]], data_x[ear[0]], data_y[ear[0]], color[k % 7]));
					break;
				}
				paths->push_back(Path(data_x[ear[i]], data_y[ear[i]], data_x[ear[i + 1]], data_y[ear[i + 1]], color[k % 7]));
			}
		}
		invalidate();
	}
}

void local_visualizer::visualize_ear_2(std::vector<int_point>*points, std::vector<int>* ear, bool clear_again) {
	if (false) {
		Color color = Color(0.0f, 0.0f, 0.0f);

		for (int i = 0; i < ear->size(); i++) {
			if (i + 1 == ear->size()) {
				paths->push_back(Path(points->at(ear->at(i)).x, points->at(ear->at(i)).y, 
					points->at(ear->at(0)).x, points->at(ear->at(0)).y, color));
				break;
			}
			paths->push_back(Path(points->at(ear->at(i)).x, points->at(ear->at(i)).y,
				points->at(ear->at(i+1)).x, points->at(ear->at(i+1)).y, color));
		}
		invalidate();


		std::cin.ignore();
		if (clear_again) {
			for (int i = 0; i < ear->size(); i++) {
				paths->pop_back();
			}
			invalidate();
		}
		else {
			scribbled_paths += ear->size();
		}
	}
}

void local_visualizer::visualize_ear(int data_x[], int data_y[], std::vector<int>* ear, bool clear_again) {
	if (false) {
		Color color = Color(0.0f, 0.0f, 0.0f);

		for (int i = 0; i < ear->size(); i++) {
			if (i + 1 == ear->size()) {
				paths->push_back(Path(data_x[ear->at(i)], data_y[ear->at(i)], data_x[ear->at(0)], data_y[ear->at(0)], color));
				break;
			}
			paths->push_back(Path(data_x[ear->at(i)], data_y[ear->at(i)], data_x[ear->at(i + 1)], data_y[ear->at(i + 1)], color));
		}
		invalidate();


		std::cin.ignore();
		if (clear_again) {
			for (int i = 0; i < ear->size(); i++) {
				paths->pop_back();
			}
			invalidate();
		}
		else {
			scribbled_paths += ear->size();
		}
	}
}

void local_visualizer::invalidate(void) {
	render_callback(nullptr);
	renderer->render();
}

void elliptical_approx::init(std::vector<Node>* node_list, int width, int height, RenderClass* renderer,
	std::vector<Path>* paths, void (*func_updater)(std::vector<float>*)) {
	this->node_list = node_list;
	this->GRID_WIDTH = width;
	this->GRID_HEIGHT = height;
	this->render_agent = local_visualizer(GRID_WIDTH, GRID_HEIGHT, renderer, paths, func_updater);
	convex_clustering clusterer = convex_clustering(&render_agent);
	this->cluster = &clusterer;

	contour_extractor2();
	//locate_obstacles();

	//ellipse ellipse1 = ellipse(77.04f, 25.04f, 15.09f, 21.54f, 46.69 * PI / 180);
	//cluster->clustering_2(node_list, GRID_WIDTH);
	for (int i = 0; i < points.size(); i++) {
		cluster->clustering_3(node_list, GRID_WIDTH, points.at(i));
	}

	quad_builder builder = quad_builder(cluster->ellipse_list, &render_agent);
	map_builder = &builder;
}

void elliptical_approx::call_next_counter_clockwise(Node* boundary_cell, int dir, Node* stopping_node, bool forward) {
	Node* next_node = nullptr;
	switch (dir) {
	case 0:		next_node = boundary_cell->left;				break;
	case 1:		next_node = boundary_cell->left->bottom;		break;
	case 2:		next_node = boundary_cell->bottom;				break;
	case 3:		next_node = boundary_cell->bottom->right;		break;
	case 4:		next_node = boundary_cell->right;				break;
	case 5:		next_node = boundary_cell->right->top;			break;
	case 6:		next_node = boundary_cell->top;					break;
	case 7:		next_node = boundary_cell->top->left;			break;
	default:	next_node = nullptr;							break;
	}

	boundary_cell->boundary_left = next_node;
	next_node->boundary_right = boundary_cell;

	if (next_node->x == stopping_node->x && next_node->y == stopping_node->y) {
		//if (next_node == stopping_node) {
		std::cout << "breaking out...\n";
		return;
	}
	contour_builder_v2(next_node, true, false, dir, stopping_node);
}

void GL_to_image_coords_temp(int GRID_WIDTH, int GRID_HEIGHT, float x_GL, float y_GL, int* x_GL_int, int* y_GL_int) {
	float x_off = (float)GRID_WIDTH / 2;
	float y_off = (float)GRID_HEIGHT / 2;
	int scale = ((GRID_WIDTH) > (GRID_HEIGHT) ? (GRID_WIDTH) : (GRID_HEIGHT)) / 2;

	float x_image = x_GL * scale + x_off;
	float y_image = y_off - y_GL * scale;
	*x_GL_int = round(x_image * 100) / 100;
	*y_GL_int = round(y_image * 100) / 100;
}

void elliptical_approx::contour_builder_v2(Node* boundary_cell, bool search_left, bool search_right, int last_operation, Node* stopping_node) {
	if (boundary_cell == nullptr)
		return;
	// for a given end point; locate its left and right neighbour.
	bool bounds[8] = { false, false, false, false, false, false, false, false };
	if (boundary_cell->left == nullptr) {
		bounds[0] = true;
		bounds[1] = true;
	}
	else {
		bounds[0] = (boundary_cell->left->type == BASE_EMPTY);
		if (boundary_cell->left->bottom == nullptr)
			bounds[1] = true;
		else
			bounds[1] = (boundary_cell->left->bottom->type == BASE_EMPTY);
	}
	if (boundary_cell->bottom == nullptr) {
		bounds[2] = true;
		bounds[3] = true;
	}
	else {
		bounds[2] = (boundary_cell->bottom->type == BASE_EMPTY);
		if (boundary_cell->bottom->right == nullptr)
			bounds[3] = true;
		else
			bounds[3] = (boundary_cell->bottom->right->type == BASE_EMPTY);
	}
	if (boundary_cell->right == nullptr) {
		bounds[4] = true;
		bounds[5] = true;
	}
	else {
		bounds[4] = (boundary_cell->right->type == BASE_EMPTY);
		if (boundary_cell->right->top == nullptr)
			bounds[5] = true;
		else
			bounds[5] = (boundary_cell->right->top->type == BASE_EMPTY);
	}
	if (boundary_cell->top == nullptr) {
		bounds[6] = true;
		bounds[7] = true;
	}
	else {
		bounds[6] = (boundary_cell->top->type == BASE_EMPTY);
		if (boundary_cell->top->left == nullptr)
			bounds[7] = true;
		else
			bounds[7] = (boundary_cell->top->left->type == BASE_EMPTY);
	}

	// -- termination step --> if all neighbours are BASE_TAKEN. STOP!!!
	int first_boundary = -1;
	for (int i = 0; i < 8; i++) {
		if (bounds[i]) {
			first_boundary = i;
			break;
		}
		if (i == 7)
			return;
	}

	// -- First find the boundary direction (left/bottom-left/bottom/bottom-right/right/top-right/top/top-left).
	// first_boundary holds that info;

	// -- move anti-clockwise from there to spot neighbour
	// ** left neighbour : 
	if (search_left) {
		for (int i = 1; i < 8; i++) {
			if (bounds[(first_boundary + i) % 8] == false) {
				call_next_counter_clockwise(boundary_cell, (first_boundary + i) % 8, stopping_node, false);
				break;
			}
		}
	}
}

void elliptical_approx::boundary_pointers_network_to_points_list(Node* start_node, Node* next_node,int first_index, int poly_index) {
	//render_agent.add_path(Path(start_node, next_node));
	int ptx, pty;
	GL_to_image_coords_temp(GRID_WIDTH, GRID_HEIGHT, start_node->x, start_node->y, &ptx, &pty);
	points.at(poly_index).push_back(int_point(ptx, pty));

	while (true) {
		start_node = next_node;
		next_node = start_node->boundary_left;
		int ptx, pty;
		GL_to_image_coords_temp(GRID_WIDTH, GRID_HEIGHT, start_node->x, start_node->y, &ptx, &pty);
		points.at(poly_index).push_back(int_point(ptx, pty));
		if (next_node->x == node_list->at(first_index).x && next_node->y == node_list->at(first_index).y) {
			std::cout << "extraction_complete\n";
			break;
		}
	}
	render_agent.invalidate();

	// add these boundary paths to render queue
	for (int i = 0; i < points.at(poly_index).size(); i++) {
		render_agent.add_path(
			Path(
				points.at(poly_index).at(i % points.at(poly_index).size()).x,
				points.at(poly_index).at(i % points.at(poly_index).size()).y,
				points.at(poly_index).at((i + 1) % points.at(poly_index).size()).x,
				points.at(poly_index).at((i + 1) % points.at(poly_index).size()).y,
				Color(1.0, 1.0, 1.0)
			)
		);
	}
	render_agent.invalidate();
}

void elliptical_approx::contour_analyzer(int first_index, std::vector<std::vector<strip>>* strips) {
	render_agent.invalidate();
	// build boundary pointers network
	contour_builder_v2(&node_list->at(first_index), true, false, 0, &node_list->at(first_index));

	// add a new entry for this polygon
	points.push_back(std::vector<int_point>());
	int poly_index = points.size() - 1;

	Node* start_node = &node_list->at(first_index);
	Node* next_node = node_list->at(first_index).boundary_left;
	boundary_pointers_network_to_points_list(start_node, next_node, first_index, poly_index);

	// store this boundary point data with "y" as key and "x" as field
	std::vector<std::vector<int>> edges;
	for (int i = 0; i < GRID_HEIGHT; i++) {
		edges.push_back(std::vector<int>());
	}
	for (int i = 0; i < points.at(poly_index).size(); i++) {
		int_point cell_point = points.at(poly_index).at(i);
		int cell_x = cell_point.x;
		int cell_y = cell_point.y;
		
		edges.at(cell_y).push_back(cell_x);
	}

	// for each "y" key in edges. sort "x" values and trim out unnecessary ones
	for (int i = 0; i<edges.size(); i++) {
		if (edges.at(i).empty()) {
			std::cout << "x x x x x\n";
			continue;
		}
		for (int j = 0; j < edges.at(i).size(); j++) {
			int biggest = edges.at(i).at(j);
			int index = j;
			for (int k = j; k < edges.at(i).size(); k++) {
				if (edges.at(i).at(k) > biggest) {
					biggest = edges.at(i).at(k);
					index = k;
				}
			}
			//std::cout << biggest << " ";
			edges.at(i).erase(edges.at(i).begin() + index);
			edges.at(i).insert(edges.at(i).begin(), biggest);
		}
		for (int j = 0; j < edges.at(i).size(); j++) {
			Node node = node_list->at(i * GRID_WIDTH + edges.at(i).at(j));
			//std::cout << node.left->type << "   " << node.right->type << "\n";
			bool flag_left_taken = false;
			bool flag_right_taken = false;
			if (node.left != nullptr)
				flag_left_taken = (node.left->type == BASE_TAKEN);
			if (node.right != nullptr)
				flag_right_taken = (node.right->type == BASE_TAKEN);

			if (flag_left_taken && flag_right_taken) {
				edges.at(i).erase(edges.at(i).begin() + j);
				j--;
			}
		}
		for (int j = 0; j < edges.at(i).size(); j++) {
			std::cout << edges.at(i).at(j) << " ";
		}
		std::cout << "\n";
	}

	// append these edge info to strips list
	for (int i = 0; i < edges.size(); i++) {

		int TRAVERSALS = edges.at(i).size();
		for (int j = 0; j < TRAVERSALS; j++) {
			if (edges.at(i).empty())
				continue;
			int start = edges.at(i).at(0);
			int end = edges.at(i).at(1);
			for (int k = 0; k < strips->at(i).size(); k++) {
				if (end < strips->at(i).at(k).start) {
					strips->at(i).insert(strips->at(i).begin() + k, strip(start, end));
					edges.at(i).erase(edges.at(i).begin());
					edges.at(i).erase(edges.at(i).begin());
					break;
				}
			}
			if (!edges.at(i).empty()) {
				if (edges.at(i).at(0) == start) {
					strips->at(i).push_back(strip(start, end));
					edges.at(i).erase(edges.at(i).begin());
					edges.at(i).erase(edges.at(i).begin());
					continue;
				}
			}
		}
	}
}

void elliptical_approx::contour_extractor2(void) {
	int first_index = -1000;
	std::vector<std::vector<strip>> strips;
	for (int i = 0; i < GRID_HEIGHT; i++) {
		strips.push_back({});
	}
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			bool continue_flag = false;

			for (int k = 0; k < strips.at(i).size(); k++) {
				if (strips.at(i).at(k).start <= j && strips.at(i).at(k).end >= j) {
					j = strips.at(i).at(k).end;
					continue_flag = true;
					break;
				}
			}
			if (continue_flag) {
				continue;
			}

			if (node_list->at(i * GRID_WIDTH + j).type == BASE_TAKEN) {
				first_index = i * GRID_WIDTH + j;
				contour_analyzer(first_index, &strips);
			}
		}
	}
}

int elliptical_approx::contour_explorer(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int start, int pass) {
	if (pass > 4)
		return 5;
	// left -> 1
	// bottom -> 2
	// right -> 3
	// top -> 4

	switch (start) {
	case 1:
		if (node->left->type == START) {
			render_agent.add_path(Path(node, node->left));
			std::cout << "New vertice : (" << (this_node_index - 1) % GRID_WIDTH << ", " << (this_node_index - 1) / GRID_WIDTH << ")\n";
			if (build_contour(node->left, travel_list, this_node_index - 1, remaining_nodes, start-1))
				return 1;
		}
		break;
	case 2:
		if (node->bottom->type == START) {
			render_agent.add_path(Path(node, node->bottom));
			std::cout << "New vertice : (" << (this_node_index + GRID_WIDTH) % GRID_WIDTH << ", " << (this_node_index + GRID_WIDTH) / GRID_WIDTH << ")\n";
			if (build_contour(node->bottom, travel_list, this_node_index + GRID_WIDTH, remaining_nodes, start-1))
				return 2;
		}
		break;
	case 3:
		if (node->right->type == START) {
			render_agent.add_path(Path(node, node->right));
			std::cout << "New vertice : (" << (this_node_index + 1) % GRID_WIDTH << ", " << (this_node_index + 1) / GRID_WIDTH << ")\n";
			if (build_contour(node->right, travel_list, this_node_index + 1, remaining_nodes, start-1))
				return 3;
		}
		break;
	case 4:
		if (node->top->type == START) {
			render_agent.add_path(Path(node, node->top));
			std::cout << "New vertice : (" << (this_node_index - GRID_WIDTH) % GRID_WIDTH << ", " << (this_node_index - GRID_WIDTH) / GRID_WIDTH << ")\n";
			if (build_contour(node->top, travel_list, this_node_index - GRID_WIDTH, remaining_nodes, start-1))
				return 4;
		}
		break;
	default:
		break;
	}
	contour_explorer(node, travel_list, this_node_index, remaining_nodes, (start + 1) % 4, pass + 1);
	return 0;
}

bool elliptical_approx::build_contour(Node* node, bool* travel_list, int this_node_index, int remaining_nodes, int starting_param) {
	if (remaining_nodes <= 0) {
		std::cout << "Target achieved hence exiting \n";
		return false;
	}
	if (node == nullptr) {
		std::cout << "Nullptr. Hence terminating \n";
		return false;
	}
	if (this_node_index < 0 || this_node_index >= node_list->size()) {
		std::cout << "Out of Bounds. Hence preventing \n";
		return false;
	}
	if (travel_list[this_node_index]) {
		std::cout << "Already explored. Stopping\n";
		return false;
	}
	travel_list[this_node_index] = true;
	remaining_nodes -= 1;

	std::cout << " self.type : " << node->type << " left : " << node->left->type << 
		" bottom : " << node->bottom->type << " right : " << node->right->type << " top : " << node->top->type << "\n";

	render_agent.invalidate();
	std::cin.ignore();

	/*if (contour_explorer(node, travel_list, this_node_index, remaining_nodes, 1, 1))
		return true;*/
	contour_explorer(node, travel_list, this_node_index, remaining_nodes, starting_param, 1);
	return true;
}

double elliptical_approx::dist(double x1, double y1, double x2, double y2)
{
	double x = x2 - x1;
	double y = y2 - y1;

	return (x * x) + (y * y);
}

Point elliptical_approx::ellipseParametric(double x0, double y0, double a, double b, double alpha, double t) {
	Point pt;
	pt.x = x0 + (a * cos(t * PI / 180) * cos(alpha * PI / 180) - b * sin(t * PI / 180) * sin(alpha * PI / 180));
	pt.y = y0 + (a * cos(t * PI / 180) * sin(alpha * PI / 180) + b * sin(t * PI / 180) * cos(alpha * PI / 180));
	return pt;
}

bool convex_clustering::leftOnly(float line_x1, float line_y1,
	float line_x2, float line_y2, float pt_x0, float pt_y0) {
	/*std::cout << "[LEFT ONLY] : [" << line_x1 << ", " << line_y1 << ", " << line_x2 << ", " <<
		line_y2 << ", " << pt_x0 << ", " << pt_y0 << "] result = " << ((line_x1 * (line_y2 - pt_y0)
			+ line_x2 * (pt_y0 - line_y1) + pt_x0 * (line_y1 - line_y2)) > 0) << "\n";*/
	return ((line_x1 * (line_y2 - pt_y0) + line_x2 * (pt_y0 - line_y1) + pt_x0 * (line_y1 - line_y2)) > 0);
}

bool convex_clustering::leftOn(float line_x1, float line_y1,
	float line_x2, float line_y2, float pt_x0, float pt_y0) {
	return ((line_x1 * (line_y2 - pt_y0) + line_x2 * (pt_y0 - line_y1) + pt_x0 * (line_y1 - line_y2)) >= 0);
}

float area_triangle(int x1, int y1, int x2, int y2, int x3, int y3) {
	float area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
	//printf("x1 :  %d. y1 = %d. x2 = %d. y2 = %d. x3 = %d. y3 = %d. Area = %f\n", x1, y1, x2, y2, x3, y3, area);
	return area;
}

float length(int x1, int y1, int x2, int y2) {
	float len = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	//std::cout << len << "\n";
	return len;
}

float convex_clustering::ear_area_2(std::vector<int>* ear, std::vector<result>* filter, bool special) {
	float area = 0;
	bool pure = false;
	for (int i = 2; i < ear->size(); i++) {
		float del_area = area_triangle(points.at(ear->at(0)).x, points.at(ear->at(0)).y,
			points.at(ear->at(i - 1)).x, points.at(ear->at(i - 1)).y,
			points.at(ear->at(i)).x, points.at(ear->at(i)).y);
		area += del_area;
		if (area / del_area > 0)
			pure = true;
		else
			pure = false;
	}

	float bridge_width = length(points.at(ear->at(0)).x, points.at(ear->at(0)).y,
		points.at(ear->at(ear->size() - 1)).x, points.at(ear->at(ear->size() - 1)).y);
	float independency_factor = area / bridge_width;
	float skewness = independency_factor / bridge_width;

	if (independency_factor > 0.3 || special) {
		printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n",
			area, independency_factor, ear->size(), bridge_width, skewness);
		filter->push_back(result(ear->at(0), area, independency_factor, ear->size(), bridge_width, skewness));
		render_agent->visualize_ear_2(&points, ear, false);
	}
	else
		render_agent->visualize_ear_2(&points, ear, true);

	return independency_factor;
}

float ear_area(std::vector<int>* ear, int data_x[], int data_y[], local_visualizer* render_agent, std::vector<result>* filter, bool special) {
	float area = 0;
	bool pure = false;
	for (int i = 2; i < ear->size(); i++) {
		float del_area = area_triangle(data_x[ear->at(0)], data_y[ear->at(0)], data_x[ear->at(i - 1)], data_y[ear->at(i - 1)], data_x[ear->at(i)], data_y[ear->at(i)]);
		area += del_area;
		if (area / del_area > 0)
			pure = true;
		else
			pure = false;
	}
	float bridge_width = length(data_x[ear->at(0)], data_y[ear->at(0)], data_x[ear->at(ear->size() - 1)], data_y[ear->at(ear->size() - 1)]);
	float independency_factor = area / bridge_width;
	// print(independency_factor
	float skewness = independency_factor / bridge_width;
	//printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n", 
		//area, independency_factor, ear->size(), bridge_width, skewness);

	/*if (abs(independency_factor) > 7.0)
		render_agent->visualize_ear(data_x, data_y, ear, false);
	else
		render_agent->visualize_ear(data_x, data_y, ear, true);*/
		/*if (area > 0) {
			render_agent->visualize_ear(data_x, data_y, ear, false);
			printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n",
				area, independency_factor, ear->size(), bridge_width, skewness);
			filter->push_back(ear->at(0));
		}*/
	if (independency_factor > 0.1 || special) {
		render_agent->visualize_ear(data_x, data_y, ear, false);
		printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n",
			area, independency_factor, ear->size(), bridge_width, skewness);
		filter->push_back(result(ear->at(0), area, independency_factor, ear->size(), bridge_width, skewness));
	}
	else
		render_agent->visualize_ear(data_x, data_y, ear, true);

	//int label;
	//scanf("%d", &label);
	//results.push_back(result(area, independency_factor, ear->size(), bridge_width, skewness, pure, label));

	return independency_factor;
}

void convex_clustering::ear_handler(int edge1, int edge2, int pt, std::vector<std::vector<int>>* poly_indice) {
	std::cout << "Concave vertex identified : node " << poly_indice->at(0).at(edge2) << ". edge1 = " << poly_indice->at(0).at(edge1) << ", pt = " << poly_indice->at(0).at(pt) << "\n";
	// ear clipping logic here
	int start = edge2;
	int end = edge2;
	for (int i = 1; i < poly_indice->at(0).size(); i++) { // for a selected line segment, iterate through next coming vertices
		bool convex_test = leftOnly(
			data_x[poly_indice->at(0).at((edge1 + i) % poly_indice->at(0).size())],
			data_y[poly_indice->at(0).at((edge1 + i) % poly_indice->at(0).size())],
			data_x[poly_indice->at(0).at((edge2 + i) % poly_indice->at(0).size())],
			data_y[poly_indice->at(0).at((edge2 + i) % poly_indice->at(0).size())],
			data_x[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())],
			data_y[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())]);
		if (!convex_test) {
			std::cout<<"[EAR] : convex vertex spotted : node "<<poly_indice->at(0).at((edge2+i)% poly_indice->at(0).size())<<"\n";
			bool concave_ear = leftOnly(
				data_x[poly_indice->at(0).at(edge2)], data_y[poly_indice->at(0).at(edge2)],
				data_x[poly_indice->at(0).at(pt)], data_y[poly_indice->at(0).at(pt)],
				data_x[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())],
				data_y[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())]);
			if (concave_ear) {
				//if ((edge2+i-1)%len(poly_indice[0]) > start):
				//   end = (edge2 + i - 1) % len(poly_indice[0])
				end = (pt + i - 1) % poly_indice->at(0).size();
				std::cout<<"preventing ear from getting concave. start = "<<start<<". end = "<<end<<"\n";
				break;
			}
		}
		else {
			printf("[EAR] : concave vertex indicates termination of ear here.\n");
			end = (edge2 + i) % poly_indice->at(0).size();
			break;
		}
	}

	if (start != end) { // separate out identified ear from the raw polygon
		std::cout<<"start : "<<start<<".end : "<<end<<"\n";
		std::vector<int> ear;
		if (end < start)
			end += poly_indice->at(0).size();
		for (int i = start; i < end + 1; i++) // copy the ear points to a new list
			ear.push_back(poly_indice->at(0).at(i % poly_indice->at(0).size()));
		if (ear.size() < 3)
			return; // that is either a point or line. and not ear
		/*float factor = ear_area(&ear, data_x, data_y, render_agent);
		if (factor < 2.0) {
			return;
		}*/
		/*a, b, c, d = ear_area(ear, data_x, data_y)
		print(f"area = {a:.2f}. factor = {b:.2f}. vertices = {c}. bridge_width = {d:.2f}")
		if (b < 1.0 or (d / b > 15)) :
			return*/
		poly_indice->push_back(ear);          // append these ear coordinates to polygon dataset
		for (int i = start + 1; i < end; i++) {
			if (start + 1 >= poly_indice->at(0).size()) // remove the excess ear coordinates from the raw polygon points set
				poly_indice->at(0).erase(poly_indice->at(0).begin() + 0);
			else
				poly_indice->at(0).erase(poly_indice->at(0).begin() + start + 1); // remove the excess ear coordinates from the raw polygon points set
		}
	}
}

int convex_clustering::convex_completion_handler(int pt, int edge2, std::vector<std::vector<int>>* poly_indice) {
	bool convex_completion_test = leftOnly(data_x[poly_indice->at(0).at(1)], data_y[poly_indice->at(0).at(1)],
		data_x[poly_indice->at(0).at(0)], data_y[poly_indice->at(0).at(0)],
		data_x[poly_indice->at(0).at(pt)], data_y[poly_indice->at(0).at(pt)]);

	if (!convex_completion_test) {
		// if (pt == 0):        // break loop on success
		//   print("Computation finished == :)")
		//   return 0 # terminate any computation ahead
		if ((edge2 == poly_indice->at(0).size()) || (poly_indice->at(0).at(edge2) == 1))
			return 1;
		std::cout<<"[CONVEX COMPLETION] : triggered. node 0 will be joined with node "<<poly_indice->at(0)[edge2]<<"\n";
		std::vector<int> ear;
		for (int i = 0; i < pt; i++) // copy the ear points to a new list
			ear.push_back(poly_indice->at(0).at(i % poly_indice->at(0).size()));
		poly_indice->push_back(ear);               // append these ear coordinates to polygon dataset
		for (int i = 1; i < pt - 1; i++)
			poly_indice->at(0).erase(poly_indice->at(0).begin() + 1);       // remove the excess ear coordinates from the raw polygon points set
			// circulation_start = edge1
			return 1; // continue with the coming steps
  }
}

void print_poly_indice(std::vector<std::vector<int>>* poly_indice) {
		//-----------
	std::cout << "[\n";
	for (int i = 0; i < poly_indice->size(); i++) {
		std::cout << "[";
		for (int j = 0; j < poly_indice->at(i).size(); j++)
			std::cout << poly_indice->at(i).at(j) << " , ";
		std::cout << "\b\b]\n";
	}
	std::cout << "]\n";
	//-----------
}

void convex_clustering::clustering(void) {
	std::vector<std::vector<int>> poly_indice = { {} };
	for (int i = 0; i < 107; i++) {
		poly_indice.at(0).push_back(i);
	}
	std::cout << "Enters here \n";
	int index = 0;
	int circulation_start = poly_indice.at(0).size() - 1;
	//while (true) { // iterate continously to pick vertices on polygon one-by-one
	for (int k=0; k<1000; k++){ // iterate continously to pick vertices on polygon one-by-one
		//std::cout << "length of raw data : " << poly_indice.at(0).size() << ".Index : " << index << " \n";
		int edge1 = (index) % poly_indice.at(0).size();
		int edge2 = (index + 1) % poly_indice.at(0).size();
		int pt = (index + 2) % poly_indice.at(0).size();

		//print_poly_indice(&poly_indice);

		if (edge1 == circulation_start) { // break loop on success
			printf("Computation finished :)\n");
			break;
		}

		bool convex_completion = convex_completion_handler(pt, edge2, &poly_indice);
		if (convex_completion == 0) {
			break;
			if (convex_completion == 1) {
				index = 0;
				continue;
			}
		}

		bool concave_test = leftOnly(data_x[poly_indice[0][edge1]], data_y[poly_indice[0][edge1]],
			data_x[poly_indice[0][edge2]], data_y[poly_indice[0][edge2]],
			data_x[poly_indice[0][pt]], data_y[poly_indice[0][pt]]);
			if (concave_test) {
				ear_handler(edge1, edge2, pt, &poly_indice);
				circulation_start = edge1;
			}
			else {
				std::cout<<"convex vertex : node "<<poly_indice[0][edge2]<<".edge1 = "<<poly_indice[0][edge1]<<", pt = "<<poly_indice[0][pt]<<"\n";
			}
				index = (index + 1) % poly_indice.at(0).size();
			// plot_current_distribution()
	}

	print_poly_indice(&poly_indice);
	render_agent->draw_ears(data_x, data_y, &poly_indice);
}

void convex_clustering::filter_dimples_2(std::vector<result>* traversal, std::vector<result>* filter, int filter_size) {
	std::cout << " length = " << data_size << "\n";
	for (int i = 0; i < traversal->size(); i++) {
		std::vector<int> ear;
		for (int k = 0; k < filter_size; k++) {
			//std::cout << ((traversal->at(i).i + k) % data_size) << "\n";
			ear.push_back((traversal->at(i).i + k) % data_size);
		}
		ear_area_2(&ear, filter, false);
	}
	//std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	//std::cin.ignore();
}

void convex_clustering::filter_dimples(std::vector<result>* traversal, std::vector<result>* filter, int filter_size) {
	std::cout << " length = " << data_size << "\n";
	for (int i = 0; i < traversal->size(); i++) {
		std::vector<int> ear;
		for (int k = 0; k < filter_size; k++) {
			//std::cout << ((traversal->at(i).i + k) % data_size) << "\n";
			ear.push_back((traversal->at(i).i + k) % data_size);
		}
		ear_area(&ear, data_x, data_y, render_agent, filter, false);
	}
	std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	std::cin.ignore();
}

void convex_clustering::cleaning_dimples(std::vector<result>* dimples) {
	int PARAM_VICINITY_RESOLUTION = 3;
	for (int i = 0; i < dimples->size(); i++) {
		int val1 = dimples->at(i % dimples->size()).i;
		int val2 = dimples->at((i + 1) % dimples->size()).i;
		if (val2 < val1)
			val2 += data_size;
		for (int j = 1; j < dimples->size(); j++) {
			int val2 = dimples->at((i + j) % dimples->size()).i;
			if (val2 < val1)
				val2 += data_size;
			if (val2 - val1 > PARAM_VICINITY_RESOLUTION)
				break;
			else {
				if (dimples->at(i % dimples->size()).independence < dimples->at((i + j) % dimples->size()).independence) {
					dimples->erase(dimples->begin() + i % dimples->size());
				}
				else {
					dimples->erase(dimples->begin() + (i + j) % dimples->size());
				}
				j--;
			}
		}
	}
}

void convex_clustering::clustrify_v2(std::vector<result>* dimples, std::vector<std::vector<int>>* clustered_data) {
	int PARAM_CLUSTER_FACTOR_THRESHOLD = 10;
	int PARAM_FILTER_SIZE_CENTER_OFFSET_VAL = 2;
	std::vector<result> results;

	std::cout << "Following pairs will be joinined \n";
	for (int i = 0; i < dimples->size(); i++) {
		std::vector<int> ear;
		int start = (dimples->at((i) % dimples->size()).i + PARAM_FILTER_SIZE_CENTER_OFFSET_VAL)%data_size;
		int end = (dimples->at((i + 1) % dimples->size()).i + PARAM_FILTER_SIZE_CENTER_OFFSET_VAL) % data_size;
		if (end < start)
			end += data_size;
		for (int k = start; k <= end; k++) {
			//std::cout << ((k) % data_size) << "\n";
			ear.push_back((k) % data_size);
		}
		ear_area_2(&ear, &results, true);
		if (abs(results.at(results.size() - 1).independence) > PARAM_CLUSTER_FACTOR_THRESHOLD) {
			std::cout << start % data_size << " will be joinined with " << end % data_size << "\n";
			// add new poly-loop to poly-indices
			clustered_data->push_back({});
			int index = clustered_data->size() - 1;
			for (int i = start; i <= end; i++) {
				clustered_data->at(index).push_back(i % data_size);
			}
			print_poly_indice(clustered_data);
			// cut that part from original poly
			for (int k = 0; k < clustered_data->at(0).size(); k++) {
				if (clustered_data->at(0).at(k) == start + 1) {
					for (int i = start + 1; i < end; i++) {
						if (k < clustered_data->at(0).size())
							clustered_data->at(0).erase(clustered_data->at(0).begin() + k);
						else
							clustered_data->at(0).erase(clustered_data->at(0).begin());
					}
					break;
				}
			}
		}
	}
}

void convex_clustering::clustrify(std::vector<result>* dimples, std::vector<std::vector<int>>* clustered_data) {
	int PARAM_CLUSTER_FACTOR_THRESHOLD = 10;
	std::vector<result> results;


	std::cout << "Following pairs will be joinined \n";
	for (int i = 0; i < dimples->size(); i++) {
		std::vector<int> ear;
		int start = dimples->at(i).i + 1;
		int end = dimples->at((i + 1) % dimples->size()).i + 1;
		if (end < start)
			end += data_size;
		for (int k = start; k <= end; k++) {
			//std::cout << ((k) % data_size) << "\n";
			ear.push_back((k) % data_size);
		}
		ear_area(&ear, data_x, data_y, render_agent, &results, true);
		if (abs(results.at(results.size() - 1).independence) > PARAM_CLUSTER_FACTOR_THRESHOLD) {
			std::cout << start % data_size << " will be joinined with " << end % data_size << "\n";
			// add new poly-loop to poly-indices
			clustered_data->push_back({});
			int index = clustered_data->size() - 1;
			for (int i = start; i <= end; i++) {
				clustered_data->at(index).push_back(i % data_size);
			}
			print_poly_indice(clustered_data);
			// cut that part from original poly
			for (int k = 0; k < clustered_data->at(0).size(); k++) {
				if (clustered_data->at(0).at(k) == start + 1) {
					for (int i = start + 1; i < end; i++) {
						if (k < clustered_data->at(0).size())
							clustered_data->at(0).erase(clustered_data->at(0).begin() + k);
						else
							clustered_data->at(0).erase(clustered_data->at(0).begin());
					}
					break;
				}
			}
		}
	}
}

void convex_clustering::clustering_3(std::vector<Node>* node_list, int GRID_WIDTH, std::vector<int_point> points) {
	this->points = points;
	std::vector<result> initial_list;
	data_size = points.size();
	for (int i = 0; i < data_size; i++) {
		initial_list.push_back(result(i));
	}

	//locating dimples
	std::vector<result> filtered;

	// setting up iterators for vertices list
	std::vector<result>* old_vertices_set, * updated_vertices_set;
	old_vertices_set = &initial_list;
	updated_vertices_set = &filtered;

	int filters[4] = {8, 5, 3 };

	for (int i = 0; i < 2; i++) {
		filter_dimples_2(old_vertices_set, updated_vertices_set, filters[i]);
		
		// transfer the new content to old list
		std::vector<result>* temp = old_vertices_set;
		old_vertices_set = updated_vertices_set;
		updated_vertices_set = temp;
		updated_vertices_set->clear();
	}

	for (int i = 0; i < old_vertices_set->size(); i++) {
		std::cout << old_vertices_set->at(i).i << "     " << old_vertices_set->at(i).area << "     " << old_vertices_set->at(i).independence << "\n";
	}

	// extracting good dimples
	for (int i = 0; i < old_vertices_set->size(); i++) {
		if (old_vertices_set->at((i + 1) % old_vertices_set->size()).i - old_vertices_set->at(i).i == 1) {
			// new set of interest zone
			std::cout << "interest zone starts at : " << i;
			float max_factor = old_vertices_set->at(i).independence;
			int best_index = i;
			for (int j = 1; j < old_vertices_set->size(); j++) {
				int i_th_index = old_vertices_set->at((i + j) % old_vertices_set->size()).i;
				int last_index = old_vertices_set->at((i + j-1) % old_vertices_set->size()).i;
				if (i_th_index < last_index)
					i_th_index += data_size;

				if (i_th_index - last_index > 1) {
					// end of that set
					updated_vertices_set->push_back(old_vertices_set->at(best_index));
					std::cout << " best performer was : " << old_vertices_set->at(best_index).i << ". factor = " << max_factor << "\n";
					i = (i + j) - 1;
					break;
				}
				if (old_vertices_set->at((i + j) % old_vertices_set->size()).independence > max_factor) {
					max_factor = old_vertices_set->at((i + j) % old_vertices_set->size()).independence;
					best_index = (i + j) % old_vertices_set->size();
				}
			}
		}
	}
	std::vector<result>* temp = old_vertices_set;
	old_vertices_set = updated_vertices_set;
	updated_vertices_set = temp;
	updated_vertices_set->clear();
	
	//highlights extracted dimples
	for (int i = 0; i < old_vertices_set->size(); i++) {
		std::cout << old_vertices_set->at(i).i + 2 << "\n";
		node_list->at(points[old_vertices_set->at(i).i + 2].y * GRID_WIDTH + points[old_vertices_set->at(i).i + 2].x).type = START;
	}

	// pair dimples
	std::vector<std::vector<int>> clustered_data = { {} };
	for (int i = 0; i < data_size; i++) {
		clustered_data.at(0).push_back(i);
	}
	clustrify_v2(old_vertices_set, &clustered_data);

	std::cout << "final outcome\n";
	//print_poly_indice(&clustered_data);

	for (int i = 0; i < clustered_data.size(); i++) {
		clustered_points.push_back(std::vector<int_point>());
		int last_index = clustered_points.size()-1;
		for (int j = 0; j < clustered_data.at(i).size(); j++) {
			clustered_points.at(last_index).push_back(points.at(clustered_data.at(i).at(j)));
		}
	}
	/*//-----------
	std::cout << "[\n";
	for (int i = 0; i < clustered_points.size(); i++) {
		std::cout << "[";
		for (int j = 0; j < clustered_points.at(i).size(); j++)
			std::cout << "[" << clustered_points.at(i).at(j).x << ", " << clustered_points.at(i).at(j).y << "] , ";
		std::cout << "\b\b]\n";
	}
	std::cout << "]\n";
	//-----------


	std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	std::cin.ignore();*/

	render_agent->draw_ears_v2(&points, &clustered_data);

	for (int i = 0; i < clustered_data.size(); i++) {
		ellipse_fitter(clustered_points.size() - 1 - i);
	}
}

void convex_clustering::clustering_2(std::vector<Node>* node_list, int GRID_WIDTH) {
	std::vector<result> initial_list;
	for (int i = 0; i < data_size; i++) {
		initial_list.push_back(result(i));
	}

	//locating dimples
	std::vector<result> filtered;
	filter_dimples(&initial_list, &filtered, 7);

	std::vector<result> second_filter;
	filter_dimples(&filtered, &second_filter, 5);

	std::vector<result> third_filter;
	filter_dimples(&second_filter, &third_filter, 4);

	std::vector<result> fourth_filter;
	filter_dimples(&third_filter, &fourth_filter, 3);

	/*std::vector<result> fourth_filter;
	filter_dimples(&second_filter, &fourth_filter, 3);*/

	// extracting good dimples
	cleaning_dimples(&fourth_filter);

	//highlights extracted dimples
	for (int i = 0; i < fourth_filter.size(); i++) {
		std::cout << fourth_filter[i].i + 1 << "\n";
		node_list->at(data_y[fourth_filter[i].i + 1] * GRID_WIDTH + data_x[fourth_filter[i].i + 1]).type = START;
	}

	// pair dimples
	std::vector<std::vector<int>> clustered_data = { {} };
	for (int i = 0; i < data_size; i++) {
		clustered_data.at(0).push_back(i);
	}
	clustrify(&fourth_filter, &clustered_data);

	std::cout << "final outcome\n";
	print_poly_indice(&clustered_data);


	std::cin.ignore();
	std::cout << "Clearing out paths\n";
	render_agent->clear_path();
	std::cin.ignore();

	render_agent->draw_ears(data_x, data_y, &clustered_data);
}

void convex_clustering::ellipse_fitter(int poly_index) {
	std::vector<int_point> data = clustered_points.at(poly_index);
	// Generate some sample boundary points
	Eigen::MatrixXd X(data.size(), 5);
	for (int i = 0; i < data.size(); i++) {
		/*double angle = 2 * M_PI * i / 100;
		X(i, 0) = 3 * cos(angle);
		X(i, 1) = 2 * sin(angle);*/
		X(i, 0) = data.at(i).x * data.at(i).x;
		X(i, 1) = data.at(i).x * data.at(i).y;
		X(i, 2) = data.at(i).y * data.at(i).y;
		X(i, 3) = data.at(i).x;
		X(i, 4) = data.at(i).y;
	}

	// Create the Y vector with the values of 1 for each boundary point
	Eigen::VectorXd Y(data.size());
	Y.fill(-1);

	// Solve the system of linear equations X * p = Y using np.linalg.lstsq
	Eigen::VectorXd p = X.colPivHouseholderQr().solve(Y);

	// Extract the parameters of the ellipse from the vector p
	std::cout << " p(0) " << p(0) << std::endl;
	std::cout << " p(1) " << p(1) << std::endl;
	std::cout << " p(2) " << p(2) << std::endl;
	std::cout << " p(3) " << p(3) << std::endl;
	std::cout << " p(4) " << p(4) << std::endl;

	double dem = p(1) * p(1) - 4 * p(0) * p(2);			// dem = b*b - 4ac
	double x0 = (2 * p(2) * p(3) - p(1) * p(4)) / dem;	// x0 = (2cd - be)/dem
	double y0 = (2 * p(0) * p(4) - p(1) * p(3)) / dem;	// y0 = (2ae - bd) / dem
	double param = std::pow((std::pow((p(0) - p(2)), 2.0) + p(1) * p(1)), 0.5);	
														// param = ((a-c)^2.0 + b*b) ^0.5
	double axi_max = -(std::pow((2 * (p(0) * p(4) * p(4) + p(2) * p(3) * p(3) - p(1) * p(3) * p(4) + dem)
		* ((p(0) + p(2)) + param)), 0.5)) / dem;
				// axi_max = -((2(ae*e + cd*d - bde + dem)*((a + c) + param))^0.5) / dem
	double axi_min = -(std::pow((2 * (p(0) * p(4) * p(4) + p(2) * p(3) * p(3) - p(1) * p(3) * p(4) + dem)
		* ((p(0) + p(2)) - param)), 0.5)) / dem;
				// axi_min = -((2(ae*e + cd*d - bde + dem)*((a + c) - param))^0.5) / dem
	
	double theta = std::atan2(p(1) , (p(2) - p(0) - param));
												// theta = atan2(b, c-a-param) * 180 / PI
	std::cout << "dem = " << dem << ", x0 = " << x0 << ", y0 = " << y0 << ", axi_max = " << axi_max << ", axi_min = " << axi_min << ", theta = " << (theta*57.2958) << std::endl;
	if (axi_max < 0) {
		std::cout << "[POSSIBLE DELETION] : axi_max negative\n";
		return;
	}
	if (axi_min < 0) {
		std::cout << "[POSSIBLE DELETION] : axi_min negative\n";
		return;
	}

	this->ellipse_list.push_back(ellipse(x0, y0, axi_max, axi_min, theta));
	render_agent->draw_ellipse(ellipse_list.at(ellipse_list.size()-1));

}

quad_builder::quad_builder(std::vector<ellipse> list, local_visualizer* render_agent) {
	obstacles = list;
	this->render_agent = render_agent;

	/*
	* Constructs the quad map from ellipses
	* Step I : find nearest neighbours first
	* Step II : compute the quad for each pair
	* Step III : look for intersection tests and create map from these quads
	*/

	// Step I : nearest neighbours
	for (int i = 0; i < obstacles.size(); i++) {
		obstacles.at(i).create_neighbour_map(&obstacles, render_agent);
	}

	// Step II : quad pair computation
	for (int i = 0; i < obstacles.size(); i++) {
		for (int j = 0; j < obstacles.at(i).neighbours.size(); j++) {
			for (int k = 0; k < quad_list.size(); k++) {
				if ((quad_list.at(k).me_A == &obstacles.at(i)) || (quad_list.at(k).me_B == &obstacles.at(i))) {
					continue;
				}
			}
			ellipse::neighbourSweep sweep = obstacles.at(i).neighbours.at(j);
			quad_list.push_back(quad(
				&obstacles.at(i),
				obstacles.at(i).neighbours.at(j).pointer,
				sweep.external_tangent1, 
				sweep.external_tangent2, 
				quad_list.size()));
			//render_agent->visualize_quads(&quad_list.at(quad_list.size()-1), true, true);
		}
	}

	// Step III : create quad map
	stich_quads();
}

void quad_builder::stich_quads(void) {
	/*
	* Step I : Keep Iterating through quad-list : we will add one quad per iteration 
				and *remove* it from the list -- need not remove as well
	* Step II : In each iteration : traverse through each quad in stitched map; if any 
				quad intersects the quad of interest. Add this quad to their list of neighbours
	* * For all this maintain a bi-directional tree and mechanism to traverse through it.
	*/
	std::vector<bool> checklist(quad_list.size(), false);
	quad_map = &quad_list.at(0);

	for (int i = 0; i < quad_list.size(); i++) {

		// add this quad to all the quads in the map that intersects with it.
		quad_map->map_expander(&quad_list.at(i), &checklist);
		render_agent->visualize_quads(&quad_list.at(i), false, false);
	}
}