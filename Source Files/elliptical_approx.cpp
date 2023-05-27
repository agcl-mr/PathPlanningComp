#include "../Header Files/elliptical_approx.h"

bool CompGeomFuncEllipseApprox::leftOn(float x1, float y1, float x2, float y2, float x0, float y0) {
	return ((x1 * (y2 - y0) + x2 * (y0 - y1) + x0 * (y1 - y2)) <= 0);
}

float CompGeomFuncEllipseApprox::leftPredicate(float x1, float y1, float x2, float y2, float x0, float y0) {
	return (x1 * (y2 - y0) + x2 * (y0 - y1) + x0 * (y1 - y2));
}

bool CompGeomFuncEllipseApprox::rightOn(float x1, float y1, float x2, float y2, float x0, float y0) {
	return ((x1 * (y2 - y0) + x2 * (y0 - y1) + x0 * (y1 - y2)) >= 0);
}

Point CompGeomFuncEllipseApprox::intersection_point(vector_segment* slicee_edge, vector_segment* slicer_edge) {
	float x1, y1, x2, y2, a1, b1, a2, b2;
	x1 = slicee_edge->x1;
	y1 = slicee_edge->y1;
	x2 = slicee_edge->x2;
	y2 = slicee_edge->y2;
	a1 = slicer_edge->x1;
	b1 = slicer_edge->y1;
	a2 = slicer_edge->x2;
	b2 = slicer_edge->y2;

	Point result;
	float denom1 = float((x2 - x1) * (b2 - b1) - (a2 - a1) * (y2 - y1));
	if (std::abs(denom1) < FLOATING_PRECISION) {
		if (show_logs)
			std::cout << "[WARNING] : Parallel lines. Assuming near mid-point as intersection point\n";
		result.x = (x1 + x2 + a1 + a2) / 4;
		result.y = (y1 + y2 + b1 + b2) / 4;
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
	float test1 = edge1->leftPredicate(edge2->x1, edge2->y1);
	if (test1 == 0)
		return true;
	float test2 = edge1->leftPredicate(edge2->x2, edge2->y2);
	if (test2 == 0)
		return true;

	if (test1 / test2 > 0)
		return false;

	float test3 = edge2->leftPredicate(edge1->x1, edge1->y1);
	if (test3 == 0)
		return true;
	float test4 = edge2->leftPredicate(edge1->x2, edge1->y2);
	if (test4 == 0)
		return true;

	if (test3 / test4 > 0)
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

float CompGeomFuncEllipseApprox::triangle_area(float x1, float y1, float x2, float y2, float x3, float y3) {
	return (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
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

float vector_segment::leftPredicate(float x3, float y3) {
	return CompGeomFuncEllipseApprox::leftPredicate(x1, y1, x2, y2, x3, y3);
}

bool vector_segment::rightOn(float x3, float y3) {
	return CompGeomFuncEllipseApprox::rightOn(x1, y1, x2, y2, x3, y3);
}

float vector_segment::subtended_angle_measure(float x3, float y3) {
	float a = length();
	if (a < FLOATING_PRECISION)
		return 0.0;
	float c = CompGeomFunc::distance(x1, y1, x3, y3);
	if (c < FLOATING_PRECISION)
		return PI;
	float b = CompGeomFunc::distance(x2, y2, x3, y3);
	if (b < FLOATING_PRECISION)
		return 0.0;
	float angle = std::acos((a * a + b * b - c * c) / (2 * a * b));
	return angle;
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

float vector_segment::normalized_dot_product(vector_segment* edge) {
	return dot_product(edge)/(edge->length()*this->length());
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

Point vector_segment::intersection_points(vector_segment* edge) {
	return CompGeomFuncEllipseApprox::intersection_point(this, edge);
}

quad::sibling::sibling(quad* neighbour, float area_measure, float length_measure,
	float length_measure2, Point ellip_center, Point farthest_point) {
	this->path = neighbour;
	this->area_measure = area_measure;
	this->length_measure = length_measure;
	this->length_measure2 = length_measure2;
	this->ellip_center = ellip_center;
	this->farthest_point = farthest_point;
}

float quad::sibling::importance_function1(void) {
	float importance;
	importance = area_measure;
	return importance;
}

bool quad::isIntersecting(quad* quad, local_visualizer* render_agent) {
	//render_agent->visualize_quads_intersection(this, quad);

	bool check;

	check = blocked1.isIntersecting(&quad->free1) && blocked1.isIntersecting(&quad->free2);
	if (check)
		return false;
	check = blocked1.isIntersecting(&quad->blocked1) && blocked1.isIntersecting(&quad->blocked2);
	if (check)
		return false;

	check = blocked2.isIntersecting(&quad->free1) && blocked2.isIntersecting(&quad->free2);
	if (check)
		return false;
	check = blocked2.isIntersecting(&quad->blocked1) && blocked2.isIntersecting(&quad->blocked2);
	if (check)
		return false;

	check = quad->blocked1.isIntersecting(&free1) && quad->blocked1.isIntersecting(&free2);
	if (check)
		return false;
	check = quad->blocked1.isIntersecting(&blocked1) && quad->blocked1.isIntersecting(&blocked2);
	if (check)
		return false;

	check = quad->blocked2.isIntersecting(&free1) && quad->blocked2.isIntersecting(&free2);
	if (check)
		return false;
	check = quad->blocked2.isIntersecting(&blocked1) && quad->blocked2.isIntersecting(&blocked2);
	if (check)
		return false;
	return true;
}

void quad::map_expander(quad* quad, std::vector<bool>* checklist, local_visualizer* render_agent) {
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
	for (int i = 0; i < neighbours_A.size(); i++) {
		if (checklist->at(neighbours_A.at(i).path->id))
			continue;
		neighbours_A.at(i).path->map_expander(quad, checklist, render_agent);
	}
	/*/
	// intersection code here!!
	if (isIntersecting(quad, render_agent))
		neighbours.push_back(sibling(quad, 0.0, 0.0));*/
}

void quad::estimate_ellipse_blockage(ellipse* ellip, vector_segment chord, float* area_reduction, 
	float* length_reduction) {
	float area = 0.0;

	// wrt ellipse frame of reference; order is {{a, 0}, {0, b}, {-a, 0}, {0, -b}}
	std::vector<std::vector<float>> quad = {
		{ellip->center_x + ellip->a * std::sin(ellip->tilt), ellip->center_y + ellip->a * std::cos(ellip->tilt)},
		{ellip->center_x + ellip->b * std::cos(ellip->tilt), ellip->center_y - ellip->b * std::sin(ellip->tilt)},
		{ellip->center_x - ellip->a * std::sin(ellip->tilt), ellip->center_y - ellip->a * std::cos(ellip->tilt)},
		{ellip->center_x - ellip->b * std::cos(ellip->tilt), ellip->center_y + ellip->b * std::sin(ellip->tilt)}
	};

	vector_segment major = vector_segment(quad[0][0], quad[0][1], quad[2][0], quad[2][1]);
	vector_segment minor = vector_segment(quad[1][0], quad[1][1], quad[3][0], quad[3][1]);
	// checking if chord is along minor axis of ellipse
	if (std::abs(major.normalized_dot_product(&chord)) < FLOATING_PRECISION) {
		//0.5*PI*a*b*(chord.length()/(2*b))
		area = 0.25 * PI * ellip->a * chord.length();
		*area_reduction = area;
		*length_reduction = area / chord.length();
		return;
	}
	// checking if chord is along major axis of ellipse
	if (std::abs(minor.normalized_dot_product(&chord)) < FLOATING_PRECISION) {
		//0.5*PI*a*b*(chord.length()/(2*a))
		area = 0.25 * PI * ellip->b * chord.length();
		*area_reduction = area;
		*length_reduction = area / chord.length();
		return;
	}

	if (chord.rightOn(ellip->center_x, ellip->center_y)) {
		area = (0.25 * PI * ellip->a * ellip->b - 0.5 * ellip->a * ellip->b) * (chord.length() / std::sqrt(ellip->a * ellip->a + ellip->b * ellip->b));
		*area_reduction = area;
		*length_reduction = area / chord.length();
		return;
	}
	else {
		area = (0.75 * PI * ellip->a * ellip->b + 0.5 * ellip->a * ellip->b) * (chord.length() / std::sqrt(ellip->a * ellip->a + ellip->b * ellip->b));
		*area_reduction = area;
		*length_reduction = area / chord.length();
		return;
	}

	return;
}

void quad::compute_params(void) {
	float area = 0.0, length = 0.0, del_area, del_length;

	area -= CompGeomFuncEllipseApprox::triangle_area(
		free1.x1, free1.y1, free2.x1, free2.y1, free1.x2, free1.y2);
	area -= CompGeomFuncEllipseApprox::triangle_area(
		free2.x1, free2.y1, free2.x2, free2.y2, free1.x2, free1.y2);

	length += CompGeomFuncEllipseApprox::distance((free1.x1 + free2.x1) / 2, (free1.y1 + free2.y1) / 2,
		(free1.x2 + free2.x2) / 2, (free1.y2 + free2.y2) / 2);

	estimate_ellipse_blockage(me_A, blocked1, &del_area, &del_length);
	area -= del_area;
	length -= del_length;

	estimate_ellipse_blockage(me_B, blocked2.invert(), &del_area, &del_length);
	area -= del_area;
	length -= del_length;

	this->mfa = area;
	this->mpl = length;
	this->mpw = area / length;
}

bool quad::clockwise_intersection_locator(std::vector<Point>* set_2, 
	vector_segment* edge1,int index_start_2, int* index_set2, Point* intersection_pt,
	bool propogation) {

	for (int i = 0; i < set_2->size(); i++) {
		if (propogation) {
			if (i == set_2->size() - 1)
				continue;
		}
		int index1 = (i + index_start_2 - 1) % set_2->size();
		int index2 = (i + index_start_2) % set_2->size();
		vector_segment edge2 = vector_segment(set_2->at(index1).x, set_2->at(index1).y,
			set_2->at(index2).x, set_2->at(index2).y);

		if (edge1->isIntersecting(&edge2)) {
			Point intersection = edge1->intersection_points(&edge2);
			*index_set2 = index2;
			*intersection_pt = intersection;
			return true;
		}
	}
	return false;
}

float quad::common_area(quad* quad, local_visualizer* render_agent) {
	/*
	* Step I : Iterate through sides of both quads in clockwise fashion until first
	*			intersection point is located. add this point to intersection polygon
	*			point set
	* 
	* Step II : To look for next point: check if the next vertex of one quad is on left
	*			to current edge of the other quad. Whichever satisfies this condition, 
	*			choose that edge
	* 
	* Step III : Keep continuing the hunt until point starts repeating
	*/
	//render_agent->visualize_quads_intersection(this, quad);
	float area = 0.0;

	std::vector<Point> set_1, set_2, common_region;

	set_1.push_back(Point(free1.x1, free1.y1));
	set_1.push_back(Point(free2.x1, free2.y1));
	set_1.push_back(Point(free2.x2, free2.y2));
	set_1.push_back(Point(free1.x2, free1.y2));

	set_2.push_back(Point(quad->free1.x1, quad->free1.y1));
	set_2.push_back(Point(quad->free2.x1, quad->free2.y1));
	set_2.push_back(Point(quad->free2.x2, quad->free2.y2));
	set_2.push_back(Point(quad->free1.x2, quad->free1.y2));

	Point intersection = Point();
	int index1 = 0, index2 = 0;

	// locate the first intersection
	for (int i = 0; i < set_1.size(); i++) {
		vector_segment edge1 = vector_segment(
			set_1.at((i - 1) % set_1.size()).x,
			set_1.at((i - 1) % set_1.size()).y,
			set_1.at(i).x,
			set_1.at(i).y
		);
		bool result = clockwise_intersection_locator(&set_2, &edge1, 0, &index2, &intersection, false);
		if (result) {
			common_region.push_back(intersection);
			index1 = i;
			break;
		}
	}

	if (common_region.empty()) {
		return 0.0;
	}

	// first intersection in located by this point
	bool flag_set1_vertice = false, flag_set2_vertice = false, success = false;
	while (true) {
		if (common_region.size() > 1) {
			/*
			* Only for propogation step
			* configures indexes and edge variable for new edge
			*/
			// finding the first intersection on new line
			vector_segment edge1;
			if (flag_set1_vertice) {
				index1++;
				edge1 = vector_segment(
					intersection.x,
					intersection.y,
					set_1.at(index1 % set_1.size()).x,
					set_1.at(index1 % set_1.size()).y
				);
				flag_set1_vertice = false;
				bool result = clockwise_intersection_locator(&set_2, &edge1, 0, &index2, &intersection, false);
				if (result) {
					common_region.push_back(intersection);
				}
			}
			else if (flag_set2_vertice) {
				index2++;
				edge1 = vector_segment(
					intersection.x,
					intersection.y,
					set_2.at(index2 % set_2.size()).x,
					set_2.at(index2 % set_2.size()).y
				);
				flag_set2_vertice = false;
				bool result = clockwise_intersection_locator(&set_1, &edge1, 0, &index1, &intersection, false);
				if (result) {
					common_region.push_back(intersection);
				}
			}
			
			if (common_region.at(0).equals(intersection)) {
				success = true;
				break;
			}
		}

		while (true) {
			vector_segment edge1;
			if (common_region.size() == 1) { // initialization case
				edge1 = vector_segment(
					intersection.x,
					intersection.y,
					set_1.at(index1 % set_1.size()).x,
					set_1.at(index1 % set_1.size()).y
				);

				bool result;
				if (edge1.leftOn(set_2.at(index2 % set_2.size()).x, set_2.at(index2 % set_2.size()).y)) {
					vector_segment edge = vector_segment(
						intersection.x, intersection.y, set_2.at(index2 % set_2.size()).x, set_2.at(index2 % set_2.size()).y);
					result = clockwise_intersection_locator(&set_1, &edge, index1 + 1, &index1, &intersection, true);
				}
				else {
					vector_segment edge = vector_segment(
						intersection.x, intersection.y, set_1.at(index1 % set_1.size()).x, set_1.at(index1 % set_1.size()).y);
					result = clockwise_intersection_locator(&set_2, &edge, index2 + 1, &index2, &intersection, true);
				}
				if (!result) {
					intersection = Point(set_1.at(index1 % set_1.size()).x, set_1.at(index1 % set_1.size()).y);
					flag_set1_vertice = true;
				}
			}
			else {
				// intersection at non-extreme point; continue with same edge
				// check which set was last edge
				if (common_region.at(common_region.size() - 2).equals(set_1.at((index1 - 1) % set_1.size()))) {
					// remaining part is from set_1
					edge1 = vector_segment(intersection.x, intersection.y, set_1.at(index1 % set_1.size()).x,
						set_1.at(index1 % set_1.size()).y);

					bool result;
					if (edge1.leftOn(set_2.at(index2 % set_2.size()).x, set_2.at(index2 % set_2.size()).y)) {
						vector_segment edge = vector_segment(
							intersection.x, intersection.y, set_2.at(index2 % set_2.size()).x, set_2.at(index2 % set_2.size()).y);
						result = clockwise_intersection_locator(&set_1, &edge, index1 + 1, &index1, &intersection, true);
					}
					else {
						vector_segment edge = vector_segment(
							intersection.x, intersection.y, set_1.at(index1 % set_1.size()).x, set_1.at(index1 % set_1.size()).y);
						result = clockwise_intersection_locator(&set_2, &edge, index2 + 1, &index2, &intersection, true);
					}
					if (!result) {
						intersection = Point(set_2.at(index2 % set_2.size()).x, set_2.at(index2 % set_2.size()).y);
						flag_set2_vertice = true;
					}
				}
				else {
					// remaining part is from set_2
					edge1 = vector_segment(intersection.x, intersection.y, set_2.at(index2 % set_2.size()).x,
						set_2.at(index2 % set_2.size()).y);

					bool result;
					if (edge1.leftOn(set_2.at(index2 % set_2.size()).x, set_2.at(index2 % set_2.size()).y)) {
						vector_segment edge = vector_segment(
							intersection.x, intersection.y, set_2.at(index2 % set_2.size()).x, set_2.at(index2 % set_2.size()).y);
						result = clockwise_intersection_locator(&set_1, &edge, index1 + 1, &index1, &intersection, true);
					}
					else {
						vector_segment edge = vector_segment(
							intersection.x, intersection.y, set_1.at(index1 % set_1.size()).x, set_1.at(index1 % set_1.size()).y);
						result = clockwise_intersection_locator(&set_2, &edge, index2 + 1, &index2, &intersection, true);
					}
					if (!result) {
						intersection = Point(set_1.at(index1 % set_1.size()).x, set_1.at(index1 % set_1.size()).y);
						flag_set1_vertice = true;
					}
				}
			}

			if (common_region.at(0).equals(intersection)) {
				success = true;
				break;
			}
			common_region.push_back(intersection);
			
			if (flag_set1_vertice || flag_set2_vertice)
				break;
		}
		if (success)
			break;
	}
	//compute area here

	for (int i = 2; i < common_region.size(); i++) {
		area -= CompGeomFuncEllipseApprox::triangle_area(
			common_region.at(0).x,
			common_region.at(0).y,
			common_region.at(i - 1).x,
			common_region.at(i - 1).y,
			common_region.at(i).x,
			common_region.at(i).y
		);
	}

	render_agent->visualize_quads_intersection(this, quad, &common_region);
	//std::cin.ignore();

	return area;
}

void quad::common_area_v2(quad* other, ellipse* common_ellipse, local_visualizer* render_agent) {
	//render_agent->visualize_quads_intersection(this, other);
	float area = 0.0;

	std::vector<Point> set_1, set_2, common_region, sorted_list;

	set_1.push_back(Point(free1.x1, free1.y1));
	set_1.push_back(Point(free2.x1, free2.y1));
	set_1.push_back(Point(free2.x2, free2.y2));
	set_1.push_back(Point(free1.x2, free1.y2));

	set_2.push_back(Point(other->free1.x1, other->free1.y1));
	set_2.push_back(Point(other->free2.x1, other->free2.y1));
	set_2.push_back(Point(other->free2.x2, other->free2.y2));
	set_2.push_back(Point(other->free1.x2, other->free1.y2));

	// finding intersections
	for (int i = 0; i < set_1.size(); i++) {
		vector_segment edge1 = vector_segment(
			set_1.at((i - 1) % set_1.size()).x,
			set_1.at((i - 1) % set_1.size()).y,
			set_1.at(i % set_1.size()).x,
			set_1.at(i % set_1.size()).y);
		for (int j = 0; j < set_2.size(); j++) {
			vector_segment edge2 = vector_segment(
				set_2.at((j - 1) % set_2.size()).x,
				set_2.at((j - 1) % set_2.size()).y,
				set_2.at(j % set_2.size()).x,
				set_2.at(j % set_2.size()).y);
			if (edge1.isIntersecting(&edge2)) {
				common_region.push_back(edge1.intersection_points(&edge2));
			}
		}
	}

	if (common_region.empty()) // case of complete overlap
		return;
	// length measure for common poly (A good enough measure to quantify free area overlap between two quads)
	float length = CompGeomFuncEllipseApprox::distance(common_ellipse->center_x,
		common_ellipse->center_y, common_region[0].x, common_region[0].y);
	Point farthest_point = Point(common_region[0].x, common_region[0].y);
	for (int i = 1; i < common_region.size(); i++) {
		float len_i = CompGeomFuncEllipseApprox::distance(common_ellipse->center_x,
			common_ellipse->center_y, common_region[i].x, common_region[i].y);
		if (len_i > length) {
			length = len_i;
			farthest_point = Point(common_region[i].x, common_region[i].y);
		}
	}
	// average radii, r = (ab)^0.5 by equation area of a circle to area of ellipse
	length = length - std::sqrt(common_ellipse->a * common_ellipse->b);
	float length_measure2 = length / std::sqrt(common_ellipse->a * common_ellipse->b);
	if (length_measure2 < 0.2) // not enough overlapped
		return;

	// for interior points
	for (int i = 0; i < set_1.size(); i++) {
		Point point = Point(set_1.at(i).x, set_1.at(i).y);

		for (int j = 0; j < set_2.size(); j++) {
			vector_segment edge = vector_segment(
				set_2.at((j - 1) % set_2.size()).x,
				set_2.at((j - 1) % set_2.size()).y,
				set_2.at(j % set_2.size()).x,
				set_2.at(j % set_2.size()).y);
			if (edge.rightOn(point.x, point.y)) {
				break;
			}
			if (j == set_2.size() - 1) {
				common_region.push_back(point);
			}
		}
	}
	for (int i = 0; i < set_2.size(); i++) {
		Point point = Point(set_2.at(i).x, set_2.at(i).y);

		for (int j = 0; j < set_1.size(); j++) {
			vector_segment edge = vector_segment(
				set_1.at((j - 1) % set_1.size()).x,
				set_1.at((j - 1) % set_1.size()).y,
				set_1.at(j % set_1.size()).x,
				set_1.at(j % set_1.size()).y);
			if (edge.rightOn(point.x, point.y)) {
				break;
			}
			if (j == set_1.size() - 1) {
				common_region.push_back(point);
			}
		}
	}

	// remove redundant points
	for (int i = 0; i < common_region.size(); i++) {
		for (int j = i + 1; j < common_region.size(); j++) {
			float del_x = std::abs(common_region.at(i).x - common_region.at(j).x);
			float del_y = std::abs(common_region.at(i).y - common_region.at(j).y);
			if (del_x < FLOATING_PRECISION && del_y < FLOATING_PRECISION) {
				common_region.erase(common_region.begin() + j);
				j--;
			}
		}
	}

	// if no common point found : 
	if (common_region.size() < 3) {
		return ;
	}

	// sort these shortlisted points
	// identifying support point
	int index = 0;
	float val = common_region.at(0).x;
	for (int i = 1; i < common_region.size(); i++) { // right-most point along x-axis
		if (common_region.at(i).x > val) {
			val = common_region.at(i).x;
			index = i;
		}
	}
	sorted_list.push_back(common_region.at(index));
	common_region.erase(common_region.begin() + index);

	// identifying support vector (2nd point)
	index = 0;
	val = (common_region[0].y - sorted_list[0].y) / (common_region[0].x - sorted_list[0].x);
	for (int i = 1; i < common_region.size(); i++) {
		float slope = (common_region[i].y - sorted_list[0].y) / (common_region[i].x - sorted_list[0].x);
		if (slope < val) {
			val = slope;
			index = i;
		}
	}
	sorted_list.push_back(common_region.at(index));
	common_region.erase(common_region.begin() + index);

	// sorting remaining points
	for (int i = 0; i < common_region.size(); i++) {
		int pos = sorted_list.size() - 1;
		vector_segment edge = vector_segment(sorted_list.at(pos - 1).x, sorted_list.at(pos - 1).y,
			sorted_list.at(pos).x, sorted_list.at(pos).y);
		val = edge.subtended_angle_measure(common_region.at(0).x, common_region.at(0).y);
		index = 0;
		
		for (int j = 1; j < common_region.size(); j++) {
			float val_j = edge.subtended_angle_measure(common_region.at(j).x, common_region.at(j).y);
			if (val_j > val) {
				val = val_j;
				index = j;
			}
		}
		sorted_list.push_back(common_region.at(index));
		common_region.erase(common_region.begin() + index);
		i--;
	}

	//compute area
	for (int i = 2; i < sorted_list.size(); i++) {
		area += CompGeomFuncEllipseApprox::triangle_area(
			sorted_list.at(0).x,
			sorted_list.at(0).y,
			sorted_list.at(i - 1).x,
			sorted_list.at(i - 1).y,
			sorted_list.at(i).x,
			sorted_list.at(i).y
		);
	}

	if (area / (this->mfa + other->mfa) < FLOATING_PRECISION) // ratio of common area to average area of quad
		return;

	//render_agent->visualize_quads_intersection(this, quad, &sorted_list);


	// common ellipses' blocking effect
	//float del_area1 = 0.0, del_area2 = 0.0, del_len1 = 0.0, del_len2 = 0.0;
	//estimate_ellipse_blockage(common_ellipse, blocked1, &del_area1, &del_len1);
	//estimate_ellipse_blockage(common_ellipse, quad->blocked1, &del_area2, &del_len2);

	if (this->me_A == common_ellipse) {
		this->neighbours_A.push_back(sibling(other, area, length, length_measure2, 
			Point(common_ellipse->center_x, common_ellipse->center_y), farthest_point));
	}
	if (this->me_B == common_ellipse) {
		this->neighbours_B.push_back(sibling(other, area, length, length_measure2,
			Point(common_ellipse->center_x, common_ellipse->center_y), farthest_point));
	}

	if (other->me_A == common_ellipse) {
		other->neighbours_A.push_back(sibling(this, area, length, length_measure2,
			Point(common_ellipse->center_x, common_ellipse->center_y), farthest_point));
	}
	if (other->me_B == common_ellipse) {
		other->neighbours_B.push_back(sibling(this, area, length, length_measure2,
			Point(common_ellipse->center_x, common_ellipse->center_y), farthest_point));
	}

	return;
}

float quad::poly_bounds(quad* quad, local_visualizer* render_agent) {
	float area = 0.0;

	std::vector<Point> set_1, set_2, unified_region;

	set_1.push_back(Point(free1.x1, free1.y1));
	set_1.push_back(Point(free1.x2, free1.y2));
	set_1.push_back(Point(free2.x2, free2.y2));
	set_1.push_back(Point(free2.x1, free2.y1));

	set_2.push_back(Point(quad->free1.x1, quad->free1.y1));
	set_2.push_back(Point(quad->free1.x2, quad->free1.y2));
	set_2.push_back(Point(quad->free2.x2, quad->free2.y2));
	set_2.push_back(Point(quad->free2.x1, quad->free2.y1));

	Point intersection = Point();
	int index1 = 0, index2 = 0;

	for (int i = 0; i < set_1.size(); i++) {
		unified_region.push_back(set_1.at((i) % set_1.size()));
		vector_segment edge1 = vector_segment(
			set_1.at((i - 1) % set_1.size()).x,
			set_1.at((i - 1) % set_1.size()).y,
			set_1.at(i).x,
			set_1.at(i).y);
		for (int j = 0; j < set_2.size(); j++) {
			if (edge1.rightOn(set_2.at(j).x, set_2.at(j).y)) {
				index2 = j;
				break;
			}
		}
		for (int j = 0; j < set_2.size(); j++) {
			vector_segment edge2 = vector_segment(
				set_2.at((j + index2) % set_2.size()).x,
				set_2.at((j + index2) % set_2.size()).y,
				set_2.at((j + index2 + 1) % set_2.size()).x,
				set_2.at((j + index2 + 1) % set_2.size()).y);
			bool flag_intersection_found = false;

			if (edge1.isIntersecting(&edge2)) {
				intersection = edge1.intersection_points(&edge2);
				unified_region.push_back(intersection);
				for (int k = 0; k < set_2.size(); k++) {
					unified_region.push_back(set_2.at((k + j + index2 + 1) % set_2.size()));
					edge2 = vector_segment(
						set_2.at((k + j + index2 + 1) % set_2.size()).x,
						set_2.at((k + j + index2 + 1) % set_2.size()).y,
						set_2.at((k + j + index2 + 2) % set_2.size()).x,
						set_2.at((k + j + index2 + 2) % set_2.size()).y);
					if (edge2.isIntersecting(&edge1)) {
						flag_intersection_found = true;
						break;
					}
				}
			}

			if (flag_intersection_found) {
				intersection = edge1.intersection_points(&edge2);
				unified_region.push_back(intersection);
				break;
			}
			else {
				// get next edge1 for the original edge2
				// reset j th loop
				j--;

			}
		}
	}

	return area;
}

float quad::importance_function1(void) {
	float importance;
	importance = mfa;
	return mfa;
}

float quad::distance(float x, float y) {
	if (isInside(x, y))
		return 0.0;

	float dist =0.0, temp=0.0;

	dist = free1.min_distance_from_line_segment(x, y);

	temp = free2.min_distance_from_line_segment(x, y);
	if (temp < dist)
		dist = temp;

	temp = blocked1.min_distance_from_line_segment(x, y);
	if (temp < dist)
		dist = temp;

	temp = blocked2.min_distance_from_line_segment(x, y);
	if (temp < dist)
		dist = temp;

	return dist;
}

bool quad::isInside(float x, float y) {
	if (free1.leftOn(x, y))
		return false;
	if (free2.rightOn(x, y))
		return false;
	if (blocked1.rightOn(x, y))
		return false;
	if (blocked2.leftOn(x, y))
		return false;

	return true;
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
	this->importance = importance;
	return importance;
}

bool ellipse::neighbourSweep::is_degenerate(void) {
	/*/float alpha1 = std::atan2(pointer->center_y - this->center_y,
		pointer->center_x-this->center_x)*/
	return false;
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
	if (show_logs)
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
		if (check0)
			continue;
		vector_segment* intersection_test0 = nullptr;
		internal_chord.intersection_points(neighbour, intersection_test0);
		if (intersection_test0 != nullptr)
			continue;

		// check if the ellipse is outside the quadrant(formed by 2 tangents) of interest
		bool check1 = neighbours.at(i).internal_tangent1.leftOn(neighbour->center_x, neighbour->center_y);
		bool check2 = neighbours.at(i).internal_tangent2.rightOn(neighbour->center_x, neighbour->center_y);
		if (check1 || check2)
			continue;

		vector_segment* intersection1 = nullptr, * intersection2 = nullptr;
		neighbours.at(i).internal_tangent1.intersection_points(neighbour, intersection1);
		neighbours.at(i).internal_tangent2.intersection_points(neighbour, intersection2);
		
		if (intersection1 == nullptr && intersection2 == nullptr) {
			render_agent->shielding_visualizer(this, neighbour);
			return true;
		}

		if (expose_vector.empty) {
			expose_vector = boundingChords(intersection1, intersection2);
		}
		else {
			//check which tangent is involved
			boundingChords update = boundingChords(intersection1, intersection2);
			expose_vector.intersection_update(update);
		}
	}
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
				Point intersection_points = exposed_segments.at(j).intersection_points(
					&neighbours.at(i).internal_tangent1);
				if (check1) { // leading end of edge in left most interest zone
					if (check4) {
						// double cut
						Point secondary_intersection = exposed_segments.at(j).intersection_points(
							&neighbours.at(i).internal_tangent2);
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
				if (check2) { // trailing(rear) end of edge in left most interest zone
					if (check3) {
						// double cut
						Point secondary_intersection = exposed_segments.at(j).intersection_points(
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
				Point intersection_points = exposed_segments.at(j).intersection_points(
					&neighbours.at(i).internal_tangent2);
				if (check3) { // leading end of edge in right most interest zone
					if (check2) {
						// double cut
						Point secondary_intersection = exposed_segments.at(j).intersection_points(
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
						Point secondary_intersection = exposed_segments.at(j).intersection_points(
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
		return true;
	}

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

void ellipse::visibility_handler_reloaded(local_visualizer* render_agent) {

	for (int i = 0; i < neighbours.size(); i++) {

		ellipse::neighbourSweep* petal1 = &neighbours.at(i);

		for (int j = 0; j < neighbours.size(); j++) {

			if (i == j)
				continue;
			ellipse::neighbourSweep* petal2 = &neighbours.at(j);

			if (petal2->visibility_limit_left.isIntersecting(&petal1->visibility_range)
				&& petal2->visibility_limit_right.isIntersecting(&petal1->visibility_range)) { // ith edge is completely blocked by this edge
				// mark ith neighbour invisible in the list
				neighbours.erase(neighbours.begin() + j);
				j--;
				continue;
			}

			modify_visibility_zones_reloaded(petal2, petal1, false, render_agent);
			modify_visibility_zones_reloaded(petal2, petal1, true, render_agent);
		}
	}
	//render_agent->visualize_petals(this);
}

bool ellipse::modify_visibility_zones_reloaded(ellipse::neighbourSweep* to_be_blocked,
	ellipse::neighbourSweep* blocker, bool right, local_visualizer* render_agent) {
	vector_segment* tangent_member_to_be_blocked, * tangent_member_blocker, * new_limit, * the_other_tangent;
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
		if (neighbours.at(i).pointer == blocker->pointer) {
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
		Point intersection = to_be_blocked->visibility_range.intersection_points(new_limit);
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

	/*/if (new_limit->isStrictlyVectorExtendedIntersecting(the_other_tangent)) {
		std::cout << "USING VETO :P\n";
		std::cout << "You should keep a check on veto...\n";
		std::cout << "new_limit : (" << new_limit->x1 << ", " << new_limit->y1 << ") ; (" << new_limit->x2 << ", " << new_limit->y2 << ") " << std::endl;
		std::cout << "the_other_tangent : (" << the_other_tangent->x1 << ", " << the_other_tangent->y1 << ") ; (" << the_other_tangent->x2 << ", " << the_other_tangent->y2 << ") " << std::endl;
		to_be_blocked->visible = false;
	}*/

	return false;
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

bool ellipse::isIntersecting(ellipse* ellipse) {
	return false;
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

void local_visualizer::draw_quad(quad* quad) {
	if (true) {
		add_path(quad->free1);
		add_path(quad->free2);
		add_path(quad->blocked1);
		add_path(quad->blocked2);

		invalidate();
	}
}

void local_visualizer::draw_quad(quad* quad, Color color) {
	if (true) {
		add_path(quad->free1, color);
		add_path(quad->free2, color);
		add_path(quad->blocked1, color);
		add_path(quad->blocked2, color);

		invalidate();
	}
}

void local_visualizer::visualize_quads_intersection(quad* quad1, quad* quad2) {
	if (true) {
		int path_count = 0;
		add_path(quad1->blocked1, Color(1.0, 0.0, 0.0));
		add_path(quad1->blocked2, Color(1.0, 0.0, 0.0));
		add_path(quad1->free1, Color(0.0, 0.0, 1.0));
		add_path(quad1->free2, Color(0.0, 0.0, 1.0));
		path_count += 4;

		add_path(quad2->blocked1, Color(1.0, 1.0, 1.0));
		add_path(quad2->blocked2, Color(1.0, 1.0, 1.0));
		add_path(quad2->free1, Color(0.0, 0.0, 0.0));
		add_path(quad2->free2, Color(0.0, 0.0, 0.0));
		path_count += 4;

		invalidate();

		std::cin.ignore();
		if (true)
			clear_paths(path_count);
	}
}

void local_visualizer::visualize_quads_intersection(quad* quad1, quad* quad2, std::vector<Point>* poly) {
	if (true) {
		int path_count = 0;
		add_path(quad1->blocked1, Color(0.0, 0.0, 0.0));
		add_path(quad1->blocked2, Color(0.0, 0.0, 0.0));
		add_path(quad1->free1, Color(0.0, 0.0, 0.0));
		add_path(quad1->free2, Color(0.0, 0.0, 0.0));
		path_count += 4;

		add_path(quad2->blocked1, Color(0.0, 0.0, 0.0));
		add_path(quad2->blocked2, Color(0.0, 0.0, 0.0));
		add_path(quad2->free1, Color(0.0, 0.0, 0.0));
		add_path(quad2->free2, Color(0.0, 0.0, 0.0));
		path_count += 4;

		for (int i = 0; i < poly->size(); i++) {
			add_path(Path(poly->at((i + 1) % poly->size()).x, poly->at((i + 1) % poly->size()).y,
				poly->at(i % poly->size()).x, poly->at(i % poly->size()).y, Color(1.0, 1.0, 1.0)));
			path_count++;
		}

		invalidate();

		//std::cin.ignore();
		clear_paths(path_count);
	}
}

void local_visualizer::visualize_petals(ellipse* obstacle) {
	if (true) {
		int path_count = 0;
		
		for (int i = 0; i < obstacle->neighbours.size(); i++) {
			add_path(obstacle->neighbours.at(i).visibility_limit_left, Color(1.0, 1.0, 0.0));
			add_path(obstacle->neighbours.at(i).visibility_limit_right, Color(1.0, 1.0, 0.0));
			add_path(obstacle->neighbours.at(i).visibility_range, Color(1.0, 1.0, 0.0));
			path_count += 3;
		}
		invalidate();

		std::cin.ignore();
		clear_paths(path_count);
		invalidate();
	}
}

void local_visualizer::visualize_path(std::vector<quad*>* path) {
	if (true) {
		int path_count = 0;
		for (int i = 0; i < path->size(); i++) {
			if (i == 0)
				draw_quad(path->at(i), Color(1.0, 0.0, 0.0));
			else if (i == path->size() - 1)
				draw_quad(path->at(i), Color(0.0, 1.0, 0.0));
			else
				draw_quad(path->at(i), Color(0.0, 0.0, 0.0));
			path_count += 4;
		}
		invalidate();

		/*std::cin.ignore();
		clear_paths(path_count);
		invalidate();*/
	}
}

void local_visualizer::visualize_path2(std::vector<quad*>* path, vector_segment start_goal) {
	if (true) {
		int path_count = 0;
		float center_x0 = (path->at(0)->free1.x1 + path->at(0)->free1.x2 +
			path->at(0)->free2.x1 + path->at(0)->free2.x2) / 4;
		float center_y0 = (path->at(0)->free1.y1 + path->at(0)->free1.y2 +
			path->at(0)->free2.y1 + path->at(0)->free2.y2) / 4;
		add_path(Path(start_goal.x1, start_goal.y1, center_x0, center_y0, Color(1.0, 0.0, 0.0)));

		for (int i = 1; i < path->size(); i++) {
			float center_x1 = (path->at(i - 1)->free1.x1 + path->at(i - 1)->free1.x2 +
				path->at(i - 1)->free2.x1 + path->at(i - 1)->free2.x2) / 4;
			float center_y1 = (path->at(i - 1)->free1.y1 + path->at(i - 1)->free1.y2 +
				path->at(i - 1)->free2.y1 + path->at(i - 1)->free2.y2) / 4;
			float center_x2 = (path->at(i)->free1.x1 + path->at(i)->free1.x2 +
				path->at(i)->free2.x1 + path->at(i)->free2.x2) / 4;
			float center_y2 = (path->at(i)->free1.y1 + path->at(i)->free1.y2 +
				path->at(i)->free2.y1 + path->at(i)->free2.y2) / 4;
			add_path(Path(center_x1, center_y1, center_x2, center_y2, Color(0.937f, 0.749f, 0.2196f)));
		}

		center_x0 = (path->at(path->size() - 1)->free1.x1 + path->at(path->size() - 1)->free1.x2 +
			path->at(path->size() - 1)->free2.x1 + path->at(path->size() - 1)->free2.x2) / 4;
		center_y0 = (path->at(path->size() - 1)->free1.y1 + path->at(path->size() - 1)->free1.y2 +
			path->at(path->size() - 1)->free2.y1 + path->at(path->size() - 1)->free2.y2) / 4;
		add_path(Path(center_x0, center_y0, start_goal.x2, start_goal.y2, Color(0.0, 1.0, 0.0)));

		path_count = path->size() + 1;
		invalidate();

		/*std::cin.ignore();
		clear_paths(path_count);
		invalidate();*/
	}
}

void local_visualizer::visualize_path3(std::vector<local_path_node> path) {
	if (true) {
		for (int i = 1; i < path.size(); i++) {
			add_path(vector_segment(path.at(i - 1).coords.x, path.at(i - 1).coords.y,
				//path.at(i).x, path.at(i).y), Color(0.0, 0.0, 0.0));
			path.at(i).coords.x, path.at(i).coords.y), Color(1.0, 0.0, 0.0));
		}
		invalidate();
	}
}

void local_visualizer::visualize_graph_exploration_options(quad* node) {
	if (true) {
		int path_count = 0;
		for (int i = 0; i < node->neighbours_A.size(); i++) {
			draw_quad(node->neighbours_A.at(i).path);
			path_count += 4;
		}
		for (int i = 0; i < node->neighbours_B.size(); i++) {
			draw_quad(node->neighbours_B.at(i).path);
			path_count += 4;
		}
		invalidate();

		std::cin.ignore();
		clear_paths(path_count);
		invalidate();
	}
}

void local_visualizer::visualize_quads(quad* quad, bool ellipses, bool clear_back) {
	if (false) {
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
	if (show_logs) {
		render_callback(nullptr);
		renderer->render();
	}
}

void elliptical_approx::init(std::vector<Node>* node_list, int width, int height, RenderClass* renderer,
	std::vector<Path>* paths, void (*func_updater)(std::vector<float>*)) {
	this->node_list = node_list;
	this->GRID_WIDTH = width;
	this->GRID_HEIGHT = height;
	this->render_agent = local_visualizer(GRID_WIDTH, GRID_HEIGHT, renderer, paths, func_updater);
	this->cluster = convex_clustering(&render_agent);

	contour_extractor2();
	//locate_obstacles();

	//ellipse ellipse1 = ellipse(77.04f, 25.04f, 15.09f, 21.54f, 46.69 * PI / 180);
	//cluster->clustering_2(node_list, GRID_WIDTH);
	for (int i = 0; i < points.size(); i++) {
		cluster.clustering_3(node_list, GRID_WIDTH, points.at(i));
	}

	map_builder = quad_builder(cluster.ellipse_list, &render_agent);
}

void elliptical_approx::finder(int start_cell_index, int goal_cell_index, 
	std::chrono::time_point<std::chrono::high_resolution_clock> start,
	consolidated_result* result) {
	this->start_cell_index = start_cell_index;
	this->goal_cell_index = goal_cell_index;

	start_x = this->start_cell_index % GRID_WIDTH;
	start_y = this->start_cell_index / GRID_WIDTH;
	goal_x = this->goal_cell_index % GRID_WIDTH;
	goal_y = this->goal_cell_index / GRID_WIDTH;

	std::vector<quad*> feasible_path;
	std::vector<quad>* quads = &(map_builder.quad_list);

	quad* start_node = nearest_quad(start_x, start_y);
	quad* goal_node = nearest_quad(goal_x, goal_y);
	if (show_logs)
		std::cout << "start_node : " << start_node->id << " goal_node : " << goal_node->id << "\n";


	search_path(start_node, goal_node, start_node->me_A, &feasible_path);
	if (feasible_path.at(feasible_path.size()-1)!=goal_node)
		search_path(start_node, goal_node, start_node->me_B, &feasible_path);

	// path cleanup
	//path_cleanup(&feasible_path);

	// path smoothening
	for (int i = 1; i < feasible_path.size(); i++) {
		//int additions = smoothen_paths(i, &feasible_path);
		//i += additions;
	}

	// local path planning
	std::vector<local_path_node> local_path;
	local_path_planning(&feasible_path, &local_path);

	path_cleanup(&feasible_path, &local_path);

	auto elapsed = std::chrono::high_resolution_clock::now() - start;
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
		elapsed).count();
	std::cout << "path processed in : " << microseconds << " microseconds\n";
	run_time.push_back(microseconds);

	if (result != nullptr)
		result->me_ea = algo_result(path_length(&local_path), microseconds, stringify(&local_path));

	long long net_sum = 0;
	for (int i = 0; i < run_time.size(); i++) {
		net_sum += run_time.at(i);
	}
	net_sum = net_sum / run_time.size();
	std::cout << "average path processing time : " << net_sum << " microseconds after " << run_time.size() << "iterations\n";

	//render_agent.visualize_path(&feasible_path);
	//render_agent.visualize_path2(&feasible_path, vector_segment(start_x, start_y, goal_x, goal_y));
	render_agent.visualize_path3(local_path);
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
				//Color(0.0, 0.0, 0.0)
				Color(0.937f, 0.749f, 0.2196f)
			)
		);
	}
	render_agent.invalidate();
}

void elliptical_approx::contour_analyzer(int first_index, std::vector<std::vector<strip>>* strips) {
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
	}

	// append these edge info to strips list
	for (int i = 0; i < edges.size(); i++) {

		int TRAVERSALS = edges.at(i).size();
		for (int j = 0; j < TRAVERSALS; j++) {
			/*/[LAST MINUTE EDITS] : if (edges.at(i).empty())
				continue;*/
			if (edges.at(i).size() < 2)
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

int count = 0;
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
			if (build_contour(node->left, travel_list, this_node_index - 1, remaining_nodes, start-1))
				return 1;
		}
		break;
	case 2:
		if (node->bottom->type == START) {
			render_agent.add_path(Path(node, node->bottom));
			if (build_contour(node->bottom, travel_list, this_node_index + GRID_WIDTH, remaining_nodes, start-1))
				return 2;
		}
		break;
	case 3:
		if (node->right->type == START) {
			render_agent.add_path(Path(node, node->right));
			if (build_contour(node->right, travel_list, this_node_index + 1, remaining_nodes, start-1))
				return 3;
		}
		break;
	case 4:
		if (node->top->type == START) {
			render_agent.add_path(Path(node, node->top));
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
		return false;
	}
	if (node == nullptr) {
		return false;
	}
	if (this_node_index < 0 || this_node_index >= node_list->size()) {
		return false;
	}
	if (travel_list[this_node_index]) {
		return false;
	}
	travel_list[this_node_index] = true;
	remaining_nodes -= 1;

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

quad* elliptical_approx::nearest_quad(float x, float y) {
	float distance = map_builder.quad_list.at(0).distance(x, y);
	int index = 0;

	for (int i = 1; i < map_builder.quad_list.size(); i++) {
		float temp = map_builder.quad_list.at(i).distance(x, y);
		if (temp < distance) {
			distance = temp;
			index = i;
		}
	}
	return &(map_builder.quad_list.at(index));
}

float elliptical_approx::heuristic_function(quad::sibling* node, ellipse* pivot_point) {
	float importance;

	// method I : minimize distance from goal
	importance = 1/node->path->distance(goal_x, goal_y);

	// method II : maximize path width
	//importance = importance * node->path->mpw;

	// method III : maximize length of common junction 
	importance = importance * node->length_measure2;

	return importance;
}

void elliptical_approx::compute_search_order(std::vector<quad::sibling>* list, std::vector<int>* order, ellipse* common_ellip) {
	std::vector<int> indices;
	for (int i = 0; i < list->size(); i++) {
		indices.push_back(i);
	}
	while (!indices.empty()) {
		float importance = heuristic_function(&list->at(indices.at(0)), common_ellip);
		int index = 0;
		//std::cout << "index : " << index << " importance : " << importance << "\n";
		for (int i = 1; i < indices.size(); i++) {
			float temp = heuristic_function(&list->at(indices.at(i)), common_ellip);
			//std::cout << "index : " << j << " importance : " << temp << "\n";
			if (temp > importance) {
				importance = temp;
				index = i;
			}
		}
		order->push_back(indices.at(index));
		indices.erase(indices.begin() + index);
	}
	indices.clear();
}

bool elliptical_approx::search_path(quad* start, quad* goal, ellipse* pivot, std::vector<quad*>* path) {
	// mark this nodes signature on path
	path->push_back(start);

	// if goal point is inside current node; path finding succeed.
	if (start->isInside(goal_x, goal_y))
		return true;
	
	// if goal reached
	if (path->at(path->size() - 1) == goal)
		return true; // force terminate all recursive functions

	if (pivot == start->me_A) {
		std::vector<int> search_order;
		compute_search_order(&start->neighbours_A, &search_order, start->me_A);
		for (int i = 0; i < start->neighbours_A.size(); i++) {
			//quad* node_i = start->neighbours_A.at(start->order_A.at(i)).path;
			quad* node_i = start->neighbours_A.at(search_order.at(i)).path;
			// check if a quad is already in the path list
			bool already_taken = false;
			for (int j = 0; j < path->size(); j++) {
				if (node_i == path->at(j)) {
					already_taken = true;
					break;
				}
			}
			if (already_taken)
				continue;

			// otherwise keep exploring in serial order until goal is reached
			float dist_A = CompGeomFuncEllipseApprox::distance(goal_x, goal_y,
				node_i->me_A->center_x, node_i->me_A->center_y);
			float dist_B = CompGeomFuncEllipseApprox::distance(goal_x, goal_y,
				node_i->me_B->center_x, node_i->me_B->center_y);

			if (dist_A < dist_B) {
				if (search_path(node_i, goal, node_i->me_A, path))
					return true;
				if (search_path(node_i, goal, node_i->me_B, path))
					return true;
			}
			else {
				if (search_path(node_i, goal, node_i->me_B, path))
					return true;
				if (search_path(node_i, goal, node_i->me_A, path))
					return true;
			}
		}
	}

	if (pivot == start->me_B) {
		std::vector<int> search_order;
		compute_search_order(&start->neighbours_B, &search_order, start->me_B);
		for (int i = 0; i < start->neighbours_B.size(); i++) {
			//quad* node_i = start->neighbours_B.at(start->order_B.at(i)).path;
			quad* node_i = start->neighbours_B.at(search_order.at(i)).path;
			// check if a quad is already in the path list
			bool already_taken = false;
			for (int j = 0; j < path->size(); j++) {
				if (node_i == path->at(j)) {
					already_taken = true;
					break;
				}
			}
			if (already_taken)
				continue;

			// otherwise keep exploring in serial order until goal is reached
			float dist_A = CompGeomFuncEllipseApprox::distance(goal_x, goal_y,
				node_i->me_A->center_x, node_i->me_A->center_y);
			float dist_B = CompGeomFuncEllipseApprox::distance(goal_x, goal_y,
				node_i->me_B->center_x, node_i->me_B->center_y);

			if (dist_A < dist_B) {
				if (search_path(node_i, goal, node_i->me_A, path))
					return true;
				if (search_path(node_i, goal, node_i->me_B, path))
					return true;
			}
			else {
				if (search_path(node_i, goal, node_i->me_B, path))
					return true;
				if (search_path(node_i, goal, node_i->me_A, path))
					return true;
			}
		}
	}

	// not a part of this journey. removing this node from path
	path->pop_back();
	return false;
}

bool elliptical_approx::scrape_common_area_measure(quad* quad1, quad* quad2, float* metrics) {
	// metrics : area, len1, len2, centerX, centerY, farX, farY
	for (int i = 0; i < map_builder.quad_list.size(); i++) {
		if (&map_builder.quad_list.at(i) == quad1) {
			for (int j = 0; j < map_builder.quad_list.at(i).neighbours_A.size(); j++) {
				if (map_builder.quad_list.at(i).neighbours_A.at(j).path == quad2) {
					//return &(map_builder.quad_list.at(i).neighbours_A.at(j));
					quad::sibling exchange_zone = map_builder.quad_list.at(i).neighbours_A.at(j);
					metrics[0] = exchange_zone.area_measure;
					metrics[1] = exchange_zone.length_measure;
					metrics[2] = exchange_zone.length_measure2;
					metrics[3] = exchange_zone.ellip_center.x;
					metrics[4] = exchange_zone.ellip_center.y;
					metrics[5] = exchange_zone.farthest_point.x;
					metrics[6] = exchange_zone.farthest_point.y;
					return true;
				}
			}
			for (int j = 0; j < map_builder.quad_list.at(i).neighbours_B.size(); j++) {
				if (map_builder.quad_list.at(i).neighbours_B.at(j).path == quad2) {
					//return &(map_builder.quad_list.at(i).neighbours_B.at(j));
					quad::sibling exchange_zone = map_builder.quad_list.at(i).neighbours_B.at(j);
					metrics[0] = exchange_zone.area_measure;
					metrics[1] = exchange_zone.length_measure;
					metrics[2] = exchange_zone.length_measure2;
					metrics[3] = exchange_zone.ellip_center.x;
					metrics[4] = exchange_zone.ellip_center.y;
					metrics[5] = exchange_zone.farthest_point.x;
					metrics[6] = exchange_zone.farthest_point.y;
					return true;
				}
			}
		}
	}
	return false;
}

void elliptical_approx::path_cleanup(std::vector<quad*>* feasible_path) {
	for (int i = 0; i < feasible_path->size(); i++) {
		if (i != 0) {
			if (feasible_path->at(i)->isInside(start_x, start_y)) {
				for (int j = 0; j < i; j++) {
					feasible_path->erase(feasible_path->begin() + 0);
				}
				i = -1; // this will start next iteration as i=0
				continue;
			}
		}
		if (feasible_path->at(i)->isInside(goal_x, goal_y)) {
			for (int j = i + 1; j < feasible_path->size(); j++) {
				feasible_path->erase(feasible_path->begin() + i + 1);
			}
			continue;
		}
	}
	for (int i = 0; i < feasible_path->size(); i++) {
		for (int j = feasible_path->size() - 1; j > i; j--) {

			float center_x = (feasible_path->at(i)->free1.x1 + feasible_path->at(i)->free1.x2 +
				feasible_path->at(i)->free2.x1 + feasible_path->at(i)->free2.x2) / 4;
			float center_y = (feasible_path->at(i)->free1.y1 + feasible_path->at(i)->free1.y2 +
				feasible_path->at(i)->free2.y1 + feasible_path->at(i)->free2.y2) / 4;
			if (feasible_path->at(j)->isInside(center_x, center_y)) {
				for (int k = i + 1; k < j; k++) {
					feasible_path->erase(feasible_path->begin() + i + 1);
				}
				break;
			}

			float exchange_zone[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
			bool intersects = scrape_common_area_measure(feasible_path->at(i), feasible_path->at(j), exchange_zone);
			if (intersects) {
				float exchange_length = exchange_zone[1];
				float exchange_length_ratio = exchange_zone[2];
				//if (exchange_length_ratio > 1.0) {
				if (exchange_length > this->ROBOT_SIZE) {
					for (int k = i + 1; k < j; k++) {
						feasible_path->erase(feasible_path->begin() + i + 1);
					}
					break;
				}
			}
		}
	}

	for (int i = 0; i < feasible_path->size(); i++) {
		std::cout << "node[" << i << "] : id " << feasible_path->at(i)->id << std::endl;
	}
}

void elliptical_approx::path_cleanup(std::vector<quad*>* feasible_path, std::vector<local_path_node>* local_path) {
	bool break_flag = false;
	for (int i = 0; i < local_path->size(); i++) {
		break_flag = false;
		for (int j = 0; j < feasible_path->size(); j++) {
			if (feasible_path->at(j)->isInside(local_path->at(i).coords.x, local_path->at(i).coords.y)) {
				for (int k = local_path->size() - 1; k >i; k--) {
					if (feasible_path->at(j)->isInside(local_path->at(k).coords.x, local_path->at(k).coords.y)) {
						for (int l = i + 1; l < k; l++) {
							local_path->erase(local_path->begin() + i + 1);
						}
						break_flag = true;
						break;
					}
				}
			}
			if (break_flag)
				break;
		}
	}

	for (int i = 2; i < local_path->size() - 1; i++) {
		if (local_path->at(i - 1).freeCell == local_path->at(i).freeCell)
			continue;
		std::vector<quad*> quads;
		quads.push_back(local_path->at(i - 1).freeCell);
		quads.push_back(local_path->at(i).freeCell);
		std::vector<vector_segment> path;
		path.push_back(vector_segment(local_path->at(i - 1).coords.x,
			local_path->at(i - 1).coords.y, local_path->at(i).coords.x,
			local_path->at(i).coords.y));
		std::vector<quad*> quads_copy;
		std::vector<vector_segment> path_copy;
		for (int j = 0; j < quads.size(); j++) {
			quads_copy.push_back(quads.at(j));
		}
		for (int j = 0; j < path.size(); j++) {
			path_copy.push_back(path.at(j));
		}
		if (check_contains(&quads_copy, &path_copy))
			continue;

		ellipse* common_ellip, * obstacle1, * obstacle2;
		vector_segment sweep_start, sweep_end;
		if (local_path->at(i - 1).freeCell->me_A == local_path->at(i).freeCell->me_A) {
			common_ellip = local_path->at(i).freeCell->me_A;
			obstacle1 = local_path->at(i - 1).freeCell->me_B;
			obstacle2 = local_path->at(i).freeCell->me_B;
		}
		else if (local_path->at(i - 1).freeCell->me_A == local_path->at(i).freeCell->me_B) {
			common_ellip = local_path->at(i).freeCell->me_B;
			obstacle1 = local_path->at(i - 1).freeCell->me_B;
			obstacle2 = local_path->at(i).freeCell->me_A;
		}
		else if (local_path->at(i - 1).freeCell->me_B == local_path->at(i).freeCell->me_A) {
			common_ellip = local_path->at(i).freeCell->me_A;
			obstacle1 = local_path->at(i - 1).freeCell->me_A;
			obstacle2 = local_path->at(i).freeCell->me_B;
		}
		else if (local_path->at(i - 1).freeCell->me_B == local_path->at(i).freeCell->me_B) {
			common_ellip = local_path->at(i).freeCell->me_B;
			obstacle1 = local_path->at(i - 1).freeCell->me_A;
			obstacle2 = local_path->at(i).freeCell->me_A;
		}
		else
			continue;

		sweep_start = vector_segment(common_ellip->center_x, common_ellip->center_y,
			obstacle1->center_x, obstacle1->center_y);
		sweep_end = vector_segment(common_ellip->center_x, common_ellip->center_y,
			obstacle2->center_x, obstacle2->center_y);
		std::vector<quad*> support;
		for (int j = 0; j < map_builder.quad_list.size(); j++) {
			vector_segment sweep;
			if (map_builder.quad_list.at(j).me_A == common_ellip) {
				if ((map_builder.quad_list.at(j).me_B == obstacle1) ||
					(map_builder.quad_list.at(j).me_B == obstacle2))
					continue;
				float center_x = (map_builder.quad_list.at(j).free1.x1 +
					map_builder.quad_list.at(j).free1.x2 +
					map_builder.quad_list.at(j).free2.x1 +
					map_builder.quad_list.at(j).free2.x2) / 4;
				float center_y = (map_builder.quad_list.at(j).free1.y1 +
					map_builder.quad_list.at(j).free1.y2 +
					map_builder.quad_list.at(j).free2.y1 +
					map_builder.quad_list.at(j).free2.y2) / 4;
				sweep = vector_segment(common_ellip->center_x, common_ellip->center_y,
					center_x, center_y);
				if (sweep_start.leftOn(sweep.x2, sweep.y2) == sweep.leftOn(sweep_end.x2, sweep_end.y2))
					support.push_back(&map_builder.quad_list.at(j));
			}
			if (map_builder.quad_list.at(j).me_B == common_ellip) {
				if ((map_builder.quad_list.at(j).me_A == obstacle1) ||
					(map_builder.quad_list.at(j).me_A == obstacle2))
					continue;
				float center_x = (map_builder.quad_list.at(j).free1.x1 +
					map_builder.quad_list.at(j).free1.x2 +
					map_builder.quad_list.at(j).free2.x1 +
					map_builder.quad_list.at(j).free2.x2) / 4;
				float center_y = (map_builder.quad_list.at(j).free1.y1 +
					map_builder.quad_list.at(j).free1.y2 +
					map_builder.quad_list.at(j).free2.y1 +
					map_builder.quad_list.at(j).free2.y2) / 4;
				sweep = vector_segment(common_ellip->center_x, common_ellip->center_y,
					center_x, center_y);
				if (sweep_start.leftOn(sweep.x2, sweep.y2) == sweep.leftOn(sweep_end.x2, sweep_end.y2))
					support.push_back(&map_builder.quad_list.at(j));
			}
		}
		bool dir = sweep_end.leftOn(sweep_start.x2, sweep_start.y2);
		for (int j = 0; j < support.size(); j++) {
			int index = 0;
			float center_x = (support.at(0)->free1.x1 +	support.at(0)->free1.x2 +
				support.at(0)->free2.x1 + support.at(0)->free2.x2) / 4;
			float center_y = (support.at(0)->free1.y1 +	support.at(0)->free1.y2 +
				support.at(0)->free2.y1 + support.at(0)->free2.y2) / 4;
			vector_segment sweep = vector_segment(common_ellip->center_x,
				common_ellip->center_y, center_x, center_y);
			for (int k = 1; k < support.size(); k++) {
				center_x = (support.at(j)->free1.x1 + support.at(j)->free1.x2 +
					support.at(j)->free2.x1 + support.at(j)->free2.x2) / 4;
				center_y = (support.at(j)->free1.y1 + support.at(j)->free1.y2 +
					support.at(j)->free2.y1 + support.at(j)->free2.y2) / 4;
				vector_segment temp = vector_segment(common_ellip->center_x,
					common_ellip->center_y, center_x, center_y);
				if (sweep.leftOn(temp.x2, temp.y2) == dir) {
					index = k;
					sweep = temp;
				}
			}
			// check if that point is already in the path
			bool found = false;
			for (int k = 0; k < local_path->size(); k++) {
				if (local_path->at(k).coords.equals(Point(sweep.x2, sweep.y2))) {
					found = true;
					break;
				}
			}
			if (found == false)
				local_path->insert(local_path->begin() + i, local_path_node(sweep.x2, sweep.y2, support.at(index)));
			support.erase(support.begin() + index);
		}
	}

	for (int i = 0; i < local_path->size(); i++) {
		for (int j = local_path->size() - 1; j > i; j--) {
			if (local_path->at(i).coords.equals(local_path->at(j).coords)) {
				for (int k = i; k < j; k++) {
					local_path->erase(local_path->begin() + i);
				}
				i--;
				break;
			}
		}
	}
}

void elliptical_approx::local_path_planning(std::vector<quad*>* feasible_path, std::vector<local_path_node>* local_path) {
	local_path->push_back(local_path_node(start_x, start_y, nullptr));
	float center_x0 = (feasible_path->at(0)->free1.x1 + feasible_path->at(0)->free1.x2 +
		feasible_path->at(0)->free2.x1 + feasible_path->at(0)->free2.x2) / 4;
	float center_y0 = (feasible_path->at(0)->free1.y1 + feasible_path->at(0)->free1.y2 +
		feasible_path->at(0)->free2.y1 + feasible_path->at(0)->free2.y2) / 4;
	local_path->push_back(local_path_node(center_x0, center_y0, feasible_path->at(0)));
	for (int i = 1; i < feasible_path->size(); i++) {
		/*/float exchange_zone[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
		bool intersects = scrape_common_area_measure(feasible_path->at(i - 1), feasible_path->at(i), exchange_zone);
		if (intersects) {
			Point center = Point(exchange_zone[3], exchange_zone[4]);
			Point far_point = Point(exchange_zone[5], exchange_zone[6]);
			float ratio = (1 + 2 / exchange_zone[2]);
			Point pivot = Point((ratio * far_point.x + center.x) / (ratio + 1),
				(ratio * far_point.y + center.y) / (ratio + 1));
			local_path->push_back(pivot);
		}*/
		center_x0 = (feasible_path->at(i)->free1.x1 + feasible_path->at(i)->free1.x2 +
			feasible_path->at(i)->free2.x1 + feasible_path->at(i)->free2.x2) / 4;
		center_y0 = (feasible_path->at(i)->free1.y1 + feasible_path->at(i)->free1.y2 +
			feasible_path->at(i)->free2.y1 + feasible_path->at(i)->free2.y2) / 4;
		local_path->push_back(local_path_node(center_x0, center_y0, feasible_path->at(i)));
	}
	center_x0 = (feasible_path->at(feasible_path->size() - 1)->free1.x1 + feasible_path->at(feasible_path->size() - 1)->free1.x2 +
		feasible_path->at(feasible_path->size() - 1)->free2.x1 + feasible_path->at(feasible_path->size() - 1)->free2.x2) / 4;
	center_y0 = (feasible_path->at(feasible_path->size() - 1)->free1.y1 + feasible_path->at(feasible_path->size() - 1)->free1.y2 +
		feasible_path->at(feasible_path->size() - 1)->free2.y1 + feasible_path->at(feasible_path->size() - 1)->free2.y2) / 4;
	local_path->push_back(local_path_node(center_x0, center_y0, feasible_path->at(feasible_path->size() - 1)));
	local_path->push_back(local_path_node(goal_x, goal_y, nullptr));
}

int elliptical_approx::smoothen_paths(int index, std::vector<quad*>* path_list) {
	float exchange_zone[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	bool intersects = scrape_common_area_measure(
		path_list->at(index - 1), path_list->at(index), exchange_zone);

	// find the common ellipse obstacle for the two quads
	ellipse* common_obstacle, *obstacle1, *obstacle2;
	vector_segment sweep_start, sweep_end;

	if (path_list->at(index - 1)->me_A == path_list->at(index)->me_A) {
		common_obstacle = path_list->at(index)->me_A;
		sweep_start = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index - 1)->me_B->center_x,
			path_list->at(index - 1)->me_B->center_y);
		sweep_end = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index)->me_B->center_x,
			path_list->at(index)->me_B->center_y);
		obstacle1 = path_list->at(index - 1)->me_B;
		obstacle2 = path_list->at(index)->me_B;
	}
	else if (path_list->at(index - 1)->me_A == path_list->at(index)->me_B) {
		common_obstacle = path_list->at(index)->me_B;
		sweep_start = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index - 1)->me_B->center_x,
			path_list->at(index - 1)->me_B->center_y);
		sweep_end = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index)->me_A->center_x,
			path_list->at(index)->me_A->center_y);
		obstacle1 = path_list->at(index - 1)->me_B;
		obstacle2 = path_list->at(index)->me_A;
	}
	else if (path_list->at(index - 1)->me_B == path_list->at(index)->me_A) {
		common_obstacle = path_list->at(index)->me_A;
		sweep_start = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index - 1)->me_A->center_x,
			path_list->at(index - 1)->me_A->center_y);
		sweep_end = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index)->me_B->center_x,
			path_list->at(index)->me_B->center_y);
		obstacle1 = path_list->at(index - 1)->me_A;
		obstacle2 = path_list->at(index)->me_B;
	}
	else if (path_list->at(index - 1)->me_B == path_list->at(index)->me_B) {
		common_obstacle = path_list->at(index)->me_B;
		sweep_start = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index - 1)->me_A->center_x,
			path_list->at(index - 1)->me_A->center_y);
		sweep_end = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
			path_list->at(index)->me_A->center_x,
			path_list->at(index)->me_A->center_y);
		obstacle1 = path_list->at(index - 1)->me_A;
		obstacle2 = path_list->at(index)->me_A;
	}
	else
		return 0;
	render_agent.draw_quad(path_list->at(index - 1), Color(1.0, 1.0, 1.0));
	render_agent.draw_quad(path_list->at(index), Color(0.0, 0.0, 0.0));
	render_agent.draw_ellipse(*common_obstacle);
	render_agent.invalidate();
	std::cin.ignore();
	render_agent.clear_paths(44);
	render_agent.add_path(sweep_start, Color(1.0, 1.0, 1.0));
	render_agent.add_path(sweep_end, Color(0.0, 0.0, 0.0));
	std::cin.ignore();
	render_agent.clear_paths(2);

	// create a list of quad with its sweep vector lying between sweep_start and sweep_end
	std::vector<quad*> options;
	std::vector<quad>* quads = &(map_builder.quad_list);

	for (int i = 0; i < quads->size(); i++) {
		if ((&quads->at(i) == path_list->at(index - 1)) ||
			(&quads->at(i) == path_list->at(index)))
			continue;
		vector_segment sweep;
		if (quads->at(i).me_A == common_obstacle) {
			sweep = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
				quads->at(i).me_B->center_x, quads->at(i).me_B->center_y);
		}
		else if (quads->at(i).me_B == common_obstacle) {
			sweep = vector_segment(common_obstacle->center_x, common_obstacle->center_y,
				quads->at(i).me_A->center_x, quads->at(i).me_A->center_y);
		}
		else
			continue;

		// check if this sweep is between sweep_start and sweep_end
		if (sweep_start.leftOn(sweep.x2, sweep.y2) != sweep.leftOn(sweep_end.x2, sweep_end.y2)) {
			continue;
		}
		else {
			options.push_back(&quads->at(i));
			render_agent.add_path(sweep, Color(1.0, 0.0, 0.0));
		}
	}
	render_agent.invalidate();
	std::cin.ignore();
	render_agent.clear_paths(options.size());

	/*/for (int i = 0; i < options.size(); i++) {
		path_list->insert(path_list->begin() + index, options.at(i));
	}
	return options.size();*/
	return 0;
}

bool elliptical_approx::check_contains(std::vector<quad*>* quads, std::vector<vector_segment>* edges) {
	quad* this_quad = quads->at(0);
	quads->erase(quads->begin());
	for (int i = 0; i < edges->size(); i++) {
		if (edges->at(i).length()<FLOATING_PRECISION) {
			edges->erase(edges->begin() + i);
			i--;
			continue;
		}
		bool test_end1 = this_quad->isInside(edges->at(i).x1, edges->at(i).y1);
		bool test_end2 = this_quad->isInside(edges->at(i).x2, edges->at(i).y2);
		if (test_end1 && test_end2) {
			edges->erase(edges->begin() + i);
			i--;
			continue;
		}
		Point point;
		if (this_quad->free1.isIntersecting(&edges->at(i))) {
			point = this_quad->free1.intersection_points(&edges->at(i));
		}
		else if (this_quad->free2.isIntersecting(&edges->at(i))) {
			point = this_quad->free2.intersection_points(&edges->at(i));
		}
		else
			continue;

		if (!test_end1 && !test_end2) {
			if (point.equals(Point(edges->at(i).x1, edges->at(i).y1)))
				continue;
			if (point.equals(Point(edges->at(i).x2, edges->at(i).y2)))
				continue;
			vector_segment segment2 = vector_segment(edges->at(i).x1, edges->at(i).y1,
				point.x, point.y);
			edges->push_back(segment2);
			edges->at(i).x1 = point.x;
			edges->at(i).y1 = point.y;
			i--;
			continue;
		}
		if (test_end1 == false) {
			edges->at(i).x2 = point.x;
			edges->at(i).y2 = point.y;
			continue;
		}
		if (test_end2 == false) {
			edges->at(i).x1 = point.x;
			edges->at(i).y1 = point.y;
			continue;
		}
	}
	if (!quads->empty())
		check_contains(quads, edges);
	if (edges->empty())
		return true;
	return false;
}

float elliptical_approx::path_length(std::vector<local_path_node>* path) {
	float length = 0.0f;
	for (int i = 1; i < path->size(); i++) {
		length += CompGeomFuncEllipseApprox::distance(path->at(i - 1).coords.x
			, path->at(i - 1).coords.y, path->at(i).coords.x, path->at(i).coords.y);
	}
	return length;
}

std::string elliptical_approx::stringify(std::vector<local_path_node>* path) {
	std::string order = "[";
	for (int i = 0; i < path->size(); i++) {
		order = order + " {" + std::to_string(path->at(i).coords.x) + ", " + 
			std::to_string(path->at(i).coords.y) + "}, ";
	}
	order = order + "]";
	return order;
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
		//printf("area = %f. factor = %f. vertices = %d. bridge_width = %f. skewness = %f\n",
		//	area, independency_factor, ear->size(), bridge_width, skewness);
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
			bool concave_ear = leftOnly(
				data_x[poly_indice->at(0).at(edge2)], data_y[poly_indice->at(0).at(edge2)],
				data_x[poly_indice->at(0).at(pt)], data_y[poly_indice->at(0).at(pt)],
				data_x[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())],
				data_y[poly_indice->at(0).at((pt + i) % poly_indice->at(0).size())]);
			if (concave_ear) {
				//if ((edge2+i-1)%len(poly_indice[0]) > start):
				//   end = (edge2 + i - 1) % len(poly_indice[0])
				end = (pt + i - 1) % poly_indice->at(0).size();
				break;
			}
		}
		else {
			end = (edge2 + i) % poly_indice->at(0).size();
			break;
		}
	}

	if (start != end) { // separate out identified ear from the raw polygon
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
	if (show_logs) {
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
}

void convex_clustering::clustering(void) {
	std::vector<std::vector<int>> poly_indice = { {} };
	for (int i = 0; i < 107; i++) {
		poly_indice.at(0).push_back(i);
	}
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
			}
				index = (index + 1) % poly_indice.at(0).size();
			// plot_current_distribution()
	}

	print_poly_indice(&poly_indice);
	render_agent->draw_ears(data_x, data_y, &poly_indice);
}

void convex_clustering::filter_dimples_2(std::vector<result>* traversal, std::vector<result>* filter, int filter_size) {
	for (int i = 0; i < traversal->size(); i++) {
		std::vector<int> ear;
		for (int k = 0; k < filter_size; k++) {
			//std::cout << ((traversal->at(i).i + k) % data_size) << "\n";
			ear.push_back((traversal->at(i).i + k) % data_size);
		}
		ear_area_2(&ear, filter, false);
	}
	//std::cin.ignore();
	render_agent->clear_path();
	//std::cin.ignore();
}

void convex_clustering::filter_dimples(std::vector<result>* traversal, std::vector<result>* filter, int filter_size) {
	for (int i = 0; i < traversal->size(); i++) {
		std::vector<int> ear;
		for (int k = 0; k < filter_size; k++) {
			//std::cout << ((traversal->at(i).i + k) % data_size) << "\n";
			ear.push_back((traversal->at(i).i + k) % data_size);
		}
		ear_area(&ear, data_x, data_y, render_agent, filter, false);
	}
	std::cin.ignore();
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

	// extracting good dimples
	for (int i = 0; i < old_vertices_set->size(); i++) {
		if (old_vertices_set->at((i + 1) % old_vertices_set->size()).i - old_vertices_set->at(i).i == 1) {
			// new set of interest zone
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
		node_list->at(points[old_vertices_set->at(i).i + 2].y * GRID_WIDTH + points[old_vertices_set->at(i).i + 2].x).type = START;
	}

	// pair dimples
	std::vector<std::vector<int>> clustered_data = { {} };
	for (int i = 0; i < data_size; i++) {
		clustered_data.at(0).push_back(i);
	}
	clustrify_v2(old_vertices_set, &clustered_data);

	//print_poly_indice(&clustered_data);

	for (int i = 0; i < clustered_data.size(); i++) {
		clustered_points.push_back(std::vector<int_point>());
		int last_index = clustered_points.size()-1;
		for (int j = 0; j < clustered_data.at(i).size(); j++) {
			clustered_points.at(last_index).push_back(points.at(clustered_data.at(i).at(j)));
		}
	}

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
		node_list->at(data_y[fourth_filter[i].i + 1] * GRID_WIDTH + data_x[fourth_filter[i].i + 1]).type = START;
	}

	// pair dimples
	std::vector<std::vector<int>> clustered_data = { {} };
	for (int i = 0; i < data_size; i++) {
		clustered_data.at(0).push_back(i);
	}
	clustrify(&fourth_filter, &clustered_data);

	print_poly_indice(&clustered_data);


	std::cin.ignore();
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
	if (axi_max < 0) {
		//std::cout << "[POSSIBLE DELETION] : axi_max negative\n";
		return;
	}
	if (axi_min < 0) {
		//std::cout << "[POSSIBLE DELETION] : axi_min negative\n";
		return;
	}

	this->ellipse_list.push_back(ellipse(x0, y0, axi_max, axi_min, theta));
	//render_agent->draw_ellipse(ellipse_list.at(ellipse_list.size()-1));

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
		obstacles.at(i).visibility_handler_reloaded(render_agent);
		int starting_index = 0;
		for (int j = 0; j < obstacles.at(i).neighbours.size(); j++) {

			if (j == 0)
				starting_index = quad_list.size();
			ellipse::neighbourSweep sweep = obstacles.at(i).neighbours.at(j);

			// check if this quad is already registered in list
			bool flag_available = false;
			for (int k = 0; k < quad_list.size(); k++) {
				if (quad_list.at(k).me_A == &obstacles.at(i)) {
					if (quad_list.at(k).me_B == sweep.pointer) {
						flag_available = true;
						break;
					}
				}
				if (quad_list.at(k).me_B == &obstacles.at(i)) {
					if (quad_list.at(k).me_A == sweep.pointer) {
						flag_available = true;
						break;
					}
				}
			}
			if (flag_available)
				continue;

			quad_list.push_back(quad(
				&obstacles.at(i),
				sweep.pointer,
				sweep.visibility_limit_left,
				sweep.visibility_limit_right,
				quad_list.size()));
			//render_agent->visualize_quads(&quad_list.at(quad_list.size()-1), true, true);
		}
	}

	// STEP II.5 Remove too much skewed out quads -- these includes degenerate cases as well
	float mfa = 0.0, mpl = 0.0, mpw = 0.0;
	for (int i = 0; i < quad_list.size(); i++) {
		mfa += quad_list.at(i).mfa;
		mpl += quad_list.at(i).mpl;
		mpw += quad_list.at(i).mpw;
	}
	mfa = mfa / quad_list.size();
	mpl = mpl / quad_list.size();
	mpw = mpw / quad_list.size();
	for (int i = 0; i < quad_list.size(); i++) {
		if (quad_list.at(i).mpl < mpl / 3 || quad_list.at(i).mpw < mpw / 3) {
			quad_list.erase(quad_list.begin() + i);
			i--;
		}
	}

	// Step III : create quad map
	// stitch quads into map
	for (int i = 0; i < obstacles.size(); i++) {
		for (int j = 0; j < quad_list.size(); j++) {
			bool flag_A_j = (quad_list.at(j).me_A == &obstacles.at(i));
			bool flag_B_j = (quad_list.at(j).me_B == &obstacles.at(i));

			if (!flag_A_j && !flag_B_j)
				continue;
			for (int k = j+1; k < quad_list.size(); k++) {
				bool flag_A_k = (quad_list.at(k).me_A == &obstacles.at(i));
				bool flag_B_k = (quad_list.at(k).me_B == &obstacles.at(i));

				if (!flag_A_k && !flag_B_k)
					continue;
				/* by this point we have got every pair of quads which share a
					common ellipse obstacle[i] */
				quad_list.at(j).common_area_v2(&quad_list.at(k), &obstacles.at(i), render_agent);
			}
		}
	}

	// Step III : create quad map
	// sorting quadlist;


	/*/std::vector<int> indices;
	for (int i = 0; i < quad_list.size(); i++) {
		indices.push_back(i);
	}
	while (!indices.empty()) {
		float importance = quad_list.at(indices.at(0)).importance_function1();
		int index = 0;
		//std::cout << "index : " << index << " importance : " << importance << "\n";
		for (int i = 1; i < indices.size(); i++) {
			float temp = quad_list.at(indices.at(i)).importance_function1();
			//std::cout << "index : " << j << " importance : " << temp << "\n";
			if (temp > importance) {
				importance = temp;
				index = i;
			}
		}
		//std::cin.ignore();
		order.push_back(indices.at(index));
		indices.erase(indices.begin() + index);
	}
	indices.clear();

	for (int i = 0; i < quad_list.size(); i++) {
		quad* path = &quad_list.at(i);

		// sorting each of the neighbours' list -- series A;
		indices.clear();
		for (int j = 0; j < path->neighbours_A.size(); j++) {
			indices.push_back(j);
		}
		while (!indices.empty()) {
			float importance = path->neighbours_A.at(indices.at(0)).importance_function1();
			int index = 0;

			for (int j = 1; j < indices.size(); j++) {
				float temp = path->neighbours_A.at(indices.at(j)).importance_function1();
				if (temp > importance) {
					importance = temp;
					index = j;
				}
			}
			path->order_A.push_back(indices.at(index));
			indices.erase(indices.begin() + index);
		}
		indices.clear();

		// sorting each of the neighbours' list -- series B;
		indices.clear();
		for (int j = 0; j < path->neighbours_B.size(); j++) {
			indices.push_back(j);
		}
		while (!indices.empty()) {
			float importance = path->neighbours_B.at(indices.at(0)).importance_function1();
			int index = 0;

			for (int j = 1; j < indices.size(); j++) {
				float temp = path->neighbours_B.at(indices.at(j)).importance_function1();
				if (temp > importance) {
					importance = temp;
					index = j;
				}
			}
			path->order_B.push_back(indices.at(index));
			indices.erase(indices.begin() + index);
		}
		indices.clear();
	}*/
}

void quad_builder::stich_quads(void) {
	/*
	* Step I : Keep Iterating through quad-list : we will add one quad per iteration 
				and *remove* it from the list -- need not remove as well
	* Step II : In each iteration : traverse through each quad in stitched map; if any 
				quad intersects the quad of interest. Add this quad to their list of neighbours
	* * For all this maintain a bi-directional tree and mechanism to traverse through it.
	*/

	/*
	* Idea II : 
	* For the map traversal part. 
	* Let it be in the vector format.
	* -- For each quad in quad_list, run a nested loop to iterate through every other 
	*/

	std::vector<bool> checklist(quad_list.size(), false);
	quad_map = &quad_list.at(0);

	for (int i = 0; i < quad_list.size(); i++) {

		// add this quad to all the quads in the map that intersects with it.
		quad_map->map_expander(&quad_list.at(i), &checklist, render_agent);
		render_agent->visualize_quads(&quad_list.at(i), false, false);

		for (int j = 0; j < quad_list.size(); j++) {
			checklist.at(j) = false;
		}
	}
}