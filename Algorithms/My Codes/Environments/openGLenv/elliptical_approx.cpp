#include "elliptical_approx.h"

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

local_visualizer::local_visualizer(void) {
	GRID_WIDTH = 0;
	GRID_HEIGHT = 0;
	paths = nullptr;
	renderer = nullptr;
	render_callback = nullptr;
}

local_visualizer::local_visualizer(int GRID_WIDTH, int GRID_HEIGHT, RenderClass* renderer, std::vector<Path>* paths,
	void (*func_updater)(std::vector<float>*)) {
	this->GRID_WIDTH = GRID_WIDTH;
	this->GRID_HEIGHT = GRID_HEIGHT;
		this->paths = paths; // for visualization
		this->renderer = renderer;
		this->render_callback = func_updater;
}

void local_visualizer::draw_ellipse(ellipse ellipse) {
	int DIVISIONS = 36;
	for (int i = 0; i < DIVISIONS; i++) {
		float x_i1_no_tilt = ellipse.a * cos(2 * i * PI / DIVISIONS);
		float y_i1_no_tilt = ellipse.b * sin(2 * i * PI / DIVISIONS);
		float x_i2_no_tilt = ellipse.a * cos(2 * (i+1) * PI / DIVISIONS);
		float y_i2_no_tilt = ellipse.b * sin(2 * (i+1) * PI / DIVISIONS);

		paths->push_back(Path(
			GRID_WIDTH / 2 + ellipse.center_x +
			x_i1_no_tilt * cos(ellipse.tilt) - y_i1_no_tilt * sin(ellipse.tilt),
			GRID_HEIGHT / 2 - (ellipse.center_y +
			x_i1_no_tilt * sin(ellipse.tilt) + y_i1_no_tilt * cos(ellipse.tilt)),
			GRID_WIDTH / 2 + ellipse.center_x +
			x_i2_no_tilt * cos(ellipse.tilt) - y_i2_no_tilt * sin(ellipse.tilt),
			GRID_HEIGHT / 2 - (ellipse.center_y +
			x_i2_no_tilt * sin(ellipse.tilt) + y_i2_no_tilt * cos(ellipse.tilt))
		));
	}
	invalidate();
	return;
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
	ellipse ellipse1 = ellipse(0.0f, 0.0f, 30.0f, 20.0f, 0.0 * PI / 180);
	//this->render_agent.draw_ellipse(ellipse1);
	//find_in(ellipse1);.
	//find_circle1();
	//find_circle2();
	find_ellipse1();
}

void elliptical_approx::find_in(ellipse ellipse1) {
	std::cout << "environment variables | (WIDTH, HEIGHT) -> (" << GRID_WIDTH << ", " << GRID_HEIGHT << ")\n";
	for (int i = 0; i < node_list->size(); i++) {
		int x = i % GRID_WIDTH;
		int y = i / GRID_WIDTH;
		if (ellipse1.isInside(node_list->at(i).x*GRID_WIDTH/2, node_list->at(i).y*GRID_HEIGHT/2, i))
			node_list->at(i).type = GOAL;
		else
			node_list->at(i).type = START;

		if (false) {
			std::cin.ignore();
			render_agent.invalidate();
		}
	}
}

void elliptical_approx::find_circle1(void) {
	int deviation = 100000000;
	float radii = 0.0;
	for (int r = 1; r < GRID_WIDTH / 2 && r < GRID_HEIGHT / 2; r++) {
		int mis_classified_count = 0;
		for (int i = 0; i < node_list->size(); i++) {
			ellipse circle = ellipse(0.0f, 0.0f, r, r, 0.0 * PI / 180);
			bool verdict = circle.isInside(node_list->at(i).x * GRID_WIDTH / 2, node_list->at(i).y * GRID_HEIGHT / 2, i);
			if (verdict && node_list->at(i).type == BASE_EMPTY)
				mis_classified_count++;
			if (!verdict && node_list->at(i).type == BASE_TAKEN)
				mis_classified_count++;
		}
		if (mis_classified_count < deviation) {
			deviation = mis_classified_count;
			radii = r;
		}
	}
	std::cout << "radii == " << radii << std::endl;
	std::cout << "deviation == " << deviation << std::endl;
	this->render_agent.draw_ellipse(ellipse(0.0f, 0.0f, radii, radii, 0.0 * PI / 180));
}

void elliptical_approx::find_circle2(void) {
	int deviation = 100000000;
	float radii = 0.0;
	float major_a = 0.0;
	float minor_b = 0.0;
	float center_x = 0.0;
	float center_y = 0.0;
	float tilt_ellip = 0;
	for (int tilt = 0; tilt < 360; tilt++) {
		std::cout << "tilt : " << tilt << std::endl;
		for (int x = 1; x < GRID_WIDTH / 2; x++) {
			for (int y = 1; y < GRID_HEIGHT / 2; y++) {
				for (int a = -GRID_WIDTH / 2; a < GRID_WIDTH / 2; a++) {
					for (int b = -GRID_HEIGHT / 2; b < GRID_HEIGHT / 2; b++) {
						int mis_classified_count = 0;
						for (int i = 0; i < node_list->size(); i++) {
							ellipse circle = ellipse(x, y, a, b, tilt * PI / 180);
							bool verdict = circle.isInside(node_list->at(i).x * GRID_WIDTH / 2, node_list->at(i).y * GRID_HEIGHT / 2, i);
							if (verdict && node_list->at(i).type == BASE_EMPTY)
								mis_classified_count++;
							if (!verdict && node_list->at(i).type == BASE_TAKEN)
								mis_classified_count++;
						}
						if (mis_classified_count < deviation) {
							deviation = mis_classified_count;
							major_a = a;
							minor_b = b;
							center_x = x;
							center_y = y;
							tilt_ellip = tilt;
						}
					}
				}
			}
		}
	}
	std::cout << "center_x == " << center_x << std::endl;
	std::cout << "center_y == " << center_y << std::endl;
	std::cout << "major_a == " << major_a << std::endl;
	std::cout << "minor_b == " << minor_b << std::endl;
	std::cout << "tilt_ellip == " << tilt_ellip << std::endl;
	std::cout << "deviation == " << deviation << std::endl;
	this->render_agent.draw_ellipse(ellipse(center_x, center_y, major_a, minor_b, tilt_ellip * PI / 180));
}

void elliptical_approx::find_ellipse1(void) {
	//cv::Mat img = cv::imread("E:/2022/DDP_ME18B074/Data/Images/exp6.bmp");
	//cv::namedWindow("First OpenCV Application", cv::WindowFlags::WINDOW_AUTOSIZE);
	//cv::imshow("First OpenCV Application", img);
	//cv::moveWindow("First OpenCV Application", 0, 45);
	//cv::waitKey(0);
	//cv::destroyAllWindows();
	std::cout << GRID_WIDTH << std::endl;
	int** map = new int* [GRID_HEIGHT];
	for (int i = 0; i < GRID_HEIGHT; i++) {
		map[i] = new int[GRID_WIDTH];
	}
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			map[i][j] = node_list->at(i * GRID_WIDTH + j).type;
		}
	}
	for (int i = 0; i < GRID_HEIGHT; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			if (i > 0 && j > 0 && i < GRID_HEIGHT - 1 && j < GRID_WIDTH - 1) {
				int up = node_list->at((i-1) * GRID_WIDTH + j).type;
				int down = node_list->at((i+1) * GRID_WIDTH + j).type;
				int left = node_list->at(i * GRID_WIDTH + (j-1)).type;
				int right = node_list->at(i * GRID_WIDTH + (j+1)).type;
				//if (up != down || left != right)
				if (map[i - 1][j] != map[i + 1][j] || map[i][j - 1] != map[i][j + 1])
					node_list->at(i * GRID_WIDTH + j).type = GOAL;
				else {
					node_list->at(i * GRID_WIDTH + j).type = START;
				}
			}
		}
	}
}