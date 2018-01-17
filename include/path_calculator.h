#ifndef PATH_CALCULATOR_H
#define PATH_CALCULATOR_H
#include <cmath>
#include <misc_math.h>
#include <opencv2/opencv.hpp>
#include <renderer.h>
#include <tinysplinecpp.h>
#include <vector>
#include <socket.h>
#include <memory>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <shared_network_types.h>

class Path {
	public:

		Path(tinyspline::BSpline spline, float wheel_distance, float max_change_time, bool reverse);
		bool next_point(TankDriveMotionUnit* output);

		//Inherited members TODO: Remove from header?
		void render();
		tinyspline::BSpline spline;

		struct Traversal { //Contains info about the persistent info along a path
			float spline_index = 0.0;
			float left_accum = 0.0;
			float right_accum = 0.0;
		} official_traversal; //Official is the front facing one for external use
		void get_current_loc_nondestruct(cv::Point2f* out_center, float* out_angle);
	private: 
		bool next_point_raw(TankDriveMotionUnit* output, Traversal* traversal, cv::Point2f* out_left, cv::Point2f* out_right);

		tinyspline::BSpline spline_derive;
		tinyspline::BSpline spline_derive_sq;

		const float pi = acos(-1);

		float wheel_distance = 0.0;
		float max_change_time = 0.0;
		bool reverse = false;
};
#endif
