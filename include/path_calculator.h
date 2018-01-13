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

class Path {
	public:
		struct TalonPoint {
			float position_left;
			float velocity_left;
			float position_right;
			float velocity_right;
			float delta_time;
			enum Special {
				Beginning = 0,
				Middle = 1,
				End = 2
			} special;
		};

		Path(tinyspline::BSpline spline, float wheel_distance, float max_change_time);
		bool next_point(TalonPoint* output);

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
		bool next_point_raw(TalonPoint* output, Traversal* traversal, cv::Point2f* out_left, cv::Point2f* out_right);

		tinyspline::BSpline spline_derive;
		tinyspline::BSpline spline_derive_sq;

		const float pi = acos(-1);

		float wheel_distance = 0.0;
		float max_change_time = 0.0;
};
#endif
