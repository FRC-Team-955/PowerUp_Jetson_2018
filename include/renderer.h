#ifndef RENDERER_H
#define RENDERER_H
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <tinysplinecpp.h>
#include <sq_derivable.h>

namespace Renderer {
	extern void init();
	extern void display();
	extern void draw_string();
	extern void color_by(float input);
	extern void render_spline(tinyspline::BSpline* spline);
	extern void draw_string(float x, float y, char *input);
	extern void bound(cv::Rect2f input, float max_z);
	extern void grid(float step_x, float step_y, float r, float g, float b, cv::Rect2f screen_dim);
	extern void clear();
	extern void draw_robot(float angle, cv::Point2f position, float length, float width, float r, float g, float b);
	extern cv::Rect2f get_spline_size(tinyspline::BSpline* spline);
	void render_function_tank_drive (
			SQDerivable* function,
			float wheel_distance, 
			float max_change_time);
	
} // namespace Renderer
#endif
