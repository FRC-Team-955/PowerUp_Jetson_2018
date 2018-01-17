#ifndef FIELD_RENDERER_H
#define FIELD_RENDERER_H

#include <field_dimensions.h>
#include <opencv2/opencv.hpp>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

namespace FieldRenderer {
	void render(char* config, bool we_are_blue);
	void render_rect(cv::Rect2f input, cv::Point3f rgb);
};

#endif
