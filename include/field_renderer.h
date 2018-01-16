#ifndef FIELD_RENDERER_H
#define FIELD_RENDERER_H

#include <field_dimensions.h>
#include <opencv2/opencv.hpp>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

class FieldRenderer {
	public:
		FieldRenderer(char* config);
		cv::Rect2f render();
	private:
		void render_rect(cv::Rect2f input, float r, float g, float b);
};

#endif
