#ifndef FIELD_RENDERER_H
#define FIELD_RENDERER_H

#include <field_dimensions.h>
#include <opencv2/opencv.hpp>
#include <shared_network_types.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

namespace FieldRenderer {

	const cv::Point3f white (1.0, 1.0, 1.0);
	const cv::Point3f grey (0.8, 0.8, 0.8);
	const cv::Point3f red (1.0, 0.3, 0.3);
	const cv::Point3f blue (0.2, 0.5, 1.0);

	void render(char* config, bool we_are_blue);
	void render(JetsonCommand::Setup::LayoutBits bits);
	void render_rect(cv::Rect2f input);
	void color_by_3f(cv::Point3f rgb);
};

#endif
