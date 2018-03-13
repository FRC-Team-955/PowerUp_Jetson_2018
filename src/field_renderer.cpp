#include <field_renderer.h>

namespace FD = FieldDimension;

void FieldRenderer::render(JetsonCommand::Setup::LayoutBits bits) {
	cv::Point3f home_color = bits & JetsonCommand::Setup::LayoutBits::we_are_blue ? blue : red;
	cv::Point3f away_color = bits & JetsonCommand::Setup::LayoutBits::we_are_blue ? red : blue;

	//Field bounds
	color_by_3f(home_color);
	glBegin(GL_LINES);
	glVertex2f(0.0, FD::side_station_h);	
	glVertex2f(FD::side_station_w, 0.0);	
	glVertex2f(FD::field_back_w, 0.0);	
	glVertex2f(FD::field_back_w + FD::side_station_w, FD::side_station_h);	
	glEnd();
	render_rect(FD::field_bounds);

	//Switch
	color_by_3f(grey);
	render_rect(FD::Switch::boom);
	
	color_by_3f(bits & JetsonCommand::Setup::LayoutBits::switch_left ? home_color : away_color);
	render_rect(FD::Switch::left_plate);
	
	color_by_3f(bits & JetsonCommand::Setup::LayoutBits::switch_left ? away_color : home_color);
	render_rect(FD::Switch::right_plate);

	//Scale
	color_by_3f(grey);
	render_rect(FD::Scale::boom);
	
	color_by_3f(bits & JetsonCommand::Setup::LayoutBits::scale_left ? home_color : away_color);
	render_rect(FD::Scale::left_plate);
	
	color_by_3f(bits & JetsonCommand::Setup::LayoutBits::scale_left ? away_color : home_color);
	render_rect(FD::Scale::right_plate);

#undef RENDER_SWSC
}

void FieldRenderer::render(char* config, bool we_are_blue) {

	//Field bounds
	color_by_3f(we_are_blue ? blue : red);
	glBegin(GL_LINES);
	glVertex2f(0.0, FD::side_station_h);	
	glVertex2f(FD::side_station_w, 0.0);	
	glVertex2f(FD::field_back_w, 0.0);	
	glVertex2f(FD::field_back_w + FD::side_station_w, FD::side_station_h);	
	glEnd();

	render_rect(FD::field_bounds);

#define RENDER_SWSC(NAME, NUM) \
	color_by_3f(grey); \
	render_rect(FD::NAME::boom); \
	\
	color_by_3f((we_are_blue == (config[NUM] == 'L')) ? blue : red);  \
	render_rect(FD::NAME::left_plate); \
	\
	color_by_3f((we_are_blue == (config[NUM] == 'R')) ? blue : red);  \
	render_rect(FD::NAME::right_plate);

	RENDER_SWSC(Switch, 0);
	RENDER_SWSC(Scale, 1);

#undef RENDER_SWSC
}

void FieldRenderer::render_rect(cv::Rect2f input) {
	glLineWidth(2);
	glBegin(GL_LINES);

	glVertex2f(input.tl().x, input.tl().y);
	glVertex2f(input.br().x, input.tl().y);

	glVertex2f(input.br().x, input.tl().y);
	glVertex2f(input.br().x, input.br().y);

	glVertex2f(input.br().x, input.br().y);
	glVertex2f(input.tl().x, input.br().y);

	glVertex2f(input.tl().x, input.br().y);
	glVertex2f(input.tl().x, input.tl().y);
	glEnd();
}

void FieldRenderer::color_by_3f(cv::Point3f rgb) {
	glColor3f(rgb.x, rgb.y, rgb.z);
}
