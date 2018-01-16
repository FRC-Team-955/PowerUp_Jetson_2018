#include <field_renderer.h>

namespace FD = FieldDimension;

void FieldRenderer::render(char* config) {
	//Field bounds
	render_rect(FD::field_bounds, 1.0, 1.0, 1.0);

	//Left plate
	render_rect(FD::Switch::left_plate, 1.0, 0.1, 0.1);
	render_rect(FD::Switch::right_plate, 0.1, 0.1, 1.0);
	render_rect(FD::Switch::boom, 0.8, 0.8, 0.8);
}

void FieldRenderer::render_rect(cv::Rect2f input, float r, float g, float b) {
	glLineWidth(2);
	glColor3f(r, g, b);
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
