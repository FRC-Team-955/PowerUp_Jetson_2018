#include <field_renderer.h>

namespace FD = FieldDimension;

FieldRenderer::FieldRenderer (char* config) {
}

cv::Rect2f FieldRenderer::render() {
	//Field bounds
	cv::Rect2f field_bounds (0, 0, FD::field_width, FD::field_height);	
	render_rect(field_bounds, 1.0, 1.0, 1.0);

	//Left plate
	cv::Rect2f left_switch_plate = cv::Rect2f(
			FD::switch_plate_horizontal_offset,
			FD::switch_plate_vertical_offset,
			FD::switch_plate_width,
			FD::switch_plate_height
			);
	render_rect(left_switch_plate, 1.0, 0.1, 0.1);

	cv::Rect2f right_switch_plate = cv::Rect2f(
			FD::switch_plate_horizontal_offset + FD::switch_boom_width + FD::switch_plate_width,
			FD::switch_plate_vertical_offset,
			FD::switch_plate_width,
			FD::switch_plate_height
			);
	render_rect(right_switch_plate, 0.1, 0.1, 1.0);

	cv::Rect2f switch_boom = cv::Rect2f(
			FD::switch_plate_horizontal_offset + FD::switch_plate_width,
			FD::switch_plate_vertical_offset + (FD::switch_plate_height / 2.0) - (FD::switch_boom_height / 2.0),
			FD::switch_boom_width,
			FD::switch_boom_height
			);
	render_rect(switch_boom, 0.8, 0.8, 0.8);

	cv::Rect2f display_bounds = field_bounds;
	/*
	display_bounds.x *= 1.1;
	display_bounds.y *= 1.1;
	display_bounds.width *= 1.2;
	display_bounds.height += 1.2;
	*/
	return field_bounds;
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
