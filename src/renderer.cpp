#include <renderer.h>

namespace Renderer {
	void clear() { glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }

	void reshape(int w, int h) { 
		int min = std::min(w, h);
		glViewport(0, 0, (GLsizei)min, (GLsizei)min);
	}

	void draw_string(float x, float y, char *input) {
		glRasterPos2f(x, y);
		size_t len = (int)strlen(input);
		for (size_t i = 0; i < len; i++)
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, input[i]);
	}

	GLUnurbsObj *nurbs;
	void render_spline(tinyspline::BSpline* spline)
	{
		if (spline->dim() == 3) {
			glColor3f(0.8, 0.8, 0.8); //TODO: Make this configurable
			glLineWidth(5);
			gluBeginCurve(nurbs);
			gluNurbsCurve(nurbs, (GLint)spline->nKnots(), &spline->knots()[0],
					(GLint)spline->dim(), &spline->ctrlp()[0],
					(GLint)spline->order(), GL_MAP1_VERTEX_3);
			gluEndCurve(nurbs);

			glColor3f(1.0, 0.0, 0.0);
			glPointSize(9);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < spline->nCtrlp(); i++)
				glVertex3fv(&spline->ctrlp()[spline->dim() * i]);
			glEnd();

			/*
			char string_buf[300];
			for (size_t i = 0; i < spline->nCtrlp() * spline->dim(); i += 3) {
				sprintf(string_buf, "(%f, %f)", spline->ctrlp()[i], spline->ctrlp()[i + 1]);
				draw_string(spline->ctrlp()[i], spline->ctrlp()[i + 1], string_buf);
			}
			*/
		}
	}

	cv::Rect2f get_spline_size(tinyspline::BSpline* spline)
	{
		cv::Rect2f rect;
		for (int i = 0; i < spline->nCtrlp(); i++) {
			cv::Point2f position = cv::Point2f(spline->ctrlp()[i * spline->dim()], spline->ctrlp()[(i * spline->dim()) + 1]);

			if (position.x < rect.x) {
				rect.x = position.x;
			}
			if (position.y < rect.y) {
				rect.y = position.y;
			}
			if (position.x > rect.br().x) {
				rect.width = fabs(rect.x - position.x);
			}
			if (position.y > rect.br().y) {
				rect.height = fabs(rect.y - position.y);
			}
		}
		return rect;
	}

	void color_by(float input) { //Green to black to red from 1.0 to 0.0 to -1.0 respectively
		if (input > 0) {
			glColor3f(0.0, input, 0.0);
		} else {
			glColor3f(fabs(input), 0.0, 0.0);
		}
	}

	void draw_robot(float angle, cv::Point2f position, float length, float width, float r, float g, float b) {
		glColor3f(r, g, b); //TODO: Make this configurable
		glLineWidth(3);
		std::vector<cv::Point2f> wireframe;
		wireframe.push_back(cv::Point2f ((length / 2), (width / 2)));
		wireframe.push_back(cv::Point2f (-(length / 2), (width / 2)));
		wireframe.push_back(cv::Point2f (-(length / 2), -(width / 2)));
		wireframe.push_back(cv::Point2f ((length / 2), -(width / 2)));

		cv::Mat rot_mat( 2, 3, CV_32FC1 );
   	rot_mat = cv::getRotationMatrix2D(cv::Point2f(0.0, 0.0), -angle * (180.0 / acos(-1)), 1.0);
		cv::transform(wireframe, wireframe, rot_mat);

		cv::Point2f last = wireframe.back();
		glBegin(GL_LINES);
		for (auto& point : wireframe) {
			glVertex2f(point.x + position.x, point.y + position.y);
			glVertex2f(last.x + position.x, last.y + position.y);
			last = point;
		}
		glEnd();
	}

	void grid(float step_x, float step_y, float r, float g, float b, cv::Rect2f screen_dim)
	{
		screen_dim.x -= step_x;
		screen_dim.y -= step_y;
		screen_dim.width += step_x;
		screen_dim.height += step_y;
		glLineWidth(1);
		glColor3f(r, g, b);
		for (float i = step_y * (int)(screen_dim.y / step_y); i < screen_dim.br().y; i += step_y) {
			if (i == 0.0) {
				glColor3f(r + 0.2, g + 0.2, b + 0.2);
			} else {
				glColor3f(r, g, b);
			}
			glBegin(GL_LINES);
			glVertex2f(screen_dim.x, i);
			glVertex2f(screen_dim.br().x, i);
			glEnd();
		}
		for (float i = step_x * (int)(screen_dim.x / step_x); i < screen_dim.br().x; i += step_x) {
			if (i == 0.0) {
				glColor3f(r + 0.2, g + 0.2, b + 0.2);
			} else {
				glColor3f(r, g, b);
			}
			glBegin(GL_LINES);
			glVertex2f(i, screen_dim.y);
			glVertex2f(i, screen_dim.br().y);
			glEnd();
		}
	}

	//TODO: Simplify the crap out of this
	//      Add a center line
	//      Find a better way to store progress through the spline
	void render_function_tank_drive (
			SQDerivable* function,
			float wheel_distance, 
			float max_change_time) {
		float temp_copy = function->index;
		function->index = 0.0;
		function->advance(0.0);
		cv::Point3f last_left;
		cv::Point3f last_right;
		TankDriveCalculator::TankOutput output = TankDriveCalculator::evaluate(
				function, 
				wheel_distance, 
				max_change_time, 
				false, 
				true);
		glColor3f(0.0, 0.0, 0.0);
		glLineWidth(3);
		glBegin(GL_LINES);

		last_left = output.left_position;
		last_right = output.right_position;

		do {
			output = TankDriveCalculator::evaluate(
					function, 
					wheel_distance, 
					max_change_time, 
					false, 
					true);
			Renderer::color_by(output.motion.velocity_left);
			glVertex2f(last_left.x, last_left.y);
			glVertex2f(output.left_position.x, output.left_position.y);

			Renderer::color_by(output.motion.velocity_right);
			glVertex2f(last_right.x, last_right.y);
			glVertex2f(output.right_position.x, output.right_position.y);
			last_left = output.left_position;
			last_right = output.right_position;
		} while (output.motion.special != TankDriveMotionUnit::Special::End);
		glEnd();

		//TODO: Write non-destructive advance function, or even a stack of different function slots
		function->index = temp_copy;
		function->advance(0.0);
	}

	void bound(cv::Rect2f input, float max_z)
	{
		glLoadIdentity();
		float min = std::min(input.tl().x, input.tl().y);
		float max = std::max(input.br().x, input.br().y);
		glOrtho(min, max, min, max, max_z, -max_z);
	}

	void display() {
		glutSwapBuffers();
		glutPostRedisplay();
		glutMainLoopEvent();
	}

	void fake_display () {}

	void init()
	{
		glClearColor(0.0, 0.0, 0.0, 0.0);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		int argc = 0;
		glutInit(&argc, nullptr);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
		glutInitWindowSize(1400, 1400);
		glutInitWindowPosition(0, 0);
		glutCreateWindow("Spline Display Out");
		glutReshapeFunc(reshape);
		nurbs = gluNewNurbsRenderer();
		glutDisplayFunc(fake_display);
	}
} // namespace Renderer
