#ifndef SQ_DERIVABLE_H
#define SQ_DERIVABLE_H

#include <opencv2/opencv.hpp>
#include <misc_math.h>

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

using namespace MiscMath;

//TODO: Implement caching of evaluations at any index, refresh caches after advance
class SQDerivable {
	public:
		cv::Point3f position; //f(index)
		cv::Point3f velocity; //f'(index)
		cv::Point3f acceleration; //f''(index)

		//These MUST be assigned
		float start_index = 0.0;
		float stop_index = 0.0;

		//returns false if at end
		//Advance index by amount, returns true if successful, and sets position, velocity, etc.
		virtual bool evaluate(float index) = 0;

		//TODO: Move these functions to a .cpp (Headers take longer to compile/aren't cached by default
		
		//The total velocity over dj
		float velocity_magnitude_xy() {
			return cv::norm(From3f_xy(velocity));
		}

		//Derivative of the above (Over dj)
		float velocity_magnitude_derivative_xy () {
			return ((acceleration.y * velocity.y) + (acceleration.x * velocity.x)) / velocity_magnitude_xy();
		}

		//Angle of the tangent line relative to the origin
		float direction_xy() {
			return std::atan2(velocity.y, velocity.x);
		};

		//x'(j)^2 + y'(j)^2
		float sum_derivative_squares_xy () {
			return powf(velocity.x, 2.0) + powf(velocity.y, 2.0);
		}

		//Normalized perpendicular vector to the tangent line
		cv::Point2f perpendicular_unit_vector_xy () {
			return NormalTo(From3f_xy(velocity)) / velocity_magnitude_xy();
		}

		//The change in position of the unit vector above over dj
		cv::Point2f perpendicular_unit_vector_derivative_xy () {
			return ((NormalTo(From3f_xy(acceleration)) * velocity_magnitude_xy()) -
				(NormalTo(From3f_xy(velocity)) * velocity_magnitude_derivative_xy()))
				/ sum_derivative_squares_xy();
		}

		//d(slope)/dj
		float change_in_slope() {
			return ((acceleration.y * velocity.x) - (acceleration.x * velocity.y)) / powf(velocity.x, 2.0);
		}

		//d(tan^-1 (y'(j)/x'(j))) / dj
		float change_in_angle() {
			return (1.0 / (1.0 + powf(velocity.y / velocity.x, 2.0))) * change_in_slope();
		}

		//Make sure the given index is possible for this function
		bool within_bounds (float index) {
			if (stop_index > start_index) {
				return index <= stop_index && index >= start_index;
			} else {
				return index >= stop_index && index <= start_index;
			}
		}

		void render_robot(float wheel_distance, float index) {
			evaluate(index);
			glColor3f(0.8, 0.8, 0.8);
			glLineWidth(3);
			std::vector<cv::Point2f> wireframe;
			wireframe.push_back(cv::Point2f ((wheel_distance), (wheel_distance)));
			wireframe.push_back(cv::Point2f (-(wheel_distance), (wheel_distance)));
			wireframe.push_back(cv::Point2f (-(wheel_distance), -(wheel_distance)));
			wireframe.push_back(cv::Point2f ((wheel_distance), -(wheel_distance)));

			cv::Mat rot_mat( 2, 3, CV_32FC1 );
			rot_mat = cv::getRotationMatrix2D(cv::Point2f(0.0, 0.0), -direction_xy() * (180.0 / acos(-1)), 1.0);
			cv::transform(wireframe, wireframe, rot_mat);

			cv::Point2f last = wireframe.back();
			glBegin(GL_LINES);
			for (auto& point : wireframe) {
				glVertex2f(point.x + position.x, point.y + position.y);
				glVertex2f(last.x + position.x, last.y + position.y);
				last = point;
			}
			glVertex2f(position.x, position.y);
			glVertex2f(position.x + (cos(direction_xy()) * (wheel_distance)), position.y + (sin(direction_xy()) * (wheel_distance)));
			glEnd();
		}

};

#endif
