#ifndef FIELD_DIMENSIONS_H
#define FIELD_DIMENSIONS_H

#include <opencv2/opencv.hpp>
#define IN_TO_MM_CONVERSION_FACTOR 25.4
#define INTO_MM(in) (in * IN_TO_MM_CONVERSION_FACTOR)

namespace FieldDimension {
	const float plate_width = INTO_MM(36.0);
	const float plate_height = INTO_MM(48.0);

	const float field_width = INTO_MM(314.0);
	const float field_height = INTO_MM(400.0);

	const cv::Rect2f field_bounds (0, 0, field_width, field_height);	

	namespace Switch {
		const float boom_width = INTO_MM(72.0);
		const float boom_height = INTO_MM(12.0);
		const float plate_vertical_offset = INTO_MM(140.0);
		const float plate_horizontal_offset = INTO_MM(85.25);

		const cv::Rect2f left_plate (
				plate_horizontal_offset,
				plate_vertical_offset,
				plate_width,
				plate_height);

		const cv::Rect2f right_plate (
				plate_horizontal_offset + boom_width + plate_width,
				plate_vertical_offset,
				plate_width,
				plate_height);

		const cv::Rect2f boom (
				plate_horizontal_offset + plate_width,
				plate_vertical_offset + (plate_height / 2.0) - (boom_height / 2.0),
				boom_width,
				boom_height);

		const cv::Point2f front_center_left ((left_plate.br().x + left_plate.tl().x) / 2.0, left_plate.tl().y);
		const cv::Point2f back_center_left ((left_plate.br().x + left_plate.tl().x) / 2.0, left_plate.br().y);
	}
	
	namespace Scale {
		const float boom_width = INTO_MM(104.0);
		const float boom_height = INTO_MM(12.0);
		const float plate_vertical_offset = INTO_MM(299.65);
		const float plate_horizontal_offset = INTO_MM(71.57);

		const cv::Rect2f left_plate (
				plate_horizontal_offset,
				plate_vertical_offset,
				plate_width,
				plate_height);

		const cv::Rect2f right_plate (
				plate_horizontal_offset + boom_width + plate_width,
				plate_vertical_offset,
				plate_width,
				plate_height);

		const cv::Rect2f boom (
				plate_horizontal_offset + plate_width,
				plate_vertical_offset + (plate_height / 2.0) - (boom_height / 2.0),
				boom_width,
				boom_height);

		const cv::Point2f front_center_left ((left_plate.br().x + left_plate.tl().x) / 2.0, left_plate.tl().y);
	}

}

#endif
