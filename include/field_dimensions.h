#ifndef FIELD_DIMENSIONS_H
#define FIELD_DIMENSIONS_H

namespace FieldDimension {
#define IN_TO_MM_CONVERSION_FACTOR 25.4
#define INTO_MM(in) (in * IN_TO_MM_CONVERSION_FACTOR)
	const float switch_plate_width = INTO_MM(36.0);
	const float switch_plate_height = INTO_MM(48.0);
	const float switch_boom_width = INTO_MM(72.0);
	const float switch_boom_height = INTO_MM(12.0);
	const float switch_plate_vertical_offset = INTO_MM(140.0);
	const float switch_plate_horizontal_offset = INTO_MM(85.25);

	const float field_width = INTO_MM(314.0);
	const float field_height = INTO_MM(400.0);
}

#endif
