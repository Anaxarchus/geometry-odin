package geometry

import "triangle"


triangle_normal :: triangle.triangle3_normal

triangle_bounds :: proc {
    triangle.triangle2_rect,
    triangle.triangle3_aabb,
}

triangle_cast :: proc {
    triangle.triangle2_cast,
    triangle.triangle3_cast,
}

triangle_centroid :: proc {
    triangle.triangle3_centroid,
    triangle.triangle2_centroid,
}

triangle_area :: proc {
	triangle.triangle2_float_area,
	triangle.triangle2_int_area_precision,
	triangle.triangle2_int_area,
	triangle.triangle3_float_area,
	triangle.triangle3_int_area_precision,
	triangle.triangle3_int_area,
}

// 2d only
triangle_signed_area :: proc {
	triangle.triangle2_float_signed_area,
	triangle.triangle2_int_signed_area_precision,
	triangle.triangle2_int_signed_area,
}