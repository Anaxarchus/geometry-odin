package triangle

import "base:intrinsics"
import "core:math"
import "core:math/linalg"


@(private)
_triangle3_normal_generic_float :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [3]T where intrinsics.type_is_float(T) {
	edge1 := t[1] - t[0]
	edge2 := t[2] - t[0]
	n := linalg.cross(edge1, edge2)
	return linalg.normalize0(n)
}

triangle3_cast :: #force_inline proc "contextless" (
	t: [3][3]$T,
	$C: typeid,
) -> [3][3]C where intrinsics.type_is_numeric(T) &&
	intrinsics.type_is_numeric(C) {
	return {
		{C(t[0][0]), C(t[0][1]), C(t[0][2])},
		{C(t[1][0]), C(t[1][1]), C(t[1][2])},
		{C(t[2][0]), C(t[2][1]), C(t[2][2])},
	}
}

triangle3_float_normal :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [3]T where intrinsics.type_is_float(T) {
	return _triangle3_normal_generic_float(t)
}

triangle3_int_normal :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [3]T where intrinsics.type_is_integer(T) {
	return linalg.cross(t[1] - t[0], t[2] - t[0])
}

triangle3_normal :: proc {
	triangle3_float_normal,
	triangle3_int_normal,
}

triangle3_float_area :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> T where intrinsics.type_is_float(T) {
	edge1 := t[1] - t[0]
	edge2 := t[2] - t[0]
	cross_prod := linalg.cross(edge1, edge2)
	return linalg.length(cross_prod) * 0.5
}

triangle3_int_area_precision :: #force_inline proc "contextless" (
	t: [3][3]$T,
	$P: typeid,
) -> P where intrinsics.type_is_integer(T) &&
	intrinsics.type_is_float(P) {
	edge1 := linalg.array_cast(t[1] - t[0], i64)
	edge2 := linalg.array_cast(t[2] - t[0], i64)
	cp := linalg.cross(edge1, edge2)
	length_squared := P(linalg.dot(cp, cp))
	return math.sqrt(length_squared) * 0.5
}

triangle3_int_area :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> f32 where intrinsics.type_is_integer(T) {
	return triangle3_int_area_precision(t, f32)
}

triangle3_area :: proc {
	triangle3_float_area,
	triangle3_int_area_precision,
	triangle3_int_area,
}

triangle3_aabb :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [2][3]T where intrinsics.type_is_numeric(T) {
	return _triangle_min_max(t)
}

triangle3_centroid :: proc(t: [3][3]$T) -> [3]T
where intrinsics.type_is_numeric(T) {
	return (t[0] + t[1] + t[2]) / 3
}
