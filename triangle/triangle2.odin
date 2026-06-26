package triangle

import "base:intrinsics"
import "core:math"
import "core:math/linalg"

triangle2_cast :: #force_inline proc "contextless" (
	t: [3][2]$T,
	$C: typeid,
) -> [3][2]C where intrinsics.type_is_numeric(T) &&
	intrinsics.type_is_numeric(C) {
	return {{C(t[0][0]), C(t[0][1])}, {C(t[1][0]), C(t[1][1])}, {C(t[2][0]), C(t[2][1])}}
}

triangle2_float_signed_area :: #force_inline proc "contextless" (
	t: [3][2]$T,
) -> T where intrinsics.type_is_float(T) {
	cross_prod :=
		(t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[1][1] - t[0][1]) * (t[2][0] - t[0][0])
	return cross_prod * 0.5
}

triangle2_int_signed_area_precision :: #force_inline proc "contextless" (
	t: [3][2]$T,
	$P: typeid,
) -> P where intrinsics.type_is_integer(T) &&
	intrinsics.type_is_float(P) {
	cross_prod :=
		i64(t[1][0] - t[0][0]) * i64(t[2][1] - t[0][1]) -
		i64(t[1][1] - t[0][1]) * i64(t[2][0] - t[0][0])
	return P(cross_prod) * 0.5
}

triangle2_int_signed_area :: #force_inline proc "contextless" (
	t: [3][2]$T,
) -> f32 where intrinsics.type_is_integer(T) {
	return triangle2_int_signed_area_precision(t, f32)
}

triangle2_signed_area :: proc {
	triangle2_float_signed_area,
	triangle2_int_signed_area_precision,
	triangle2_int_signed_area,
}

triangle2_float_area :: #force_inline proc "contextless" (
	t: [3][2]$T,
) -> T where intrinsics.type_is_float(T) {
	return abs(triangle2_float_signed_area(t))
}

triangle2_int_area_precision :: #force_inline proc "contextless" (
	t: [3][2]$T,
	$P: typeid,
) -> P where intrinsics.type_is_integer(T) &&
	intrinsics.type_is_float(P) {
	return abs(triangle2_int_signed_area_precision(t, P))
}

triangle2_int_area :: #force_inline proc "contextless" (
	t: [3][2]$T,
) -> f32 where intrinsics.type_is_integer(T) {
	return triangle2_int_area_precision(t, f32)
}

triangle2_area :: proc {
	triangle2_float_area,
	triangle2_int_area_precision,
	triangle2_int_area,
}

triangle2_rect :: #force_inline proc "contextless" (
	t: [3][2]$T,
) -> [2][2]T where intrinsics.type_is_numeric(T) {
	return _triangle_min_max(t)
}

triangle2_centroid :: #force_inline proc "contextless" (
    t: [3][2]$T,
) -> [2]T where intrinsics.type_is_numeric(T) {
    return (t[0] + t[1] + t[2]) / 3
}