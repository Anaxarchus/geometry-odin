package triangle

import "base:intrinsics"
import "core:math"
import "core:math/linalg"



@(private)
is_float :: intrinsics.type_is_float
@(private)
is_int :: intrinsics.type_is_integer
@(private)
is_scalar :: intrinsics.type_is_ordered_numeric

triangle_cast :: proc(
	t: [3][$N]$T,
	$K: typeid,
) -> [3][N]K where is_scalar(T) && is_scalar(K) {
	return {linalg.array_cast(t[0], K), linalg.array_cast(t[1], K), linalg.array_cast(t[2], K)}
}

triangle_reverse_winding :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> [3][N]T where is_scalar(T) {
	return {t[0], t[2], t[1]}
}

triangle_area_float :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> T where is_float(T) {
	a := linalg.distance(t[0], t[1])
	b := linalg.distance(t[1], t[2])
	c := linalg.distance(t[2], t[0])
	s := (a + b + c) * 0.5
	return math.sqrt(s * (s - a) * (s - b) * (s - c))
}

triangle_area_int :: #force_inline proc(
	t: [3][$N]$T,
) -> f64 where is_int(T) {
	tc := triangle_cast(t, f64)
	return triangle_area_float(tc)
}

triangle_area :: proc {
	triangle_area_float,
	triangle_area_int,
}

triangle_perimeter_float :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> T where is_float(T) {
	return linalg.distance(t[0], t[1]) + linalg.distance(t[1], t[2]) + linalg.distance(t[2], t[0])
}

triangle_perimeter_int :: #force_inline proc (
	t: [3][$N]$T,
) -> f64 where is_int(T) {
	tc := triangle_cast(t, f64)
	return(
		linalg.distance(tc[0], tc[1]) +
		linalg.distance(tc[1], tc[2]) +
		linalg.distance(tc[2], tc[0]) \
	)
}

triangle_perimeter :: proc {
	triangle_perimeter_float,
	triangle_perimeter_int,
}

triangle_centroid :: proc(
	t: [3][$N]$T,
) -> [N]T where is_scalar(T) {
	return (t[0] + t[1] + t[2]) / 3
}

triangle_bounds :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> [2][N]T where is_scalar(T) {
	return {linalg.min(t[0], linalg.min(t[1], t[2])), linalg.max(t[0], linalg.max(t[1], t[2]))}
}

triangle3_normal_float :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [3]T where is_float(T) {
	edge1 := t[1] - t[0]
	edge2 := t[2] - t[0]
	return linalg.normalize(linalg.cross(edge1, edge2))
}

triangle3_normal_int :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [3]f64 where is_int(T) {
	tc := triangle_cast(t, f64)
	return triangle3_normal_float(tc)
}

triangle_normal :: proc {
	triangle3_normal_float,
	triangle3_normal_int,
}

@(private)
_triangle_barycentric :: proc "contextless" (
	t: [3][$N]$T,
	p: [N]T,
) -> [3]T where is_float(T) {
	a, b, c := t[0], t[1], t[2]

	v0 := b - a
	v1 := c - a
	v2 := p - a

	d00 := linalg.dot(v0, v0)
	d01 := linalg.dot(v0, v1)
	d11 := linalg.dot(v1, v1)
	d20 := linalg.dot(v2, v0)
	d21 := linalg.dot(v2, v1)

	denom := d00 * d11 - d01 * d01
	if denom == 0.0 do return {1.0, 0.0, 0.0}

	v := (d11 * d20 - d01 * d21) / denom
	w := (d00 * d21 - d01 * d20) / denom
	u := 1.0 - v - w

	return {u, v, w}
}

triangle_barycentric :: #force_inline proc(
	t: [3][$N]$T,
	p: [N]$K,
) -> [3]K where is_scalar(T) && is_float(K) {
	when T != K {
		return _triangle_barycentric(triangle_cast(t, K), p)
	} else {
		return _triangle_barycentric(t, p)
	}
}

triangle_circumbounds_float :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> (center: [N]T, radius: T) where is_float(T) {
	v0, v1, v2 := t[0], t[1], t[2]
	a := linalg.dot(v1 - v2, v1 - v2)
	b := linalg.dot(v2 - v0, v2 - v0)
	c := linalg.dot(v0 - v1, v0 - v1)

	alpha := a * linalg.dot(v2 - v0, v0 - v1)
	beta  := b * linalg.dot(v0 - v1, v1 - v2)
	gamma := c * linalg.dot(v1 - v2, v2 - v0)

	denom := alpha + beta + gamma
	if denom == 0.0 do return v0, 0.0

	center = (v0 * alpha + v1 * beta + v2 * gamma) / denom
	radius = linalg.distance(center, v0)
	return center, radius
}

triangle_circumbounds_int :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> (center: [N]f64, radius: f64) where is_int(T) {
	return triangle_circumbounds_float(triangle_cast(t, f64))
}

triangle_circumbounds :: proc {
	triangle_circumbounds_float,
	triangle_circumbounds_int,
}

triangle_contains_point :: #force_inline proc "contextless" (
	t: [3][$N]$T,
	p: [N]$K,
) -> bool where is_scalar(T) && is_float(K) {
	bary := triangle_barycentric(t, p)
	return bary[0] >= 0.0 && bary[1] >= 0.0 && bary[2] >= 0.0
}