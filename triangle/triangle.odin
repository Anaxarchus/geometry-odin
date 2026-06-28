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

// Explicitly casts a 3-vertex triangle structure from type `T` to type `K`.
// Useful for batch converting integer triangles into floating-point equivalents.
cast_to :: proc(
	t: [3][$N]$T,
	$K: typeid,
) -> [3][N]K where is_scalar(T) && is_scalar(K) {
	return {linalg.array_cast(t[0], K), linalg.array_cast(t[1], K), linalg.array_cast(t[2], K)}
}

// Flips the winding order of a triangle's vertices by swapping the last two points.
// Changes the sequence from `0 -> 1 -> 2` to `0 -> 2 -> 1`, effectively inverting 
// the triangle's normal vector.
reverse_winding :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> [3][N]T where is_scalar(T) {
	return {t[0], t[2], t[1]}
}

// Calculates the surface area of a triangle using Heron's Formula.
// Works with floating-point types.
area_float :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> T where is_float(T) {
	a := linalg.distance(t[0], t[1])
	b := linalg.distance(t[1], t[2])
	c := linalg.distance(t[2], t[0])
	s := (a + b + c) * 0.5
	return math.sqrt(s * (s - a) * (s - b) * (s - c))
}

// Calculates the surface area of a triangle.
// Accepts integer coordinates and promotes them to `f64` to return a precise fractional area.
area_int :: #force_inline proc(
	t: [3][$N]$T,
) -> f64 where is_int(T) {
	tc := cast_to(t, f64)
	return area_float(tc)
}

// Explicit overload group for calculating the surface area of a triangle.
area :: proc {
	area_float,
	area_int,
}

// Calculates the cumulative perimeter length of a triangle by summing its three side lengths.
// Works with floating-point types.
perimeter_float :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> T where is_float(T) {
	return linalg.distance(t[0], t[1]) + linalg.distance(t[1], t[2]) + linalg.distance(t[2], t[0])
}

// Calculates the cumulative perimeter length of a triangle.
// Accepts integer coordinates and promotes them to `f64` to return a precise perimeter value.
perimeter_int :: #force_inline proc (
	t: [3][$N]$T,
) -> f64 where is_int(T) {
	tc := cast_to(t, f64)
	return linalg.distance(tc[0], tc[1]) + linalg.distance(tc[1], tc[2]) + linalg.distance(tc[2], tc[0])
}

// Explicit overload group for calculating the perimeter of a triangle.
perimeter :: proc {
	perimeter_float,
	perimeter_int,
}

// Computes the arithmetic centroid (center of mass) of a triangle.
// Returns the average of the three vertex positions.
centroid :: proc(
	t: [3][$N]$T,
) -> [N]T where is_scalar(T) {
	return (t[0] + t[1] + t[2]) / 3
}

// Generates an axis-aligned bounding box (AABB) enclosing the triangle.
// Returns a matrix of two points: `{min_bounds, max_bounds}`.
bounds :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> [2][N]T where is_scalar(T) {
	return {linalg.min(t[0], linalg.min(t[1], t[2])), linalg.max(t[0], linalg.max(t[1], t[2]))}
}

// Calculates the unit surface normal vector of a 3D triangle.
// Assumes a counter-clockwise (CCW) winding order for the positive normal face direction.
// Works with floating-point types.
normal_float :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [3]T where is_float(T) {
	edge1 := t[1] - t[0]
	edge2 := t[2] - t[0]
	return linalg.normalize(linalg.cross(edge1, edge2))
}

// Calculates the unit surface normal vector of a 3D triangle.
// Accepts integer coordinates and promotes them to `f64` to complete the cross product normalization.
normal_int :: #force_inline proc "contextless" (
	t: [3][3]$T,
) -> [3]f64 where is_int(T) {
	tc := cast_to(t, f64)
	return normal_float(tc)
}

// Explicit overload group for calculating the surface normal vector of a 3D triangle.
normal :: proc {
	normal_float,
	normal_int,
}

// Internal barycentric coordinator utility using an optimized Cramer's rule implementation.
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

// Computes the Barycentric coordinates `{u, v, w}` of a point `p` relative to a triangle `t`.
//
// The returned weights map directly to the vertices `{t[0], t[1], t[2]}`.
// - If all coordinates are `>= 0.0` and `<= 1.0`, the point lies inside or on the triangle.
// - Automatically matches scalar integer or float triangle types to the precision of point `p`.
barycentric :: #force_inline proc(
	t: [3][$N]$T,
	p: [N]$K,
) -> [3]K where is_scalar(T) && is_float(K) {
	when T != K {
		return _triangle_barycentric(cast_to(t, K), p)
	} else {
		return _triangle_barycentric(t, p)
	}
}

// Computes the circumscribed bounding sphere/circle of a triangle.
// Returns the unique geographic center point and the exact radius that intersects all three vertices.
// Handles degenerate collinear triangles safely by returning `v0` and a radius of `0.0`.
// Works with floating-point types.
circumbounds_float :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> (center: [N]T, radius: T) where is_float(T) {
	v0, v1, v2 := t[0], t[1], t[2]
	a2 := linalg.dot(v1 - v2, v1 - v2)
	b2 := linalg.dot(v2 - v0, v2 - v0)
	c2 := linalg.dot(v0 - v1, v0 - v1)

	alpha := a2 * (b2 + c2 - a2)
	beta  := b2 * (c2 + a2 - b2)
	gamma := c2 * (a2 + b2 - c2)

	denom := alpha + beta + gamma
	if denom == 0.0 do return v0, 0.0

	center = (v0 * alpha + v1 * beta + v2 * gamma) / denom
	radius = linalg.distance(center, v0)
	return center, radius
}

// Computes the circumscribed bounding sphere/circle of a triangle.
// Accepts integer coordinates and upcasts them to `f64` to compute the continuous fractional circumcenter.
circumbounds_int :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> (center: [N]f64, radius: f64) where is_int(T) {
	return circumbounds_float(cast_to(t, f64))
}

// Explicit overload group for determining a triangle's circumcenter and circumradius.
circumbounds :: proc {
	circumbounds_float,
	circumbounds_int,
}

// Tests if a target point `p` is situated within or on the boundary edges of a triangle `t`.
// Employs a highly efficient barycentric sign check under the hood.
contains_point :: #force_inline proc (
	t: [3][$N]$T,
	p: [N]$K,
) -> bool where is_scalar(T) && is_float(K) {
	bary := barycentric(t, p)
	return bary[0] >= 0.0 && bary[1] >= 0.0 && bary[2] >= 0.0
}