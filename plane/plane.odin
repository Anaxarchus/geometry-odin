package plane

import "base:intrinsics"
import "core:math/linalg"

// Default epsilon values matching standard game engine / CAD tolerances
EPSILON_32 :: 1e-5 // 0.00001
EPSILON_64 :: 1e-9 // 0.000000001

@(private)
is_float :: intrinsics.type_is_float

// per-precision default tolerance, selected by the float width
@(private)
default_epsilon :: #force_inline proc "contextless" ($T: typeid) -> T where is_float(T) {
	when size_of(T) >= 8 {
		return T(EPSILON_64)
	} else {
		return T(EPSILON_32)
	}
}

// builds a plane from three points (CW winding defines the front face)
plane_from_points :: proc "contextless" (p0, p1, p2: [3]$T) -> [4]T where is_float(T) {
	// edge vectors; their cross product is the (unnormalized) normal
	v := p1 - p0
	w := p2 - p0
	normal := linalg.normalize(linalg.cross(v, w))
	d := linalg.dot(normal, p0)
	return {normal.x, normal.y, normal.z, d}
}

// returns the plane's normal
plane_normal :: #force_inline proc "contextless" (p: [4]$T) -> [3]T where is_float(T) {
	return p.xyz
}

// returns the signed distance from a point to the plane
plane_distance :: #force_inline proc "contextless" (
	plane: [4]$T,
	point: [3]T,
) -> T where is_float(T) {
	return linalg.dot(plane.xyz, point) - plane.w
}

// projects a point onto the plane (its nearest point on the plane)
plane_project :: #force_inline proc "contextless" (
	plane: [4]$T,
	point: [3]T,
) -> [3]T where is_float(T) {
	// subtract the signed distance along the normal
	dist := plane_distance(plane, point)
	return point - plane.xyz * dist
}

// returns true if the point lies on the positive (normal-facing) side
plane_is_above :: #force_inline proc "contextless" (
	plane: [4]$T,
	point: [3]T,
) -> bool where is_float(T) {
	return plane_distance(plane, point) > 0
}

// returns true if two planes are equal within epsilon. A negative epsilon (the
// default) selects the per-precision tolerance from default_epsilon.
plane_is_equal_approx :: proc "contextless" (
	a, b: [4]$T,
) -> bool where is_float(T) {
	eps := default_epsilon(T)
	// distance term first, then the normal components
	if abs(a.w - b.w) > eps {
		return false
	}
	return abs(a.x - b.x) <= eps && abs(a.y - b.y) <= eps && abs(a.z - b.z) <= eps
}
