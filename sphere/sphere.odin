package sphere

import "base:intrinsics"
import "core:math"
import "core:math/linalg"

is_float :: intrinsics.type_is_float

// angle convention for the parametric boundary:
//   theta = azimuth in the xy-plane, measured from +x (matches circle)
//   phi   = polar angle from +z (0 at the north pole, PI at the south pole)
// at phi = PI/2 (the equator) the boundary reduces to the circle in the xy-plane.

volume :: proc(radius: $T) -> T where is_float(T) {
	return 4.0 / 3.0 * math.PI * radius * radius * radius
}

surface_area :: proc(radius: $T) -> T where is_float(T) {
	return 4 * math.PI * radius * radius
}

// gets the nearest point on the sphere's boundary to the given point
project_to_boundary :: proc(point, origin: [3]$T, radius: T) -> [3]T where is_float(T) {
	d := point - origin
	dist := linalg.length(d)
	if dist == 0 {
		// point is at the center: direction is undefined, pick +x
		return origin + [3]T{radius, 0, 0}
	}
	return origin + d / dist * radius
}

// clamps a point to within the sphere (interior points are returned unchanged)
clamp_point :: proc(point, origin: [3]$T, radius: T) -> [3]T where is_float(T) {
	d := point - origin
	dist := linalg.length(d)
	if dist <= radius {
		return point
	}
	return origin + d / dist * radius
}

// returns true if the sphere contains the point
has_point :: proc(point, origin: [3]$T, radius: T) -> bool where is_float(T) {
	d := point - origin
	return linalg.dot(d, d) <= radius * radius
}

// returns the point on the boundary at the given spherical angles
point_at :: proc(origin: [3]$T, radius: T, theta, phi: T) -> [3]T where is_float(T) {
	sp := math.sin(phi)
	return origin + [3]T{sp * math.cos(theta), sp * math.sin(theta), math.cos(phi)} * radius
}

// returns the outward unit normal at the given spherical angles. (A sphere's
// tangent space at a point is a plane, so there is no single tangent direction;
// build a frame from this normal if one is needed.)
normal_at :: proc(theta, phi: $T) -> [3]T where is_float(T) {
	sp := math.sin(phi)
	return {sp * math.cos(theta), sp * math.sin(theta), math.cos(phi)}
}

// returns the spherical angles of a point as seen from the center
angles_of :: proc(point, origin: [3]$T) -> (theta, phi: T) where is_float(T) {
	d := point - origin
	dist := linalg.length(d)
	if dist == 0 {
		return 0, 0
	}
	theta = math.atan2(d.y, d.x)
	// clamp guards against acos(NaN) from rounding just outside [-1, 1]
	phi = math.acos(clamp(d.z / dist, -1, 1))
	return
}
