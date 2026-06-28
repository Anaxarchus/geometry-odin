package circle

import "base:intrinsics"
import "core:math"
import "core:math/linalg"

is_float :: intrinsics.type_is_float

circle_area :: proc(radius: $T) -> T where is_float(T) {
	return math.PI * radius * radius
}

circle_circumference :: proc(radius: $T) -> T where is_float(T) {
	return math.TAU * radius
}

// gets the nearest point on the circle's boundary to the given point
circle_project_to_boundary :: proc(point, origin: [2]$T, radius: T) -> [2]T where is_float(T) {
	d := point - origin
	dist := linalg.length(d)
	if dist == 0 {
		// point is at the center: direction is undefined, pick +x
		return origin + [2]T{radius, 0}
	}
	return origin + d / dist * radius
}

// clamps a point to within the circle (interior points are returned unchanged)
circle_clamp_point :: proc(point, origin: [2]$T, radius: T) -> [2]T where is_float(T) {
	d := point - origin
	dist := linalg.length(d)
	if dist <= radius {
		return point
	}
	return origin + d / dist * radius
}

// returns true if the circle contains the point
circle_has_point :: proc(point, origin: [2]$T, radius: T) -> bool where is_float(T) {
	d := point - origin
	return linalg.dot(d, d) <= radius * radius
}

// returns the point on the boundary at the given angle (radians, 0 = +x axis)
circle_point_at :: proc(origin: [2]$T, radius: T, angle: T) -> [2]T where is_float(T) {
	return origin + [2]T{math.cos(angle), math.sin(angle)} * radius
}

// returns the unit tangent (counter-clockwise direction) at the given angle
circle_tangent_at :: proc(angle: $T) -> [2]T where is_float(T) {
	return {-math.sin(angle), math.cos(angle)}
}

// returns the outward unit normal at the given angle
circle_normal_at :: proc(angle: $T) -> [2]T where is_float(T) {
	return {math.cos(angle), math.sin(angle)}
}

// returns the angle (radians, 0 = +x axis) of a point as seen from the center
circle_angle_of :: proc(point, origin: [2]$T) -> T where is_float(T) {
	d := point - origin
	return math.atan2(d.y, d.x)
}

// fits a circle to a closed polygon. The origin is the centroid of the contour
// vertices; the fit mode selects how the radius is derived from the vertex
// distances to that centroid.
Circle_Fit :: enum {
	Min,
	Max,
	Average,
}
circle_from_contour :: proc(
	contour: [][2]$T,
	fit: Circle_Fit,
) -> (origin: [2]T, radius: T) where is_float(T) {
	if len(contour) == 0 {
		return {}, 0
	}

	// centroid of the vertices
	for v in contour {
		origin += v
	}
	origin /= T(len(contour))

	switch fit {
	case .Min:
		// distance to the nearest vertex (largest inscribed-ish circle)
		radius = max(T)
		for v in contour {
			radius = min(radius, linalg.length(v - origin))
		}
	case .Max:
		// distance to the farthest vertex (smallest enclosing-ish circle)
		for v in contour {
			radius = max(radius, linalg.length(v - origin))
		}
	case .Average:
		// arithmetic mean of all vertex distances
		for v in contour {
			radius += linalg.length(v - origin)
		}
		radius /= T(len(contour))
	}
	return
}

// builds a polygon approximating the circle, with `segments` evenly spaced
// vertices starting at angle 0 (the +x axis) and winding counter-clockwise
circle_to_contour :: proc(
	origin: [2]$T,
	radius: T,
	segments := 32,
	allocator := context.allocator,
) -> [][2]T where is_float(T) {
	points := make([][2]T, segments, allocator)
	for i in 0 ..< segments {
		angle := math.TAU * T(i) / T(segments)
		points[i] = origin + [2]T{math.cos(angle), math.sin(angle)} * radius
	}
	return points
}
