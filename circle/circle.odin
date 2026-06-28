package circle

import "base:intrinsics"
import "core:math"
import "core:math/linalg"

is_float :: intrinsics.type_is_float

// Calculates the surface area of a circle ($A = \pi r^2$).
// Works with floating-point types.
area :: proc(radius: $T) -> T where is_float(T) {
	return math.PI * radius * radius
}

// Calculates the true bounding circumference of a circle ($C = 2\pi r$).
// Works with floating-point types.
circumference :: proc(radius: $T) -> T where is_float(T) {
	return math.TAU * radius
}

// Snaps an external or internal point orthogonally onto the circle's perimeter edge boundary.
//
// If the target point lands perfectly at the circle's exact center coordinate, the projection 
// direction defaults along the positive X-axis (`{radius, 0}`).
// Works with floating-point types.
project_to_boundary :: proc(point, origin: [2]$T, radius: T) -> [2]T where is_float(T) {
	d := point - origin
	dist := linalg.length(d)
	if dist == 0 {
		// point is at the center: direction is undefined, pick +x
		return origin + [2]T{radius, 0}
	}
	return origin + d / dist * radius
}

// Clamps an arbitrary point `point` so that it remains inside or on the circle's disk.
// Points already situated inside the circle are returned completely unaltered.
// Works with floating-point types.
clamp_point :: proc(point, origin: [2]$T, radius: T) -> [2]T where is_float(T) {
	d := point - origin
	dist := linalg.length(d)
	if dist <= radius {
		return point
	}
	return origin + d / dist * radius
}

// Tests whether a given point sits strictly within or on the bounding radius perimeter of a circle.
// Employs a squared dot-product distance calculation to avoid expensive square roots.
has_point :: proc(point, origin: [2]$T, radius: T) -> bool where is_float(T) {
	d := point - origin
	return linalg.dot(d, d) <= radius * radius
}

// Resolves a 2D coordinate point directly on the circle perimeter boundary at a specific angle.
// The angle argument is parsed in radians, traveling counter-clockwise from the positive X-axis (0.0).
point_at :: proc(origin: [2]$T, radius: T, angle: T) -> [2]T where is_float(T) {
	return origin + [2]T{math.cos(angle), math.sin(angle)} * radius
}

// Computes the unit counter-clockwise tangent direction vector of a circle at a specific radial angle.
// Invaluable for physics reflections, surface sliding, and orbital movement vectors.
tangent_at :: proc(angle: $T) -> [2]T where is_float(T) {
	return {-math.sin(angle), math.cos(angle)}
}

// Computes the outward-facing unit normal vector stretching from a circle center at a given radial angle.
normal_at :: proc(angle: $T) -> [2]T where is_float(T) {
	return {math.cos(angle), math.sin(angle)}
}

// Derives the directional angular heading (in radians) of a point relative to the circle's center.
// Returns an angle metric spanning between $-\pi$ and $+\pi$, with 0.0 resting along the positive X-axis.
angle_of :: proc(point, origin: [2]$T) -> T where is_float(T) {
	d := point - origin
	return math.atan2(d.y, d.x)
}

// Configuration options determining how a circle's radius is approximated from an input shape.
Circle_Fit :: enum {
	Min,     // Distance to the nearest vertex (largest inscribed circle approximation).
	Max,     // Distance to the furthest vertex (smallest bounding/enclosing circle approximation).
	Mean, // The arithmetic mean of all vertex distances relative to the shape center.
}

// Approximates a circle representation fitted to a closed 2D polygon contour map.
//
// The calculated circle origin is extracted from the arithmetic centroid of the shape vertices. 
// The chosen `Circle_Fit` metric determines how the final boundary radius is selected.
from_contour :: proc(
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

	radius = 0 // Reset radius accumulator before evaluation
	switch fit {
	case .Min:
		radius = max(T)
		for v in contour {
			radius = min(radius, linalg.length(v - origin))
		}
	case .Max:
		for v in contour {
			radius = max(radius, linalg.length(v - origin))
		}
	case .Mean:
		for v in contour {
			radius += linalg.length(v - origin)
		}
		radius /= T(len(contour))
	}
	return
}

// Generates a slice of evenly-spaced vertex coordinates tracking the path of a circle.
//
// The output contour array starts at angle 0.0 (positive X-axis) and winds counter-clockwise.
// Memory allocation is handled explicitly through the provided `allocator` pipeline parameter.
to_contour :: proc(
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