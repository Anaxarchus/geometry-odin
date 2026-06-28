package intersect

import "base:intrinsics"
import "core:math"
import "core:math/linalg"
import "../types"

// Ray and overlap primitives, generic over the scalar type and expressed in raw
// vectors so they drop straight into existing code (e.g. algo/bvh).
//
// Rays are parameterized as P(t) = origin + dir*t. Ray tests return the entry
// distance t along the ray together with a hit flag; use ray_at to recover the
// point. dir need not be normalized, in which case t is in units of |dir|.

@(private)
is_scalar :: intrinsics.type_is_ordered_numeric

@(private)
is_float :: intrinsics.type_is_float

// ---------------------------------------------------------------------------
// Ray utilities
// ---------------------------------------------------------------------------

// returns the point at parametric distance t along the ray
ray_at :: #force_inline proc "contextless" (origin, dir: [3]$T, t: T) -> [3]T where is_scalar(T) {
	return origin + dir * t
}

// ---------------------------------------------------------------------------
// Ray vs primitive
// ---------------------------------------------------------------------------

// ray vs plane, where the plane is { x | dot(normal, x) = d }
ray_plane :: proc "contextless" (
	origin, dir, normal: [3]$T,
	d: T,
) -> (hit: bool, t: T) where is_float(T) {
	denom := linalg.dot(normal, dir)
	if abs(denom) < 1e-6 do return false, 0 // ray parallel to plane
	t = (d - linalg.dot(normal, origin)) / denom
	if t < 0 do return false, 0 // plane behind ray origin
	return true, t
}

// ray vs aabb (slab test) using a precomputed reciprocal direction; t is the
// entry distance and may be negative when the origin is inside the box. This is
// the hot-path variant for traversal loops that reuse inv_dir across many boxes.
ray_aabb_inv :: #force_inline proc "contextless" (
	origin, inv_dir, aabb_min, aabb_max: [3]$T,
) -> (hit: bool, t: T) where is_float(T) {
	t_min := (aabb_min - origin) * inv_dir
	t_max := (aabb_max - origin) * inv_dir
	t1 := linalg.min(t_min, t_max)
	t2 := linalg.max(t_min, t_max)
	t_near := max(max(t1.x, t1.y), t1.z)
	t_far := min(min(t2.x, t2.y), t2.z)
	return t_far >= 0 && t_near <= t_far, t_near
}

// ray vs aabb (slab test); computes the reciprocal direction internally
ray_aabb :: #force_inline proc "contextless" (
	origin, dir, aabb_min, aabb_max: [3]$T,
) -> (hit: bool, t: T) where is_float(T) {
	inv_dir := [3]T{1 / dir.x, 1 / dir.y, 1 / dir.z}
	return ray_aabb_inv(origin, inv_dir, aabb_min, aabb_max)
}

// ray vs triangle (Moller-Trumbore); max_t bounds the accepted hit distance
ray_triangle :: proc "contextless" (
	origin, dir, v0, v1, v2: [3]$T,
	max_t: T,
) -> (hit: bool, t: T) where is_float(T) {
	EPSILON :: 0.000001

	edge1 := v1 - v0
	edge2 := v2 - v0

	h := linalg.cross(dir, edge2)
	det := linalg.dot(edge1, h)

	if det > -EPSILON && det < EPSILON do return false, max_t // ray parallel to triangle

	inv_det := 1.0 / det
	s := origin - v0

	u := linalg.dot(s, h) * inv_det
	if u < 0.0 || u > 1.0 do return false, max_t

	q := linalg.cross(s, edge1)
	v := linalg.dot(dir, q) * inv_det
	if v < 0.0 || u + v > 1.0 do return false, max_t

	out_t := linalg.dot(edge2, q) * inv_det
	if out_t > EPSILON && out_t < max_t {
		return true, out_t
	}
	return false, max_t
}

// ray vs sphere; returns the nearest non-negative intersection distance. When
// the origin is inside the sphere the exit distance is returned.
ray_sphere :: proc "contextless" (
	origin, dir, center: [3]$T,
	radius: T,
) -> (hit: bool, t: T) where is_float(T) {
	oc := origin - center
	a := linalg.dot(dir, dir)
	b := 2 * linalg.dot(oc, dir)
	c := linalg.dot(oc, oc) - radius * radius

	disc := b * b - 4 * a * c
	if disc < 0 do return false, 0

	sq := math.sqrt(disc)
	t0 := (-b - sq) / (2 * a)
	t1 := (-b + sq) / (2 * a)
	if t0 > t1 do t0, t1 = t1, t0

	if t0 >= 0 do return true, t0
	if t1 >= 0 do return true, t1 // origin inside the sphere
	return false, 0
}

// ---------------------------------------------------------------------------
// Overlap (boolean) tests
// ---------------------------------------------------------------------------

// aabb vs aabb overlap (inclusive of touching faces)
overlap_aabb_aabb :: #force_inline proc "contextless" (
	a_min, a_max, b_min, b_max: [3]$T,
) -> bool where is_scalar(T) {
	return(
		!(a_max.x < b_min.x ||
			b_max.x < a_min.x ||
			a_max.y < b_min.y ||
			b_max.y < a_min.y ||
			a_max.z < b_min.z ||
			b_max.z < a_min.z) \
	)
}

// sphere vs aabb overlap: clamp the center into the box and compare squared
// distance to r^2 (no sqrt needed)
overlap_sphere_aabb :: #force_inline proc "contextless" (
	center: [3]$T,
	radius: T,
	aabb_min, aabb_max: [3]T,
) -> bool where is_scalar(T) {
	q := [3]T {
		clamp(center.x, aabb_min.x, aabb_max.x),
		clamp(center.y, aabb_min.y, aabb_max.y),
		clamp(center.z, aabb_min.z, aabb_max.z),
	}
	d := q - center
	return d.x * d.x + d.y * d.y + d.z * d.z <= radius * radius
}

// oriented box vs aabb overlap via the separating-axis theorem (15 axes).
// Core form taking the box's three orthonormal local axes explicitly; obb_half
// are its half extents along those axes. Prefer this in hot loops where the axes
// can be derived once and reused across many AABBs (e.g. BVH traversal). This is
// Ericson's OBB-OBB test specialized to an axis-aligned second box.
overlap_obb_aabb_axes :: proc "contextless" (
	obb_center, obb_half: [3]$T,
	obb_axes: [3][3]T,
	aabb_min, aabb_max: [3]T,
) -> bool where is_float(T) {
	EPS :: 1e-6

	bc := (aabb_min + aabb_max) / 2 // aabb center
	be := (aabb_max - aabb_min) / 2 // aabb half extents
	ae := obb_half

	// R[i][j] = dot(obb_axis[i], world_axis[j]) = obb_axes[i][j], since the
	// aabb's axes are the identity basis. AbsR adds EPS to absorb the parallel-
	// edge degeneracy when cross products vanish.
	R, AbsR: [3][3]T
	for i in 0 ..< 3 {
		for j in 0 ..< 3 {
			R[i][j] = obb_axes[i][j]
			AbsR[i][j] = abs(R[i][j]) + EPS
		}
	}

	// translation from obb center to aabb center, in the obb's frame
	tw := bc - obb_center
	t := [3]T {
		linalg.dot(tw, obb_axes[0]),
		linalg.dot(tw, obb_axes[1]),
		linalg.dot(tw, obb_axes[2]),
	}

	// L = obb axes (A0, A1, A2)
	for i in 0 ..< 3 {
		ra := ae[i]
		rb := be[0] * AbsR[i][0] + be[1] * AbsR[i][1] + be[2] * AbsR[i][2]
		if abs(t[i]) > ra + rb do return false
	}

	// L = aabb axes (B0, B1, B2)
	for j in 0 ..< 3 {
		ra := ae[0] * AbsR[0][j] + ae[1] * AbsR[1][j] + ae[2] * AbsR[2][j]
		rb := be[j]
		if abs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]) > ra + rb do return false
	}

	// L = A0 x B0
	{
		ra := ae[1] * AbsR[2][0] + ae[2] * AbsR[1][0]
		rb := be[1] * AbsR[0][2] + be[2] * AbsR[0][1]
		if abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb do return false
	}
	// L = A0 x B1
	{
		ra := ae[1] * AbsR[2][1] + ae[2] * AbsR[1][1]
		rb := be[0] * AbsR[0][2] + be[2] * AbsR[0][0]
		if abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb do return false
	}
	// L = A0 x B2
	{
		ra := ae[1] * AbsR[2][2] + ae[2] * AbsR[1][2]
		rb := be[0] * AbsR[0][1] + be[1] * AbsR[0][0]
		if abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb do return false
	}
	// L = A1 x B0
	{
		ra := ae[0] * AbsR[2][0] + ae[2] * AbsR[0][0]
		rb := be[1] * AbsR[1][2] + be[2] * AbsR[1][1]
		if abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb do return false
	}
	// L = A1 x B1
	{
		ra := ae[0] * AbsR[2][1] + ae[2] * AbsR[0][1]
		rb := be[0] * AbsR[1][2] + be[2] * AbsR[1][0]
		if abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb do return false
	}
	// L = A1 x B2
	{
		ra := ae[0] * AbsR[2][2] + ae[2] * AbsR[0][2]
		rb := be[0] * AbsR[1][1] + be[1] * AbsR[1][0]
		if abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb do return false
	}
	// L = A2 x B0
	{
		ra := ae[0] * AbsR[1][0] + ae[1] * AbsR[0][0]
		rb := be[1] * AbsR[2][2] + be[2] * AbsR[2][1]
		if abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb do return false
	}
	// L = A2 x B1
	{
		ra := ae[0] * AbsR[1][1] + ae[1] * AbsR[0][1]
		rb := be[0] * AbsR[2][2] + be[2] * AbsR[2][0]
		if abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb do return false
	}
	// L = A2 x B2
	{
		ra := ae[0] * AbsR[1][2] + ae[1] * AbsR[0][2]
		rb := be[0] * AbsR[2][1] + be[1] * AbsR[2][0]
		if abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb do return false
	}

	// no separating axis found -> the boxes overlap
	return true
}

// oriented box (types.Box) vs aabb overlap. Derives the box's world-space axes
// from its quaternion and forwards to overlap_obb_aabb_axes. half_extents pass
// straight through, so no size conversion is needed.
overlap_box_f32_aabb :: proc "contextless" (
	box: types.Box_f32,
	aabb_min, aabb_max: [3]f32,
) -> bool {
	// columns of the rotation matrix are the box's local axes in world space
	m := linalg.matrix3_from_quaternion_f32(box.rotation)
	axes := [3][3]f32 {
		{m[0, 0], m[1, 0], m[2, 0]},
		{m[0, 1], m[1, 1], m[2, 1]},
		{m[0, 2], m[1, 2], m[2, 2]},
	}
	return overlap_obb_aabb_axes(box.origin, box.half_extents, axes, aabb_min, aabb_max)
}

overlap_box_f64_aabb :: proc "contextless" (
	box: types.Box_f64,
	aabb_min, aabb_max: [3]f64,
) -> bool {
	m := linalg.matrix3_from_quaternion_f64(box.rotation)
	axes := [3][3]f64 {
		{m[0, 0], m[1, 0], m[2, 0]},
		{m[0, 1], m[1, 1], m[2, 1]},
		{m[0, 2], m[1, 2], m[2, 2]},
	}
	return overlap_obb_aabb_axes(box.origin, box.half_extents, axes, aabb_min, aabb_max)
}

// dispatches across the explicit-axes core and the Box_f32/Box_f64 convenience
// forms. Call as overlap_obb_aabb(box, min, max) or
// overlap_obb_aabb(center, half, axes, min, max).
overlap_obb_aabb :: proc {
	overlap_obb_aabb_axes,
	overlap_box_f32_aabb,
	overlap_box_f64_aabb,
}
