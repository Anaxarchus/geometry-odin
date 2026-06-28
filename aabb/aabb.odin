package aabb

import "base:intrinsics"
import "core:math/linalg"

// An Aabb is stored as [2][3]T == {min, max}, the 3D analog of a Rect.

// the 8 corners, indexed by axis-sign bits (bit0 = x, bit1 = y, bit2 = z; 0 = min, 1 = max)
Aabb_Corner :: enum {
	Min_X_Min_Y_Min_Z, // 0
	Max_X_Min_Y_Min_Z, // 1
	Min_X_Max_Y_Min_Z, // 2
	Max_X_Max_Y_Min_Z, // 3
	Min_X_Min_Y_Max_Z, // 4
	Max_X_Min_Y_Max_Z, // 5
	Min_X_Max_Y_Max_Z, // 6
	Max_X_Max_Y_Max_Z, // 7
}

// the 6 axis-aligned faces
Aabb_Face :: enum {
	Neg_X,
	Pos_X,
	Neg_Y,
	Pos_Y,
	Neg_Z,
	Pos_Z,
}

@(private)
is_scalar :: intrinsics.type_is_ordered_numeric

// builds an aabb from a center point and full size
aabb_from_center_size :: proc(center, size: [3]$T) -> [2][3]T where is_scalar(T) {
	hs := size / 2
	return {center - hs, center + hs}
}

// returns the axis-aligned bounding box of the given points
aabb_from_points :: proc(points: [][3]$T) -> [2][3]T where is_scalar(T) {
	result := [2][3]T{points[0], points[0]}
	for v in points[1:] {
		result[0] = linalg.min(result[0], v)
		result[1] = linalg.max(result[1], v)
	}
	return result
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

// returns the size (width, height, depth) of the aabb
aabb_get_size :: #force_inline proc "contextless" (a: [2][3]$T) -> [3]T where is_scalar(T) {
	return a[1] - a[0]
}

// returns the volume of the aabb
aabb_get_volume :: #force_inline proc "contextless" (a: [2][3]$T) -> T where is_scalar(T) {
	size := aabb_get_size(a)
	return size.x * size.y * size.z
}

// returns the total surface area of the aabb
aabb_get_surface_area :: #force_inline proc "contextless" (a: [2][3]$T) -> T where is_scalar(T) {
	size := aabb_get_size(a)
	return 2 * (size.x * size.y + size.y * size.z + size.z * size.x)
}

// returns the center point of the aabb
aabb_get_center :: #force_inline proc "contextless" (a: [2][3]$T) -> [3]T where is_scalar(T) {
	return (a[0] + a[1]) / 2
}

// returns the position of a single named corner
aabb_get_corner :: #force_inline proc "contextless" (
	a: [2][3]$T,
	corner: Aabb_Corner,
) -> [3]T where is_scalar(T) {
	i := int(corner)
	return {
		a[0].x if (i & 1) == 0 else a[1].x,
		a[0].y if (i & 2) == 0 else a[1].y,
		a[0].z if (i & 4) == 0 else a[1].z,
	}
}

// returns all eight corners in Aabb_Corner order
aabb_get_corners :: #force_inline proc "contextless" (a: [2][3]$T) -> [8][3]T where is_scalar(T) {
	corners: [8][3]T
	for i in 0 ..< 8 {
		corners[i] = aabb_get_corner(a, Aabb_Corner(i))
	}
	return corners
}

// returns the named face as its own (degenerate) aabb in the face plane
aabb_get_face :: #force_inline proc "contextless" (
	a: [2][3]$T,
	face: Aabb_Face,
) -> [2][3]T where is_scalar(T) {
	switch face {
	case .Neg_X:
		return {a[0], {a[0].x, a[1].y, a[1].z}}
	case .Pos_X:
		return {{a[1].x, a[0].y, a[0].z}, a[1]}
	case .Neg_Y:
		return {a[0], {a[1].x, a[0].y, a[1].z}}
	case .Pos_Y:
		return {{a[0].x, a[1].y, a[0].z}, a[1]}
	case .Neg_Z:
		return {a[0], {a[1].x, a[1].y, a[0].z}}
	case .Pos_Z:
		return {{a[0].x, a[0].y, a[1].z}, a[1]}
	}
	return {}
}

// ---------------------------------------------------------------------------
// Relationships & queries
// ---------------------------------------------------------------------------

// returns true if an aabb contains a given point
aabb_has_point :: #force_inline proc "contextless" (
	a: [2][3]$T,
	p: [3]T,
) -> bool where is_scalar(T) {
	return(
		p.x >= a[0].x &&
		p.x <= a[1].x &&
		p.y >= a[0].y &&
		p.y <= a[1].y &&
		p.z >= a[0].z &&
		p.z <= a[1].z \
	)
}

// returns true if `outer` fully encloses `inner`
aabb_contains_aabb :: #force_inline proc "contextless" (
	outer, inner: [2][3]$T,
) -> bool where is_scalar(T) {
	return(
		inner[0].x >= outer[0].x &&
		inner[0].y >= outer[0].y &&
		inner[0].z >= outer[0].z &&
		inner[1].x <= outer[1].x &&
		inner[1].y <= outer[1].y &&
		inner[1].z <= outer[1].z \
	)
}

// returns true if the two aabbs overlap
aabb_intersects :: #force_inline proc "contextless" (a, b: [2][3]$T) -> bool where is_scalar(T) {
	return(
		a[0].x <= b[1].x &&
		a[1].x >= b[0].x &&
		a[0].y <= b[1].y &&
		a[1].y >= b[0].y &&
		a[0].z <= b[1].z &&
		a[1].z >= b[0].z \
	)
}

// returns the overlapping aabb; ok is false when they don't intersect
aabb_intersection :: #force_inline proc "contextless" (
	a, b: [2][3]$T,
) -> (overlap: [2][3]T, ok: bool) where is_scalar(T) {
	lo := linalg.max(a[0], b[0])
	hi := linalg.min(a[1], b[1])
	if lo.x > hi.x || lo.y > hi.y || lo.z > hi.z {
		return {}, false
	}
	return {lo, hi}, true
}

// returns a new aabb enclosing two given aabbs (their union / bounding box)
aabb_merge :: #force_inline proc "contextless" (a, b: [2][3]$T) -> [2][3]T where is_scalar(T) {
	return {linalg.min(a[0], b[0]), linalg.max(a[1], b[1])}
}

// returns the vertex of the aabb closest to the given point, and which corner it is
aabb_get_nearest_vertex :: proc "contextless" (
	a: [2][3]$T,
	p: [3]T,
) -> (vertex: [3]T, corner: Aabb_Corner) where is_scalar(T) {
	corners := aabb_get_corners(a)
	best := 0
	d := corners[0] - p
	best_dist := d.x * d.x + d.y * d.y + d.z * d.z
	for i in 1 ..< 8 {
		d = corners[i] - p
		dist := d.x * d.x + d.y * d.y + d.z * d.z
		if dist < best_dist {
			best_dist = dist
			best = i
		}
	}
	return corners[best], Aabb_Corner(best)
}

// returns the face of the aabb closest to the given point
aabb_get_nearest_face :: proc "contextless" (
	a: [2][3]$T,
	p: [3]T,
) -> Aabb_Face where is_scalar(T) {
	dnx := abs(p.x - a[0].x)
	dpx := abs(p.x - a[1].x)
	dny := abs(p.y - a[0].y)
	dpy := abs(p.y - a[1].y)
	dnz := abs(p.z - a[0].z)
	dpz := abs(p.z - a[1].z)
	face := Aabb_Face.Neg_X
	best := dnx
	if dpx < best {best = dpx;face = .Pos_X}
	if dny < best {best = dny;face = .Neg_Y}
	if dpy < best {best = dpy;face = .Pos_Y}
	if dnz < best {best = dnz;face = .Neg_Z}
	if dpz < best {best = dpz;face = .Pos_Z}
	return face
}

// returns the nearest point on the boundary to the given point (snaps even from inside)
aabb_project_to_boundary :: proc "contextless" (a: [2][3]$T, p: [3]T) -> [3]T where is_scalar(T) {
	c := aabb_clamp_point(a, p)
	dnx := abs(c.x - a[0].x)
	dpx := abs(a[1].x - c.x)
	dny := abs(c.y - a[0].y)
	dpy := abs(a[1].y - c.y)
	dnz := abs(c.z - a[0].z)
	dpz := abs(a[1].z - c.z)
	m := min(dnx, dpx, dny, dpy, dnz, dpz)
	result := c
	switch {
	case m == dnx:
		result.x = a[0].x
	case m == dpx:
		result.x = a[1].x
	case m == dny:
		result.y = a[0].y
	case m == dpy:
		result.y = a[1].y
	case m == dnz:
		result.z = a[0].z
	case:
		result.z = a[1].z
	}
	return result
}

// clamps a point into the aabb; interior points are returned unchanged
aabb_clamp_point :: #force_inline proc "contextless" (
	a: [2][3]$T,
	p: [3]T,
) -> [3]T where is_scalar(T) {
	return {
		clamp(p.x, a[0].x, a[1].x),
		clamp(p.y, a[0].y, a[1].y),
		clamp(p.z, a[0].z, a[1].z),
	}
}

// ---------------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------------

// moves the aabb by an offset without resizing
aabb_translate :: #force_inline proc "contextless" (
	a: [2][3]$T,
	offset: [3]T,
) -> [2][3]T where is_scalar(T) {
	return {a[0] + offset, a[1] + offset}
}

// scales the aabb about its center by a per-axis factor
aabb_scale :: #force_inline proc "contextless" (
	a: [2][3]$T,
	factor: [3]T,
) -> [2][3]T where is_scalar(T) {
	c := aabb_get_center(a)
	hs := aabb_get_size(a) / 2 * factor
	return {c - hs, c + hs}
}

// grows an aabb outward on all sides by a given vector (each extent grows by 2*amount)
aabb_grow :: #force_inline proc "contextless" (
	a: [2][3]$T,
	amount: [3]T,
) -> [2][3]T where is_scalar(T) {
	return {a[0] - amount, a[1] + amount}
}

// grows an aabb to enclose a given point
aabb_grow_to :: #force_inline proc "contextless" (
	a: [2][3]$T,
	p: [3]T,
) -> [2][3]T where is_scalar(T) {
	return {linalg.min(a[0], p), linalg.max(a[1], p)}
}

// constrains an aabb to sit fully inside `bounds` (translating, and shrinking if needed)
aabb_clamp_aabb :: proc "contextless" (
	a: [2][3]$T,
	bounds: [2][3]T,
) -> [2][3]T where is_scalar(T) {
	size := linalg.min(aabb_get_size(a), aabb_get_size(bounds))
	lo := a[0]
	lo = linalg.min(lo, bounds[1] - size)
	lo = linalg.max(lo, bounds[0])
	return {lo, lo + size}
}

// ---------------------------------------------------------------------------
// Partitioning
// ---------------------------------------------------------------------------

// splits an aabb into two new aabbs at the given x coordinate
aabb_split_x :: #force_inline proc "contextless" (
	a: [2][3]$T,
	x: T,
) -> (lo, hi: [2][3]T) where is_scalar(T) {
	lo = {a[0], {x, a[1].y, a[1].z}}
	hi = {{x, a[0].y, a[0].z}, a[1]}
	return
}

// splits an aabb into two new aabbs at the given y coordinate
aabb_split_y :: #force_inline proc "contextless" (
	a: [2][3]$T,
	y: T,
) -> (lo, hi: [2][3]T) where is_scalar(T) {
	lo = {a[0], {a[1].x, y, a[1].z}}
	hi = {{a[0].x, y, a[0].z}, a[1]}
	return
}

// splits an aabb into two new aabbs at the given z coordinate
aabb_split_z :: #force_inline proc "contextless" (
	a: [2][3]$T,
	z: T,
) -> (lo, hi: [2][3]T) where is_scalar(T) {
	lo = {a[0], {a[1].x, a[1].y, z}}
	hi = {{a[0].x, a[0].y, z}, a[1]}
	return
}

// subdivides an aabb into its eight octants about its center, in Aabb_Corner order
aabb_subdivide :: proc "contextless" (a: [2][3]$T) -> [8][2][3]T where is_scalar(T) {
	c := aabb_get_center(a)
	octants: [8][2][3]T
	for i in 0 ..< 8 {
		corner := aabb_get_corner(a, Aabb_Corner(i))
		octants[i] = {linalg.min(corner, c), linalg.max(corner, c)}
	}
	return octants
}
