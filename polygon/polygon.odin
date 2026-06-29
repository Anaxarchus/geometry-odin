package polygon

import "base:intrinsics"
import "core:math"
import "core:math/linalg"
import "core:slice"
import "../libtess2"
import earcut "../algo/earcut"

// Polygons are treated as closed contours (the last vertex connects back to the
// first) wound counter-clockwise in a y-up convention: signed area is positive
// for CCW, negative for CW.
//
// The package is generic over the float type $T. The pure analytic ops run
// natively in $T. The boolean/offset/triangulate ops are backed by libtess2,
// which is f64-only, so they round-trip through f64: the input is cast to f64,
// processed, and the result cast back to $T. Callers pay that conversion per
// call. If you are chaining several tess-backed ops, work in f64 end-to-end (or
// call the libtess2 package directly) to avoid repeated round-trips -- and note
// that libtess2 is not a high-performance backend regardless.

@(private)
is_float :: intrinsics.type_is_float

// ---------------------------------------------------------------------------
// Analytic queries (native, no casting)
// ---------------------------------------------------------------------------

@(private)
_signed_area :: proc(polygon: [][2]$T) -> T where is_float(T) {
	area: T
	n := len(polygon)
	for i in 0 ..< n {
		j := (i + 1) % n
		area += polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y
	}
	return area * 0.5
}

@(private)
_point_line_distance :: proc(p, a, b: [2]$T) -> T where is_float(T) {
	ab := b - a
	l := linalg.length(ab)
	if l < 1e-12 {
		return linalg.distance(p, a)
	}
	ap := p - a
	cross := ab.x * ap.y - ab.y * ap.x
	return abs(cross) / l
}

// Newell's method
normal :: proc(profile: [][3]$T) -> [3]T where is_float(T) {
	n := len(profile)
	normal: [3]T
	for i in 0 ..< n {
		curr := profile[i]
		next := profile[(i + 1) % n]
		normal.x += (curr.y - next.y) * (curr.z + next.z)
		normal.y += (curr.z - next.z) * (curr.x + next.x)
		normal.z += (curr.x - next.x) * (curr.y + next.y)
	}
	return linalg.normalize0(normal)
}

// returns true if a point is inside the polygon
Interior_Rule :: enum {
	Even_Odd,
	Winding_Number,
}
has_point :: proc(
	polygon: [][2]$T,
	point: [2]T,
	rule := Interior_Rule.Even_Odd,
) -> bool where is_float(T) {
	n := len(polygon)
	if n < 3 {
		return false
	}

	switch rule {
	case .Even_Odd:
		inside := false
		j := n - 1
		for i in 0 ..< n {
			pi := polygon[i]
			pj := polygon[j]
			if (pi.y > point.y) != (pj.y > point.y) {
				x_cross := (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x
				if point.x < x_cross {
					inside = !inside
				}
			}
			j = i
		}
		return inside

	case .Winding_Number:
		wn := 0
		for i in 0 ..< n {
			a := polygon[i]
			b := polygon[(i + 1) % n]
			// is_left > 0 if the point is left of the directed edge a->b
			is_left := (b.x - a.x) * (point.y - a.y) - (point.x - a.x) * (b.y - a.y)
			if a.y <= point.y {
				if b.y > point.y && is_left > 0 {
					wn += 1
				}
			} else {
				if b.y <= point.y && is_left < 0 {
					wn -= 1
				}
			}
		}
		return wn != 0
	}
	return false
}

// returns true if the polygon is convex
is_convex :: proc(polygon: [][2]$T) -> bool where is_float(T) {
	n := len(polygon)
	if n < 3 {
		return false
	}
	got_pos := false
	got_neg := false
	for i in 0 ..< n {
		a := polygon[i]
		b := polygon[(i + 1) % n]
		c := polygon[(i + 2) % n]
		cross := (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)
		if cross > 0 {
			got_pos = true
		} else if cross < 0 {
			got_neg = true
		}
		if got_pos && got_neg {
			return false
		}
	}
	return true
}

// returns true if the polygon is wound clockwise (negative signed area, y-up)
is_clockwise :: proc(polygon: [][2]$T) -> bool where is_float(T) {
	return _signed_area(polygon) < 0
}

// returns the area of the polygon via shoelace
area :: proc(polygon: [][2]$T) -> T where is_float(T) {
	return abs(_signed_area(polygon))
}

// returns the perimeter length of the polygon (including the closing edge)
perimeter :: proc(polygon: [][2]$T) -> T where is_float(T) {
	per: T
	n := len(polygon)
	for i in 0 ..< n {
		per += linalg.distance(polygon[i], polygon[(i + 1) % n])
	}
	return per
}

// returns the bounding rect of the polygon as {min, max} (rect-compatible)
min_max :: proc(polygon: [][2]$T) -> [2][2]T where is_float(T) {
	if len(polygon) == 0 {
		return {}
	}
	lo := polygon[0]
	hi := polygon[0]
	for p in polygon[1:] {
		lo = linalg.min(lo, p)
		hi = linalg.max(hi, p)
	}
	return {lo, hi}
}

// returns a list of vertex pairs for each edge (caller owns the result)
edges :: proc(
	polygon: [][2]$T,
	allocator := context.allocator,
) -> [][2][2]T where is_float(T) {
	n := len(polygon)
	edges := make([][2][2]T, n, allocator)
	for i in 0 ..< n {
		edges[i] = {polygon[i], polygon[(i + 1) % n]}
	}
	return edges
}

// reduces the resolution of a polygon using an angular threshold (radians);
// drops vertices whose turn angle is at or below the threshold. Lossy: alters
// the shape. Caller owns the result.
decimate :: proc(
	polygon: [][2]$T,
	angle_threshold: T,
	allocator := context.allocator,
) -> [][2]T where is_float(T) {
	n := len(polygon)
	if n < 3 {
		return slice.clone(polygon, allocator)
	}
	out := make([dynamic][2]T, 0, n, allocator)
	for i in 0 ..< n {
		prev := polygon[(i - 1 + n) % n]
		curr := polygon[i]
		next := polygon[(i + 1) % n]
		v1 := linalg.normalize0(curr - prev)
		v2 := linalg.normalize0(next - curr)
		turn := math.acos(clamp(linalg.dot(v1, v2), -1, 1))
		if turn > angle_threshold {
			append(&out, curr)
		}
	}
	return out[:]
}

// removes (near-)collinear points within epsilon perpendicular distance.
// Topology-preserving: does not change the shape. Caller owns the result.
simplify :: proc(
	polygon: [][2]$T,
	epsilon: T,
	allocator := context.allocator,
) -> [][2]T where is_float(T) {
	n := len(polygon)
	if n < 3 {
		return slice.clone(polygon, allocator)
	}
	out := make([dynamic][2]T, 0, n, allocator)
	for i in 0 ..< n {
		prev := polygon[(i - 1 + n) % n]
		curr := polygon[i]
		next := polygon[(i + 1) % n]
		if _point_line_distance(curr, prev, next) > epsilon {
			append(&out, curr)
		}
	}
	return out[:]
}

// ---------------------------------------------------------------------------
// f64 cast helpers for the libtess2-backed ops
// ---------------------------------------------------------------------------

@(private)
_contour_to_f64 :: proc(c: [][2]$T, allocator := context.allocator) -> [][2]f64 {
	out := make([][2]f64, len(c), allocator)
	for p, i in c {
		out[i] = linalg.array_cast(p, f64)
	}
	return out
}

@(private)
_contours_to_f64 :: proc(cs: [][][2]$T, allocator := context.allocator) -> [][][2]f64 {
	out := make([][][2]f64, len(cs), allocator)
	for c, i in cs {
		out[i] = _contour_to_f64(c, allocator)
	}
	return out
}

@(private)
_contours_from_f64 :: proc(
	cs: [][][2]f64,
	$T: typeid,
	allocator := context.allocator,
) -> [][][2]T {
	out := make([][][2]T, len(cs), allocator)
	for c, i in cs {
		inner := make([][2]T, len(c), allocator)
		for p, j in c {
			inner[j] = linalg.array_cast(p, T)
		}
		out[i] = inner
	}
	return out
}

@(private)
_triangles_from_f64 :: proc(
	ts: [][3][2]f64,
	$T: typeid,
	allocator := context.allocator,
) -> [][3][2]T {
	out := make([][3][2]T, len(ts), allocator)
	for tri, i in ts {
		out[i] = {
			linalg.array_cast(tri[0], T),
			linalg.array_cast(tri[1], T),
			linalg.array_cast(tri[2], T),
		}
	}
	return out
}

@(private)
_join :: proc(j: Polygon_Join_Type) -> libtess2.Join_Type {
	switch j {
	case .Miter:
		return .Miter
	case .Bevel:
		return .Bevel
	case .Round:
		return .Round
	}
	return .Miter
}

// ---------------------------------------------------------------------------
// libtess2-backed ops (round-trip through f64; see package note)
// ---------------------------------------------------------------------------

Polygon_Join_Type :: enum {
	Miter,
	Bevel,
	Round,
}

// uniform delta offset; arc_resolution is the chord-deviation tolerance for
// round joins (polygon units), miter_limit <= 0 means no limit. Returns the
// offset boundary as a set of contours. Caller owns the result.
offset :: proc(
	polygon: [][2]$T,
	delta: T,
	join: Polygon_Join_Type,
	arc_resolution: f64 = 0.1,
	miter_limit: f64 = 0,
	allocator := context.allocator,
) -> [][][2]T where is_float(T) {
	f := _contour_to_f64(polygon, context.temp_allocator)
	deltas := make([]f64, len(f), context.temp_allocator)
	for i in 0 ..< len(deltas) {
		deltas[i] = f64(delta)
	}
	res := libtess2.offset_polygon_edges(f, deltas, _join(join), arc_resolution, miter_limit)
	defer libtess2.delete_contours(res)
	return _contours_from_f64(res, T, allocator)
}

// per-edge delta offset; deltas[i] applies to the edge leaving vertex i.
// Caller owns the result.
offset_edges :: proc(
	polygon: [][2]$T,
	deltas: []T,
	join: Polygon_Join_Type,
	arc_resolution: f64 = 0.1,
	miter_limit: f64 = 0,
	allocator := context.allocator,
) -> [][][2]T where is_float(T) {
	f := _contour_to_f64(polygon, context.temp_allocator)
	fd := make([]f64, len(deltas), context.temp_allocator)
	for d, i in deltas {
		fd[i] = f64(d)
	}
	res := libtess2.offset_polygon_edges(f, fd, _join(join), arc_resolution, miter_limit)
	defer libtess2.delete_contours(res)
	return _contours_from_f64(res, T, allocator)
}

// Triangulation backend.
//   Robust: libtess2 (f64 round-trip) -- tolerant of messy/self-touching input
//           and shares the boolean pipeline.
//   Fast:   earcut (native $T, no f64 round-trip) -- quicker for a single
//           simple (possibly concave) contour.
Triangulate_Mode :: enum {
	Robust,
	Fast,
}

// returns the triangles of the polygon. Caller owns the result.
triangulate :: proc(
	polygon: [][2]$T,
	mode := Triangulate_Mode.Robust,
	allocator := context.allocator,
) -> [][3][2]T where is_float(T) {
	switch mode {
	case .Robust:
		set := make([][][2]f64, 1, context.temp_allocator)
		set[0] = _contour_to_f64(polygon, context.temp_allocator)
		tris := libtess2.triangulate_polygons(set)
		defer delete(tris)
		return _triangles_from_f64(tris, T, allocator)

	case .Fast:
		// earcut works on 3D coplanar polygons; lift to z = 0 then drop back
		lifted := make([][3]T, len(polygon), context.temp_allocator)
		for p, i in polygon {
			lifted[i] = {p.x, p.y, 0}
		}
		tris := earcut.earcut_triangulate(lifted, context.temp_allocator)
		out := make([][3][2]T, len(tris), allocator)
		for tr, i in tris {
			out[i] = {{tr[0].x, tr[0].y}, {tr[1].x, tr[1].y}, {tr[2].x, tr[2].y}}
		}
		return out
	}
	return {}
}

// returns the union of the input polygons (all should be CCW). Caller owns the
// result.
union_polygon :: proc(
	polygons: [][][2]$T,
	allocator := context.allocator,
) -> [][][2]T where is_float(T) {
	f := _contours_to_f64(polygons, context.temp_allocator)
	res := libtess2.union_polygons(f)
	defer libtess2.delete_contours(res)
	return _contours_from_f64(res, T, allocator)
}

// returns the regions covered by at least two of the input polygons. For
// exactly two this is their intersection; for three or more it is "covered >= 2
// times", not the strict n-way intersection. Caller owns the result.
intersect :: proc(
	polygons: [][][2]$T,
	allocator := context.allocator,
) -> [][][2]T where is_float(T) {
	f := _contours_to_f64(polygons, context.temp_allocator)
	res := libtess2.intersect_polygons(f)
	defer libtess2.delete_contours(res)
	return _contours_from_f64(res, T, allocator)
}

// subtracts the cutters (polygons[1:]) from the subject (polygons[0]). Exact
// when the cutters do not mutually overlap. Caller owns the result.
difference :: proc(
	polygons: [][][2]$T,
	allocator := context.allocator,
) -> [][][2]T where is_float(T) {
	f := _contours_to_f64(polygons, context.temp_allocator)
	res := libtess2.difference_polygons(f)
	defer libtess2.delete_contours(res)
	return _contours_from_f64(res, T, allocator)
}

// returns the symmetric difference of the input polygons: regions covered an
// odd number of times. Caller owns the result.
xor :: proc(
	polygons: [][][2]$T,
	allocator := context.allocator,
) -> [][][2]T where is_float(T) {
	f := _contours_to_f64(polygons, context.temp_allocator)
	res := libtess2.xor_polygons(f)
	defer libtess2.delete_contours(res)
	return _contours_from_f64(res, T, allocator)
}


// returns 1 or more convex hulls from a given polygon.
convex_hulls :: proc(
	polygon: [][2]$T,
	allocator := context.allocator,
) -> [][][2]T where is_float(T) {

}