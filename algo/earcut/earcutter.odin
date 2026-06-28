package triangles

import "core:fmt"
import "core:slice"
import "core:math/linalg"

// ============================================================
// Earcut Triangulator
// Pure Odin implementation.
// Based on the mapbox/earcut algorithm:
//   https://github.com/mapbox/earcut
//
// Supports simple polygons and polygons with holes.
// Hole support works via a horizontal bridge that merges each
// hole into the outer contour before running the core earcut.
// ============================================================


// Helper struct to sort holes by their maximum X coordinate descending
@(private)
_Ec_Hole_Sort :: struct {
    index: int,
    max_x: f32, // Replace with $T or f64 if you want to keep it strictly generic
}

// Standard shoelace signed area.
// Positive = CCW (standard math orientation with y-up).
@(private)
_ec_signed_area :: proc(verts: [][2]$T) -> T {
	n := len(verts)
	area: T
	for i in 0 ..< n {
		j := (i + 1) % n
		area += verts[i].x * verts[j].y - verts[j].x * verts[i].y
	}
	return area
}

// 2D cross product at vertex o from vectors o→a and o→b.
// Positive = left turn (CCW), negative = right turn (CW).
@(private)
_ec_cross2 :: #force_inline proc(o, a, b: [2]$T) -> T {
	return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

// Returns true if point p is inside (or on the boundary of) CCW triangle (a, b, c).
// Uses a small negative epsilon so points within numerical noise of an edge count
// as "inside" — this conservatively rejects potential ears near degenerate geometry.
@(private)
_ec_in_triangle :: proc(p, a, b, c: [2]$T) -> bool {
	EPS :: -1e-10
	return _ec_cross2(a, b, p) > EPS &&
	       _ec_cross2(b, c, p) > EPS &&
	       _ec_cross2(c, a, p) > EPS
}

// Recomputes and stores the ear flag for vertex i in the polygon ring
// defined by the prv/nxt doubly-linked index arrays.
@(private)
_ec_update_ear :: proc(verts: [][2]$T, prv, nxt: []int, is_ear: []bool, i: int) {
	p := verts[prv[i]]
	a := verts[i]
	q := verts[nxt[i]]

	// Convex vertex check: the ear triangle must have positive (CCW) area.
	if _ec_cross2(p, a, q) <= 0 {
		is_ear[i] = false
		return
	}

	// No other active vertex may lie inside this ear triangle.
	// The loop skips prv[i] and nxt[i] (the triangle corners) by design.
	// Vertices coincident with a corner are skipped too: hole merging inserts
	// zero-width bridge duplicates that sit exactly on p/a/q and would otherwise
	// (via the on-edge epsilon in _ec_in_triangle) falsely reject every ear along
	// the bridge, leaving the polygon untriangulable.
	j := nxt[nxt[i]]
	for j != prv[i] {
		vj := verts[j]
		if vj != p && vj != a && vj != q && _ec_in_triangle(vj, p, a, q) {
			is_ear[i] = false
			return
		}
		j = nxt[j]
	}
	is_ear[i] = true
}

// Core earcut: triangulate a simple 2D polygon.
// Any winding is accepted — the polygon is normalised to CCW internally.
// Returns triangle indices (groups of 3) into verts. Caller must delete.
@(private)
_ec_triangulate :: proc(verts: [][2]$T, allocator := context.allocator) -> []int {
	n := len(verts)
	if n < 3 do return {}
	if n == 3 {
		r := make([]int, 3, allocator)
		r[0] = 0
		r[1] = 1
		r[2] = 2
		return r
	}

	// Build doubly-linked ring.
	prv := make([]int, n)
	defer delete(prv)
	nxt := make([]int, n)
	defer delete(nxt)
	for i in 0 ..< n {
		prv[i] = (i - 1 + n) % n
		nxt[i] = (i + 1) % n
	}

	// Normalise to CCW: if the signed area is negative the polygon is CW,
	// so we reverse the traversal direction by swapping prv and nxt.
	if _ec_signed_area(verts) < 0 {
		for i in 0 ..< n {
			prv[i], nxt[i] = nxt[i], prv[i]
		}
	}

	// Initialise ear flags.
	is_ear := make([]bool, n)
	defer delete(is_ear)
	for i in 0 ..< n {
		_ec_update_ear(verts, prv, nxt, is_ear, i)
	}

	// Result buffer: at most (n-2) triangles.
	result := make([]int, (n - 2) * 3, allocator)
	tri_count := 0
	active := n

	// Walk the ring, clipping one ear per iteration.
	// The safety cap on max_iters guards against degenerate input that would
	// otherwise loop forever (e.g. all-collinear vertices).
	i := 0
	max_iters := n * n + n
	for iters := 0; active > 3 && iters < max_iters; iters += 1 {
		if !is_ear[i] {
			i = nxt[i]
			continue
		}

		pi := prv[i]
		ni := nxt[i]

		result[tri_count * 3 + 0] = pi
		result[tri_count * 3 + 1] = i
		result[tri_count * 3 + 2] = ni
		tri_count += 1

		// Excise vertex i from the ring.
		nxt[pi] = ni
		prv[ni] = pi

		// Only the two neighbours can change ear status after a clip.
		_ec_update_ear(verts, prv, nxt, is_ear, pi)
		_ec_update_ear(verts, prv, nxt, is_ear, ni)

		active -= 1
		i = ni
	}

	// Emit the final remaining triangle.
	if active == 3 {
		result[tri_count * 3 + 0] = prv[i]
		result[tri_count * 3 + 1] = i
		result[tri_count * 3 + 2] = nxt[i]
		tri_count += 1
	}

	if tri_count < n - 2 {
		fmt.printfln(
			"EARCUT WARNING: expected %d triangles, produced %d — polygon may be degenerate",
			n - 2,
			tri_count,
		)
	}

	return result[:tri_count * 3]
}

// Newell's method
@(private)
polygon_normal :: proc(profile: [][3]$T) -> [3]T {
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

// Project a 3D planar polygon onto its own plane as 2D coordinates.
// Builds an orthonormal {u, v} basis from the polygon's Newell normal.
@(private)
_ec_project_2d :: proc(polygon: [][3]$T) -> [][2]T {
	n := len(polygon)
	if n == 0 do return {}

	normal := polygon_normal(polygon)

	// Choose a reference vector that is not nearly parallel to the normal.
	ref := [3]T{0, 0, 1}
	if abs(linalg.dot(normal, ref)) > 0.9 {
		ref = [3]T{1, 0, 0}
	}
	u := linalg.normalize0(linalg.cross(normal, ref))
	v := linalg.cross(normal, u)

	result := make([][2]T, n)
	for p, i in polygon {
		result[i] = {linalg.dot(p, u), linalg.dot(p, v)}
	}
	return result
}

// ---------------------------------------------------------------------------
// Hole support — bridge technique
//
// Each hole is merged into the outer contour via a horizontal bridge:
//   1. Find the hole vertex with the largest x (rightmost).
//   2. Cast a ray in the +x direction and find the nearest outer edge.
//   3. Walk potential bridge candidates near the intersection to find the
//      mutually-visible vertex on the outer contour.
//   4. Splice the hole ring into the outer ring through the bridge.
// ---------------------------------------------------------------------------

// Finds a vertex on the current outer contour that the hole at (hx, hy) can
// bridge to. Walks the entire live ring starting at `start` — which grows as
// earlier holes get merged in — so multiple holes are handled correctly. Casts a
// +x ray from the hole and takes the nearest visible edge vertex.
@(private)
_ec_find_hole_bridge :: proc(pts: [][2]$T, prv, nxt: []int, start: int, h_idx: int) -> int {
    p := pts[h_idx]
    hx, hy := p.x, p.y
    
    best_x := -max(T)
    best_idx := -1

    // Step 1: Find all outer edges intersecting the ray y = hy to the right of hx
    i := start
    for {
        a := pts[i]
        b := pts[nxt[i]]

        if (a.y > hy) != (b.y > hy) && a.y != b.y {
            ix := a.x + (hy - a.y) * (b.x - a.x) / (b.y - a.y)
            if ix >= hx {
                if ix > best_x {
                    best_x = ix
                    best_idx = a.x > b.x ? i : nxt[i]
                }
            }
        }
        i = nxt[i]
        if i == start do break
    }

    if best_idx == -1 do return start

    // Step 2: Check if any reflex vertices lie inside the triangle (hole, intersection, candidate)
    // If they do, choose the one that minimizes the strict geometric distance/angle metric.
    bridge := best_idx
    v_candidate := pts[bridge]
    
    // Define the filtering triangle bounds
    p0 := p
    p1 := [2]T{best_x, hy}
    p2 := v_candidate
    
    // We invert the triangle test if needed to ensure CCW order for the visibility check
    if _ec_cross2(p0, p1, p2) < 0 {
        p1, p2 = p2, p1
    }

    i = start
    for {
        // Only check vertices that could potentially obstruct line of sight
        if pts[i].x >= hx && pts[i].x <= best_x && i != bridge {
            if _ec_in_triangle(pts[i], p0, p1, p2) {
                // If it's inside, this vertex becomes our new strictly visible candidate
                bridge = i
                p2 = pts[bridge]
                // Re-verify orientation
                if _ec_cross2(p0, p1, p2) < 0 {
                    p1, p2 = p2, p1
                }
            }
        }
        i = nxt[i]
        if i == start do break
    }

    return bridge
}

// Triangulate a single 2D polygon, potentially with holes.
// outer:  outer contour vertices (any winding).
// holes:  each hole as a slice of 2D vertices (any winding).
// Returns triangle indices into a merged vertex array. Caller must delete.
@(private)
_ec_triangulate_with_holes :: proc(
    outer: [][2]$T,
    holes: [][][2]T,
    allocator := context.allocator,
) -> (indices: []int, merged_verts: [][2]T) {
    if len(holes) == 0 {
        merged_verts = make([][2]T, len(outer), allocator)
        copy(merged_verts, outer)
        indices = _ec_triangulate(merged_verts, allocator)
        return
    }

    // Allocate and flatten vertices
    total := len(outer)
    for h in holes do total += len(h)

    merged_verts = make([][2]T, total, allocator)
    copy(merged_verts[:len(outer)], outer)
    
    // Keep track of where each hole starts in the flat array
    hole_offsets := make([]int, len(holes), context.temp_allocator)
    
    off := len(outer)
    for h, idx in holes {
        hole_offsets[idx] = off
        copy(merged_verts[off:off + len(h)], h)
        off += len(h)
    }

    // Sort holes by rightmost X coordinate descending
    hole_sort := make([]_Ec_Hole_Sort, len(holes), context.temp_allocator)
    for h, idx in holes {
        max_x := -max(T)
        for p in h {
            if p.x > max_x do max_x = p.x
        }
        hole_sort[idx] = { index = idx, max_x = f32(max_x) }
    }
    
    slice.sort_by(hole_sort, proc(a, b: _Ec_Hole_Sort) -> bool {
        return a.max_x > b.max_x
    })

    // Setup working arrays with room for bridge duplicates
    n_work := total + 2 * len(holes)
    pts := make([][2]T, n_work)
    defer delete(pts)
    copy(pts[:total], merged_verts)
    
    orig := make([]int, n_work)
    defer delete(orig)
    for i in 0 ..< n_work do orig[i] = i

    prv := make([]int, n_work)
    defer delete(prv)
    nxt := make([]int, n_work)
    defer delete(nxt)

    // Setup outer ring (CCW)
    n := len(outer)
    for i in 0 ..< n {
        prv[i] = (i - 1 + n) % n
        nxt[i] = (i + 1) % n
    }
    if _ec_signed_area(pts[:n]) < 0 {
        for i in 0 ..< n {
            prv[i], nxt[i] = nxt[i], prv[i]
        }
    }

    next_free := total

    // Process sorted holes
    for hs in hole_sort {
        h_idx := hs.index
        h := holes[h_idx]
        hn := len(h)
        hole_off := hole_offsets[h_idx]

        for i in 0 ..< hn {
            gi := hole_off + i
            prv[gi] = hole_off + (i - 1 + hn) % hn
            nxt[gi] = hole_off + (i + 1) % hn
        }

        // Holes must be CW
        if _ec_signed_area(pts[hole_off:hole_off + hn]) > 0 {
            for i in 0 ..< hn {
                gi := hole_off + i
                prv[gi], nxt[gi] = nxt[gi], prv[gi]
            }
        }

        // Find rightmost vertex of this hole
        rightmost := hole_off
        for i in 1 ..< hn {
            gi := hole_off + i
            if pts[gi].x > pts[rightmost].x {
                rightmost = gi
            }
        }

        // Find the safe, unobstructed bridge point
        bridge := _ec_find_hole_bridge(pts, prv, nxt, 0, rightmost)

        // Inject splitting bridge nodes
        b2 := next_free; next_free += 1
        m2 := next_free; next_free += 1
        pts[b2] = pts[bridge];    orig[b2] = orig[bridge]
        pts[m2] = pts[rightmost]; orig[m2] = orig[rightmost]

        an := nxt[bridge]
        bp := prv[rightmost]
        
        nxt[bridge] = rightmost; prv[rightmost] = bridge
        nxt[b2] = an;            prv[an] = b2
        nxt[m2] = b2;            prv[b2] = m2
        nxt[bp] = m2;            prv[m2] = bp
    }

    // Recover index execution sequence
    ring := make([]int, next_free + 1)
    defer delete(ring)
    ring_count := 0
    cur := 0
    for _ in 0 ..< next_free + 1 {
        ring[ring_count] = cur
        ring_count += 1
        cur = nxt[cur]
        if cur == 0 do break
    }

    ordered := make([][2]T, ring_count)
    defer delete(ordered)
    for k in 0 ..< ring_count {
        ordered[k] = pts[ring[k]]
    }

    raw_indices := _ec_triangulate(ordered)
    defer delete(raw_indices)

    indices = make([]int, len(raw_indices), allocator)
    for v, i in raw_indices {
        indices[i] = orig[ring[v]]
    }

    return
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Triangulate a single planar 3D polygon.
// Returns triangles as [][3][3]T. Caller must delete.
earcut_triangulate :: proc(polygon: [][3]$T, allocator := context.allocator) -> [][3][3]T {
	if len(polygon) < 3 do return {}

	verts2d := _ec_project_2d(polygon)
	defer delete(verts2d)

	indices := _ec_triangulate(verts2d)
	defer delete(indices)

	tri_count := len(indices) / 3
	result := make([][3][3]T, tri_count, allocator)
	for t in 0 ..< tri_count {
		i0 := indices[t * 3 + 0]
		i1 := indices[t * 3 + 1]
		i2 := indices[t * 3 + 2]
		result[t] = {polygon[i0], polygon[i1], polygon[i2]}
	}
	return result
}

// Project a polygon onto the plane basis {u, v}.
// Polymorphic on its own so it can live at top level for any float type.
@(private)
_ec_project_onto_basis :: proc(poly: [][3]$T, u, v: [3]T) -> [][2]T {
	r := make([][2]T, len(poly))
	for p, i in poly {
		r[i] = {linalg.dot(p, u), linalg.dot(p, v)}
	}
	return r
}

// Drop-in replacement for libtess2.triangulate_polygons.
// polygons[0] is the outer boundary; polygons[1..] are holes.
// All polygons must be coplanar. Caller must delete the result.
// Drop-in replacement for libtess2.triangulate_polygons with zero-area culling.
triangulate_polygons :: proc(
    polygons: [][][3]$T,
    allocator := context.allocator,
) -> [][3][3]T {
    if len(polygons) == 0 do return {}

    outer3d := polygons[0]
    if len(outer3d) < 3 do return {}

    // 1. Plane Projection Setup
    normal := polygon_normal(outer3d)
    ref := [3]T{0, 0, 1}
    if abs(linalg.dot(normal, ref)) > 0.9 {
        ref = [3]T{1, 0, 0}
    }
    u := linalg.normalize0(linalg.cross(normal, ref))
    v := linalg.cross(normal, u)

    outer2d := _ec_project_onto_basis(outer3d, u, v)
    defer delete(outer2d)

    holes2d := make([][][2]T, len(polygons) - 1)
    defer {
        for h in holes2d do delete(h)
        delete(holes2d)
    }
    for i in 1 ..< len(polygons) {
        holes2d[i - 1] = _ec_project_onto_basis(polygons[i], u, v)
    }

    // 2. Perform Triangulation
    indices, merged_verts := _ec_triangulate_with_holes(outer2d, holes2d)
    defer delete(indices)
    defer delete(merged_verts)

    // 3. Assemble 3D positions matching the merged_verts layout
    total3d := len(outer3d)
    for i in 1 ..< len(polygons) do total3d += len(polygons[i])

    merged3d := make([][3]T, total3d)
    defer delete(merged3d)
    copy(merged3d[:len(outer3d)], outer3d)
    off := len(outer3d)
    for i in 1 ..< len(polygons) {
        copy(merged3d[off:off + len(polygons[i])], polygons[i])
        off += len(polygons[i])
    }

    // 4. Culling Pass: Count and keep only non-degenerate triangles
    raw_tri_count := len(indices) / 3
    valid_indices := make([][3]int, raw_tri_count, context.temp_allocator)
    valid_count := 0

    // Epsilon threshold adjusted for standard precision limits
    AREA_EPSILON :: 1e-7 

    for t in 0 ..< raw_tri_count {
        i0 := indices[t * 3 + 0]
        i1 := indices[t * 3 + 1]
        i2 := indices[t * 3 + 2]

        // Sample 2D projected coordinates to check true flat area
        p0 := merged_verts[i0]
        p1 := merged_verts[i1]
        p2 := merged_verts[i2]

        // Standard 2D double triangle area formula
        double_area := abs(p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y))
        
        // If the triangle has meaningful geometric surface area, keep it
        if double_area > AREA_EPSILON {
            valid_indices[valid_count] = {i0, i1, i2}
            valid_count += 1
        }
    }

    // 5. Build final clean geometry array
    result := make([][3][3]T, valid_count, allocator)
    for t in 0 ..< valid_count {
        ids := valid_indices[t]
        result[t] = {merged3d[ids[0]], merged3d[ids[1]], merged3d[ids[2]]}
    }

    return result
}
