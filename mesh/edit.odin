package mesh

// edit.odin holds the "inspect / modify" half of the package: programmatic
// traversal of the half-edge structure, the low-level additive builders, and
// the topology-editing operations (translate / subdivide / inset / extrude).
// The data model and constructors live in mesh.odin; both files share one
// package so private helpers cross the file boundary freely.

import "core:math/linalg"

// --- Low-level builders ---------------------------------------------------
//
// These are the only places (besides _add_face_loop in mesh.odin) that grow the
// topology arrays. Because the arrays are dynamic they remember the allocator
// they were created with, so the edit operations don't need to thread an
// allocator through.

@(private)
_add_vertex :: proc(mesh: ^Mesh, position: [3]f64) -> int {
	append_soa(&mesh.vertices, Mesh_Vertex{position = position})
	return len(mesh.vertices) - 1
}

@(private)
_add_edge :: proc(mesh: ^Mesh, vertex, face: int) -> int {
	append(&mesh.edges, Half_Edge{vertex = vertex, face = face, twin = -1, next = -1, prev = -1})
	return len(mesh.edges) - 1
}

// Appends a fresh face together with a dedicated 1:1 surface. The surface plane
// is left zeroed and is filled in by refresh(). Returns the new face index.
@(private)
_add_face :: proc(mesh: ^Mesh, half_edge: int) -> int {
	fi := len(mesh.faces)
	si := len(mesh.surfaces)
	append(&mesh.surfaces, Surface{first_face = fi, face_count = 1})
	append(&mesh.faces, Face{surface = si, half_edge = half_edge})
	return fi
}

// Links two half-edges together as a -> b in their shared face loop.
@(private)
_link :: #force_inline proc(mesh: ^Mesh, a, b: int) {
	mesh.edges[a].next = b
	mesh.edges[b].prev = a
}

// --- Topology Queries & Traversal -----------------------------------------
//
// The half-edge structure stores no per-vertex back-reference to an incident
// edge, so vertex-rooted queries start with a one-time linear scan
// (vertex_incoming_halfedge) and then rotate locally. Face- and edge-rooted
// queries are purely local. All procs that return a [dynamic] hand ownership to
// the caller; pass context.temp_allocator for throwaway results.

// face_loop_edges returns the half-edge indices that bound a face, in CCW loop
// order starting at face.half_edge.
face_loop_edges :: proc(mesh: ^Mesh, face_idx: int, allocator := context.allocator) -> [dynamic]int {
	out := make([dynamic]int, 0, 8, allocator)
	start := mesh.faces[face_idx].half_edge
	e := start
	for {
		append(&out, e)
		e = mesh.edges[e].next
		if e == start do break
	}
	return out
}

// face_vertices returns the vertex indices around a face, in CCW loop order.
// Vertex k is the one half-edge k of the loop points TO.
face_vertices :: proc(mesh: ^Mesh, face_idx: int, allocator := context.allocator) -> [dynamic]int {
	out := make([dynamic]int, 0, 8, allocator)
	start := mesh.faces[face_idx].half_edge
	e := start
	for {
		append(&out, mesh.edges[e].vertex)
		e = mesh.edges[e].next
		if e == start do break
	}
	return out
}

face_centroid :: proc(mesh: ^Mesh, face_idx: int) -> [3]f64 {
	verts := face_vertices(mesh, face_idx, context.temp_allocator)
	c: [3]f64
	for v in verts do c += mesh.vertices[v].position
	if len(verts) > 0 do c /= f64(len(verts))
	return c
}

// _face_plane fits a plane to a face loop using Newell's method, which is robust
// for non-triangular and slightly non-planar loops.
@(private)
_face_plane :: proc(mesh: ^Mesh, face_idx: int) -> Plane {
	verts := face_vertices(mesh, face_idx, context.temp_allocator)
	cnt := len(verts)
	if cnt < 3 do return {0, 1, 0, 0}

	n: [3]f64
	c: [3]f64
	for k in 0 ..< cnt {
		cur := mesh.vertices[verts[k]].position
		nxt := mesh.vertices[verts[(k + 1) % cnt]].position
		n.x += (cur.y - nxt.y) * (cur.z + nxt.z)
		n.y += (cur.z - nxt.z) * (cur.x + nxt.x)
		n.z += (cur.x - nxt.x) * (cur.y + nxt.y)
		c += cur
	}
	if linalg.length(n) > 1e-12 do n = linalg.normalize(n)
	c /= f64(cnt)
	d := linalg.dot(n, c)
	return {n.x, n.y, n.z, d}
}

face_normal :: proc(mesh: ^Mesh, face_idx: int) -> [3]f64 {
	return _face_plane(mesh, face_idx).xyz
}

// edge_endpoints returns the (from, to) vertex indices of a half-edge.
edge_endpoints :: proc(mesh: ^Mesh, edge_idx: int) -> (v0, v1: int) {
	v1 = mesh.edges[edge_idx].vertex
	v0 = mesh.edges[mesh.edges[edge_idx].prev].vertex
	return
}

// is_boundary_edge reports whether a half-edge has no twin (an open boundary).
is_boundary_edge :: proc(mesh: ^Mesh, edge_idx: int) -> bool {
	return mesh.edges[edge_idx].twin < 0
}

// vertex_incoming_halfedge returns some half-edge that points TO `vertex_idx`,
// or -1 if none exists. The mesh stores no per-vertex edge reference, so this is
// a linear scan; callers traversing many vertices should cache or batch.
vertex_incoming_halfedge :: proc(mesh: ^Mesh, vertex_idx: int) -> int {
	for i in 0 ..< len(mesh.edges) {
		if mesh.edges[i].vertex == vertex_idx do return i
	}
	return -1
}

// vertex_incoming_edges walks the half-edges that point TO `vertex_idx`, one per
// incident face, by rotating around the vertex via next->twin. For a closed
// (manifold) neighbourhood it returns the full ring; on a boundary it returns
// the partial fan reachable before hitting an open edge.
vertex_incoming_edges :: proc(mesh: ^Mesh, vertex_idx: int, allocator := context.allocator) -> [dynamic]int {
	out := make([dynamic]int, 0, 8, allocator)
	start := vertex_incoming_halfedge(mesh, vertex_idx)
	if start < 0 do return out
	h := start
	for {
		append(&out, h)
		// h points to v; h.next leaves v; its twin points back to v in the
		// neighbouring face — the next incoming half-edge around the fan.
		t := mesh.edges[mesh.edges[h].next].twin
		if t < 0 do break
		h = t
		if h == start do break
	}
	return out
}

// vertex_faces returns the faces incident to `vertex_idx` (its one-ring fan).
vertex_faces :: proc(mesh: ^Mesh, vertex_idx: int, allocator := context.allocator) -> [dynamic]int {
	ring := vertex_incoming_edges(mesh, vertex_idx, context.temp_allocator)
	out := make([dynamic]int, 0, len(ring), allocator)
	for e in ring do append(&out, mesh.edges[e].face)
	return out
}

// vertex_neighbors returns the vertices sharing an edge with `vertex_idx` (its
// one-ring of adjacent vertices).
vertex_neighbors :: proc(mesh: ^Mesh, vertex_idx: int, allocator := context.allocator) -> [dynamic]int {
	ring := vertex_incoming_edges(mesh, vertex_idx, context.temp_allocator)
	out := make([dynamic]int, 0, len(ring), allocator)
	for e in ring {
		from, _ := edge_endpoints(mesh, e) // e: from -> vertex_idx
		append(&out, from)
	}
	return out
}

// face_neighbors returns the faces adjacent to `face_idx` across each of its
// edges, skipping open boundary edges that have no twin.
face_neighbors :: proc(mesh: ^Mesh, face_idx: int, allocator := context.allocator) -> [dynamic]int {
	H := face_loop_edges(mesh, face_idx, context.temp_allocator)
	out := make([dynamic]int, 0, len(H), allocator)
	for e in H {
		t := mesh.edges[e].twin
		if t >= 0 do append(&out, mesh.edges[t].face)
	}
	return out
}

// --- Triangulation / Picking Queries --------------------------------------
//
// The mesh package exposes geometry; it does not implement picking or selection.
// Ray-vs-element intersection, nearest-element-to-ray hover policy, screen-space
// thresholds and tie-breaking are caller concerns. get_triangles is the bridge:
// it hands callers a triangle soup they can intersect however they like.

// A single triangle emitted by get_triangles, tagged with the index of the face
// it was fan-triangulated from so a hit can be mapped back to topology.
Mesh_Triangle :: struct {
	v0, v1, v2: [3]f64,
	face:       int,
}

// get_triangles fan-triangulates every face into a flat triangle list, tagging
// each triangle with its source face index. It is the query-friendly counterpart
// to bake(): it preserves the face->topology mapping that bake() discards and
// returns world-space f64 positions, so callers can run exact intersection tests
// at full precision, or downcast to f32 and feed the triangles into algo/bvh for
// accelerated picking on larger meshes. The caller owns the returned slice.
//
// Triangulation is a simple fan from each face's first vertex, so results are
// exact only for convex faces; hits on non-convex n-gons may be approximate.
get_triangles :: proc(mesh: ^Mesh, allocator := context.allocator) -> []Mesh_Triangle {
	tris := make([dynamic]Mesh_Triangle, 0, len(mesh.faces) * 2, allocator)
	for fi in 0 ..< len(mesh.faces) {
		verts := face_vertices(mesh, fi, context.temp_allocator)
		if len(verts) < 3 do continue
		p0 := mesh.vertices[verts[0]].position
		for i in 1 ..< len(verts) - 1 {
			append(&tris, Mesh_Triangle{
				v0   = p0,
				v1   = mesh.vertices[verts[i]].position,
				v2   = mesh.vertices[verts[i + 1]].position,
				face = fi,
			})
		}
	}
	return tris[:]
}

// --- Refresh --------------------------------------------------------------

// refresh recomputes derived data (surface planes + vertex normals) after a
// geometry or topology change. Cheap enough to call after every edit.
refresh :: proc(mesh: ^Mesh) {
	for si in 0 ..< len(mesh.surfaces) {
		f := mesh.surfaces[si].first_face
		if f >= 0 && f < len(mesh.faces) {
			mesh.surfaces[si].plane = _face_plane(mesh, f)
		}
	}
	calculate_normals(mesh)
}

// --- Translation Operations -----------------------------------------------

translate_vertex :: proc(mesh: ^Mesh, vertex_index: int, translation: [3]f64) {
	mesh.vertices[vertex_index].position += translation
	refresh(mesh)
}

translate_edge :: proc(mesh: ^Mesh, edge_index: int, translation: [3]f64) {
	v0, v1 := edge_endpoints(mesh, edge_index)
	mesh.vertices[v0].position += translation
	mesh.vertices[v1].position += translation
	refresh(mesh)
}

translate_face :: proc(mesh: ^Mesh, face_index: int, translation: [3]f64) {
	verts := face_vertices(mesh, face_index, context.temp_allocator)
	for v in verts do mesh.vertices[v].position += translation
	refresh(mesh)
}

// --- Subdivision ----------------------------------------------------------

// _split_edge inserts a midpoint vertex on the undirected edge represented by
// `he` and (if present) its twin, splitting each half-edge into two collinear
// segments. It performs the raw surgery only; callers rebuild twins/refresh.
//
// Returns the new midpoint vertex and the half-edge of the *second* segment on
// he's side (the one pointing to he's original target vertex).
@(private)
_split_edge :: proc(mesh: ^Mesh, he: int) -> (mid: int, second: int) {
	v_to := mesh.edges[he].vertex             // P1 (he: P0 -> P1)
	v_from, _ := edge_endpoints(mesh, he)     // P0
	p_mid := (mesh.vertices[v_from].position + mesh.vertices[v_to].position) * 0.5
	mid = _add_vertex(mesh, p_mid)

	tw := mesh.edges[he].twin

	// he side: he becomes P0 -> M, new edge he2 becomes M -> P1
	he_next := mesh.edges[he].next
	he2 := _add_edge(mesh, v_to, mesh.edges[he].face)
	mesh.edges[he].vertex = mid
	_link(mesh, he, he2)
	_link(mesh, he2, he_next)
	second = he2

	// twin side: tw (P1 -> P0) becomes P1 -> M, new edge tw2 becomes M -> P0
	if tw >= 0 {
		v_tw_to := mesh.edges[tw].vertex      // P0
		tw_next := mesh.edges[tw].next
		tw2 := _add_edge(mesh, v_tw_to, mesh.edges[tw].face)
		mesh.edges[tw].vertex = mid
		_link(mesh, tw, tw2)
		_link(mesh, tw2, tw_next)
	}
	return
}

// subdivide_edge splits an edge at its midpoint, inserting the new vertex into
// both adjacent face loops. Returns the new midpoint vertex index.
subdivide_edge :: proc(mesh: ^Mesh, edge_index: int) -> int {
	mid, _ := _split_edge(mesh, edge_index)
	rebuild_twins(mesh)
	refresh(mesh)
	return mid
}

// subdivide_face performs one step of quad subdivision (Catmull-Clark topology):
// every boundary edge gains a shared midpoint (keeping neighbours crack-free)
// and an n-gon becomes n quads meeting at a new centroid vertex.
subdivide_face :: proc(mesh: ^Mesh, face_index: int) {
	H := face_loop_edges(mesh, face_index, context.temp_allocator)
	n := len(H)
	if n < 3 do return

	// Vertex k is what loop half-edge H[k] points to; H[k] runs V[k-1] -> V[k].
	V := make([]int, n, context.temp_allocator)
	for k in 0 ..< n do V[k] = mesh.edges[H[k]].vertex

	centroid := face_centroid(mesh, face_index)
	C := _add_vertex(mesh, centroid)

	// Split every boundary edge. M[k] is the midpoint on edge H[k] and S[k] is
	// its second segment (M[k] -> V[k]). H[k] now runs V[k-1] -> M[k].
	M := make([]int, n, context.temp_allocator)
	S := make([]int, n, context.temp_allocator)
	for k in 0 ..< n {
		M[k], S[k] = _split_edge(mesh, H[k])
	}

	// One quad per original corner V[k]: M[k] -> V[k] -> M[k+1] -> C.
	for k in 0 ..< n {
		kn := (k + 1) % n
		a := S[k]          // M[k]   -> V[k]
		b := H[kn]         // V[k]   -> M[k+1]
		c := _add_edge(mesh, C, 0)    // M[k+1] -> C
		d := _add_edge(mesh, M[k], 0) // C      -> M[k]

		fi := face_index if k == 0 else _add_face(mesh, a)
		mesh.edges[a].face = fi
		mesh.edges[b].face = fi
		mesh.edges[c].face = fi
		mesh.edges[d].face = fi
		mesh.faces[fi].half_edge = a

		_link(mesh, a, b)
		_link(mesh, b, c)
		_link(mesh, c, d)
		_link(mesh, d, a)
	}

	rebuild_twins(mesh)
	refresh(mesh)
}

// split_faces_at_centroid keeps the legacy surface-indexed name; it quad-
// subdivides the surface's first face.
split_faces_at_centroid :: proc(mesh: ^Mesh, surface_index: int) {
	subdivide_face(mesh, mesh.surfaces[surface_index].first_face)
}

// --- Inset / Extrude ------------------------------------------------------
//
// Both build a new ring of vertices from a face loop and bridge the old and new
// rings with quads. Inset keeps the ring coplanar (pulled toward the centroid);
// extrude offsets the ring along a vector. In both cases the original face is
// reused as the inner/cap face, so the operation is purely additive.

// inset_face shrinks a face inward by `distance`, leaving a border ring of quads
// around a smaller inner face (the original face, repointed to the inner ring).
inset_face :: proc(mesh: ^Mesh, face_index: int, distance: f64) {
	H := face_loop_edges(mesh, face_index, context.temp_allocator)
	n := len(H)
	if n < 3 do return
	V := make([]int, n, context.temp_allocator)
	for k in 0 ..< n do V[k] = mesh.edges[H[k]].vertex

	centroid := face_centroid(mesh, face_index)

	// Inner ring: each outer vertex pulled toward the centroid by `distance`.
	I := make([]int, n, context.temp_allocator)
	for k in 0 ..< n {
		pos := mesh.vertices[V[k]].position
		to_c := centroid - pos
		l := linalg.length(to_c)
		t := l > 1e-9 ? clamp(distance / l, 0, 1) : 0
		I[k] = _add_vertex(mesh, pos + to_c * t)
	}

	// Inner face reuses face_index. Its loop is inner[k]: I[k-1] -> I[k].
	inner := make([]int, n, context.temp_allocator)
	for k in 0 ..< n do inner[k] = _add_edge(mesh, I[k], face_index)
	for k in 0 ..< n {
		_link(mesh, inner[(k - 1 + n) % n], inner[k])
	}
	mesh.faces[face_index].half_edge = inner[0]

	// Border quad per outer edge: V[k-1] -> V[k] -> I[k] -> I[k-1].
	for k in 0 ..< n {
		kp := (k - 1 + n) % n
		a := H[k]                       // V[k-1] -> V[k]  (reused)
		b := _add_edge(mesh, I[k], 0)   // V[k]   -> I[k]
		c := _add_edge(mesh, I[kp], 0)  // I[k]   -> I[k-1]
		d := _add_edge(mesh, V[kp], 0)  // I[k-1] -> V[k-1]

		fi := _add_face(mesh, a)
		mesh.edges[a].face = fi
		mesh.edges[b].face = fi
		mesh.edges[c].face = fi
		mesh.edges[d].face = fi
		mesh.faces[fi].half_edge = a

		_link(mesh, a, b)
		_link(mesh, b, c)
		_link(mesh, c, d)
		_link(mesh, d, a)
	}

	rebuild_twins(mesh)
	refresh(mesh)
}

// extrude_face lifts a face along `offset`, creating side walls connecting the
// original boundary to the offset cap (the original face, repointed upward).
extrude_face :: proc(mesh: ^Mesh, face_index: int, offset: [3]f64) {
	H := face_loop_edges(mesh, face_index, context.temp_allocator)
	n := len(H)
	if n < 3 do return
	V := make([]int, n, context.temp_allocator)
	for k in 0 ..< n do V[k] = mesh.edges[H[k]].vertex

	// Cap ring: each boundary vertex duplicated and offset.
	T := make([]int, n, context.temp_allocator)
	for k in 0 ..< n {
		T[k] = _add_vertex(mesh, mesh.vertices[V[k]].position + offset)
	}

	// Cap face reuses face_index. Its loop is cap[k]: T[k-1] -> T[k].
	cap := make([]int, n, context.temp_allocator)
	for k in 0 ..< n do cap[k] = _add_edge(mesh, T[k], face_index)
	for k in 0 ..< n {
		_link(mesh, cap[(k - 1 + n) % n], cap[k])
	}
	mesh.faces[face_index].half_edge = cap[0]

	// Side wall per boundary edge: V[k-1] -> V[k] -> T[k] -> T[k-1].
	for k in 0 ..< n {
		kp := (k - 1 + n) % n
		a := H[k]                       // V[k-1] -> V[k]  (reused; keeps base twin)
		b := _add_edge(mesh, T[k], 0)   // V[k]   -> T[k]
		c := _add_edge(mesh, T[kp], 0)  // T[k]   -> T[k-1]
		d := _add_edge(mesh, V[kp], 0)  // T[k-1] -> V[k-1]

		fi := _add_face(mesh, a)
		mesh.edges[a].face = fi
		mesh.edges[b].face = fi
		mesh.edges[c].face = fi
		mesh.edges[d].face = fi
		mesh.faces[fi].half_edge = a

		_link(mesh, a, b)
		_link(mesh, b, c)
		_link(mesh, c, d)
		_link(mesh, d, a)
	}

	rebuild_twins(mesh)
	refresh(mesh)
}

// --- Legacy surface-indexed wrappers --------------------------------------

translate_surface :: proc(mesh: ^Mesh, surface_index: int, translation: [3]f64) {
	translate_face(mesh, mesh.surfaces[surface_index].first_face, translation)
}

extrude_surface :: proc(mesh: ^Mesh, surface_index: int, vector: [3]f64) {
	extrude_face(mesh, mesh.surfaces[surface_index].first_face, vector)
}

inset_surface :: proc(mesh: ^Mesh, surface_index: int, distance: f64) {
	inset_face(mesh, mesh.surfaces[surface_index].first_face, distance)
}

// --- Unimplemented edit stubs (not required by the mesh modeler) ----------

add_surface_from_contours :: proc(mesh: ^Mesh, contours: [][][3]f64, plane_normal: [3]f64, allocator := context.allocator) {}
bevel_surface :: proc(mesh: ^Mesh, surface_index: int, distance: f64) {}
bevel_edge :: proc(mesh: ^Mesh, edge_index: int, distance: f64) {}
