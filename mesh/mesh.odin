package mesh

import "core:math/linalg"

import "../plane"
import "../types"

// {x, y, z, d}
Plane :: types.Plane_f64

// types.Mesh_Vertex_f64 :: struct {
//  position: Vector3f64,
//  normal: Vector3f64,
//  tangent: Vector4f64,
//  uv: Vector2f64,
// }
Mesh_Vertex :: types.Mesh_Vertex_f64

// types.Indexed_Mesh_f32 :: struct {
//  vertices: []Mesh_Vertex_f32,
//  indices: []u32,
// }
Indexed_Mesh :: types.Indexed_Mesh_f32

// --- Structural Topology ---

Surface :: struct {
	plane:      Plane,
	first_face: int,
	face_count: int,
}

Face :: struct {
	surface:   int,
	half_edge: int,
}

Half_Edge :: struct {
	vertex: int, // Vertex it points TO
	face:   int, // Parent face
	twin:   int, // Opposing half-edge (-1 if open boundary)
	next:   int, // CCW next
	prev:   int, // CCW prev
}

// The mesh uses #soa[dynamic]/[dynamic] storage so editing operations can grow
// the topology. Every required edit (subdivide / inset / extrude) is designed to
// be *additive* — it only ever appends vertices, edges, faces and surfaces and
// repoints existing links. Nothing is ever deleted, so the arrays never need
// compaction and indices stay stable across edits.
Mesh :: struct {
	vertices: #soa[dynamic]Mesh_Vertex,
	edges:    [dynamic]Half_Edge,
	faces:    [dynamic]Face,
	surfaces: [dynamic]Surface,
}

// --- Lifecycle & Destruction ---

destroy_mesh :: proc(mesh: ^Mesh, allocator := context.allocator) {
	if mesh == nil do return
	delete(mesh.vertices)
	delete(mesh.edges)
	delete(mesh.faces)
	delete(mesh.surfaces)
	mesh^ = {}
}

destroy_indexed_mesh :: proc(mesh: ^Indexed_Mesh, allocator := context.allocator) {
	if mesh == nil do return
	delete(mesh.vertices, allocator)
	delete(mesh.indices, allocator)
	mesh^ = {}
}

// --- Low-level builders ---------------------------------------------------
//
// These are the only places that grow the topology arrays. Because the arrays
// are dynamic they remember the allocator they were created with, so the edit
// operations don't need to thread an allocator through.

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

// --- Topology Queries -----------------------------------------------------

// face_loop_edges returns the half-edge indices that bound a face, in CCW loop
// order starting at face.half_edge. The caller owns the result.
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
// Vertex k is the one half-edge k of the loop points TO. The caller owns it.
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

// --- Twin resolution & refresh -------------------------------------------

// rebuild_twins recomputes every half-edge twin from scratch by matching each
// directed edge (start -> end) with one running the opposite direction. It is
// O(n) via a hash map and requires only that .prev and .vertex are correct, so
// edit operations can build clean loops and let this resolve adjacency.
rebuild_twins :: proc(mesh: ^Mesh) {
	dir := make(map[[2]int]int, len(mesh.edges))
	defer delete(dir)

	for i in 0 ..< len(mesh.edges) {
		s := mesh.edges[mesh.edges[i].prev].vertex
		e := mesh.edges[i].vertex
		dir[[2]int{s, e}] = i
	}
	for i in 0 ..< len(mesh.edges) {
		s := mesh.edges[mesh.edges[i].prev].vertex
		e := mesh.edges[i].vertex
		if t, ok := dir[[2]int{e, s}]; ok {
			mesh.edges[i].twin = t
		} else {
			mesh.edges[i].twin = -1
		}
	}
}

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

// --- Primitive Factories: From Box ---

from_box :: proc(size: [3]f64, allocator := context.allocator) -> Mesh {
	mesh: Mesh
	h := size * 0.5

	// 1. Define the 8 corner vertices of a box
	v_positions := [8][3]f64{
		{-h.x, -h.y,  h.z}, { h.x, -h.y,  h.z}, { h.x,  h.y,  h.z}, {-h.x,  h.y,  h.z},
		{-h.x, -h.y, -h.z}, { h.x, -h.y, -h.z}, { h.x,  h.y, -h.z}, {-h.x,  h.y, -h.z},
	}

	mesh.vertices = make(#soa[dynamic]Mesh_Vertex, 0, 8, allocator)
	for p in v_positions {
		append_soa(&mesh.vertices, Mesh_Vertex{position = p})
	}

	// 2. Define the 6 quad faces via vertex index loops (CCW looking from outside)
	face_vertex_indices := [6][4]int{
		{0, 1, 2, 3}, // Front (+Z)
		{1, 5, 6, 2}, // Right (+X)
		{5, 4, 7, 6}, // Back (-Z)
		{4, 0, 3, 7}, // Left (-X)
		{3, 2, 6, 7}, // Top (+Y)
		{4, 5, 1, 0}, // Bottom (-Y)
	}

	// A cube has 6 faces, 6 surfaces, and 24 half-edges (6 faces * 4 edges each)
	mesh.surfaces = make([dynamic]Surface, 6, allocator)
	mesh.faces    = make([dynamic]Face, 6, allocator)
	mesh.edges    = make([dynamic]Half_Edge, 24, allocator)

	edge_idx := 0
	for f_idx in 0 ..< 6 {
		v_idx_loop := face_vertex_indices[f_idx]

		p0 := mesh.vertices[v_idx_loop[0]].position
		p1 := mesh.vertices[v_idx_loop[1]].position
		p2 := mesh.vertices[v_idx_loop[2]].position

		mesh.surfaces[f_idx] = Surface{
			plane      = plane.from_points(p0, p1, p2),
			first_face = f_idx,
			face_count = 1,
		}

		mesh.faces[f_idx] = Face{
			surface   = f_idx,
			half_edge = edge_idx,
		}

		for i in 0 ..< 4 {
			curr := edge_idx + i
			next := edge_idx + ((i + 1) % 4)
			prev := edge_idx + ((i + 3) % 4)

			mesh.edges[curr] = Half_Edge{
				vertex = v_idx_loop[i],
				face   = f_idx,
				twin   = -1,
				next   = next,
				prev   = prev,
			}
		}
		edge_idx += 4
	}

	// 3. Resolve internal Half-Edge Twin mappings
	rebuild_twins(&mesh)

	calculate_normals(&mesh)
	return mesh
}

// --- Triangulation / Queries ----------------------------------------------
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

// --- Translation Operations ----------------------------------------------

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

// --- Legacy surface-indexed wrappers -------------------------------------

translate_surface :: proc(mesh: ^Mesh, surface_index: int, translation: [3]f64) {
	translate_face(mesh, mesh.surfaces[surface_index].first_face, translation)
}

extrude_surface :: proc(mesh: ^Mesh, surface_index: int, vector: [3]f64) {
	extrude_face(mesh, mesh.surfaces[surface_index].first_face, vector)
}

inset_surface :: proc(mesh: ^Mesh, surface_index: int, distance: f64) {
	inset_face(mesh, mesh.surfaces[surface_index].first_face, distance)
}

// --- Unimplemented stubs (not required by the mesh modeler) ---------------

from_sphere :: proc(radius: f64, allocator := context.allocator) -> Mesh { return {} }
from_capsule :: proc(length, radius: f64, allocator := context.allocator) -> Mesh { return {} }
add_surface_from_contours :: proc(mesh: ^Mesh, contours: [][][3]f64, plane_normal: [3]f64, allocator := context.allocator) {}
bevel_surface :: proc(mesh: ^Mesh, surface_index: int, distance: f64) {}
bevel_edge :: proc(mesh: ^Mesh, edge_index: int, distance: f64) {}
calculate_uvs :: proc(mesh: ^Mesh) {}

// --- Structural Analysis Routines ---

calculate_normals :: proc(mesh: ^Mesh) {
	// Reset current vertex normals
	for i in 0 ..< len(mesh.vertices) {
		mesh.vertices[i].normal = {0, 0, 0}
	}

	// Traverse every half-edge to accumulate area-weighted surface plane vectors
	for e_idx in 0 ..< len(mesh.edges) {
		edge := mesh.edges[e_idx]

		// Fetch positions tracking our current local vertex corner loop
		p_curr := mesh.vertices[edge.vertex].position
		p_next := mesh.vertices[mesh.edges[edge.next].vertex].position
		p_prev := mesh.vertices[mesh.edges[edge.prev].vertex].position

		// Compute edge corner cross product vectors
		v1 := p_next - p_curr
		v2 := p_prev - p_curr
		face_normal := linalg.cross(v1, v2)

		mesh.vertices[edge.vertex].normal += face_normal
	}

	// Apply final normalization sweeps over vectors
	for i in 0 ..< len(mesh.vertices) {
		n := mesh.vertices[i].normal
		if linalg.length(n) > 0.00001 {
			mesh.vertices[i].normal = linalg.normalize(n)
		}
	}
}

// --- Compilation Pipeline Pass: Bake ---

bake :: proc(mesh: Mesh, allocator := context.allocator) -> Indexed_Mesh {
	out_vertices := make([dynamic][4]f64, 0, 128, context.temp_allocator)
	out_normals  := make([dynamic][4]f64, 0, 128, context.temp_allocator)
	out_indices  := make([dynamic]u32, 0, 256, context.temp_allocator)

	// Keep track of which vertex data has already been emitted per face to handle flat shading
	Face_Vertex_Pair :: struct { v_idx, f_idx: int }
	v_map := make(map[Face_Vertex_Pair]u32, 64, context.temp_allocator)

	for f_idx in 0 ..< len(mesh.faces) {
		face := mesh.faces[f_idx]

		face_v_indices := make([dynamic]int, 0, 16, context.temp_allocator)
		start_edge := face.half_edge
		curr_edge := start_edge

		for {
			append(&face_v_indices, mesh.edges[curr_edge].vertex)
			curr_edge = mesh.edges[curr_edge].next
			if curr_edge == start_edge do break
		}

		if len(face_v_indices) < 3 do continue

		for i in 1 ..< len(face_v_indices) - 1 {
			tri_v_indices := [3]int{ face_v_indices[0], face_v_indices[i], face_v_indices[i+1] }

			for v_idx in tri_v_indices {
				pair := Face_Vertex_Pair{v_idx, f_idx}

				if baked_idx, exists := v_map[pair]; exists {
					append(&out_indices, baked_idx)
				} else {
					new_idx := u32(len(out_vertices))

					p0 := mesh.vertices[tri_v_indices[0]].position
					p1 := mesh.vertices[tri_v_indices[1]].position
					p2 := mesh.vertices[tri_v_indices[2]].position
					n  := linalg.normalize(linalg.cross(p1 - p0, p2 - p0))

					append(&out_vertices, [4]f64{mesh.vertices[v_idx].position.x, mesh.vertices[v_idx].position.y, mesh.vertices[v_idx].position.z, 1.0})
					append(&out_normals,  [4]f64{n.x, n.y, n.z, 0.0})
					append(&out_indices,  new_idx)

					v_map[pair] = new_idx
				}
			}
		}
	}

	baked_mesh: Indexed_Mesh
	baked_mesh.vertices = make([]types.Mesh_Vertex_f32, len(out_vertices), allocator)
	baked_mesh.indices  = make([]u32, len(out_indices), allocator)

	for i in 0 ..< len(out_vertices) {
		baked_mesh.vertices[i].position = [3]f32{f32(out_vertices[i].x), f32(out_vertices[i].y), f32(out_vertices[i].z)}
		baked_mesh.vertices[i].normal   = [3]f32{f32(out_normals[i].x),  f32(out_normals[i].y),  f32(out_normals[i].z)}
	}
	copy(baked_mesh.indices, out_indices[:])

	return baked_mesh
}
