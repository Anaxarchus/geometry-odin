package mesh

// mesh.odin holds the data model and the "produce / compile" half of the
// package: type definitions, the primitive constructors, normal/UV computation
// and bake(). All traversal queries and topology-editing operations live in
// edit.odin. The two files share one package, so private (`@(private)`,
// package-scoped) helpers are visible across both.

import "core:math/linalg"

import "../plane"
import "../polygon"
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
	vertex: int,    // Vertex it points TO
	face:   int,    // Parent face
	twin:   int,    // Opposing half-edge (-1 if open boundary)
	next:   int,    // CCW next
	prev:   int,    // CCW prev
	uv:     [2]f64, // Texture coordinate at this corner (vertex `vertex` within `face`)
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

// --- Twin resolution ------------------------------------------------------

// rebuild_twins recomputes every half-edge twin from scratch by matching each
// directed edge (start -> end) with one running the opposite direction. It is
// O(n) via a hash map and requires only that .prev and .vertex are correct, so
// constructors and edit operations can build clean loops and let this resolve
// adjacency.
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

// _add_face_loop appends a face (with its dedicated 1:1 surface) built from a
// CCW vertex loop, wiring a closed next/prev half-edge ring. Twins are left
// unresolved; the caller runs rebuild_twins() once all loops are added. If
// `uvs` is non-nil it must run parallel to `loop` and supplies the per-corner
// texture coordinate for each half-edge. Returns the new face index.
@(private)
_add_face_loop :: proc(mesh: ^Mesh, loop: []int, uvs: [][2]f64 = nil) -> int {
	fi   := len(mesh.faces)
	si   := len(mesh.surfaces)
	base := len(mesh.edges)
	m    := len(loop)
	append(&mesh.surfaces, Surface{first_face = fi, face_count = 1})
	append(&mesh.faces, Face{surface = si, half_edge = base})
	for i in 0 ..< m {
		uv: [2]f64
		if uvs != nil do uv = uvs[i]
		append(&mesh.edges, Half_Edge{
			vertex = loop[i],
			face   = fi,
			twin   = -1,
			next   = base + (i + 1) % m,
			prev   = base + (i + m - 1) % m,
			uv     = uv,
		})
	}
	return fi
}

// --- Primitive Factories: From Box ----------------------------------------

// Box_Face names the six quad faces from_box emits, in build order. The enum
// values equal the face indices, so a box face can be edited by name:
//   mesh.extrude_face(&m, mesh.box_face(.Top), {0, 1, 0})
Box_Face :: enum int {
	Front  = 0, // +Z
	Right  = 1, // +X
	Back   = 2, // -Z
	Left   = 3, // -X
	Top    = 4, // +Y
	Bottom = 5, // -Y
}

// box_face returns the face index for a named box face produced by from_box.
box_face :: proc(f: Box_Face) -> int { return int(f) }

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

	// 2. Define the 6 quad faces via vertex index loops (CCW looking from
	//    outside). Order matches Box_Face.
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

// --- Primitive Factories: From Polygon ------------------------------------

// from_polygon3 emits faces in the order [wall_0 .. wall_{n-1}, top, bottom],
// where n == len(polygon3). These helpers map that layout onto face indices so
// a prism can be edited by feature, e.g. mesh.extrude_face(&m, mesh.prism_top(n), v).
// `n` is the contour vertex count (len of the polygon passed to from_polygon3).
prism_wall   :: proc(edge_index: int) -> int { return edge_index }
prism_top    :: proc(n: int) -> int { return n }
prism_bottom :: proc(n: int) -> int { return n + 1 }

// from_polygon3 extrudes a closed 3D polygon contour into a prism Mesh. The
// contour is wound CCW (y-up convention) and `length` is the extrusion distance
// along the contour's normal (as computed by polygon.normal). The result is a
// closed half-edge mesh.
//
// Face layout: [wall_0 .. wall_{n-1}, top, bottom] (see prism_wall/top/bottom).
// Wall i bridges contour edge V[i] -> V[i+1]. The top cap is the contour offset
// by `length` along the normal; the bottom cap is the original contour. Bottom
// vertices occupy indices 0..n-1, top vertices n..2n-1.
//
// UVs are unwrapped analytically (the prism is developable, so this is exact):
// caps use an orthographic projection onto the contour plane, and the side
// walls unroll into one continuous strip where U is arc length around the
// contour and V is the extrusion height. UVs are stored per corner (per
// half-edge) so the cap/wall seam textures cleanly; bake() carries them through.
from_polygon3 :: proc(polygon3: [][3]f64, length: f64, allocator := context.allocator) -> Mesh {
	mesh: Mesh
	n := len(polygon3)
	if n < 3 do return mesh

	pn     := polygon.normal(polygon3)
	offset := pn * length

	// Vertices: 0..n-1 = bottom (original contour), n..2n-1 = top (offset).
	mesh.vertices = make(#soa[dynamic]Mesh_Vertex, 0, n * 2, allocator)
	for p in polygon3 do append_soa(&mesh.vertices, Mesh_Vertex{position = p})
	for p in polygon3 do append_soa(&mesh.vertices, Mesh_Vertex{position = p + offset})

	// Faces emitted as [walls.., top, bottom]. Edges: 4*n (walls) + 2*n (caps).
	face_cap := n + 2
	mesh.surfaces = make([dynamic]Surface,   0, face_cap, allocator)
	mesh.faces    = make([dynamic]Face,      0, face_cap, allocator)
	mesh.edges    = make([dynamic]Half_Edge, 0, n * 6,    allocator)

	// Orthonormal frame in the contour plane for the planar cap projection. The
	// `up` reference is swung off-axis when pn is near +/-Z to keep the cross
	// product well-conditioned.
	up: [3]f64 = {0, 0, 1}
	if abs(pn.z) >= 0.999 do up = {0, 1, 0}
	tan    := linalg.normalize(linalg.cross(up, pn))
	bit    := linalg.cross(pn, tan)
	origin := polygon3[0]

	cap_uv :: proc(p, origin, tan, bit: [3]f64) -> [2]f64 {
		rel := p - origin
		return {linalg.dot(rel, tan), linalg.dot(rel, bit)}
	}

	// Side walls first (faces 0..n-1): face k bridges edge V[k] -> V[k+1], wound
	// CCW seen from outside (bottom[k] -> bottom[k+1] -> top[k+1] -> top[k]). U is
	// the cumulative arc length around the contour, V the extrusion height.
	height := linalg.length(offset)
	u: f64 = 0
	for k in 0 ..< n {
		kn     := (k + 1) % n
		seg    := linalg.length(polygon3[kn] - polygon3[k])
		u_next := u + seg
		quad := [4]int{k, kn, n + kn, n + k}
		quad_uv := [4][2]f64{
			{u,      0},
			{u_next, 0},
			{u_next, height},
			{u,      height},
		}
		_add_face_loop(&mesh, quad[:], quad_uv[:])
		u = u_next
	}

	// Top cap (face n): outward normal +pn, contour order over the offset verts.
	top    := make([]int,    n, context.temp_allocator)
	top_uv := make([][2]f64, n, context.temp_allocator)
	for k in 0 ..< n {
		top[k]    = n + k
		top_uv[k] = cap_uv(polygon3[k] + offset, origin, tan, bit)
	}
	_add_face_loop(&mesh, top, top_uv)

	// Bottom cap (face n+1): outward normal -pn, so the contour is wound reversed.
	bot    := make([]int,    n, context.temp_allocator)
	bot_uv := make([][2]f64, n, context.temp_allocator)
	for k in 0 ..< n {
		bot[k]    = n - 1 - k
		bot_uv[k] = cap_uv(polygon3[n - 1 - k], origin, tan, bit)
	}
	_add_face_loop(&mesh, bot, bot_uv)

	rebuild_twins(&mesh)
	calculate_normals(&mesh)
	return mesh
}

// --- Unimplemented primitive stubs ----------------------------------------

from_sphere :: proc(radius: f64, allocator := context.allocator) -> Mesh { return {} }
from_capsule :: proc(length, radius: f64, allocator := context.allocator) -> Mesh { return {} }

// --- Normals --------------------------------------------------------------

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

// --- UVs ------------------------------------------------------------------

// calculate_uvs assigns per-corner UVs by orthographically projecting every face
// onto its own plane (a tangent frame built from the face normal). This is exact
// for planar faces and gives each face its own UV island; it overwrites any UVs
// set by a primitive factory, so prefer the factory's own unwrap (e.g.
// from_polygon3) when a continuous layout matters.
calculate_uvs :: proc(mesh: ^Mesh) {
	for fi in 0 ..< len(mesh.faces) {
		nrm := _face_plane(mesh, fi).xyz
		up: [3]f64 = {0, 0, 1}
		if abs(nrm.z) >= 0.999 do up = {0, 1, 0}
		tan := linalg.normalize(linalg.cross(up, nrm))
		bit := linalg.cross(nrm, tan)

		start  := mesh.faces[fi].half_edge
		origin := mesh.vertices[mesh.edges[start].vertex].position
		e := start
		for {
			rel := mesh.vertices[mesh.edges[e].vertex].position - origin
			mesh.edges[e].uv = {linalg.dot(rel, tan), linalg.dot(rel, bit)}
			e = mesh.edges[e].next
			if e == start do break
		}
	}
}

// _triangle_tangent derives a tangent vector for a triangle from its UV gradient
// (the standard Lengyel construction). The returned xyz is orthonormalized
// against `n`; w is the handedness sign for reconstructing the bitangent. Falls
// back to an arbitrary axis orthogonal to `n` when the UVs are degenerate.
@(private)
_triangle_tangent :: proc(p0, p1, p2: [3]f64, uv0, uv1, uv2: [2]f64, n: [3]f64) -> [4]f64 {
	e1  := p1 - p0
	e2  := p2 - p0
	du1 := uv1 - uv0
	du2 := uv2 - uv0
	det := du1.x * du2.y - du2.x * du1.y

	if abs(det) < 1e-12 {
		ref: [3]f64 = {1, 0, 0}
		if abs(n.x) > 0.9 do ref = {0, 1, 0}
		t := linalg.normalize(linalg.cross(ref, n))
		return {t.x, t.y, t.z, 1}
	}

	r := 1.0 / det
	t := (e1 * du2.y - e2 * du1.y) * r
	b := (e2 * du1.x - e1 * du2.x) * r

	// Gram-Schmidt orthogonalize the tangent against the normal.
	t = t - n * linalg.dot(n, t)
	if linalg.length(t) > 1e-12 do t = linalg.normalize(t)
	w: f64 = linalg.dot(linalg.cross(n, t), b) < 0 ? -1 : 1
	return {t.x, t.y, t.z, w}
}

// --- Compilation Pipeline Pass: Bake --------------------------------------

bake :: proc(mesh: Mesh, allocator := context.allocator) -> Indexed_Mesh {
	out_vertices := make([dynamic][4]f64, 0, 128, context.temp_allocator)
	out_normals  := make([dynamic][4]f64, 0, 128, context.temp_allocator)
	out_uvs      := make([dynamic][2]f64, 0, 128, context.temp_allocator)
	out_tangents := make([dynamic][4]f64, 0, 128, context.temp_allocator)
	out_indices  := make([dynamic]u32, 0, 256, context.temp_allocator)

	// Keep track of which vertex data has already been emitted per face to handle
	// flat shading and per-corner UV seams.
	Face_Vertex_Pair :: struct { v_idx, f_idx: int }
	v_map := make(map[Face_Vertex_Pair]u32, 64, context.temp_allocator)

	for f_idx in 0 ..< len(mesh.faces) {
		face := mesh.faces[f_idx]

		// Walk the loop, recording both the vertex and the half-edge for each
		// corner (the half-edge carries that corner's UV).
		face_v_indices := make([dynamic]int, 0, 16, context.temp_allocator)
		face_e_indices := make([dynamic]int, 0, 16, context.temp_allocator)
		start_edge := face.half_edge
		curr_edge := start_edge

		for {
			append(&face_v_indices, mesh.edges[curr_edge].vertex)
			append(&face_e_indices, curr_edge)
			curr_edge = mesh.edges[curr_edge].next
			if curr_edge == start_edge do break
		}

		if len(face_v_indices) < 3 do continue

		for i in 1 ..< len(face_v_indices) - 1 {
			tri_v := [3]int{ face_v_indices[0], face_v_indices[i], face_v_indices[i+1] }
			tri_e := [3]int{ face_e_indices[0], face_e_indices[i], face_e_indices[i+1] }

			p0 := mesh.vertices[tri_v[0]].position
			p1 := mesh.vertices[tri_v[1]].position
			p2 := mesh.vertices[tri_v[2]].position
			n  := linalg.normalize(linalg.cross(p1 - p0, p2 - p0))

			uv0 := mesh.edges[tri_e[0]].uv
			uv1 := mesh.edges[tri_e[1]].uv
			uv2 := mesh.edges[tri_e[2]].uv
			tangent := _triangle_tangent(p0, p1, p2, uv0, uv1, uv2, n)

			for k in 0 ..< 3 {
				v_idx := tri_v[k]
				e_idx := tri_e[k]
				pair := Face_Vertex_Pair{v_idx, f_idx}

				if baked_idx, exists := v_map[pair]; exists {
					append(&out_indices, baked_idx)
				} else {
					new_idx := u32(len(out_vertices))
					pos := mesh.vertices[v_idx].position
					uv  := mesh.edges[e_idx].uv

					append(&out_vertices, [4]f64{pos.x, pos.y, pos.z, 1.0})
					append(&out_normals,  [4]f64{n.x, n.y, n.z, 0.0})
					append(&out_uvs,      uv)
					append(&out_tangents, tangent)
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
		baked_mesh.vertices[i].uv       = [2]f32{f32(out_uvs[i].x), f32(out_uvs[i].y)}
		baked_mesh.vertices[i].tangent  = [4]f32{f32(out_tangents[i].x), f32(out_tangents[i].y), f32(out_tangents[i].z), f32(out_tangents[i].w)}
	}
	copy(baked_mesh.indices, out_indices[:])

	return baked_mesh
}
