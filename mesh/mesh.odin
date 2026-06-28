package mesh

import "core:math/linalg"

import "../plane"
import "../triangle"
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

Mesh :: struct {
	vertices: #soa[]Mesh_Vertex,
	edges:    []Half_Edge,
	faces:    []Face,
	surfaces: []Surface,
}

// --- Lifecycle & Destruction ---

destroy_mesh :: proc(mesh: ^Mesh, allocator := context.allocator) {
	if mesh == nil do return
	// Because vertices is #soa, delete operates on the internal structural arrays cleanly
	delete(mesh.vertices, allocator)
	delete(mesh.edges, allocator)
	delete(mesh.faces, allocator)
	delete(mesh.surfaces, allocator)
	mesh^ = {}
}

destroy_indexed_mesh :: proc(mesh: ^Indexed_Mesh, allocator := context.allocator) {
	if mesh == nil do return
	delete(mesh.vertices, allocator)
	delete(mesh.indices, allocator)
	mesh^ = {}
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

	// Allocate space for the 8 structural vertices
	mesh.vertices = make(#soa[]Mesh_Vertex, 8, allocator)
	for p, i in v_positions {
		mesh.vertices[i].position = p
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

	// Pre-allocate the arrays for our BRep structure
	// A cube has 6 faces, 6 surfaces, and 24 half-edges (6 faces * 4 edges each)
	mesh.surfaces = make([]Surface, 6, allocator)
	mesh.faces    = make([]Face, 6, allocator)
	mesh.edges    = make([]Half_Edge, 24, allocator)

	edge_idx := 0
	for f_idx in 0..<6 {
		v_idx_loop := face_vertex_indices[f_idx]
		
		// Setup the surface plane equation using three points on the face boundary loop
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

		// Build the 4 cyclical internal half-edges for this quad face loop
		for i in 0..<4 {
			curr := edge_idx + i
			next := edge_idx + ((i + 1) % 4)
			prev := edge_idx + ((i + 3) % 4)

			mesh.edges[curr] = Half_Edge{
				vertex    = v_idx_loop[i],
				face      = f_idx,
				twin      = -1, // We will pair twins on a follow-up compilation pass
				next      = next,
				prev      = prev,
			}
		}
		edge_idx += 4
	}

	// 3. Resolve internal Half-Edge Twin mappings
	for i in 0..<len(mesh.edges) {
		if mesh.edges[i].twin != -1 do continue // Already mapped
		
		// Find the starting vertex of edge 'i' by looking at its cyclical previous edge target
		v_start_i := mesh.edges[mesh.edges[i].prev].vertex
		v_end_i   := mesh.edges[i].vertex

		// Match it with an edge 'j' running in the inverse direction
		for j in (i + 1)..<len(mesh.edges) {
			v_start_j := mesh.edges[mesh.edges[j].prev].vertex
			v_end_j   := mesh.edges[j].vertex

			if v_start_i == v_end_j && v_end_i == v_start_j {
				mesh.edges[i].twin = j
				mesh.edges[j].twin = i
				break
			}
		}
	}

	calculate_normals(&mesh)
	return mesh
}

// Temporary stubs to maintain compilation compliance
from_sphere :: proc(radius: f64, allocator := context.allocator) -> Mesh { return {} }
from_capsule :: proc(length, radius: f64, allocator := context.allocator) -> Mesh { return {} }
add_surface_from_contours :: proc(mesh: ^Mesh, contours: [][][3]f64, plane_normal: [3]f64, allocator := context.allocator) {}
extrude_surface :: proc(mesh: ^Mesh, surface_index: int, vector: [3]f64) {}
translate_surface :: proc(mesh: ^Mesh, surface_index: int, translation: [3]f64) {}
inset_surface :: proc(mesh: ^Mesh, surface_index: int, distance: f64) {}
bevel_surface :: proc(mesh: ^Mesh, surface_index: int, distance: f64) {}
split_faces_at_centroid :: proc(mesh: ^Mesh, surface_index: int) {}
translate_edge :: proc(mesh: ^Mesh, edge_index: int, translation: [3]f64) {}
subdivide_edge :: proc(mesh: ^Mesh, edge_index: int) -> int { return -1 }
bevel_edge :: proc(mesh: ^Mesh, edge_index: int, distance: f64) {}
calculate_uvs :: proc(mesh: ^Mesh) {}

// --- Structural Analysis Routines ---

calculate_normals :: proc(mesh: ^Mesh) {
	// Reset current vertex normals
	for i in 0..<len(mesh.vertices) {
		mesh.vertices[i].normal = {0, 0, 0}
	}

	// Traverse every half-edge to accumulate area-weighted surface plane vectors
	for e_idx in 0..<len(mesh.edges) {
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
	for i in 0..<len(mesh.vertices) {
		n := mesh.vertices[i].normal
		if linalg.length(n) > 0.00001 {
			mesh.vertices[i].normal = linalg.normalize(n)
		}
	}
}

// --- Compilation Pipeline Pass: Bake ---

// --- Compilation Pipeline Pass: Bake ---

// --- Compilation Pipeline Pass: Bake ---

bake :: proc(mesh: Mesh, allocator := context.allocator) -> Indexed_Mesh {
	out_vertices := make([dynamic][4]f64, 0, 128, context.temp_allocator)
	out_normals  := make([dynamic][4]f64, 0, 128, context.temp_allocator)
	out_indices  := make([dynamic]u32, 0, 256, context.temp_allocator)

	// Keep track of which vertex data has already been emitted per face to handle flat shading
	Face_Vertex_Pair :: struct { v_idx, f_idx: int }
	v_map := make(map[Face_Vertex_Pair]u32, 64, context.temp_allocator)

	for f_idx in 0..<len(mesh.faces) {
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

		for i in 1..<len(face_v_indices) - 1 {
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

	for i in 0..<len(out_vertices) {
		baked_mesh.vertices[i].position = [3]f32{f32(out_vertices[i].x), f32(out_vertices[i].y), f32(out_vertices[i].z)}
		baked_mesh.vertices[i].normal   = [3]f32{f32(out_normals[i].x),  f32(out_normals[i].y),  f32(out_normals[i].z)}
	}
	copy(baked_mesh.indices, out_indices[:])

	return baked_mesh
}