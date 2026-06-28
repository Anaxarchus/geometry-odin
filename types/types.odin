package types

import "core:math/linalg"

Vector2f32 :: linalg.Vector2f32
Vector2f64 :: linalg.Vector2f64
Vector2int :: [2]int
Vector2i32 :: [2]i32
Vector2i64 :: [2]i64

Vector3f32 :: linalg.Vector3f32
Vector3f64 :: linalg.Vector3f64
Vector3int :: [3]int
Vector3i32 :: [3]i32
Vector3i64 :: [3]i64

Vector4f32 :: linalg.Vector4f32
Vector4f64 :: linalg.Vector4f64

Aabb_f32 :: [2]Vector3f32 // min, max
Aabb_f64 :: [2]Vector3f64
Aabb_int :: [2]Vector3int
Aabb_i32 :: [2]Vector3i32
Aabb_i64 :: [2]Vector3i64

Rect_f32 :: [2]Vector2f32 // min, max
Rect_f64 :: [2]Vector2f64
Rect_int :: [2]Vector2int
Rect_i32 :: [2]Vector2i32
Rect_i64 :: [2]Vector2i64

Ray2_f32 :: distinct [2]Vector2f32 // origin, direction
Ray2_f64 :: distinct [2]Vector2f64

Ray3_f32 :: distinct [2]Vector3f32 // origin, direction
Ray3_f64 :: distinct [2]Vector3f64

Plane_f32 :: Vector4f32 // x, y, z, d
Plane_f64 :: Vector4f64

Triangle2_f32 :: [3]Vector2f32 // p0, p1, p2
Triangle2_f64 :: [3]Vector2f64

Triangle3_f32 :: [3]Vector3f32 // p0, p1, p2
Triangle3_f64 :: [3]Vector3f64

Quad2_f32 :: [4]Vector2f32 // p0, p1, p2, p3
Quad2_f64 :: [4]Vector2f64

Quad3_f32 :: [4]Vector3f32 // p0, p1, p2, p3
Quad3_f64 :: [4]Vector3f64

Sphere_f32 :: distinct Vector4f32 // {x, y, z, radius}
Sphere_f64 :: distinct Vector4f64

Circle2_f32 :: struct {
    origin: Vector2f32,
    radius: f32,
}
Circle2_f64 :: struct {
    origin: Vector2f64,
    radius: f64,
}

Circle3_f32 :: struct {
    origin: Vector3f32,
    normal: Vector3f32,
    radius: f32,
}

Circle3_f64 :: struct {
    origin: Vector3f64,
    normal: Vector3f64,
    radius: f64,
}

Arc2_f32 :: struct {
    origin: Vector2f32,
    radius: f32,
    angle_start: f32,
    angle_end: f32,
}

Arc2_f64 :: struct {
    origin: Vector2f64,
    radius: f64,
    angle_start: f64,
    angle_end: f64,
}

Box_f32 :: struct {
	origin:       Vector3f32,
	half_extents: Vector3f32,
	rotation:     quaternion128,  // w >= 0
}
Box_f64 :: struct {
	origin:       Vector3f64,
	half_extents: Vector3f64,
	rotation:     quaternion256,
}

Polygon2_f32 :: []Vector2f32
Polygon2_f64 :: []Vector2f64

Polygon3_f32 :: []Vector3f32
Polygon3_f64 :: []Vector3f64

Prism_f32 :: struct {
    polygon: Polygon3_f32,
    length: f32,
}

Prism_f64 :: struct {
    polygon: Polygon3_f64,
    length: f64,
}

Mesh_Vertex_f32 :: struct {
    position: Vector3f32,
    normal: Vector3f32,
    tangent: Vector4f32,
    uv: Vector2f32,
}

Mesh_Vertex_f64 :: struct {
    position: Vector3f64,
    normal: Vector3f64,
    tangent: Vector4f64,
    uv: Vector2f64,
}

Mesh_f32 :: #soa []Mesh_Vertex_f32
Mesh_f64 :: #soa []Mesh_Vertex_f64

Indexed_Mesh_f32 :: struct {
    vertices: []Mesh_Vertex_f32,
    indices:  []u32,
}
