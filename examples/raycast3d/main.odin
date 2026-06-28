package main

// A triangle soup (a few cubes) is loaded into a BVH. A ray sweeps across the
// scene each frame; bvh.ray_query finds the nearest triangle hit and the hit
// point is marked with a sphere. Demonstrates algo/bvh + intersect.ray_at.

import rl "vendor:raylib"
import "core:math"
import "core:math/linalg"
import "../../algo/bvh"
import "../../intersect"

WIDTH :: 1000
HEIGHT :: 720

// appends the 12 triangles of an axis-aligned cube to `tris`
add_cube :: proc(tris: ^[dynamic]bvh.Primitive(f32), center: [3]f32, size: f32, id: int) {
	h := size * 0.5
	c := [8][3]f32 {
		center + {-h, -h, -h},
		center + {h, -h, -h},
		center + {h, h, -h},
		center + {-h, h, -h},
		center + {-h, -h, h},
		center + {h, -h, h},
		center + {h, h, h},
		center + {-h, h, h},
	}
	faces := [6][4]int {
		{0, 1, 2, 3},
		{5, 4, 7, 6},
		{4, 0, 3, 7},
		{1, 5, 6, 2},
		{4, 5, 1, 0},
		{3, 2, 6, 7},
	}
	for f in faces {
		append(tris, bvh.make_triangle_primitive(c[f[0]], c[f[1]], c[f[2]], id))
		append(tris, bvh.make_triangle_primitive(c[f[0]], c[f[2]], c[f[3]], id))
	}
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: 3D BVH raycast")
	rl.SetTargetFPS(60)

	tris: [dynamic]bvh.Primitive(f32)
	add_cube(&tris, {0, 1, 0}, 2, 0)
	add_cube(&tris, {3.5, 1.5, -1}, 3, 1)
	add_cube(&tris, {-3, 0.75, 2}, 1.5, 2)
	add_cube(&tris, {-1, 2.5, -3.5}, 1.5, 3)

	b := bvh.make_bvh(tris[:], 4)
	delete(tris) // make_bvh keeps its own copy

	camera := rl.Camera3D {
		position   = {12, 9, 12},
		target     = {0, 1, 0},
		up         = {0, 1, 0},
		fovy       = 45,
		projection = .PERSPECTIVE,
	}

	for !rl.WindowShouldClose() {
		//rl.UpdateCamera(&camera, .ORBITAL)

		t := f32(rl.GetTime())
		origin := [3]f32{9, 7, 9}
		target := [3]f32{math.cos(t) * 4, 1, math.sin(t * 0.7) * 4}
		dir := linalg.normalize(target - origin)

		hit, index, _, dist := bvh.ray_query(b, origin, dir)
		ray_end := origin + dir * 30
		if hit {
			ray_end = intersect.ray_at(origin, dir, dist)
		}

		rl.BeginDrawing()
		rl.ClearBackground({22, 24, 30, 255})

		rl.BeginMode3D(camera)
		rl.DrawGrid(20, 1)

		// triangle wireframe (bright, to stand out from the gray grid)
		wire := rl.SKYBLUE
		for p in b.primitives {
			rl.DrawLine3D(rl.Vector3(p.v0), rl.Vector3(p.v1), wire)
			rl.DrawLine3D(rl.Vector3(p.v1), rl.Vector3(p.v2), wire)
			rl.DrawLine3D(rl.Vector3(p.v2), rl.Vector3(p.v0), wire)
		}

		// the ray
		rl.DrawLine3D(rl.Vector3(origin), rl.Vector3(ray_end), rl.GREEN)

		if hit {
			p := b.primitives[index]
			v0 := rl.Vector3(p.v0)
			v1 := rl.Vector3(p.v1)
			v2 := rl.Vector3(p.v2)
			face := rl.Fade(rl.RED, 0.6)
			rl.DrawTriangle3D(v0, v1, v2, face)
			rl.DrawTriangle3D(v0, v2, v1, face)
			rl.DrawSphere(rl.Vector3(ray_end), 0.2, rl.RED)
		}

		rl.EndMode3D()

		rl.DrawText("BVH ray query: green ray, red = nearest hit", 12, 12, 20, rl.RAYWHITE)
		rl.DrawText("ray: HIT" if hit else "ray: miss", 12, 40, 18, rl.LIGHTGRAY)
		rl.EndDrawing()
	}

	bvh.delete_bvh(b)
	rl.CloseWindow()
}
