package tests

import "core:math"
import "core:testing"
import "../intersect"

@(test)
test_ray_at :: proc(t: ^testing.T) {
	testing.expect(
		t,
		veq(intersect.ray_at([3]f32{0, 0, 0}, [3]f32{1, 0, 0}, 5), [3]f32{5, 0, 0}),
		"ray_at",
	)
}

@(test)
test_ray_plane :: proc(t: ^testing.T) {
	hit, dist := intersect.ray_plane([3]f32{0, 0, 5}, [3]f32{0, 0, -1}, [3]f32{0, 0, 1}, 0)
	testing.expect(t, hit && feq(dist, 5), "ray_plane hit")

	// parallel ray never meets the plane
	miss, _ := intersect.ray_plane([3]f32{0, 0, 5}, [3]f32{1, 0, 0}, [3]f32{0, 0, 1}, 0)
	testing.expect(t, !miss, "ray_plane parallel")

	// plane behind the origin
	behind, _ := intersect.ray_plane([3]f32{0, 0, 5}, [3]f32{0, 0, 1}, [3]f32{0, 0, 1}, 0)
	testing.expect(t, !behind, "ray_plane behind")
}

@(test)
test_ray_aabb :: proc(t: ^testing.T) {
	mn := [3]f32{-1, -1, -1}
	mx := [3]f32{1, 1, 1}

	hit, dist := intersect.ray_aabb([3]f32{-5, 0, 0}, [3]f32{1, 0, 0}, mn, mx)
	testing.expect(t, hit && feq(dist, 4), "ray_aabb hit")

	// passes beside the box in y
	miss, _ := intersect.ray_aabb([3]f32{-5, 5, 0}, [3]f32{1, 0, 0}, mn, mx)
	testing.expect(t, !miss, "ray_aabb miss")
}

@(test)
test_ray_triangle :: proc(t: ^testing.T) {
	v0 := [3]f32{0, 0, 0}
	v1 := [3]f32{1, 0, 0}
	v2 := [3]f32{0, 1, 0}

	hit, dist := intersect.ray_triangle([3]f32{0.25, 0.25, -1}, [3]f32{0, 0, 1}, v0, v1, v2, 100)
	testing.expect(t, hit && feq(dist, 1), "ray_triangle hit")

	miss, _ := intersect.ray_triangle([3]f32{2, 2, -1}, [3]f32{0, 0, 1}, v0, v1, v2, 100)
	testing.expect(t, !miss, "ray_triangle miss")
}

@(test)
test_ray_sphere :: proc(t: ^testing.T) {
	c := [3]f32{0, 0, 0}

	hit, dist := intersect.ray_sphere([3]f32{-5, 0, 0}, [3]f32{1, 0, 0}, c, 1)
	testing.expect(t, hit && feq(dist, 4), "ray_sphere hit (entry)")

	miss, _ := intersect.ray_sphere([3]f32{-5, 5, 0}, [3]f32{1, 0, 0}, c, 1)
	testing.expect(t, !miss, "ray_sphere miss")

	// origin inside returns the exit distance
	inside, dist_in := intersect.ray_sphere([3]f32{0, 0, 0}, [3]f32{1, 0, 0}, c, 1)
	testing.expect(t, inside && feq(dist_in, 1), "ray_sphere inside (exit)")
}

@(test)
test_overlap_aabb :: proc(t: ^testing.T) {
	testing.expect(
		t,
		intersect.overlap_aabb_aabb(
			[3]f32{0, 0, 0},
			[3]f32{2, 2, 2},
			[3]f32{1, 1, 1},
			[3]f32{3, 3, 3},
		),
		"overlap true",
	)
	testing.expect(
		t,
		!intersect.overlap_aabb_aabb(
			[3]f32{0, 0, 0},
			[3]f32{1, 1, 1},
			[3]f32{2, 2, 2},
			[3]f32{3, 3, 3},
		),
		"overlap false",
	)
}

@(test)
test_overlap_sphere_aabb :: proc(t: ^testing.T) {
	mn := [3]f32{-1, -1, -1}
	mx := [3]f32{1, 1, 1}
	testing.expect(t, !intersect.overlap_sphere_aabb([3]f32{3, 0, 0}, 1.5, mn, mx), "too far")
	testing.expect(t, intersect.overlap_sphere_aabb([3]f32{3, 0, 0}, 2.5, mn, mx), "reaches")
	testing.expect(t, intersect.overlap_sphere_aabb([3]f32{0, 0, 0}, 0.5, mn, mx), "inside")
}

@(test)
test_overlap_obb_aabb :: proc(t: ^testing.T) {
	identity := [3][3]f32{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	mn := [3]f32{-1, -1, -1}
	mx := [3]f32{1, 1, 1}

	testing.expect(
		t,
		intersect.overlap_obb_aabb_axes([3]f32{0, 0, 0}, [3]f32{1, 1, 1}, identity, mn, mx),
		"identity overlap",
	)
	testing.expect(
		t,
		!intersect.overlap_obb_aabb_axes([3]f32{5, 0, 0}, [3]f32{1, 1, 1}, identity, mn, mx),
		"identity separated",
	)

	// 45-degree rotation about z
	c := math.cos(f32(math.PI / 4))
	s := math.sin(f32(math.PI / 4))
	rot := [3][3]f32{{c, s, 0}, {-s, c, 0}, {0, 0, 1}}

	testing.expect(
		t,
		intersect.overlap_obb_aabb_axes([3]f32{0, 0, 0}, [3]f32{1, 1, 1}, rot, mn, mx),
		"rotated overlap",
	)
	testing.expect(
		t,
		!intersect.overlap_obb_aabb_axes(
			[3]f32{0, 0, 0},
			[3]f32{1, 1, 1},
			rot,
			[3]f32{10, 10, 10},
			[3]f32{11, 11, 11},
		),
		"rotated separated",
	)
}
