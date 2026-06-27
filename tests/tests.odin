package tests

import "core:math"
import "core:math/linalg"
import "core:testing"
import "core:log"

import "../triangle"

@(test)
test_triangles :: proc(t: ^testing.T) {
	all_passed := true

	log.info("--- Starting Triangle Package Validation Suite ---")

	// 1. Test Centroid & Bounds (Heterogeneous Numeric verification)
	{
		triangle_int := [3][2]int{{0, 0}, {6, 0}, {0, 6}}
		
		centroid := triangle.triangle_centroid(triangle_int)
		if centroid != {2, 2} {
			log.info("[FAIL] Centroid calculation wrong. Got: %v, Expected: [2, 2]\n", centroid)
			all_passed = false
		}

		bounds := triangle.triangle_bounds(triangle_int)
		if bounds[0] != {0, 0} || bounds[1] != {6, 6} {
			log.info("[FAIL] Bounds extraction failed. Got: %v\n", bounds)
			all_passed = false
		}
	}

	// 2. Test Perimeter (Float vs Overloaded Int paths)
	{
		t_int := [3][2]int{{0, 0}, {3, 0}, {0, 4}} // Classic 3-4-5 Triangle
		perim_int := triangle.triangle_perimeter(t_int)
		if perim_int != 12.0 {
			log.info("[FAIL] Integer perimeter overload failed. Got: %f, Expected: 12.0\n", perim_int)
			all_passed = false
		}

		t_float := [3][2]f32{{0.0, 0.0}, {3.0, 0.0}, {0.0, 4.0}}
		perim_float := triangle.triangle_perimeter(t_float)
		if perim_float != 12.0 {
			log.info("[FAIL] Float perimeter overload failed. Got: %f\n", perim_float)
			all_passed = false
		}
	}

	// 3. Test Area Verification (Heron N-Dimensional Validation)
	{
		t_int := [3][2]int{{0, 0}, {4, 0}, {0, 4}} // Right triangle, area should be (4*4)/2 = 8
		area := triangle.triangle_area(t_int)
		if math.abs(area - 8.0) > 0.0001 {
			log.info("[FAIL] Area calculation returned incorrect values. Got: %f, Expected: 8.0\n", area)
			all_passed = false
		}
	}

	// 4. Test 3D Normal Vector 
	{
		t_3d := [3][3]f32{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}} // Lying on flat XY Plane
		normal := triangle.triangle_normal(t_3d)
		expected := [3]f32{0, 0, 1} // Normal points completely Up into Z axis
		if linalg.distance(normal, expected) > 0.0001 {
			log.info("[FAIL] 3D Face Normal vector incorrect. Got: %v\n", normal)
			all_passed = false
		}
	}

	// 5. Test Heterogeneous Barycentric Tracking
	{
		// Scenario A: Identical Types (Float Triangle, Float Point)
		t_float := [3][2]f32{{0, 0}, {10, 0}, {0, 10}}
		p_float := [2]f32{2.5, 2.5}
		bary_aa := triangle.triangle_barycentric(t_float, p_float)
		
		// Scenario B: Heterogeneous Types (Int Triangle, Float Point)
		t_int   := [3][2]int{{0, 0}, {10, 0}, {0, 10}}
		bary_ba := triangle.triangle_barycentric(t_int, p_float)

		// The center coordinate of this configuration must evaluate to exactly {0.5, 0.25, 0.25}
		expected_bary := [3]f32{0.5, 0.25, 0.25}

		if linalg.distance(bary_aa, expected_bary) > 0.0001 {
			log.info("[FAIL] Homogeneous Barycentric coordinates invalid. Got: %v\n", bary_aa)
			all_passed = false
		}
		if linalg.distance(bary_ba, expected_bary) > 0.0001 {
			log.info("[FAIL] Heterogeneous Barycentric branch transformation failed. Got: %v\n", bary_ba)
			all_passed = false
		}
	}

	if all_passed {
		log.info("[SUCCESS] All geometric triangle procedures validated cleanly!")
	} else {
		log.info("[FAILURE] One or more test segments did not pass specifications.")
	}
}