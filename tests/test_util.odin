package tests

import "base:intrinsics"

// Shared approximate-comparison helpers for the geometry test suite.

EPS :: 1e-4

// scalar float comparison within EPS
feq :: proc(a, b: $T) -> bool where intrinsics.type_is_float(T) {
	return abs(a - b) <= T(EPS)
}

// element-wise vector comparison within EPS (any length, any float type)
veq :: proc(a, b: [$N]$T) -> bool where intrinsics.type_is_float(T) {
	for i in 0 ..< N {
		if abs(a[i] - b[i]) > T(EPS) {
			return false
		}
	}
	return true
}
