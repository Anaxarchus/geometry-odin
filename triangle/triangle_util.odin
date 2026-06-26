package triangle

import "base:intrinsics"
import "core:math"
import "core:math/linalg"


@(private)
_triangle_min_max :: #force_inline proc "contextless" (
	t: [3][$N]$T,
) -> [2][N]T where intrinsics.type_is_numeric(T) {
	return {linalg.min(t[0], linalg.min(t[1], t[2])), linalg.max(t[0], linalg.max(t[1], t[2]))}
}