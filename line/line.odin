package line

import "base:intrinsics"
import "core:math"
import "core:math/linalg"

@(private)
is_scalar :: intrinsics.type_is_ordered_numeric
@(private)
is_float :: intrinsics.type_is_float
@(private)
is_int :: intrinsics.type_is_integer

// casts a line to another scalar type
cast_to :: #force_inline proc "contextless" (l: [2][$N]$T, $to: typeid) -> [2][N]to where is_scalar(T) && is_scalar(to) {
    return [2][N]to{
        linalg.array_cast(l[0], to),
        linalg.array_cast(l[1], to)
    }
}

// returns the direction normal of the line by normalizing the difference from end to start.
direction_normal_float :: #force_inline proc "contextless" (l: [2][$N]$T) -> [N]T where is_float(T) {
    return linalg.normalize0(l[1] - l[0])
}

// returns the f64 direction normal of the line, by first casting the line to f64, and then normalizing the difference from end to start.
direction_normal_int :: #force_inline proc "contextless" (l: [2][$N]$T) -> [N]f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    return linalg.normalize0(lf64[1] - lf64[0])
}

// takes an integer or floating point line and returns the normalized difference from end to start.
direction_normal :: proc {direction_normal_float, direction_normal_int}

// returns the normal of the line by computing the orthogonal of it's direction normal.
normal_float :: #force_inline proc "contextless" (l: [2][2]$T) -> [2]T where is_float(T) {
    return linalg.orthogonal(direction_normal(l))
}

// returns the normal of the line by computing the orthogonal of it's direction normal.
normal_int :: #force_inline proc "contextless" (l: [2][2]$T) -> [2]f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    return linalg.orthogonal(direction_normal(lf64))
}

// returns the normal of the line by computing the orthogonal of it's direction normal.
normal :: proc {normal_float, normal_int}

// returns the center of the line by computing the average of it's two end points.
center :: #force_inline proc "contextless" (l: [2][$N]$T) -> [N]T where is_scalar(T) {
    return l[0] + (l[1] - l[0]) / 2
}

// returns the point on a line linearly interpolated by factor `t` (0.0 = start, 1.0 = end).
point_at :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    t: T,
) -> [N]T where is_scalar(T) {
    return l[0] + (l[1] - l[0]) * t
}

// Projects a point `p` onto the infinite line defined by segment `l` and returns 
// the normalized interpolation parameter `t`.
//
// - Returns 0.0 if `p` projects exactly onto the start point `l[0]`.
// - Returns 1.0 if `p` projects exactly onto the end point `l[1]`.
// - Can return values outside [0, 1] if `p` projects beyond the segment endpoints.
// - Returns 0.0 if the line segment has zero length (degenerate line).
project_parameter_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> T where is_float(T) {
    d := l[1] - l[0]

    denom := linalg.dot(d, d) // Squared length of the line segment
    if denom == 0 {
        return 0
    }

    return linalg.dot(p - l[0], d) / denom
}

// Projects a point `p` onto the infinite line defined by segment `l` and returns 
// the normalized interpolation parameter `t`.
//
// It first casts the integer line to float
//
// - Returns 0.0 if `p` projects exactly onto the start point `l[0]`.
// - Returns 1.0 if `p` projects exactly onto the end point `l[1]`.
// - Can return values outside [0, 1] if `p` projects beyond the segment endpoints.
// - Returns 0.0 if the line segment has zero length (degenerate line).
project_parameter_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return project_parameter_float(lf64, pf64)
}

// Projects a point `p` onto the infinite line defined by segment `l` and returns 
// the normalized interpolation parameter `t`.
//
// - Returns 0.0 if `p` projects exactly onto the start point `l[0]`.
// - Returns 1.0 if `p` projects exactly onto the end point `l[1]`.
// - Can return values outside [0, 1] if `p` projects beyond the segment endpoints.
// - Returns 0.0 if the line segment has zero length (degenerate line).
project_parameter :: proc {project_parameter_float, project_parameter_int}

// Projects a point `p` orthogonally onto the infinite line defined by `l` 
// and returns the closest coordinate point on that line.
// Works with floating-point types.
project_infinite_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> [N]T where is_float(T) {
    t := project_parameter(l, p)
    return point_at(l, t)
}

// Projects a point `p` orthogonally onto the infinite line defined by `l`.
// Accepts integer inputs and automatically promotes them to `f64` to prevent 
// geometric truncation or precision loss during projection math.
project_infinite_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> [N]f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return project_infinite_float(lf64, pf64)
}

// Explicit overload group for projecting a point onto an infinite line.
// Dynamically resolves to float preservation or f64 integer promotion based on type.
project_infinite :: proc {
	project_infinite_float,
	project_infinite_int,
}

// Projects a point `p` orthogonally onto a bounded line segment `l`
// and returns the closest coordinate point on that segment.
//
// The result is strictly clamped to the segment boundaries: if the projection falls 
// outside the segment, it returns either the start point `l[0]` or the end point `l[1]`.
// Works with floating-point types.
project_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> [N]T where is_float(T) {
    t := project_parameter(l, p)
    t = clamp(t, 0, 1)

    return point_at(l, t)
}

// Projects a point `p` orthogonally onto a bounded line segment `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to prevent 
// geometric truncation, returning the exact fractional coordinate clamped to the segment.
project_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> [N]f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return project_float(lf64, pf64)
}

// Explicit overload group for projecting a point onto a bounded line segment.
// Automatically handles type matching and integer-to-f64 precision promotion.
project :: proc {
	project_float,
	project_int,
}

// Calculates the squared length of a line segment `l`.
//
// This is significantly faster than calculating the true length because it avoids a 
// square root operation. Ideal for distance comparisons (e.g., comparing `length2(a) < length2(b)`).
// Works with floating-point types.
length2_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
) -> T where is_float(T) {
    d := l[1] - l[0]
    return linalg.dot(d, d)
}

// Calculates the squared length of a line segment `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to prevent 
// integer overflow errors during the multiplication/dot-product phase.
length2_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    return length2_float(lf64)
}

// Explicit overload group for calculating the squared length of a line segment.
// Maximizes speed for distance sorting, boundary radius checks, and threshold tests.
length2 :: proc {length2_float, length2_int}

// Calculates the true geometric (Euclidean) length of a line segment `l`.
// Works with floating-point types.
length_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
) -> T where is_float(T) {
    return math.sqrt(length2(l))
}

// Calculates the true geometric (Euclidean) length of a line segment `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to return 
// a precise fractional length vector magnitude.
length_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    return length_float(lf64)
}

// Explicit overload group for calculating the true geometric length of a line segment.
// Returns the exact distance between the segment's endpoints.
length :: proc {length_float, length_int}

// Calculates the squared shortest distance from a point `p` to a bounded line segment `l`.
//
// This is significantly faster than calculating the true distance because it avoids a 
// square root operation. Highly recommended for proximity checks or sorting objects by distance.
// Works with floating-point types.
distance2_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> T where is_float(T) {
    d := p - project(l, p)
    return linalg.dot(d, d)
}

// Calculates the squared shortest distance from a point `p` to a bounded line segment `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to prevent 
// coordinate truncation or integer overflow errors during the distance squaring calculation.
distance2_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return distance2_float(lf64, pf64)
}

// Explicit overload group for calculating the squared distance from a point to a line segment.
// Maximizes efficiency when checking thresholds (e.g., `distance2(l, p) < radius * radius`).
distance2 :: proc {distance2_float, distance2_int}

// Calculates the true geometric (Euclidean) shortest distance from a point `p` to a bounded line segment `l`.
// Works with floating-point types.
distance_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> T where is_float(T) {
    return math.sqrt(distance2(l, p))
}

// Calculates the true geometric (Euclidean) shortest distance from a point `p` to a bounded line segment `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to return 
// a precise fractional distance value.
distance_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return distance_float(lf64, pf64)
}

// Explicit overload group for calculating the true geometric distance from a point to a line segment.
distance :: proc {distance_float, distance_int}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

// returns the (non-normalized) direction vector from l[0] to l[1]
direction :: #force_inline proc "contextless" (l: [2][$N]$T) -> [N]T where is_scalar(T) {
    return l[1] - l[0]
}

// returns the axis-aligned bounding box of the segment as {min, max}
min_max :: #force_inline proc "contextless" (l: [2][$N]$T) -> [2][N]T where is_scalar(T) {
    lo := l[0]
    hi := l[1]
    for i in 0 ..< N {
        if l[1][i] < l[0][i] {
            lo[i] = l[1][i]
            hi[i] = l[0][i]
        }
    }
    return {lo, hi}
}

// Checks if a point `p` lies within a specified `epsilon` distance of the bounded line segment `l`.
//
// This effectively tests if the point lies inside a capsule-shaped tolerance zone centered on 
// the segment. Because it uses the clamped projection under the hood, points near the ends 
// of the segment are evaluated against a spherical radius around the endpoints `l[0]` and `l[1]`.
// Works with floating-point types.
has_point_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
    epsilon: f64 = 1e-6,
) -> bool where is_float(T) {
    d := p - project(l, p)
    e := T(epsilon)
    return linalg.dot(d, d) <= e * e
}

// Checks if a point `p` lies within a specified `epsilon` distance of the bounded line segment `l`.
//
// Accepts integer coordinates for the line and point, automatically promoting them to `f64` 
// to execute the precision threshold check against the floating-point `epsilon`.
has_point_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
    epsilon: f64 = 1e-6,
) -> bool where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return has_point_float(lf64, pf64, epsilon)
}

// Explicit overload group for testing point-on-segment inclusion within an epsilon tolerance.
// Highly useful for UI mouse-clicking/interaction detection on lines, vector snapping, 
// and filtering colinear structural intersections.
has_point :: proc {has_point_float, has_point_int}

// returns the squared perpendicular distance from p to the infinite line
// Calculates the squared shortest perpendicular distance from a point `p` to the infinite line defined by `l`.
//
// Unlike segment distance, this ignores the endpoints and treats the line as stretching infinitely 
// in both directions. This is significantly faster than calculating the true distance because it 
// avoids a square root operation.
// Works with floating-point types.
distance2_infinite_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> T where is_float(T) {
    d := p - project_infinite(l, p)
    return linalg.dot(d, d)
}

// Calculates the squared shortest perpendicular distance from a point `p` to the infinite line defined by `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to prevent 
// coordinate truncation or integer overflow errors during the distance squaring phase.
distance2_infinite_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return distance2_infinite_float(lf64, pf64)
}

// Explicit overload group for calculating the squared perpendicular distance from a point to an infinite line.
// Excellent for high-performance threshold filters (e.g., finding if a point is within a corridor ray).
distance2_infinite :: proc {distance2_infinite_float, distance2_infinite_int}

// Calculates the true geometric (Euclidean) shortest perpendicular distance from a point `p` to the infinite line defined by `l`.
// Works with floating-point types.
distance_infinite_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> T where is_float(T) {
    return math.sqrt(distance2_infinite(l, p))
}

// Calculates the true geometric (Euclidean) shortest perpendicular distance from a point `p` to the infinite line defined by `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to return 
// a precise fractional perpendicular distance value.
distance_infinite_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    p: [N]T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return distance_infinite_float(lf64, pf64)
}

// Explicit overload group for calculating the true geometric perpendicular distance from a point to an infinite line.
distance_infinite :: proc {distance_infinite_float, distance_infinite_int}

// Computes the signed side of a point `p` relative to a directed 2D line segment `l`.
//
// Mathematically calculates the 2D cross product (determinant) of the line vector 
// and the vector pointing from `l[0]` to `p`. 
//
// - Returns a positive value if `p` lies strictly to the left of the directed line `l[0] -> l[1]`.
// - Returns a negative value if `p` lies strictly to the right.
// - Returns 0.0 if `p` is perfectly collinear with the line.
// Works with floating-point types.
side_float :: #force_inline proc "contextless" (
    l: [2][2]$T,
    p: [2]T,
) -> T where is_float(T) {
    d := l[1] - l[0]
    r := p - l[0]
    return d.x * r.y - d.y * r.x
}

// Computes the signed side of a point `p` relative to a directed 2D line segment `l`.
//
// Accepts integer inputs and automatically promotes them to `f64` to calculate 
// the precise orientation value without risk of integer overflow.
side_int :: #force_inline proc "contextless" (
    l: [2][2]$T,
    p: [2]T,
) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    pf64 := linalg.array_cast(p, f64)
    return side_float(lf64, pf64)
}

// Explicit overload group for evaluating point orientation relative to a directed 2D line.
// This is the fundamental primitive for polygon winding numbers, clockwise tests, 
// convex hull generation, and ray-casting intersection filters.
side :: proc {side_float, side_int}

// Calculates the absolute directional angle of a 2D segment `l` in radians.
//
// Uses `math.atan2` under the hood to return an angle ranging from -π to +π.
// An angle of 0.0 aligns perfectly with the positive X-axis, progressing counter-clockwise.
// Works with floating-point types.
angle_float :: #force_inline proc "contextless" (l: [2][2]$T) -> T where is_float(T) {
    d := l[1] - l[0]
    return math.atan2(d.y, d.x)
}

// Calculates the absolute directional angle of a 2D segment `l` in radians.
//
// Accepts integer inputs and promotes them to `f64` to return a continuous, 
// precise floating-point angle orientation.
angle_int :: #force_inline proc "contextless" (l: [2][2]$T) -> f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    return angle_float(lf64)
}

// Explicit overload group for calculating the directional angle of a 2D segment.
// Invaluable for vector steering, turning calculations, and path orientation adjustments.
angle :: proc {angle_float, angle_int}

// ---------------------------------------------------------------------------
// Relationships
// ---------------------------------------------------------------------------

// Checks if two line segments `a` and `b` are approximately equal, respecting endpoint order.
//
// This validates directed equality: `a[0]` must be within `epsilon` of `b[0]`, 
// and `a[1]` must be within `epsilon` of `b[1]`.
// Works with floating-point types.
equal_approx_float :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
    epsilon: f64 = 1e-6,
) -> bool where is_float(T) {
    d0 := a[0] - b[0]
    d1 := a[1] - b[1]
    e := T(epsilon)
    e2 := e * e
    return linalg.dot(d0, d0) <= e2 && linalg.dot(d1, d1) <= e2
}

// Checks if two line segments `a` and `b` are approximately equal, respecting endpoint order.
//
// Accepts integer coordinates and promotes them to `f64` to evaluate proximity 
// against the floating-point `epsilon` threshold.
equal_approx_int :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
    epsilon: f64 = 1e-6,
) -> bool where is_int(T) {
    af := cast_to(a, f64)
    bf := cast_to(b, f64)
    return equal_approx_float(af, bf, epsilon)
}

// Explicit overload group for strict, directed segment equality within an epsilon tolerance.
// Useful when vector heading or winding order (e.g., clockwise vs. counter-clockwise) matters.
equal_approx :: proc {equal_approx_float, equal_approx_int}

// Checks if two line segments `a` and `b` occupy the same spatial position, ignoring endpoint order.
//
// This validates topological edge equality. It returns true if the segments match forward or if 
// segment `b` is reversed, meaning they represent the exact same geometric boundary.
// Works with floating-point types.
equal_approx_unordered_float :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
    epsilon: f64 = 1e-6,
) -> bool where is_float(T) {
    return equal_approx(a, b, epsilon) || equal_approx(a, reverse(b), epsilon)
}

// Checks if two line segments `a` and `b` occupy the same spatial position, ignoring endpoint order.
//
// Accepts integer coordinates and promotes them to `f64` to evaluate proximity 
// against the floating-point `epsilon` threshold.
equal_approx_unordered_int :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
    epsilon: f64 = 1e-6,
) -> bool where is_int(T) {
    af := cast_to(a, f64)
    bf := cast_to(b, f64)
    return equal_approx_unordered_float(af, bf, epsilon)
}

// Explicit overload group for spatial segment equality, ignoring edge direction.
// Essential for deduplicating mesh edges, resolving shared polygon boundaries, and graph structural analysis.
equal_approx_unordered :: proc {equal_approx_unordered_float, equal_approx_unordered_int}

// ---------------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------------

// Reverses the direction of a line segment `l` by swapping its start and end points.
//
// Effectively flips the heading vector and the signed side orientation (`side()`) 
// of the segment without moving it in world space.
reverse :: #force_inline proc "contextless" (l: [2][$N]$T) -> [2][N]T where is_scalar(T) {
    return {l[1], l[0]}
}

// Displaces a line segment `l` uniformly along a spatial vector `offset`.
//
// Translates both endpoints simultaneously, preserving the segment's exact 
// geometric length, heading angle, and orientation.
translate :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    offset: [N]T,
) -> [2][N]T where is_scalar(T) {
    return {l[0] + offset, l[1] + offset}
}

// Scales a line segment `l` uniformly by a scalar `factor`, expanding or contracting 
// symmetrically about its midpoint.
//
// - A factor > 1.0 lengthens the segment.
// - A factor between 0.0 and 1.0 shortens the segment.
// - A negative factor flips the segment endpoints across the center point.
// The segment's geometric center (`center(l)`) remains invariant during this operation.
scale :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    factor: T,
) -> [2][N]T where is_scalar(T) {
    c := center(l)
    return {c + (l[0] - c) * factor, c + (l[1] - c) * factor}
}

Line_Anchor :: enum {
    Begin,
    Center,
    End,
}

// Rescales a line segment `l` to a specified `new_length` along its original direction, 
// anchoring the transformation at the specified pivot point.
//
// - `Line_Anchor.Begin`: The start point `l[0]` stays fixed; the segment extends or shrinks from `l[1]`.
// - `Line_Anchor.Center`: The midpoint remains fixed; the segment scales symmetrically from both ends.
// - `Line_Anchor.End`: The end point `l[1]` stays fixed; the segment extends or shrinks from `l[0]`.
// Works with floating-point types.
set_length_float :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    new_length: T,
    anchor := Line_Anchor.Begin,
) -> [2][N]T where is_float(T) {
    dir := direction_normal(l)
    
    switch anchor {
    case .Begin:
        return {l[0], l[0] + dir * new_length}
    case .End:
        return {l[1] - dir * new_length, l[1]}
    case .Center:
        c := center(l)
        half_ext := dir * (new_length * 0.5)
        return {c - half_ext, c + half_ext}
    }
    return l
}

// Rescales a line segment `l` to a specified `new_length` along its original direction, 
// anchoring the transformation at the specified pivot point.
//
// Accepts integer coordinates and promotes them to `f64` to calculate precise fractional 
// endpoint adjustments.
set_length_int :: #force_inline proc "contextless" (
    l: [2][$N]$T,
    new_length: f64,
    anchor := Line_Anchor.Begin,
) -> [2][N]f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    return set_length_float(lf64, new_length, anchor)
}

// Explicit overload group for modifying a segment's length while locking a specific anchor point.
set_length :: proc {set_length_float, set_length_int}

// Rotates a 2D line segment `l` by a specified `angle` (in radians) about a given anchor point.
//
// Positive angles rotate counter-clockwise, negative angles rotate clockwise.
// - `Line_Anchor.Begin`: Rotates around the start coordinate `l[0]`.
// - `Line_Anchor.Center`: Rotates around the midpoint of the segment.
// - `Line_Anchor.End`: Rotates around the end coordinate `l[1]`.
// Works with floating-point types.
rotate_float :: proc "contextless" (
    l: [2][2]$T,
    angle: T,
    anchor := Line_Anchor.Center,
) -> [2][2]T where is_float(T) {
    pivot: [2]T
    switch anchor {
    case .Begin:  pivot = l[0]
    case .End:    pivot = l[1]
    case .Center: pivot = center(l)
    }

    s := math.sin(angle)
    co := math.cos(angle)
    d0 := l[0] - pivot
    d1 := l[1] - pivot
    return {
        pivot + [2]T{d0.x * co - d0.y * s, d0.x * s + d0.y * co},
        pivot + [2]T{d1.x * co - d1.y * s, d1.x * s + d1.y * co},
    }
}

// Rotates a 2D line segment `l` by a specified `angle` (in radians) about a given anchor point.
//
// Accepts integer inputs and promotes them to `f64` coordinates to accurately preserve 
// geometric circular arcs without integer quantization snapping.
rotate_int :: #force_inline proc "contextless" (
    l: [2][2]$T,
    angle: f64,
    anchor := Line_Anchor.Center,
) -> [2][2]f64 where is_int(T) {
    lf64 := cast_to(l, f64)
    return rotate_float(lf64, angle, anchor)
}

// Explicit overload group for rotating a 2D segment about a fixed anchor point.
rotate :: proc {rotate_float, rotate_int}

// ---------------------------------------------------------------------------
// Closest approach (segment to segment)
// ---------------------------------------------------------------------------

// Computes the closest points `c1` on segment `a` and `c2` on segment `b`.
//
// This executes an optimized 3D/N-Dimensional segment-to-segment proximity test. 
// It effectively finds the shortest bridging vector between two lines in space.
//
// - If the segments intersect, `c1` and `c2` will be identical (the intersection point).
// - If the segments are parallel or overlapping, it returns a valid pair of closest points.
// - Handles degenerate conditions (if either or both segments have zero length) without 
//   crashing or throwing divisions-by-zero.
// Works with floating-point types.
closest_points_float :: proc "contextless" (
    a, b: [2][$N]$T,
) -> (c1: [N]T, c2: [N]T) where is_float(T) {
    d1 := a[1] - a[0]
    d2 := b[1] - b[0]
    r := a[0] - b[0]
    aa := linalg.dot(d1, d1)
    ee := linalg.dot(d2, d2)
    f := linalg.dot(d2, r)

    s, t: T

    if aa <= 1e-12 && ee <= 1e-12 {
        // both segments degenerate to points
        return a[0], b[0]
    }

    if aa <= 1e-12 {
        // segment a is a point
        s = 0
        t = clamp(f / ee, 0, 1)
    } else {
        c := linalg.dot(d1, r)
        if ee <= 1e-12 {
            // segment b is a point
            t = 0
            s = clamp(-c / aa, 0, 1)
        } else {
            bb := linalg.dot(d1, d2)
            denom := aa * ee - bb * bb
            if denom != 0 {
                s = clamp((bb * f - c * ee) / denom, 0, 1)
            } else {
                s = 0
            }
            t = (bb * s + f) / ee
            if t < 0 {
                t = 0
                s = clamp(-c / aa, 0, 1)
            } else if t > 1 {
                t = 1
                s = clamp((bb - c) / aa, 0, 1)
            }
        }
    }

    c1 = a[0] + d1 * s
    c2 = b[0] + d2 * t
    return c1, c2
}

// Computes the closest points `c1` on segment `a` and `c2` on segment `b`.
//
// Accepts integer segment inputs and promotes them to `f64` coordinates to evaluate 
// precise, non-truncated fractional spatial closest points.
closest_points_int :: proc "contextless" (
    a, b: [2][$N]$T,
) -> (c1: [N]f64, c2: [N]f64) where is_int(T) {
    af := cast_to(a, f64)
    bf := cast_to(b, f64)
    return closest_points_float(af, bf)
}

// Explicit overload group for determining the closest points between two bounded line segments.
// Fundamental for computing 3D line-of-sight distance thresholds, processing broad-phase capsule 
// collision physics, and evaluating edge clearances in computational geometry.
closest_points :: proc {closest_points_float, closest_points_int}

// Calculates the minimum squared distance between two bounded line segments `a` and `b`.
//
// Leverages `closest_points` under the hood to find the shortest clearance vector. 
// This is significantly faster than calculating the true distance because it avoids a 
// square root operation. Highly recommended for narrow-phase collision optimization.
// Works with floating-point types.
closest_distance2_float :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
) -> T where is_float(T) {
    c1, c2 := closest_points(a, b)
    d := c1 - c2
    return linalg.dot(d, d)
}

// Calculates the minimum squared distance between two bounded line segments `a` and `b`.
//
// Accepts integer segment inputs and promotes them to `f64` coordinates to compute 
// the precise squared geometric clearance value without integer truncation.
closest_distance2_int :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
) -> f64 where is_int(T) {
    af := cast_to(a, f64)
    bf := cast_to(b, f64)
    return closest_distance2_float(af, bf)
}

// Explicit overload group for calculating the minimum squared distance between two segments.
// Essential for fast overlap/clearance queries (e.g., checking if two capsule colliders intersect 
// via `closest_distance2(a, b) <= (r1 + r2) * (r1 + r2)`).
closest_distance2 :: proc {closest_distance2_float, closest_distance2_int}

// Calculates the true geometric (Euclidean) minimum distance between two bounded line segments `a` and `b`.
// Works with floating-point types.
closest_distance_float :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
) -> T where is_float(T) {
    return math.sqrt(closest_distance2(a, b))
}

// Calculates the true geometric (Euclidean) minimum distance between two bounded line segments `a` and `b`.
//
// Accepts integer segment inputs and promotes them to `f64` coordinates to return 
// a precise fractional distance clearance value.
closest_distance_int :: #force_inline proc "contextless" (
    a, b: [2][$N]$T,
) -> f64 where is_int(T) {
    af := cast_to(a, f64)
    bf := cast_to(b, f64)
    return closest_distance_float(af, bf)
}

// Explicit overload group for calculating the true minimum geometric distance between two line segments.
closest_distance :: proc {closest_distance_float, closest_distance_int}