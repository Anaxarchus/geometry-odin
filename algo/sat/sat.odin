package sat

import "base:intrinsics"
import "core:math/linalg"

Winding :: enum {CCW, CW}

@(private)
get_axes :: proc(shape: [][2]$T, winding: Winding) -> [][2]T
where intrinsics.type_is_float(T) {
    C := len(shape)
    axes := make([][2]T, C)
    for i in 0..<C {
        j := (i+1)%C

        a := shape[i]
        b := shape[j]

        if abs(a.x - b.x) < 1e-7 && abs(a.y - b.y) < 1e-7 {
            continue
        }

        direction := linalg.orthogonal(b-a)
        if winding == .CCW {
            direction = -direction
        }

        axes[i] = direction
    }
    return axes
}

@(private)
project_to_axis :: proc(shape: [][2]$T, axis: [2]T) -> (min, max: T)
where intrinsics.type_is_float(T) {
    min = linalg.dot(axis, shape[0])
    max = min
    for i in 1..<len(shape) {
        p := linalg.dot(axis, shape[i])
        if p < min {
            min = p
        } else if p > max {
            max = p
        }
    }
    return
}

@(private)
overlapping :: #force_inline proc(amin, amax, bmin, bmax: $T) -> bool {
    return !(amax < bmin || bmax < amin)
}

@(private)
centroid :: proc(shape: [][2]$T) -> [2]T
where intrinsics.type_is_float(T) {
    c: [2]T
    for v in shape {
        c += v
    }
    return c / T(len(shape))
}

is_overlapping :: proc(a, b: [][2]$T, winding: Winding) -> bool
where intrinsics.type_is_float(T) {
    axes_a := get_axes(a, winding); defer delete(axes_a)
    axes_b := get_axes(b, winding); defer delete(axes_b)

    for axis in axes_a {
        amin, amax := project_to_axis(a, axis)
        bmin, bmax := project_to_axis(b, axis)
        if !overlapping(amin, amax, bmin, bmax) {
            return false
        }
    }

    for axis in axes_b {
        amin, amax := project_to_axis(a, axis)
        bmin, bmax := project_to_axis(b, axis)
        if !overlapping(amin, amax, bmin, bmax) {
            return false
        }
    }

    return true
}

is_overlapping_mtv :: proc(a, b: [][2]$T, winding: Winding) -> (bool, [2]T)
where intrinsics.type_is_float(T) {
    axes_a := get_axes(a, winding); defer delete(axes_a)
    axes_b := get_axes(b, winding); defer delete(axes_b)

    min_overlap := max(T)
    smallest: [2]T

    for axis in axes_a {
        n := linalg.normalize(axis)
        amin, amax := project_to_axis(a, n)
        bmin, bmax := project_to_axis(b, n)
        if !overlapping(amin, amax, bmin, bmax) {
            return false, {}
        }
        o := min(amax, bmax) - max(amin, bmin)
        if o < min_overlap {
            min_overlap = o
            smallest = n
        }
    }

    for axis in axes_b {
        n := linalg.normalize(axis)
        amin, amax := project_to_axis(a, n)
        bmin, bmax := project_to_axis(b, n)
        if !overlapping(amin, amax, bmin, bmax) {
            return false, {}
        }
        o := min(amax, bmax) - max(amin, bmin)
        if o < min_overlap {
            min_overlap = o
            smallest = n
        }
    }

    ca := centroid(a)
    cb := centroid(b)
    if linalg.dot(cb - ca, smallest) < 0 {
        smallest = -smallest
    }

    return true, smallest * min_overlap
}

// Gap-aware touch test: returns true if a and b overlap OR are separated by no
// more than `gap` on every separating axis. Equivalent to a Minkowski inflation
// of both shapes by `gap`, so two shapes that clear each other on every face
// normal by less than `gap` count as "approximately touching".
is_touching :: proc(a, b: [][2]$T, gap: T, winding: Winding) -> bool
where intrinsics.type_is_float(T) {
    if len(a) == 0 || len(b) == 0 do return false

    axes: [64][2]T
    axis_count: int

    includes :: #force_inline proc(slice: [][2]T, norm: [2]T, eps := T(1e-7)) -> bool {
        for axis in slice {
            if (abs(axis.x - norm.x) < eps && abs(axis.y - norm.y) < eps) ||
               (abs(axis.x + norm.x) < eps && abs(axis.y + norm.y) < eps) {
                return true
            }
        }
        return false
    }

    bb_a: [2][2]T = {{max(T), max(T)}, {min(T), min(T)}}
    for i in 0..<len(a) {
        bb_a[0] = linalg.min(bb_a[0], a[i])
        bb_a[1] = linalg.max(bb_a[1], a[i])
        j := (i+1)%len(a)
        e := a[j] - a[i]
        if linalg.length2(e) < T(1e-14) do continue   // skip degenerate edge
        o := linalg.normalize(linalg.orthogonal(e))
        if winding == .CW do o = -o
        if includes(axes[:axis_count], o) do continue
        if axis_count < len(axes) { axes[axis_count] = o; axis_count += 1 }
    }

    bb_b: [2][2]T = {{max(T), max(T)}, {min(T), min(T)}}
    for i in 0..<len(b) {
        bb_b[0] = linalg.min(bb_b[0], b[i])
        bb_b[1] = linalg.max(bb_b[1], b[i])
        j := (i+1)%len(b)
        e := b[j] - b[i]
        if linalg.length2(e) < T(1e-14) do continue   // skip degenerate edge
        o := linalg.normalize(linalg.orthogonal(e))
        if winding == .CCW do o = -o
        if includes(axes[:axis_count], o) do continue
        if axis_count < len(axes) { axes[axis_count] = o; axis_count += 1 }
    }

    // broad-phase: separated if inflated bounding boxes clear each other
    if bb_a[1].x < bb_b[0].x - gap || bb_b[1].x < bb_a[0].x - gap ||
       bb_a[1].y < bb_b[0].y - gap || bb_b[1].y < bb_a[0].y - gap {
        return false
    }

    // narrow-phase: separated if any face-normal axis clears the gap tolerance
    for axis in axes[:axis_count] {
        amin, amax := project_to_axis(a, axis)
        bmin, bmax := project_to_axis(b, axis)
        if amax < bmin - gap || bmax < amin - gap {
            return false
        }
    }

    return true
}

// used when len(a) + len(b) <= 32
// stack allocated
// goes brrr
get_mtv_small :: proc(a, b: [][2]$T, winding: Winding) -> (overlaps: bool, mtv: [2]T)
where intrinsics.type_is_float(T) {
    if len(a) == 0 || len(b) == 0 do return false, {}

    axes: [64][2]T
    axis_count: int

    includes :: #force_inline proc(slice: [][2]T, norm: [2]T, eps := T(1e-7)) -> bool {
        for axis in slice {
            if (abs(axis.x - norm.x) < eps && abs(axis.y - norm.y) < eps) ||
               (abs(axis.x + norm.x) < eps && abs(axis.y + norm.y) < eps) {
                return true
            }
        }
        return false
    }

    centroid_a: [2]T
    bb_a: [2][2]T = {{max(T), max(T)}, {min(T), min(T)}}
    for i in 0..<len(a) {
        centroid_a += a[i]
        bb_a[0] = linalg.min(bb_a[0], a[i])
        bb_a[1] = linalg.max(bb_a[1], a[i])
        j := (i+1)%len(a)
        o := linalg.normalize(linalg.orthogonal(a[j] - a[i]))
        if winding == .CW do o = -o
        if includes(axes[:axis_count], o) do continue
        axes[axis_count] = o; axis_count += 1
    }
    centroid_a /= T(len(a))

    centroid_b: [2]T
    bb_b: [2][2]T = {{max(T), max(T)}, {min(T), min(T)}}
    for i in 0..<len(b) {
        centroid_b += b[i]
        bb_b[0] = linalg.min(bb_b[0], b[i])
        bb_b[1] = linalg.max(bb_b[1], b[i])
        j := (i+1)%len(b)
        o := linalg.normalize(linalg.orthogonal(b[j] - b[i]))
        if winding == .CCW do o = -o
        if includes(axes[:axis_count], o) do continue
        axes[axis_count] = o; axis_count += 1
    }
    centroid_b /= T(len(b))

    min_overlap := max(T)
    smallest: [2]T

    // early out if bounding boxes don't overlap
    if bb_a[1].x < bb_b[0].x || bb_b[1].x < bb_a[0].x ||
    bb_a[1].y < bb_b[0].y || bb_b[1].y < bb_a[0].y {
        return false, {}
    }

    // Check overlaps
    for axis in axes[:axis_count] {
        amin, amax := project_to_axis(a, axis)
        bmin, bmax := project_to_axis(b, axis)
        if !overlapping(amin, amax, bmin, bmax) {
            return false, {}
        }
        o := min(amax, bmax) - max(amin, bmin)
        if o < min_overlap {
            min_overlap = o
            smallest = axis
        }
    }

    // Push direction correction
    if linalg.dot(centroid_b - centroid_a, smallest) < 0 {
        smallest = -smallest
    }

    return true, smallest * min_overlap
}