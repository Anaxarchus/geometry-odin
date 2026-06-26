package bvh

import "core:slice"
import "core:math/linalg"

Primitive :: struct(T: typeid) {
    min:    [3]T,
    max:    [3]T,
    v0:     [3]T,
    v1:     [3]T,
    v2:     [3]T,
    id:     int,  // Optional user/entity ID
    active: bool,
}

Node :: struct(T: typeid) {
    aabb_min:   [3]T,
    aabb_max:   [3]T,
    left:       int,
    right:      int,
    prim_start: int,
    prim_count: int,
    parent:     int,
}

BVH :: struct(T: typeid) {
    nodes:      []Node(T),
    node_count: int,
    primitives: []Primitive(T),
}

ray_plane_intersect :: proc(origin, dir, plane_normal: [3]f32, plane_d: f32) -> (pos: [3]f32, hit: bool) {
    denom := linalg.dot(plane_normal, dir)
    if abs(denom) < 1e-6 do return {}, false   // ray parallel to plane
    t := (plane_d - linalg.dot(plane_normal, origin)) / denom
    if t < 0 do return {}, false                // plane behind ray origin
    return origin + dir * t, true
}

ray_aabb_intersect :: proc(ray_origin, ray_dir, ray_inv_dir, aabb_min, aabb_max: [3]$T) -> (hit: bool, t: T) {
    t_min := (aabb_min - ray_origin) * ray_inv_dir
    t_max := (aabb_max - ray_origin) * ray_inv_dir
    t1 := linalg.min(t_min, t_max)
    t2 := linalg.max(t_min, t_max)
    t_near := max(max(t1.x, t1.y), t1.z)
    t_far  := min(min(t2.x, t2.y), t2.z)
    return t_far >= 0 && t_near <= t_far, t_near
}

// Inline Moller-Trumbore triangle intersection
ray_triangle_intersect :: proc(origin, direction: [3]$T, v0, v1, v2: [3]T, max_t: T) -> (hit: bool, t: T) {
    EPSILON :: 0.000001

    edge1 := v1 - v0
    edge2 := v2 - v0

    h := linalg.cross(direction, edge2)
    det := linalg.dot(edge1, h)

    if det > -EPSILON && det < EPSILON do return false, max_t

    inv_det := 1.0 / det
    s := origin - v0

    u := linalg.dot(s, h) * inv_det
    if u < 0.0 || u > 1.0 do return false, max_t

    q := linalg.cross(s, edge1)
    v := linalg.dot(direction, q) * inv_det
    if v < 0.0 || u + v > 1.0 do return false, max_t

    out_t := linalg.dot(edge2, q) * inv_det

    if out_t > EPSILON && out_t < max_t {
        return true, out_t
    }

    return false, max_t
}

aabb_aabb_intersect :: proc(a_min, a_max, b_min, b_max: [3]$T) -> bool {
    return !(a_max.x < b_min.x || b_max.x < a_min.x ||
             a_max.y < b_min.y || b_max.y < a_min.y ||
             a_max.z < b_min.z || b_max.z < a_min.z)
}

@(private)
compute_aabb :: proc(primitives: []Primitive($T)) -> (min, max: [3]T) {
    min = primitives[0].min
    max = primitives[0].max
    for i in 1..<len(primitives) {
        min = linalg.min(min, primitives[i].min)
        max = linalg.max(max, primitives[i].max)
    }
    return
}

@(private)
Bin :: struct(T: typeid) {
    aabb_min, aabb_max: [3]T,
    count: int,
}

@(private)
sweep :: proc(bins: [16]Bin($T)) -> (left_min, left_max: [15][3]T, left_count: [15]int, right_min, right_max: [15][3]T, right_count: [15]int) {
    MAX :: max(T)

    for i in 0..<15 {
        left_min[i]   = {MAX, MAX, MAX}
        left_max[i]   = {-MAX, -MAX, -MAX}
        left_count[i] = 0
        right_min[i]  = {MAX, MAX, MAX}
        right_max[i]  = {-MAX, -MAX, -MAX}
        right_count[i] = 0
    }

    for i in 0..<15 {
        if i > 0 {
            left_min[i]   = left_min[i-1]
            left_max[i]   = left_max[i-1]
            left_count[i] = left_count[i-1]
        }
        if bins[i].count > 0 {
            left_min[i]   = linalg.min(left_min[i], bins[i].aabb_min)
            left_max[i]   = linalg.max(left_max[i], bins[i].aabb_max)
            left_count[i] += bins[i].count
        }
    }

    for i := 13; i >= 0; i -= 1 {
        if i < 13 {
            right_min[i]   = right_min[i+1]
            right_max[i]   = right_max[i+1]
            right_count[i] = right_count[i+1]
        }
        if bins[i+1].count > 0 {
            right_min[i]   = linalg.min(right_min[i], bins[i+1].aabb_min)
            right_max[i]   = linalg.max(right_max[i], bins[i+1].aabb_max)
            right_count[i] += bins[i+1].count
        }
    }

    return
}

@(private)
surface_area :: #force_inline proc(aabb_min, aabb_max: [3]$T) -> T {
    e := aabb_max - aabb_min
    return 2 * (e.x*e.y + e.y*e.z + e.z*e.x)
}

@(private)
sah_split :: proc(primitives: []Primitive($T), node_min, node_max: [3]T) -> (split_axis, split_idx: int) {
    MAX := max(T)
    xbins, ybins, zbins: [16]Bin(T)

    bin := Bin(T){
        count    = 0,
        aabb_min = {MAX, MAX, MAX},
        aabb_max = {-MAX, -MAX, -MAX},
    }

    for i in 0..<16 {
        xbins[i] = bin; ybins[i] = bin; zbins[i] = bin
    }

    for p in primitives {
        c := (p.min + p.max) * 0.5
        t := (c - node_min) / (node_max - node_min)

        x := clamp(int(t.x * 16), 0, 15)
        y := clamp(int(t.y * 16), 0, 15)
        z := clamp(int(t.z * 16), 0, 15)

        xbins[x].count += 1
        xbins[x].aabb_min = linalg.min(xbins[x].aabb_min, p.min)
        xbins[x].aabb_max = linalg.max(xbins[x].aabb_max, p.max)

        ybins[y].count += 1
        ybins[y].aabb_min = linalg.min(ybins[y].aabb_min, p.min)
        ybins[y].aabb_max = linalg.max(ybins[y].aabb_max, p.max)

        zbins[z].count += 1
        zbins[z].aabb_min = linalg.min(zbins[z].aabb_min, p.min)
        zbins[z].aabb_max = linalg.max(zbins[z].aabb_max, p.max)
    }

    xl_min, xl_max, xl_count, xr_min, xr_max, xr_count := sweep(xbins)
    yl_min, yl_max, yl_count, yr_min, yr_max, yr_count := sweep(ybins)
    zl_min, zl_max, zl_count, zr_min, zr_max, zr_count := sweep(zbins)

    best_cost := MAX
    best_axis := 0
    best_bin  := 0

    for i in 0..<15 {
        x_cost := surface_area(xl_min[i], xl_max[i]) * T(xl_count[i]) + surface_area(xr_min[i], xr_max[i]) * T(xr_count[i])
        if x_cost < best_cost { best_cost = x_cost; best_axis = 0; best_bin = i }

        y_cost := surface_area(yl_min[i], yl_max[i]) * T(yl_count[i]) + surface_area(yr_min[i], yr_max[i]) * T(yr_count[i])
        if y_cost < best_cost { best_cost = y_cost; best_axis = 1; best_bin = i }

        z_cost := surface_area(zl_min[i], zl_max[i]) * T(zl_count[i]) + surface_area(zr_min[i], zr_max[i]) * T(zr_count[i])
        if z_cost < best_cost { best_cost = z_cost; best_axis = 2; best_bin = i }
    }

    lo := 0
    hi := len(primitives) - 1
    for lo <= hi {
        c := (primitives[lo].min + primitives[lo].max) * 0.5
        c_axis: T
        switch best_axis {
            case 0: c_axis = c.x
            case 1: c_axis = c.y
            case 2: c_axis = c.z
        }
        t := (c_axis - node_min[best_axis]) / (node_max[best_axis] - node_min[best_axis])
        bin_idx := clamp(int(t * 16), 0, 15)
        if bin_idx <= best_bin {
            lo += 1
        } else {
            primitives[lo], primitives[hi] = primitives[hi], primitives[lo]
            hi -= 1
        }
    }

    if lo == 0 || lo == len(primitives) {
        lo = len(primitives) / 2
    }

    return best_axis, lo
}

@(private)
build_node :: proc(bvh: ^BVH($T), parent_idx, prim_start, prim_end, leaf_size: int) -> int {
    node := Node(T){}
    node.aabb_min, node.aabb_max = compute_aabb(bvh.primitives[prim_start: prim_end])
    node.prim_start = prim_start
    node.prim_count = prim_end - prim_start
    node.parent = parent_idx
    node.left = -1
    node.right = -1
    id := bvh.node_count
    bvh.node_count += 1

    if node.prim_count > leaf_size {
        _, mid := sah_split(bvh.primitives[prim_start:prim_end], node.aabb_min, node.aabb_max)
        node.left  = build_node(bvh, id, prim_start, prim_start + mid, leaf_size)
        node.right = build_node(bvh, id, prim_start + mid, prim_end, leaf_size)

        node.prim_count = 0
        node.prim_start = 0
    }

    bvh.nodes[id] = node
    return id
}

make_id_map :: proc(bvh: BVH($T)) -> map[int]int {
    result := make(map[int]int)
    for p, i in bvh.primitives {
        result[p.id] = i
    }
    return result
}

make_triangle_primitive :: proc(v0, v1, v2: [3]$T, id := -1) -> Primitive(T) {
    pmin := linalg.min(linalg.min(v0, v1), v2)
    pmax := linalg.max(linalg.max(v0, v1), v2)
    return Primitive(T){
        min    = pmin,
        max    = pmax,
        v0     = v0,
        v1     = v1,
        v2     = v2,
        id     = id,
        active = true,
    }
}

make_bvh :: proc(leaves: []Primitive($T), leaf_size: int = 4) -> BVH(T) {
    if len(leaves) == 0 do return {}

    bvh := BVH(T){
        nodes      = make([]Node(T), len(leaves) * 2 - 1),
        primitives = make([]Primitive(T), len(leaves)),
        node_count = 0,
    }
    copy(bvh.primitives, leaves)
    build_node(&bvh, -1, 0, len(leaves), leaf_size)
    return bvh
}

delete_bvh :: proc(bvh: BVH($T)) {
    delete(bvh.nodes)
    delete(bvh.primitives)
}

// Ray query runs tight Möller-Trumbore intersection across leaves
ray_query :: proc(bvh: BVH($T), origin, direction: [3]T) -> (hit: bool, index, id: int, t: T) {
    if bvh.node_count == 0 do return false, -1, -1, 0

    stack:     [64]int
    stack_top: int
    stack[stack_top] = 0
    stack_top += 1

    t     = max(T)
    index = -1
    id    = -1

    inv_dir := [3]T{1/direction.x, 1/direction.y, 1/direction.z}

    for stack_top > 0 {
        stack_top -= 1
        node_idx := stack[stack_top]
        node := bvh.nodes[node_idx]

        hit_aabb, t_near := ray_aabb_intersect(origin, direction, inv_dir, node.aabb_min, node.aabb_max)

        if !hit_aabb || t_near > t do continue

        if node.prim_count > 0 {
            for i in node.prim_start..<node.prim_start + node.prim_count {
                prim := bvh.primitives[i]
                if !prim.active do continue
                
                // Directly check structural geometry intersection
                if tri_hit, tri_t := ray_triangle_intersect(origin, direction, prim.v0, prim.v1, prim.v2, t); tri_hit {
                    if tri_t < t {
                        t = tri_t
                        index = i
                        id = prim.id
                    }
                }
            }
        } else {
            left  := bvh.nodes[node.left]
            right := bvh.nodes[node.right]

            left_hit,  left_t  := ray_aabb_intersect(origin, direction, inv_dir, left.aabb_min,  left.aabb_max)
            right_hit, right_t := ray_aabb_intersect(origin, direction, inv_dir, right.aabb_min, right.aabb_max)

            if left_hit && right_hit {
                if left_t < right_t {
                    stack[stack_top] = node.right; stack_top += 1
                    stack[stack_top] = node.left;  stack_top += 1
                } else {
                    stack[stack_top] = node.left;  stack_top += 1
                    stack[stack_top] = node.right; stack_top += 1
                }
            } else if left_hit {
                stack[stack_top] = node.left;  stack_top += 1
            } else if right_hit {
                stack[stack_top] = node.right; stack_top += 1
            }
        }
    }

    hit = (index > -1)
    return
}

aabb_query :: proc(bvh: BVH($T), query_min, query_max: [3]T) -> [dynamic]int {
    if bvh.node_count == 0 do return {}

    results := make([dynamic]int)

    stack:     [64]int
    stack_top: int
    stack[stack_top] = 0
    stack_top += 1

    for stack_top > 0 {
        stack_top -= 1
        node_idx := stack[stack_top]
        node := bvh.nodes[node_idx]

        if !aabb_aabb_intersect(query_min, query_max, node.aabb_min, node.aabb_max) do continue

        if node.prim_count > 0 {
            for i in node.prim_start..<node.prim_start + node.prim_count {
                prim := bvh.primitives[i]
                if !prim.active do continue
                if aabb_aabb_intersect(query_min, query_max, prim.min, prim.max) {
                    _,exists := slice.linear_search(results[:], prim.id)
                    if !exists {
                        append(&results, prim.id)
                    }
                }
            }
        } else {
            stack[stack_top] = node.left;  stack_top += 1
            stack[stack_top] = node.right; stack_top += 1
        }
    }

    return results
}