# Examples

Interactive demos built on `vendor:raylib` (ships with the Odin toolchain).
Each is a standalone `package main`; run from the repo root:

```sh
odin run examples/booleans
odin run examples/offset
odin run examples/layout
odin run examples/raycast3d
odin run examples/lines
odin run examples/kitchensink
odin run examples/asteroids
odin run examples/hull
```

| Example | Shows | Controls |
|---|---|---|
| `booleans` | `polygon` union / intersect / difference / xor, filled via `triangulate` | mouse moves the second disk; `1`-`4` switch operation |
| `offset` | `offset` outward & inward with miter / round / bevel joins | `1`/`2`/`3` switch join style (offset distance animates) |
| `layout` | `rect` partitioning (`cut` / `grow` / `divide` / `stack`) as a responsive dashboard | resize the window to reflow |
| `raycast3d` | `algo/bvh` ray query over a triangle soup + `intersect.ray_at` | a ray sweeps the scene; the nearest hit is highlighted |
| `lines` | `line` API: `project` / `distance` / `side` / `normal` / `has_point` / `angle` / `rotate` / `closest_distance` | move the mouse; a spinning blade reports its gap to the path |
| `kitchensink` | `rect` + `circle` + `line` together: panel layout plus nearest-point / containment queries on three obstacles | move the mouse to probe each obstacle |
| `hull` | a geometry-first demo where the algorithm *is* the program: a convex hull is gift-wrapped from a draggable point cloud using `line.side` as the orientation test, then the contour is run through `polygon` (`is_convex`/`is_clockwise`/`area`/`perimeter`/`min_max`/`edges`/`triangulate`/`has_point`), its bounding box through `rect` (`center`/`size`/`area`/`corners`), centroid-fit circles through `from_contour`, and the triangles through `centroid`/`area` (summed to cross-check the polygon area). The mouse probes containment and the nearest edge via `line.project`/`distance`/`side`/`normal`/`angle` | left click/drag add or move a point, right-click delete, `R` scatter, `N` toggle edge normals |
| `asteroids` | a small Asteroids clone touring most of the lib: timed stages drop materials you spend in a `rect`-built upgrade shop (`grow`/`cut`/`divide_x`/`cut_frac`/`has_point`), `polygon` rocks burst into their component triangles via `triangulate` (+ `has_point`/`is_convex`/`area`/`min_max`), `contains_point` ship hits, a `circle` energy/destruct shield, `intersects` AABB debug, and a `line` aim-lock crosshair | arrows/WASD fly, SPACE shoot, SHIFT shield; shop: 1-4 buy, ENTER next stage; G debug, R restart |
