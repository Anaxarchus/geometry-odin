# Examples

Interactive demos built on `vendor:raylib` (ships with the Odin toolchain).
Each is a standalone `package main`; run from the repo root:

```sh
odin run examples/booleans
odin run examples/offset
odin run examples/layout
odin run examples/raycast3d
```

| Example | Shows | Controls |
|---|---|---|
| `booleans` | `polygon` union / intersect / difference / xor, filled via `polygon_triangulate` | mouse moves the second disk; `1`-`4` switch operation |
| `offset` | `polygon_offset` outward & inward with miter / round / bevel joins | `1`/`2`/`3` switch join style (offset distance animates) |
| `layout` | `rect` partitioning (`cut` / `grow` / `divide` / `stack`) as a responsive dashboard | resize the window to reflow |
| `raycast3d` | `algo/bvh` ray query over a triangle soup + `intersect.ray_at` | a ray sweeps the scene; the nearest hit is highlighted |
