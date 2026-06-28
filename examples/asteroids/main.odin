package main

// A small Asteroids clone that doubles as a tour of the geometry library, now
// with a stage / shop gameplay loop.
//
//   rect     - HUD + shield gauge (cut_frac) + AABB overlap (intersects)
//              and the whole upgrade shop (grow / cut / divide_x /
//              has_point for click hit-testing)
//   polygon  - procedural rocks, triangulate to burst them into their
//              component triangles, has_point collisions,
//              min_max / is_convex / area (debug)
//   triangle - debris shards (centroid / area) and ship hits
//              via contains_point
//   circle   - the deflector / destructive shield (has_point,
//              project_to_boundary)
//   line     - aim-lock targeting (line.distance_infinite)
//
// Loop: each stage runs ~30s with increasing rock counts; shoot rocks to drop
// materials, then spend them in the shop between stages.
//
// Controls: arrows / WASD fly, SPACE shoot, SHIFT shield, G geometry debug,
// R restart. In the shop: 1-4 (or click) to buy, ENTER for next stage.

import rl "vendor:raylib"
import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:math/rand"
import "../../circle"
import "../../line"
import "../../polygon"
import "../../rect"
import "../../triangle"
import "core:strings"

WIDTH :: 1000
HEIGHT :: 720

TURN :: 3.7
THRUST :: 340
DRAG :: 0.99
BULLET_SPEED :: 620

STAGE_LEN :: 30
GUN_MAX :: 5
GUN_SPREAD :: 0.14

SHIELD_RADIUS :: 46
SHIELD_DRAIN :: 0.45
SHIELD_REGEN :: 0.16
SHIELD_DEFLECT_COST :: 0.05
SHIELD_DESTROY_COST :: 0.12

// prices
PRICE_SHIELD :: 14
PRICE_DESTRUCT :: 28
PRICE_LIFE :: 20

// wound so the signed area is positive (contains_point convention)
ship_model := [3][2]f32{{18, 0}, {-12, 11}, {-12, -11}}

Mode :: enum {
	Playing,
	Shop,
	GameOver,
}

Ship :: struct {
	pos, vel:  [2]f32,
	ang:       f32,
	alive:     bool,
	respawn:   f32,
	inv:       f32, // invulnerability timer
	shield:    f32, // energy 0..1
	shielding: bool,
}

Asteroid :: struct {
	pos, vel:  [2]f32,
	ang, spin: f32,
	radius:    f32,
	size:      int, // 2 = large, 1 = medium, 0 = small
	kind:      int,
	shape:     [][2]f32, // model-space polygon (heap owned)
	dead:      bool,
}

Bullet :: struct {
	pos, vel: [2]f32,
	life:     f32,
	dead:     bool,
}

Debris :: struct {
	pos, vel:       [2]f32,
	ang, spin:      f32,
	local:          [3][2]f32,
	life, max_life: f32,
}

Pickup :: struct {
	pos, vel:  [2]f32,
	ang, spin: f32,
	life:      f32,
}

Game :: struct {
	ship:        Ship,
	asteroids:   [dynamic]Asteroid,
	bullets:     [dynamic]Bullet,
	debris:      [dynamic]Debris,
	pickups:     [dynamic]Pickup,
	pending:     [dynamic]Asteroid,
	score:       int,
	lives:       int,
	materials:   int,
	stage:       int,
	stage_time:  f32,
	spawn_timer: f32,
	gun_level:   int,
	has_shield:  bool,
	destructive: bool,
	locked:      int,
	debug:       bool,
	mode:        Mode,
	over:        bool,
}

// --- small helpers ----------------------------------------------------------

v2 :: proc(p: [2]f32) -> rl.Vector2 {
	return rl.Vector2(p)
}

to_rec :: proc(r: [2][2]f32) -> rl.Rectangle {
	return {r[0].x, r[0].y, r[1].x - r[0].x, r[1].y - r[0].y}
}

rndf :: proc(lo, hi: f32) -> f32 {
	return lo + rand.float32() * (hi - lo)
}

rot :: proc(v: [2]f32, a: f32) -> [2]f32 {
	c := math.cos(a)
	s := math.sin(a)
	return {v.x * c - v.y * s, v.x * s + v.y * c}
}

reflect :: proc(v, n: [2]f32) -> [2]f32 {
	return v - 2 * linalg.dot(v, n) * n
}

safe_norm :: proc(v: [2]f32) -> [2]f32 {
	l := linalg.length(v)
	return v / l if l > 1e-5 else [2]f32{1, 0}
}

wrap :: proc(p: [2]f32) -> [2]f32 {
	q := p
	if q.x < 0 do q.x += WIDTH
	if q.x >= WIDTH do q.x -= WIDTH
	if q.y < 0 do q.y += HEIGHT
	if q.y >= HEIGHT do q.y -= HEIGHT
	return q
}

radius_for_size :: proc(size: int) -> f32 {
	switch size {
	case 2:
		return 58
	case 1:
		return 34
	}
	return 19
}

kind_color :: proc(kind: int) -> rl.Color {
	switch kind {
	case 0:
		return {185, 188, 210, 255}
	case 1:
		return {175, 205, 180, 255}
	}
	return {214, 184, 168, 255}
}

gun_price :: proc(level: int) -> int {
	return 10 + 6 * level
}

// --- procedural shapes ------------------------------------------------------

gen_shape :: proc(radius: f32, kind: int) -> [][2]f32 {
	n: int
	switch kind {
	case 0:
		n = 11 // lumpy
	case 1:
		n = 8 // chunky
	case:
		n = 12 // spiky
	}
	pts := make([][2]f32, n)
	for i in 0 ..< n {
		ang := f32(i) / f32(n) * 2 * math.PI
		r: f32
		switch kind {
		case 0:
			r = radius * rndf(0.72, 1.0)
		case 1:
			r = radius * rndf(0.88, 1.0)
		case:
			r = radius * (1.0 if i % 2 == 0 else rndf(0.5, 0.66))
		}
		pts[i] = {math.cos(ang) * r, math.sin(ang) * r}
	}
	return pts
}

make_asteroid :: proc(pos: [2]f32, size: int) -> Asteroid {
	radius := radius_for_size(size)
	kind := int(rndf(0, 3))
	return Asteroid {
		pos = pos,
		vel = rot({rndf(45, 120), 0}, rndf(0, 2 * math.PI)),
		ang = rndf(0, 2 * math.PI),
		spin = rndf(-1.5, 1.5),
		radius = radius,
		size = size,
		kind = kind,
		shape = gen_shape(radius, kind),
	}
}

world_shape :: proc(a: Asteroid, allocator := context.temp_allocator) -> [][2]f32 {
	out := make([][2]f32, len(a.shape), allocator)
	for v, i in a.shape {
		out[i] = a.pos + rot(v, a.ang)
	}
	return out
}

ship_world :: proc(s: Ship) -> [3][2]f32 {
	out: [3][2]f32
	for v, i in ship_model {
		out[i] = s.pos + rot(v, s.ang)
	}
	return out
}

// --- spawning / destruction -------------------------------------------------

spawn_wave :: proc(g: ^Game, count: int) {
	for _ in 0 ..< count {
		pos: [2]f32
		for {
			pos = {rndf(0, WIDTH), rndf(60, HEIGHT)}
			if linalg.length(pos - g.ship.pos) > 200 do break
		}
		append(&g.asteroids, make_asteroid(pos, 2))
	}
}

// bursts an asteroid into debris triangles + smaller children, drops materials
destroy_asteroid :: proc(g: ^Game, a: Asteroid, world: [][2]f32) {
	tris := polygon.triangulate(world, .Robust, context.temp_allocator)
	for tr in tris {
		if triangle.area(tr) < 2 do continue
		c := triangle.centroid(tr)
		d := Debris {
			pos   = c,
			local = {tr[0] - c, tr[1] - c, tr[2] - c},
			vel   = a.vel + safe_norm(c - a.pos) * rndf(40, 120),
			spin  = rndf(-5, 5),
			life  = rndf(0.6, 1.5),
		}
		d.max_life = d.life
		append(&g.debris, d)
	}

	switch a.size {
	case 2:
		g.score += 20
	case 1:
		g.score += 50
	case:
		g.score += 100
	}

	// material drops
	drops := 2 if a.size == 2 else 1
	for _ in 0 ..< drops {
		append(
			&g.pickups,
			Pickup {
				pos = a.pos,
				vel = rot({rndf(30, 85), 0}, rndf(0, 2 * math.PI)),
				spin = rndf(-4, 4),
				life = 12,
			},
		)
	}

	if a.size > 0 {
		for _ in 0 ..< 2 {
			ch := make_asteroid(a.pos, a.size - 1)
			ch.vel = a.vel + rot({rndf(70, 140), 0}, rndf(0, 2 * math.PI))
			append(&g.pending, ch)
		}
	}
}

kill_ship :: proc(g: ^Game) {
	g.lives -= 1
	g.ship.alive = false
	g.ship.respawn = 1.3
	if g.lives <= 0 do g.over = true
}

begin_stage :: proc(g: ^Game) {
	for a in g.asteroids do delete(a.shape)
	clear(&g.asteroids)
	clear(&g.bullets)
	clear(&g.debris)
	clear(&g.pickups)
	clear(&g.pending)

	g.stage += 1
	g.stage_time = STAGE_LEN
	g.spawn_timer = 6
	g.mode = .Playing
	g.locked = -1

	g.ship.pos = {WIDTH / 2, HEIGHT / 2}
	g.ship.vel = {}
	g.ship.ang = -math.PI / 2
	g.ship.alive = true
	g.ship.inv = 2.0
	g.ship.shield = 1.0

	spawn_wave(g, 3 + g.stage)
}

end_stage :: proc(g: ^Game) {
	g.materials += len(g.pickups) // bank what you didn't grab
	for a in g.asteroids do delete(a.shape)
	clear(&g.asteroids)
	clear(&g.bullets)
	clear(&g.debris)
	clear(&g.pickups)
	clear(&g.pending)
	g.locked = -1
	g.mode = .Shop
}

reset_game :: proc(g: ^Game) {
	for a in g.asteroids do delete(a.shape)
	clear(&g.asteroids)
	clear(&g.bullets)
	clear(&g.debris)
	clear(&g.pickups)
	clear(&g.pending)
	g.ship = Ship{}
	g.score = 0
	g.lives = 3
	g.materials = 0
	g.gun_level = 0
	g.has_shield = false
	g.destructive = false
	g.stage = 0
	g.debug = false
	g.over = false
	begin_stage(g)
}

// --- upgrades ---------------------------------------------------------------

// price, status label, and whether it can be bought right now
card_info :: proc(g: ^Game, i: int) -> (status: cstring, buyable: bool) {
	switch i {
	case 0:
		if g.gun_level >= GUN_MAX do return "MAX LEVEL", false
		p := gun_price(g.gun_level)
		return fmt.ctprintf("lvl %d  -  %d mat", g.gun_level, p), g.materials >= p
	case 1:
		if g.has_shield do return "OWNED", false
		return fmt.ctprintf("%d mat", PRICE_SHIELD), g.materials >= PRICE_SHIELD
	case 2:
		if !g.has_shield do return "NEEDS SHIELD", false
		if g.destructive do return "OWNED", false
		return fmt.ctprintf("%d mat", PRICE_DESTRUCT), g.materials >= PRICE_DESTRUCT
	case:
		if g.lives >= 6 do return "MAX LIVES", false
		return fmt.ctprintf("%d mat", PRICE_LIFE), g.materials >= PRICE_LIFE
	}
	return "", false
}

try_buy :: proc(g: ^Game, i: int) {
	switch i {
	case 0:
		if g.gun_level < GUN_MAX {
			p := gun_price(g.gun_level)
			if g.materials >= p {
				g.materials -= p
				g.gun_level += 1
			}
		}
	case 1:
		if !g.has_shield && g.materials >= PRICE_SHIELD {
			g.materials -= PRICE_SHIELD
			g.has_shield = true
		}
	case 2:
		if g.has_shield && !g.destructive && g.materials >= PRICE_DESTRUCT {
			g.materials -= PRICE_DESTRUCT
			g.destructive = true
		}
	case 3:
		if g.lives < 6 && g.materials >= PRICE_LIFE {
			g.materials -= PRICE_LIFE
			g.lives += 1
		}
	}
}

// --- updates ----------------------------------------------------------------

fire :: proc(g: ^Game) {
	s := g.ship
	count := 1 + g.gun_level
	for i in 0 ..< count {
		off := (f32(i) - f32(count - 1) / 2) * GUN_SPREAD
		dir := rot({1, 0}, s.ang + off)
		nose := s.pos + dir * 18
		append(&g.bullets, Bullet{pos = nose, vel = dir * BULLET_SPEED + s.vel, life = 1.1})
	}
}

update_ship :: proc(g: ^Game, dt: f32) {
	s := &g.ship
	if s.inv > 0 do s.inv -= dt

	if !s.alive {
		if !g.over {
			s.respawn -= dt
			if s.respawn <= 0 {
				s.pos = {WIDTH / 2, HEIGHT / 2}
				s.vel = {}
				s.ang = -math.PI / 2
				s.alive = true
				s.inv = 2.0
			}
		}
		return
	}

	if rl.IsKeyDown(.LEFT) || rl.IsKeyDown(.A) do s.ang -= TURN * dt
	if rl.IsKeyDown(.RIGHT) || rl.IsKeyDown(.D) do s.ang += TURN * dt
	if rl.IsKeyDown(.UP) || rl.IsKeyDown(.W) {
		s.vel += rot({1, 0}, s.ang) * THRUST * dt
	}
	s.vel *= DRAG
	s.pos = wrap(s.pos + s.vel * dt)

	s.shielding =
		g.has_shield && (rl.IsKeyDown(.LEFT_SHIFT) || rl.IsKeyDown(.RIGHT_SHIFT)) && s.shield > 0
	if s.shielding {
		s.shield = max(0, s.shield - SHIELD_DRAIN * dt)
	} else {
		s.shield = min(1, s.shield + SHIELD_REGEN * dt)
	}

	if rl.IsKeyPressed(.SPACE) do fire(g)
}

update_world :: proc(g: ^Game, dt: f32) {
	for &b in g.bullets {
		b.pos = wrap(b.pos + b.vel * dt)
		b.life -= dt
	}
	for &a in g.asteroids {
		a.pos = wrap(a.pos + a.vel * dt)
		a.ang += a.spin * dt
	}
	for &d in g.debris {
		d.pos += d.vel * dt
		d.vel *= 0.99
		d.ang += d.spin * dt
		d.life -= dt
	}
}

update_pickups :: proc(g: ^Game, dt: f32) {
	for &p in g.pickups {
		if g.ship.alive {
			to := g.ship.pos - p.pos
			if linalg.length(to) < 135 do p.vel += safe_norm(to) * (300 * dt) // magnet
		}
		p.pos = wrap(p.pos + p.vel * dt)
		p.vel *= 0.97
		p.ang += p.spin * dt
		p.life -= dt
	}
	if g.ship.alive {
		for &p in g.pickups {
			if p.life > 0 && linalg.length(p.pos - g.ship.pos) < 20 {
				p.life = 0
				g.materials += 1
			}
		}
	}
}

// circle shield: deflect, or (if destructive) shatter rocks on contact
apply_shield :: proc(g: ^Game) {
	if !g.ship.alive || !g.ship.shielding do return
	c := g.ship.pos
	for i in 0 ..< len(g.asteroids) {
		a := g.asteroids[i]
		if a.dead do continue
		if !circle.has_point(a.pos, c, SHIELD_RADIUS + a.radius * 0.5) do continue

		if g.destructive {
			g.asteroids[i].dead = true
			destroy_asteroid(g, a, world_shape(a))
			g.ship.shield = max(0, g.ship.shield - SHIELD_DESTROY_COST)
		} else {
			n := safe_norm(a.pos - c)
			contact := circle.project_to_boundary(a.pos, c, SHIELD_RADIUS)
			g.asteroids[i].vel = reflect(a.vel, n)
			g.asteroids[i].pos = contact + n * a.radius
			g.ship.shield = max(0, g.ship.shield - SHIELD_DEFLECT_COST)
		}
		if g.ship.shield <= 0 {
			g.ship.shielding = false
			break
		}
	}
}

// aim-lock: nearest forward asteroid within its radius of the aim line
update_aim :: proc(g: ^Game) {
	g.locked = -1
	if !g.ship.alive do return
	dir := rot({1, 0}, g.ship.ang)
	nose := g.ship.pos + dir * 18
	aim := [2][2]f32{nose, nose + dir * 1300}
	best := f32(1e9)
	for a, i in g.asteroids {
		fwd := linalg.dot(a.pos - nose, dir)
		if fwd <= 0 do continue
		if line.distance_infinite(aim, a.pos) <= a.radius && fwd < best {
			best = fwd
			g.locked = i
		}
	}
}

collide :: proc(g: ^Game) {
	for &b in g.bullets {
		if b.dead do continue
		for i in 0 ..< len(g.asteroids) {
			a := g.asteroids[i]
			if a.dead do continue
			if linalg.length(b.pos - a.pos) > a.radius do continue
			world := world_shape(a)
			if polygon.has_point(world, b.pos) {
				b.dead = true
				g.asteroids[i].dead = true
				destroy_asteroid(g, a, world)
				break
			}
		}
	}

	if g.ship.alive && g.ship.inv <= 0 && !g.over {
		sv := ship_world(g.ship)
		for i in 0 ..< len(g.asteroids) {
			a := g.asteroids[i]
			if a.dead do continue
			if g.ship.shielding &&
			   circle.has_point(a.pos, g.ship.pos, SHIELD_RADIUS + a.radius) {
				continue
			}
			if linalg.length(g.ship.pos - a.pos) > a.radius + 18 do continue
			world := world_shape(a)
			hit := false
			for v in sv {
				if polygon.has_point(world, v) {
					hit = true
					break
				}
			}
			if !hit {
				for wv in world {
					if triangle.contains_point(sv, wv) {
						hit = true
						break
					}
				}
			}
			if hit {
				kill_ship(g)
				break
			}
		}
	}
}

compact :: proc(g: ^Game) {
	keep := 0
	for i in 0 ..< len(g.asteroids) {
		if g.asteroids[i].dead {
			delete(g.asteroids[i].shape)
		} else {
			g.asteroids[keep] = g.asteroids[i]
			keep += 1
		}
	}
	resize(&g.asteroids, keep)
	for ch in g.pending do append(&g.asteroids, ch)
	clear(&g.pending)

	keep = 0
	for i in 0 ..< len(g.bullets) {
		if !g.bullets[i].dead && g.bullets[i].life > 0 {
			g.bullets[keep] = g.bullets[i]
			keep += 1
		}
	}
	resize(&g.bullets, keep)

	keep = 0
	for i in 0 ..< len(g.debris) {
		if g.debris[i].life > 0 {
			g.debris[keep] = g.debris[i]
			keep += 1
		}
	}
	resize(&g.debris, keep)

	keep = 0
	for i in 0 ..< len(g.pickups) {
		if g.pickups[i].life > 0 {
			g.pickups[keep] = g.pickups[i]
			keep += 1
		}
	}
	resize(&g.pickups, keep)
}

update_play :: proc(g: ^Game, dt: f32) {
	if rl.IsKeyPressed(.G) do g.debug = !g.debug
	update_ship(g, dt)
	update_world(g, dt)
	apply_shield(g)
	update_aim(g)
	update_pickups(g, dt)
	collide(g)
	compact(g)

	g.stage_time -= dt
	g.spawn_timer -= dt
	if g.spawn_timer <= 0 {
		g.spawn_timer = 4
		if len(g.asteroids) < 28 do spawn_wave(g, 1 + g.stage)
	}

	if g.over {
		g.mode = .GameOver
	} else if g.stage_time <= 0 {
		end_stage(g)
	}
}

total_mass :: proc(g: ^Game) -> f32 {
	m: f32 = 0
	for a in g.asteroids {
		m += abs(polygon.area(world_shape(a)))
	}
	return m
}

// --- shop layout (shared by update + draw) ----------------------------------

shop_layout :: proc(
) -> (
	panel: [2][2]f32,
	header: [2][2]f32,
	cards: [4][2][2]f32,
	cont: [2][2]f32,
) {
	screen := [2][2]f32{{0, 0}, {WIDTH, HEIGHT}}
	panel = rect.grow(screen, [2]f32{-150, -120})
	inner := rect.grow(panel, [2]f32{-22, -22})
	rest: [2][2]f32
	header, rest = rect.cut(inner, .Top, 60)
	foot, body := rect.cut(rest, .Bottom, 64)
	cols := rect.divide_x(body, 4, 16, context.temp_allocator)
	for i in 0 ..< 4 {
		cards[i] = rect.grow(cols[i], [2]f32{-4, -8})
	}
	cx := (foot[0].x + foot[1].x) / 2
	cont = {{cx - 150, foot[0].y + 8}, {cx + 150, foot[1].y - 8}}
	return
}

update_shop :: proc(g: ^Game) {
	_, _, cards, cont := shop_layout()

	if rl.IsKeyPressed(.ONE) do try_buy(g, 0)
	if rl.IsKeyPressed(.TWO) do try_buy(g, 1)
	if rl.IsKeyPressed(.THREE) do try_buy(g, 2)
	if rl.IsKeyPressed(.FOUR) do try_buy(g, 3)
	if rl.IsKeyPressed(.ENTER) || rl.IsKeyPressed(.KP_ENTER) do begin_stage(g)

	if rl.IsMouseButtonPressed(.LEFT) {
		mp := rl.GetMousePosition()
		p := [2]f32{mp.x, mp.y}
		for i in 0 ..< 4 {
			if rect.has_point(cards[i], p) do try_buy(g, i)
		}
		if rect.has_point(cont, p) do begin_stage(g)
	}
}

// --- rendering --------------------------------------------------------------

draw_outline :: proc(pts: [][2]f32, thick: f32, col: rl.Color) {
	n := len(pts)
	for i in 0 ..< n {
		rl.DrawLineEx(v2(pts[i]), v2(pts[(i + 1) % n]), thick, col)
	}
}

draw_crosshair :: proc(g: ^Game) {
	if g.locked < 0 || g.locked >= len(g.asteroids) do return
	ap := g.asteroids[g.locked].pos
	r := g.asteroids[g.locked].radius + 8
	col := rl.Fade(rl.RED, 0.85)
	rl.DrawCircleLines(i32(ap.x), i32(ap.y), r, col)
	rl.DrawLineEx(v2(ap - {r + 7, 0}), v2(ap - {r - 5, 0}), 2, col)
	rl.DrawLineEx(v2(ap + {r - 5, 0}), v2(ap + {r + 7, 0}), 2, col)
	rl.DrawLineEx(v2(ap - {0, r + 7}), v2(ap - {0, r - 5}), 2, col)
	rl.DrawLineEx(v2(ap + {0, r - 5}), v2(ap + {0, r + 7}), 2, col)
}

draw_pickups :: proc(g: ^Game) {
	for p in g.pickups {
		s := 6 + 1.5 * math.sin(f32(rl.GetTime()) * 6 + p.ang)
		alpha := u8(255 * clamp(p.life / 2, 0, 1))
		col := rl.Color{90, 220, 200, alpha}
		a := p.pos + rot({s, 0}, p.ang)
		b := p.pos + rot({0, s}, p.ang)
		c := p.pos + rot({-s, 0}, p.ang)
		d := p.pos + rot({0, -s}, p.ang)
		rl.DrawLineEx(v2(a), v2(b), 2, col)
		rl.DrawLineEx(v2(b), v2(c), 2, col)
		rl.DrawLineEx(v2(c), v2(d), 2, col)
		rl.DrawLineEx(v2(d), v2(a), 2, col)
	}
}

draw_debug :: proc(g: ^Game) {
	boxes := make([][2][2]f32, len(g.asteroids), context.temp_allocator)
	for a, i in g.asteroids {
		w := world_shape(a)
		boxes[i] = polygon.min_max(w)
		col := rl.GREEN if polygon.is_convex(w) else rl.YELLOW
		rl.DrawRectangleLinesEx(to_rec(boxes[i]), 1, rl.Fade(col, 0.6))
	}
	for i in 0 ..< len(boxes) {
		for j in i + 1 ..< len(boxes) {
			if rect.intersects(boxes[i], boxes[j]) {
				rl.DrawRectangleLinesEx(to_rec(boxes[i]), 2, rl.Fade(rl.RED, 0.9))
				rl.DrawRectangleLinesEx(to_rec(boxes[j]), 2, rl.Fade(rl.RED, 0.9))
			}
		}
	}
	sv := ship_world(g.ship)
	lo := sv[0]
	hi := sv[0]
	for k in 1 ..< 3 {
		lo = linalg.min(lo, sv[k])
		hi = linalg.max(hi, sv[k])
	}
	rl.DrawRectangleLinesEx(to_rec({lo, hi}), 1, rl.Fade(rl.SKYBLUE, 0.8))
	rl.DrawText(
		fmt.ctprintf("rocks %d   mass %.0f", len(g.asteroids), total_mass(g)),
		18,
		HEIGHT - 52,
		16,
		rl.YELLOW,
	)
}

draw_hud :: proc(g: ^Game) {
	screen := [2][2]f32{{0, 0}, {WIDTH, HEIGHT}}
	bar, _ := rect.cut(screen, .Top, 50)
	rl.DrawRectangleRec(to_rec(bar), {16, 18, 26, 235})
	rl.DrawLineEx(v2({0, 50}), v2({WIDTH, 50}), 1, {60, 66, 84, 255})

	cells := rect.divide_x(bar, 3, 0, context.temp_allocator)

	score := fmt.ctprintf("SCORE  %d", g.score)
	rl.DrawText(score, i32(cells[0][0].x) + 18, 14, 24, rl.RAYWHITE)

	mid := cells[1]
	timer := fmt.ctprintf("STAGE %d    %.0fs", g.stage, max(0, g.stage_time))
	tx := i32((mid[0].x + mid[1].x) / 2) - rl.MeasureText(timer, 24) / 2
	rl.DrawText(timer, tx, 14, 24, rl.SKYBLUE)

	lives := fmt.ctprintf("LIVES  %d", g.lives)
	lx := i32(cells[2][1].x) - rl.MeasureText(lives, 24) - 18
	rl.DrawText(lives, lx, 14, 24, rl.RAYWHITE)

	// second row: materials + gun level
	rl.DrawText(fmt.ctprintf("MATERIALS  %d", g.materials), 18, 58, 18, rl.LIME)
	rl.DrawText(fmt.ctprintf("GUN  x%d", 1 + g.gun_level), 210, 58, 18, {150, 156, 176, 255})

	// shield gauge (only once owned)
	if g.has_shield {
		track := [2][2]f32{{330, 58}, {520, 72}}
		rl.DrawRectangleRec(to_rec(track), {30, 34, 44, 255})
		if g.ship.shield > 0 {
			filled, _ := rect.cut_frac(track, .Left, g.ship.shield)
			base := rl.Color{220, 110, 90, 255} if g.destructive else rl.Color{96, 150, 184, 255}
			col := rl.SKYBLUE if g.ship.shielding else base
			rl.DrawRectangleRec(to_rec(filled), col)
		}
		label: cstring = "SHIELD"
		if g.destructive do label = "SHIELD *"
		rl.DrawText(label, 528, 56, 16, rl.GRAY)
	}
}

draw_shop :: proc(g: ^Game) {
	rl.DrawRectangle(0, 0, WIDTH, HEIGHT, {0, 0, 0, 185})
	panel, header, cards, cont := shop_layout()

	rl.DrawRectangleRec(to_rec(panel), {20, 22, 30, 255})
	rl.DrawRectangleLinesEx(to_rec(panel), 2, rl.SKYBLUE)

	rl.DrawText("UPGRADE SHOP", i32(header[0].x) + 4, i32(header[0].y) + 8, 32, rl.RAYWHITE)
	bal := fmt.ctprintf("MATERIALS  %d", g.materials)
	rl.DrawText(bal, i32(header[1].x) - rl.MeasureText(bal, 24) - 4, i32(header[0].y) + 14, 24, rl.LIME)

	titles := [4]cstring{"GUN +1 SHOT", "ENERGY SHIELD", "DESTRUCT SHIELD", "EXTRA LIFE"}
	descs := [4]cstring {
		"+1 conical barrel",
		"SHIFT to deflect",
		"shatters rocks",
		"one more chance",
	}

	mp := rl.GetMousePosition()
	mpos := [2]f32{mp.x, mp.y}

	for i in 0 ..< 4 {
		status, buyable := card_info(g, i)
		c := cards[i]
		bg := rl.Color{34, 38, 50, 255}
		if rect.has_point(c, mpos) do bg = {46, 52, 68, 255}
		rl.DrawRectangleRec(to_rec(c), bg)
		rl.DrawRectangleLinesEx(to_rec(c), 2, rl.LIME if buyable else rl.Color{80, 86, 104, 255})

		x := i32(c[0].x) + 12
		rl.DrawText(fmt.ctprintf("[%d]", i + 1), x, i32(c[0].y) + 12, 22, rl.SKYBLUE)
		rl.DrawText(titles[i], x, i32(c[0].y) + 44, 19, rl.RAYWHITE)
		rl.DrawText(descs[i], x, i32(c[0].y) + 78, 14, {150, 156, 176, 255})
		rl.DrawText(status, x, i32(c[1].y) - 34, 18, rl.LIME if buyable else rl.GRAY)
	}

	if rect.has_point(cont, mpos) {
		rl.DrawRectangleRec(to_rec(cont), {40, 70, 90, 255})
	} else {
		rl.DrawRectangleRec(to_rec(cont), {30, 40, 52, 255})
	}
	rl.DrawRectangleLinesEx(to_rec(cont), 2, rl.SKYBLUE)
	ct: cstring = "NEXT STAGE  [ENTER]"
	rl.DrawText(
		ct,
		i32((cont[0].x + cont[1].x) / 2) - rl.MeasureText(ct, 22) / 2,
		i32((cont[0].y + cont[1].y) / 2) - 11,
		22,
		rl.RAYWHITE,
	)
}

draw_game_over :: proc(g: ^Game) {
	rl.DrawRectangle(0, 0, WIDTH, HEIGHT, {0, 0, 0, 160})
	title: cstring = "GAME OVER"
	rl.DrawText(title, WIDTH / 2 - rl.MeasureText(title, 64) / 2, HEIGHT / 2 - 80, 64, rl.RAYWHITE)
	sub := fmt.ctprintf("REACHED STAGE %d   -   SCORE %d", g.stage, g.score)
	rl.DrawText(sub, WIDTH / 2 - rl.MeasureText(sub, 24) / 2, HEIGHT / 2 + 4, 24, rl.LIGHTGRAY)
	hint: cstring = "PRESS  R  TO  RESTART"
	rl.DrawText(hint, WIDTH / 2 - rl.MeasureText(hint, 22) / 2, HEIGHT / 2 + 48, 22, rl.GRAY)
}

draw :: proc(g: ^Game) {
	rl.BeginDrawing()
	rl.ClearBackground({10, 11, 16, 255})

	if g.mode == .Playing do draw_crosshair(g)

	for d in g.debris {
		w: [3][2]f32
		for j in 0 ..< 3 {
			w[j] = d.pos + rot(d.local[j], d.ang)
		}
		a := u8(clamp(d.life / d.max_life, 0, 1) * 220)
		col := rl.Color{220, 220, 230, a}
		rl.DrawLineEx(v2(w[0]), v2(w[1]), 1.5, col)
		rl.DrawLineEx(v2(w[1]), v2(w[2]), 1.5, col)
		rl.DrawLineEx(v2(w[2]), v2(w[0]), 1.5, col)
	}

	for a in g.asteroids {
		draw_outline(world_shape(a), 2, kind_color(a.kind))
	}

	draw_pickups(g)

	if g.debug && g.mode == .Playing do draw_debug(g)

	for b in g.bullets {
		rl.DrawCircleV(v2(b.pos), 2.5, rl.RAYWHITE)
	}

	s := g.ship
	if s.alive && g.mode != .Shop {
		show := s.inv <= 0 || int(rl.GetTime() * 18) % 2 == 0
		if show {
			sv := ship_world(s)
			col := rl.SKYBLUE if s.inv <= 0 else rl.Fade(rl.SKYBLUE, 0.6)
			draw_outline(sv[:], 2, col)
		}
		if s.shielding {
			pulse := u8(120 + 80 * math.sin(f32(rl.GetTime()) * 12))
			ring := rl.Color{230, 120, 100, pulse} if g.destructive else rl.Color{90, 170, 230, pulse}
			rl.DrawCircleLines(i32(s.pos.x), i32(s.pos.y), SHIELD_RADIUS, ring)
		}
	}

	draw_hud(g)

	if g.mode == .Playing {
		rl.DrawText(
			"arrows/WASD fly   SPACE shoot   SHIFT shield   G debug   R restart",
			18,
			HEIGHT - 28,
			16,
			{90, 96, 116, 255},
		)
	}

	if g.mode == .Shop do draw_shop(g)
	if g.mode == .GameOver do draw_game_over(g)

	rl.EndDrawing()
}

screenshots: int = 0
capture_screenshot :: proc() {
	// Dynamically format the filename using the temp_allocator
	filename_str := fmt.tprintf("screenshot_%d.jpg", screenshots)
	
	// Convert the Odin string to a null-terminated cstring for Raylib
	filename_c := strings.clone_to_cstring(filename_str, context.temp_allocator)
	
	// Raylib saves it out safely based on the extension provided
	rl.TakeScreenshot(filename_c)
	
	screenshots += 1
}

// --- entry point ------------------------------------------------------------

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: asteroids")
	rl.SetTargetFPS(60)

	g: Game
	reset_game(&g)

	for !rl.WindowShouldClose() {
		dt := rl.GetFrameTime()

		if rl.IsKeyPressed(.R) do reset_game(&g)
		if rl.IsKeyPressed(.S) do capture_screenshot()

		switch g.mode {
		case .Playing:
			update_play(&g, dt)
		case .Shop:
			update_shop(&g)
		case .GameOver:
		// waiting for R
		}

		draw(&g)
		free_all(context.temp_allocator)
	}

	for a in g.asteroids do delete(a.shape)
	delete(g.asteroids)
	delete(g.bullets)
	delete(g.debris)
	delete(g.pickups)
	delete(g.pending)
	rl.CloseWindow()
}
