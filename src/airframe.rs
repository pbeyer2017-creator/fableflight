//! The toolkit aircraft are modelled with: procedural meshes built from
//! analytic surfaces (lofted sections, NACA-section lifting surfaces, lathes,
//! tubes, extrusions) so that panel seams, rivet rows, fasteners, lights and
//! decals can be placed exactly on the skin; the pieces a model is made of,
//! how each one moves and what it is to the damage model; and the ambient
//! occlusion bake. The aircraft themselves live in `aircraft/`; their
//! materials are drawn by `plane.wgsl`.
//!
//! Body axes: +X right, +Y up, -Z forward. Every piece keeps its vertices in
//! the body frame at rest, so the shader can paint the livery and wear from
//! the body-space position whatever the piece is doing.
use crate::render::PartVertex;
use glam::{Mat4, Quat, Vec2, Vec3};
use std::f32::consts::{FRAC_PI_2, TAU};

// ---------------------------------------------------------------- materials

/// Material ids (vertex `mat.x`); keep in step with plane.wgsl.
pub mod mat {
    pub const PAINT: f32 = 0.0; // glossy paint, colour from the vertex
    pub const LIVERY: f32 = 1.0; // glossy exterior paint, colour from the paint scheme
    pub const ALU: f32 = 2.0; // bare aluminium
    pub const STEEL: f32 = 3.0; // machined / chromed steel
    pub const RUBBER: f32 = 4.0;
    pub const PLASTIC: f32 = 5.0; // textured interior plastic
    pub const FABRIC: f32 = 6.0; // upholstery, carpet, headliner
    pub const GLASS: f32 = 7.0;
    pub const LAMP: f32 = 8.0; // light lens; extra = which light
    pub const GAUGE: f32 = 9.0; // instrument face; extra = which instrument
    pub const BLACK: f32 = 10.0; // matte black finish
    pub const EXHAUST: f32 = 11.0;
    pub const SCREW: f32 = 12.0; // screw head, uv = local frame
    pub const PROP: f32 = 13.0;
    pub const DISPLAY: f32 = 14.0; // radio readout; extra = which display
    pub const SATIN: f32 = 15.0; // satin paint / primer
    pub const PROPDISC: f32 = 16.0; // the blur of a spinning propeller
}
/// Surface patterns (vertex `mat.y`); keep in step with plane.wgsl.
pub mod pat {
    pub const FUSE: f32 = 1.0; // fuselage skin: stations, stringers, doors, windows, wells
    pub const WING: f32 = 2.0;
    pub const CTRL: f32 = 3.0; // control surface with stiffening beads
    pub const COWL: f32 = 4.0;
    pub const TAIL: f32 = 5.0;
    pub const TREAD: f32 = 6.0;
    pub const LINING: f32 = 7.0; // cabin side walls and headliner (windows cut out)
    pub const SEAT: f32 = 8.0; // pleated upholstery
    pub const CARPET: f32 = 9.0;
    pub const PANEL: f32 = 10.0; // crinkle-finish instrument panel
    pub const PLACARD: f32 = 11.0; // small printed label
    pub const KNURL: f32 = 12.0;
    pub const NACELLE: f32 = 13.0; // a jet engine's cowling: seams and fasteners, no openings
    pub const LEGEND: f32 = 14.0; // printed panel legend, button face or scale: extra = style + 16 * string (b747::deck)
    pub const FLEECE: f32 = 15.0; // sheepskin seat cover
    pub const SPINNER: f32 = 16.0; // a jet engine's spinner (747: the swirl)
    // 17 is the Hornet's painted skin (fa18.py); the rest are its cockpit (fa18/pit.rs):
    pub const PIT: f32 = 18.0; // painted cockpit structure: orange peel, scuffs, grime
    pub const MESH: f32 = 19.0; // open-weave seat cushion fabric (uv in metres)
    pub const WEBBING: f32 = 20.0; // harness webbing: uv = (across, along) in metres
    pub const NONSLIP: f32 = 21.0; // gritty floor coating
    pub const STRIPE: f32 = 22.0; // yellow and black hazard stripes: extra = pitch (m); negative: rings along uv.y
    pub const HOSE: f32 = 23.0; // corrugated hose: ribs along uv.y
}
/// Light ids (vertex `mat.w` on LAMP); keep in step with plane.wgsl.
pub mod lamp {
    pub const NAV_RED: f32 = 0.0;
    pub const NAV_GREEN: f32 = 1.0;
    pub const TAIL: f32 = 2.0;
    pub const STROBE: f32 = 3.0;
    pub const BEACON: f32 = 4.0;
    pub const LANDING: f32 = 5.0;
    pub const GEAR_DOWN: f32 = 6.0;
    pub const GEAR_UP: f32 = 7.0;
    pub const ANNUNC: f32 = 8.0;
    #[allow(dead_code)] // no aircraft has one yet.
    pub const FORMATION: f32 = 9.0; // electroluminescent formation light strip (a fighter's)
}

#[derive(Clone, Copy)]
pub struct Mat {
    pub id: f32,
    pub pat: f32,
    pub col: Vec3,
    pub extra: f32,
}
pub const fn mt(id: f32, r: f32, g: f32, b: f32) -> Mat {
    Mat { id, pat: 0.0, col: Vec3::new(r, g, b), extra: 0.0 }
}
impl Mat {
    pub const fn pat(mut self, p: f32) -> Self {
        self.pat = p;
        self
    }
    pub const fn x(mut self, e: f32) -> Self {
        self.extra = e;
        self
    }
}

// Common finishes (linear colour).
pub const WHITE: Mat = mt(mat::LIVERY, 0.80, 0.80, 0.78);
pub const PAINT_WHITE: Mat = mt(mat::PAINT, 0.80, 0.80, 0.78);
pub const ALU: Mat = mt(mat::ALU, 0.62, 0.63, 0.65);
pub const STEEL: Mat = mt(mat::STEEL, 0.55, 0.56, 0.58);
pub const CHROME: Mat = mt(mat::STEEL, 0.85, 0.86, 0.88).x(1.0);
pub const RUBBER: Mat = mt(mat::RUBBER, 0.018, 0.018, 0.02);
pub const BLACK: Mat = mt(mat::BLACK, 0.025, 0.025, 0.027);
pub const PRIMER: Mat = mt(mat::SATIN, 0.20, 0.23, 0.19);
pub const GREY: Mat = mt(mat::SATIN, 0.18, 0.18, 0.18);
pub const SCREW_STEEL: Mat = mt(mat::SCREW, 0.55, 0.56, 0.58);
pub const SCREW_PAINT: Mat = mt(mat::SCREW, 0.78, 0.78, 0.76).x(1.0);
pub const SCREW_BLACK: Mat = mt(mat::SCREW, 0.03, 0.03, 0.03).x(2.0);

// ---------------------------------------------------------------- mesh building

#[derive(Default, Clone)]
pub struct Mesh {
    pub v: Vec<PartVertex>,
    pub i: Vec<u32>,
}

pub type SP = (Vec3, [f32; 2], f32);

impl Mesh {
    pub fn push(&mut self, p: Vec3, n: Vec3, uv: [f32; 2], m: &Mat, wear: f32) -> u32 {
        self.v.push(PartVertex { pos: p.into(), nrm: n.normalize_or_zero().into(), col: [m.col.x, m.col.y, m.col.z, 1.0], uv, mat: [m.id, m.pat, wear, m.extra] });
        (self.v.len() - 1) as u32
    }
    pub fn append(&mut self, o: &Mesh) {
        let b = self.v.len() as u32;
        self.v.extend_from_slice(&o.v);
        self.i.extend(o.i.iter().map(|i| i + b));
    }
    /// Move everything from vertex `from` on by `t` (rigid).
    pub fn transform(&mut self, from: usize, t: Mat4) {
        for v in &mut self.v[from..] {
            v.pos = t.transform_point3(v.pos.into()).into();
            v.nrm = t.transform_vector3(v.nrm.into()).normalize_or_zero().into();
        }
    }
    pub fn mirror_x(&self) -> Mesh {
        let mut m = self.clone();
        for v in &mut m.v {
            v.pos[0] = -v.pos[0];
            v.nrm[0] = -v.nrm[0];
        }
        m
    }

    /// A grid of precomputed points (rows x cols) with normals from the grid
    /// itself; `wrap` joins the last column back to the first.
    pub fn grid(&mut self, m: &Mat, rows: &[Vec<SP>], wrap: bool, flip: bool) {
        let nr = rows.len();
        let nc = rows[0].len();
        if nr < 2 || nc < 2 {
            return;
        }
        let base = self.v.len() as u32;
        let p = |i: usize, j: usize| rows[i][j].0;
        for i in 0..nr {
            for j in 0..nc {
                let (a, b) = (i.saturating_sub(1), (i + 1).min(nr - 1));
                let (c, d) = if wrap { ((j + nc - 2) % (nc - 1), (j + 1) % (nc - 1)) } else { (j.saturating_sub(1), (j + 1).min(nc - 1)) };
                let mut n = (p(b, j) - p(a, j)).cross(p(i, d) - p(i, c));
                // Degenerate (a pole or a collapsed row): borrow from the neighbours.
                let mut k = 1;
                while n.length_squared() < 1e-14 && k < 4 {
                    let i2 = if i + k < nr { i + k } else { i.saturating_sub(k) };
                    let (a2, b2) = (i2.saturating_sub(1), (i2 + 1).min(nr - 1));
                    n = (p(b2, j) - p(a2, j)).cross(p(i2, d) - p(i2, c));
                    if n.length_squared() < 1e-14 {
                        n = (p(b, j) - p(a, j)).cross(p(i2, d) - p(i2, c));
                    }
                    k += 1;
                }
                if flip {
                    n = -n;
                }
                let (pos, uv, w) = rows[i][j];
                self.push(pos, n, uv, m, w);
            }
        }
        for i in 0..nr as u32 - 1 {
            for j in 0..nc as u32 - 1 {
                let a = base + i * nc as u32 + j;
                let b = a + nc as u32;
                self.i.extend([a, b, a + 1, a + 1, b, b + 1]);
            }
        }
    }
    /// Parametric patch over [0,1]²: `f(u, v)` gives position, surface coordinates and wear.
    pub fn patch(&mut self, m: &Mat, nu: usize, nv: usize, flip: bool, f: impl Fn(f32, f32) -> SP) {
        let rows: Vec<Vec<SP>> = (0..=nu).map(|i| (0..=nv).map(|j| f(i as f32 / nu as f32, j as f32 / nv as f32)).collect()).collect();
        self.grid(m, &rows, false, flip);
    }
    pub fn quad(&mut self, m: &Mat, p: [Vec3; 4]) {
        let n = (p[1] - p[0]).cross(p[2] - p[0]).normalize_or_zero();
        let uvs = [[0.0, 0.0], [(p[1] - p[0]).length(), 0.0], [(p[1] - p[0]).length(), (p[2] - p[1]).length()], [0.0, (p[3] - p[0]).length()]];
        let b = self.v.len() as u32;
        for k in 0..4 {
            self.push(p[k], n, uvs[k], m, 0.0);
        }
        self.i.extend([b, b + 1, b + 2, b, b + 2, b + 3]);
    }
    /// Flat fan from a centre point to a closed ring.
    pub fn fan(&mut self, m: &Mat, c: Vec3, ring: &[Vec3], n: Vec3) {
        let b = self.push(c, n, [0.0, 0.0], m, 0.0);
        for (k, p) in ring.iter().enumerate() {
            let d = *p - c;
            self.push(*p, n, [d.x + d.z, d.y], m, 0.0);
            if k > 0 {
                self.i.extend([b, b + k as u32, b + k as u32 + 1]);
            }
        }
        let k = ring.len() as u32;
        self.i.extend([b, b + k, b + 1]);
    }

    /// Box with rounded edges, `h` half extents, `r` edge radius.
    pub fn rbox(&mut self, m: &Mat, c: Vec3, h: Vec3, r: f32, rot: Quat) {
        let start = self.v.len();
        let r = r.min(h.min_element() * 0.999).max(0.0);
        let steps = |hh: f32| -> Vec<f32> {
            let mut v: Vec<f32> = Vec::new();
            if r > 0.0 {
                for i in 0..=3 {
                    v.push(-hh + r * (1.0 - (i as f32 / 3.0 * FRAC_PI_2).cos()));
                }
            } else {
                v.push(-hh);
            }
            let mut w: Vec<f32> = v.iter().rev().map(|x| -x).collect();
            if (w[0] - v[v.len() - 1]).abs() < 1e-5 {
                w.remove(0);
            }
            v.extend(w);
            v
        };
        for a in 0..3 {
            for s in [-1.0f32, 1.0] {
                let (b, cc) = ((a + 1) % 3, (a + 2) % 3);
                let (sb, sc) = (steps(h[b]), steps(h[cc]));
                let mut rows = Vec::new();
                for &x in &sb {
                    let mut row = Vec::new();
                    for &y in &sc {
                        let mut q = Vec3::ZERO;
                        q[a] = s * h[a];
                        q[b] = x;
                        q[cc] = y;
                        let inner = q.clamp(-(h - Vec3::splat(r)), h - Vec3::splat(r));
                        let d = q - inner;
                        let n = if d.length_squared() > 1e-12 { d.normalize() } else { let mut n = Vec3::ZERO; n[a] = s; n };
                        row.push((inner + n * r, [x, y], 0.0, n));
                    }
                    rows.push(row);
                }
                let base = self.v.len() as u32;
                let nc = sc.len() as u32;
                for row in &rows {
                    for (p, uv, w, n) in row {
                        self.push(*p, *n, *uv, m, *w);
                    }
                }
                for i in 0..sb.len() as u32 - 1 {
                    for j in 0..nc - 1 {
                        let a0 = base + i * nc + j;
                        self.i.extend([a0, a0 + nc, a0 + 1, a0 + 1, a0 + nc, a0 + nc + 1]);
                    }
                }
            }
        }
        self.transform(start, Mat4::from_rotation_translation(rot, c));
    }

    /// Surface of revolution. `prof` is (radius, height) going up the outside;
    /// `smooth` shares normals between profile segments.
    pub fn lathe(&mut self, m: &Mat, c: Vec3, axis: Vec3, prof: &[[f32; 2]], segs: usize, smooth: bool) {
        let ax = axis.normalize();
        let u = ax.any_orthonormal_vector();
        let w = ax.cross(u);
        let sn: Vec<Vec2> = prof.windows(2).map(|p| Vec2::new(p[1][1] - p[0][1], -(p[1][0] - p[0][0])).normalize_or_zero()).collect();
        // (radius, height, profile normal, arc length)
        let mut rings: Vec<(f32, f32, Vec2, f32)> = Vec::new();
        let mut arc = 0.0;
        for (k, p) in prof.iter().enumerate() {
            if k > 0 {
                arc += Vec2::new(p[0] - prof[k - 1][0], p[1] - prof[k - 1][1]).length();
            }
            if smooth {
                let n = (sn.get(k.wrapping_sub(1)).copied().unwrap_or(Vec2::ZERO) + sn.get(k).copied().unwrap_or(Vec2::ZERO)).normalize_or_zero();
                rings.push((p[0], p[1], n, arc));
            } else {
                if k > 0 {
                    rings.push((p[0], p[1], sn[k - 1], arc));
                }
                if k + 1 < prof.len() {
                    rings.push((p[0], p[1], sn[k], arc));
                }
            }
        }
        let base = self.v.len() as u32;
        let nc = segs as u32 + 1;
        for &(r, hgt, n, a) in &rings {
            for k in 0..=segs {
                let t = k as f32 / segs as f32 * TAU;
                let d = u * t.cos() + w * t.sin();
                self.push(c + ax * hgt + d * r, d * n.x + ax * n.y, [t * r.max(0.004), a], m, 0.0);
            }
        }
        let pairs: Vec<u32> = if smooth { (0..rings.len() as u32 - 1).collect() } else { (0..rings.len() as u32 / 2).map(|k| k * 2).collect() };
        for i in pairs {
            for j in 0..nc - 1 {
                let a = base + i * nc + j;
                self.i.extend([a, a + nc, a + 1, a + 1, a + nc, a + nc + 1]);
            }
        }
    }

    /// Tube along a polyline with radius `r(t)`, t in [0, 1] along its length.
    pub fn tube(&mut self, m: &Mat, pts: &[Vec3], r: &dyn Fn(f32) -> f32, segs: usize, caps: bool) {
        let n = pts.len();
        let tang: Vec<Vec3> = (0..n).map(|i| (pts[(i + 1).min(n - 1)] - pts[i.saturating_sub(1)]).normalize()).collect();
        let mut len = vec![0.0f32; n];
        for i in 1..n {
            len[i] = len[i - 1] + (pts[i] - pts[i - 1]).length();
        }
        let total = len[n - 1].max(1e-6);
        let mut nrm = tang[0].any_orthonormal_vector();
        let mut rows = Vec::new();
        for i in 0..n {
            if i > 0 {
                nrm = (Quat::from_rotation_arc(tang[i - 1], tang[i]) * nrm).normalize();
            }
            let b = tang[i].cross(nrm);
            let rad = r(len[i] / total);
            let row: Vec<SP> = (0..=segs).map(|k| {
                let a = k as f32 / segs as f32 * TAU;
                (pts[i] + (nrm * a.cos() + b * a.sin()) * rad, [a * rad, len[i]], 0.0)
            }).collect();
            rows.push(row);
        }
        self.grid(m, &rows, true, true);
        if caps {
            for (i, d) in [(0, -1.0f32), (n - 1, 1.0)] {
                let ring: Vec<Vec3> = rows[i][..segs].iter().map(|s| s.0).collect();
                self.fan(m, pts[i], &ring, tang[i] * d);
            }
        }
    }

    /// Flat outline with holes, extruded by `depth` behind the plane of `ax`, `ay`.
    pub fn slab(&mut self, m: &Mat, outline: &[Vec2], holes: &[Vec<Vec2>], o: Vec3, ax: Vec3, ay: Vec3, depth: f32) {
        let az = ax.cross(ay).normalize();
        let mut flat: Vec<f64> = Vec::new();
        let mut hi = Vec::new();
        let rings: Vec<&[Vec2]> = std::iter::once(outline).chain(holes.iter().map(|h| h.as_slice())).collect();
        for (k, ring) in rings.iter().enumerate() {
            if k > 0 {
                hi.push(flat.len() / 2);
            }
            for p in ring.iter() {
                flat.extend([p.x as f64, p.y as f64]);
            }
        }
        let tris = earcutr::earcut(&flat, &hi, 2).unwrap_or_default();
        let pts: Vec<Vec2> = flat.chunks(2).map(|c| Vec2::new(c[0] as f32, c[1] as f32)).collect();
        for (off, n) in [(0.0, az), (-depth, -az)] {
            if depth == 0.0 && off != 0.0 {
                break;
            }
            let b = self.v.len() as u32;
            for p in &pts {
                self.push(o + ax * p.x + ay * p.y + az * off, n, [p.x, p.y], m, 0.0);
            }
            self.i.extend(tris.iter().map(|&t| b + t as u32));
        }
        if depth > 0.0 {
            for ring in rings {
                for k in 0..ring.len() {
                    let (p, q) = (ring[k], ring[(k + 1) % ring.len()]);
                    let (a, b) = (o + ax * p.x + ay * p.y, o + ax * q.x + ay * q.y);
                    self.quad(m, [a, b, b - az * depth, a - az * depth]);
                }
            }
        }
    }

    /// Disc facing `n`, with uv the local frame scaled to [-1, 1].
    pub fn disc(&mut self, m: &Mat, c: Vec3, n: Vec3, up: Vec3, r: f32, segs: usize) {
        let n = n.normalize();
        let y = (up - n * up.dot(n)).normalize();
        let x = y.cross(n);
        let b = self.push(c, n, [0.0, 0.0], m, 0.0);
        for k in 0..=segs {
            let a = k as f32 / segs as f32 * TAU;
            self.push(c + (x * a.cos() + y * a.sin()) * r, n, [a.cos(), a.sin()], m, 0.0);
            if k > 0 {
                self.i.extend([b, b + k as u32, b + k as u32 + 1]);
            }
        }
    }
    /// Low domed head (screw, rivet, knob) of radius `r`, height `h`; uv is a
    /// local frame turned by `spin` so slots do not all line up.
    pub fn dome(&mut self, m: &Mat, c: Vec3, n: Vec3, r: f32, h: f32, segs: usize, spin: f32) {
        let n = n.normalize();
        let x0 = n.any_orthonormal_vector();
        let y0 = n.cross(x0);
        let (x, y) = (x0 * spin.cos() + y0 * spin.sin(), -x0 * spin.sin() + y0 * spin.cos());
        let rings = [1.0f32, 0.92, 0.72, 0.42, 0.0];
        let base = self.v.len() as u32;
        for &f in &rings {
            let z = h * (1.0 - f * f).max(0.0).sqrt();
            for k in 0..=segs {
                let a = k as f32 / segs as f32 * TAU;
                let d = x * a.cos() + y * a.sin();
                let nn = d * (f / (r * r)) + n * (z / (h * h)).max(if f == 0.0 { 1.0 } else { 0.0 });
                self.push(c + d * (r * f) + n * z, nn, [f * a.cos(), f * a.sin()], m, 0.0);
            }
        }
        let nc = segs as u32 + 1;
        for i in 0..rings.len() as u32 - 1 {
            for j in 0..nc - 1 {
                let a = base + i * nc + j;
                self.i.extend([a, a + nc, a + 1, a + 1, a + nc, a + nc + 1]);
            }
        }
    }
    /// A screw head sitting on a surface.
    pub fn screw(&mut self, m: &Mat, c: Vec3, n: Vec3, r: f32, seed: f32) {
        self.dome(m, c, n, r, r * 0.32, 8, hash(seed) * TAU);
    }
    /// Hex bolt head with a washer.
    pub fn bolt(&mut self, m: &Mat, c: Vec3, n: Vec3, r: f32) {
        let n = n.normalize();
        self.lathe(m, c, n, &[[r * 1.5, 0.0], [r * 1.5, r * 0.15], [0.0, r * 0.15]], 12, false);
        self.lathe(m, c + n * r * 0.15, n, &[[r, 0.0], [r, r * 0.65], [r * 0.85, r * 0.75], [0.0, r * 0.75]], 6, false);
    }
}

pub fn hash(x: f32) -> f32 {
    ((x * 127.1 + 311.7).sin() * 43758.547).fract().abs()
}
pub fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

// ---------------------------------------------------------------- airfoils

/// NACA four-digit section: camber line height and half thickness at chord fraction x.
pub fn naca(x: f32, t: f32, m: f32) -> (f32, f32) {
    let x = x.clamp(0.0, 1.0);
    let yt = 5.0 * t * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
    let p = 0.4;
    let yc = if x < p { m / (p * p) * (2.0 * p * x - x * x) } else { m / ((1.0 - p) * (1.0 - p)) * (1.0 - 2.0 * p + 2.0 * p * x - x * x) };
    (yc, yt.max(0.0))
}

/// A lifting surface's section at span station s (metres along the span).
#[derive(Clone, Copy)]
pub struct Sect {
    pub le: Vec3,
    pub c: f32,
    pub cdir: Vec3,
    pub up: Vec3,
    /// thickness scale, for rounded tips
    pub tf: f32,
}
pub struct Foil {
    pub at: Box<dyn Fn(f32) -> Sect>,
    pub t: f32,
    pub camber: f32,
}
impl Foil {
    /// Point on the surface; `nose` >= 0 rounds the section off to start at
    /// that chord fraction (a control surface behind its hinge).
    pub fn pt(&self, s: f32, xc: f32, side: f32, nose: f32) -> Vec3 {
        let k = (self.at)(s);
        let (yc, mut yt) = naca(xc, self.t, self.camber);
        if nose >= 0.0 {
            yt *= 0.88 * ((xc - nose).max(0.0) / 0.07).min(1.0).sqrt();
        }
        k.le + k.cdir * (xc * k.c) + k.up * (k.c * (yc + side * yt * k.tf))
    }
    /// The skin between chord fractions x0(s) and x1(s), upper surface then lower.
    pub fn skin(&self, mesh: &mut Mesh, m: &Mat, s0: f32, s1: f32, ns: usize, x0: &dyn Fn(f32) -> f32, x1: &dyn Fn(f32) -> f32, nx: usize, nose: bool) {
        // Normals come from the grid's parameter directions: make sure they point out.
        let k = (self.at)((s0 + s1) * 0.5);
        let ds = (self.at)(s1).le - (self.at)(s0).le;
        let flip = ds.cross(-k.cdir).dot(k.up) < 0.0;
        let mut rows = Vec::new();
        for i in 0..=ns {
            let s = s0 + (s1 - s0) * i as f32 / ns as f32;
            let (a, b) = (x0(s), x1(s));
            let c = (self.at)(s).c;
            let mut row = Vec::new();
            for j in 0..=2 * nx {
                let (side, w) = if j <= nx { (1.0, 1.0 - j as f32 / nx as f32) } else { (-1.0, (j - nx) as f32 / nx as f32) };
                let xc = a + (b - a) * (1.0 - (w * FRAC_PI_2).cos());
                let wear = if nose { 0.2 * (1.0 - smoothstep(0.0, 0.08, xc - a)) } else { 1.0 - smoothstep(0.0, 0.1, xc) };
                row.push((self.pt(s, xc, side, if nose { a } else { -1.0 }), [s, side * xc * c], wear));
            }
            rows.push(row);
        }
        mesh.grid(m, &rows, false, flip);
    }
    /// The flat web closing an open skin at chord fraction x(s).
    pub fn web(&self, mesh: &mut Mesh, m: &Mat, s0: f32, s1: f32, ns: usize, x: &dyn Fn(f32) -> f32) {
        mesh.patch(m, ns, 2, false, |a, b| {
            let s = s0 + (s1 - s0) * a;
            let p = self.pt(s, x(s), 1.0, -1.0).lerp(self.pt(s, x(s), -1.0, -1.0), b);
            (p, [s, b], 0.0)
        });
    }
    /// Flat end cap of the section at s between chord fractions a and b.
    pub fn cap(&self, mesh: &mut Mesh, m: &Mat, s: f32, a: f32, b: f32, nose: bool) {
        let n = 16;
        let mut ring = Vec::new();
        for j in 0..=2 * n {
            let (side, w) = if j <= n { (1.0, 1.0 - j as f32 / n as f32) } else { (-1.0, (j - n) as f32 / n as f32) };
            let xc = a + (b - a) * (1.0 - (w * FRAC_PI_2).cos());
            ring.push(self.pt(s, xc, side, if nose { a } else { -1.0 }));
        }
        ring.pop();
        let k = (self.at)(s);
        let c = self.pt(s, (a + b) * 0.5, 0.0, -1.0);
        let span = k.cdir.cross(k.up).normalize();
        mesh.fan(m, c, &ring, span);
    }
}

// ---------------------------------------------------------------- cutouts

/// Exact signed distance to a polygon (negative inside).
pub fn sd_poly(p: Vec2, v: &[Vec2]) -> f32 {
    let mut d = (p - v[0]).length_squared();
    let mut s = 1.0f32;
    let n = v.len();
    let mut j = n - 1;
    for i in 0..n {
        let e = v[j] - v[i];
        let w = p - v[i];
        let b = w - e * (w.dot(e) / e.dot(e)).clamp(0.0, 1.0);
        d = d.min(b.length_squared());
        let c = [p.y >= v[i].y, p.y < v[j].y, e.x * w.y > e.y * w.x];
        if c.iter().all(|&x| x) || c.iter().all(|&x| !x) {
            s = -s;
        }
        j = i;
    }
    s * d.sqrt()
}
pub fn v2(a: [[f32; 2]; 4]) -> [Vec2; 4] {
    a.map(|p| Vec2::new(p[0], p[1]))
}
/// Does this surface pattern have the openings (windows, gear wells) cut out of it?
pub fn cut_by(pattern: f32) -> bool {
    pattern == pat::FUSE || pattern == pat::LINING || pattern == pat::COWL || pattern == crate::fgmodel::P_TEX
}
/// Only the skin has the gear wells cut in it (a cabin lining runs down to the floor above them).
pub fn well_cut_by(pattern: f32) -> bool {
    pattern == pat::FUSE || pattern == pat::COWL
}

// ---------------------------------------------------------------- moving pieces

/// What moves a piece: a value the aircraft works out each frame (see
/// `Plane::channels`). Control surfaces and levers are -1..1 or 0..1
/// commands, the gear and flaps their positions, wheels and propellers
/// their angles.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Input {
    /// the aileron on that side (-1 left, +1 right): +1 trailing edge down
    Aileron(f32),
    /// the elevator command, +1 trailing edge down (stick forward)
    Elevator,
    /// the rudder command, +1 trailing edge right
    Rudder,
    /// the smoothed stick and pedals, -1..1
    Pitch,
    Roll,
    Yaw,
    /// flap extension, 0 up .. 1 full travel; the flap lever 0..1 over its notches
    Flaps,
    FlapLever,
    /// leading-edge devices, 0 stowed .. 1 out
    Slats,
    /// spoiler panels on that side, 0 flush .. 1 fully up (speedbrake, roll and lift dump together)
    Spoiler(f32),
    /// gear position, 0 stowed .. 1 down and locked; the gear handle -1 up, +1 down
    Gear,
    GearHandle,
    /// how far the oleo on that leg is compressed (m), how far its wheels have turned (rad)
    Compress(u8),
    Wheel(u8),
    /// nose wheel steering angle (rad, + right)
    Steer,
    /// propeller or fan angle (rad)
    Prop,
    Throttle,
    /// the angle of attack the trim holds (rad)
    Trim,
    /// cockpit switches: the battery master (0 off, 1 on), the magnetos
    /// (0 off, 1 both, 1.5 starting), the mixture (0 cut-off .. 1 rich),
    /// the parking brake (0 off, 1 set)
    Master,
    Ignition,
    /// one of a pair of L/R switches on the shared fuel control or battery
    /// master (0 left, 1 right): 1 while that switch is on (`Plane::pair_off`)
    FuelControl(u8),
    Battery(u8),
    Mixture,
    ParkingBrake,
    Brake(f32),
    LandingLights,
    FuelValve,
    PropControl,
    CowlFlaps,
    /// thrust reversers, 0 stowed .. 1 deployed
    Reverser,
    /// the Mach number (inlet spikes follow it)
    Mach,
    /// a sliding canopy, 0 shut .. 1 fully open
    Canopy,
    /// the speed brake handle, 0 stowed .. 1 (`Plane::speedbrake`; the spoilers follow it in flight)
    Speedbrake,
    /// a jet's fan speed in percent (FlightGear's `engines/engine/n1`, `n2`); 0 for a piston engine
    N1,
}

/// How an input is shaped before it moves a piece.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Curve {
    Lin,
    /// 0 below a, 1 above b, smooth between
    Smooth(f32, f32),
    /// up over a..b, back down over c..d (a door that opens for the gear to pass and closes behind it)
    Pulse(f32, f32, f32, f32),
    /// 0 below a, 1 above b, straight between (a FlightGear table of two entries, held at its ends)
    Ramp(f32, f32),
}

/// `offset + gain * curve(input)`: radians for a hinge, metres for a slide.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Drive {
    pub input: Input,
    pub curve: Curve,
    pub gain: f32,
    pub offset: f32,
}

impl Drive {
    pub const NONE: Drive = Drive { input: Input::Gear, curve: Curve::Lin, gain: 0.0, offset: 0.0 };
    pub const fn new(input: Input, gain: f32) -> Self {
        Self { input, curve: Curve::Lin, gain, offset: 0.0 }
    }
    pub const fn smooth(mut self, a: f32, b: f32) -> Self {
        self.curve = Curve::Smooth(a, b);
        self
    }
    pub const fn pulse(mut self, a: f32, b: f32, c: f32, d: f32) -> Self {
        self.curve = Curve::Pulse(a, b, c, d);
        self
    }
    pub const fn offset(mut self, o: f32) -> Self {
        self.offset = o;
        self
    }
    /// The drive's value for an input value `x`.
    pub fn eval(&self, x: f32) -> f32 {
        if self.gain == 0.0 {
            return self.offset;
        }
        let y = match self.curve {
            Curve::Lin => x,
            Curve::Smooth(a, b) => smoothstep(a, b, x),
            Curve::Pulse(a, b, c, d) => smoothstep(a, b, x) * (1.0 - smoothstep(c, d, x)),
            Curve::Ramp(a, b) => ((x - a) / (b - a)).clamp(0.0, 1.0),
        };
        self.offset + self.gain * y
    }
}

/// A rigid motion: turned about the piece's hinge (pivot, axis) by the sum
/// of two drives, then slid along `dir` (body frame) by a third.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Motion {
    pub turn: [Drive; 2],
    pub slide: Drive,
    pub dir: Vec3,
}

/// How a piece moves (see `Plane::pose`).
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Anim {
    Fixed,
    Move(Motion),
    // The Cessna's gear, whose linkages are worked out in full:
    NoseLeg,
    LinkUpper,
    LinkLower,
    /// side: -1 left, +1 right
    MainLeg(f32),
}

impl Anim {
    /// Turns about the hinge by `d`.
    pub const fn hinge(d: Drive) -> Self {
        Anim::Move(Motion { turn: [d, Drive::NONE], slide: Drive::NONE, dir: Vec3::ZERO })
    }
    /// Turns about the hinge by the sum of two drives.
    pub const fn hinge2(a: Drive, b: Drive) -> Self {
        Anim::Move(Motion { turn: [a, b], slide: Drive::NONE, dir: Vec3::ZERO })
    }
    /// Slides along `dir` by `d` metres.
    pub const fn slide(d: Drive, dir: Vec3) -> Self {
        Anim::Move(Motion { turn: [Drive::NONE, Drive::NONE], slide: d, dir })
    }
    /// Turns by `t`, then slides along `dir` by `s`.
    pub const fn turn_slide(t: Drive, s: Drive, dir: Vec3) -> Self {
        Anim::Move(Motion { turn: [t, Drive::NONE], slide: s, dir })
    }
    /// Whether any of its drives follows `i`.
    pub fn uses(&self, i: Input) -> bool {
        match self {
            Anim::Move(m) => m.turn.iter().chain(std::iter::once(&m.slide)).any(|d| d.gain != 0.0 && d.input == i),
            _ => false,
        }
    }
}

/// What a piece is to the damage model (see damage.rs): which part goes
/// missing when that part is lost.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Role {
    Other,
    /// side: 0 left, 1 right
    Aileron(u8),
    Flap(u8),
    Elevator,
    Rudder,
    /// the tail surfaces (cut by the stabiliser and fin breaks)
    Stab,
    /// propeller or fan blades, spinner, blur disc
    Prop,
    /// spars and ribs inside a wing, shown once the skin over them is torn
    WingStructure(u8),
    TailStructure,
    /// the part of the fuselage the tail breaks off with
    TailCone,
    /// a landing gear leg (see `Spec::gear`)
    Gear(u8),
}

/// When a piece is drawn.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Vis {
    Always,
    /// propeller blades, while they can still be followed by eye
    SlowProp,
    /// the blur of a propeller turning too fast to see
    FastProp,
    /// a braking parachute, while it is out (no part of the airframe's bounds or wreckage)
    Chute,
    /// while `Model::conds[i]` holds (a downloaded aircraft's `select`, such as wheels shown once the gear is out)
    When(u16),
}

/// A FlightGear `select` condition on our inputs, checked every frame.
#[derive(Clone, PartialEq, Debug)]
pub enum Cond {
    All(Vec<Cond>),
    Any(Vec<Cond>),
    Not(Box<Cond>),
    /// a property we don't model, at its value at rest
    Const(bool),
    /// input times the unit (FlightGear's units), compared with a value
    Cmp(Input, f32, CmpOp, f32),
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum CmpOp {
    Eq,
    Ne,
    Gt,
    Ge,
    Lt,
    Le,
}

impl Cond {
    pub fn eval(&self, f: &impl Fn(Input) -> f32) -> bool {
        match self {
            Cond::All(c) => c.iter().all(|c| c.eval(f)),
            Cond::Any(c) => c.iter().any(|c| c.eval(f)),
            Cond::Not(c) => !c.eval(f),
            Cond::Const(b) => *b,
            Cond::Cmp(i, unit, op, y) => {
                let x = f(*i) * unit;
                match op {
                    CmpOp::Eq => (x - y).abs() < 1e-4,
                    CmpOp::Ne => (x - y).abs() >= 1e-4,
                    CmpOp::Gt => x > *y,
                    CmpOp::Ge => x >= *y,
                    CmpOp::Lt => x < *y,
                    CmpOp::Le => x <= *y,
                }
            }
        }
    }
}

pub struct Piece {
    /// Mouse control, with its centre relative to this piece’s pivot.
    pub hotspot: Option<crate::aircraft::Hotspot>,
    pub name: &'static str,
    pub mesh: Mesh,
    /// which breakable group it belongs to (see `Spec::groups`)
    pub group: usize,
    pub parent: Option<usize>,
    /// a point on the hinge axis, body frame at rest
    pub pivot: Vec3,
    pub axis: Vec3,
    pub anim: Anim,
    pub role: Role,
    pub vis: Vis,
    /// drawn in the transparent pass
    pub glass: bool,
    /// blocks ambient light in the occlusion bake
    pub occluder: bool,
}

/// The breakable groups every aircraft is divided into (names and masses in `Spec::groups`).
pub const N_GROUPS: usize = 7;
pub const G_FUSE: usize = 0;
pub const G_TAIL: usize = 1;
pub const G_LWING: usize = 2;
pub const G_RWING: usize = 3;
/// the engine (a single's) or the nose (a jet's, whose engines hang from the wings)
pub const G_ENGINE: usize = 4;
pub const G_PROP: usize = 5;
pub const G_GEAR: usize = 6;

#[derive(Default)]
pub struct Builder {
    pub pieces: Vec<Piece>,
}
impl Builder {
    pub fn add(&mut self, name: &'static str, group: usize, mesh: Mesh, anim: Anim, pivot: Vec3, axis: Vec3, parent: Option<usize>) -> usize {
        self.pieces.push(Piece { hotspot: None, name, mesh, group, parent, pivot, axis: axis.normalize_or_zero(), anim, role: Role::Other, vis: Vis::Always, glass: false, occluder: true });
        self.pieces.len() - 1
    }
    pub fn fixed(&mut self, name: &'static str, group: usize, mesh: Mesh) -> usize {
        self.add(name, group, mesh, Anim::Fixed, Vec3::ZERO, Vec3::X, None)
    }
    /// Add a right-side piece and its mirror image on the left. Returns (left, right).
    pub fn pair(&mut self, name: [&'static str; 2], group: [usize; 2], mesh: Mesh, anim: impl Fn(f32) -> Anim, pivot: Vec3, axis: Vec3, parent: [Option<usize>; 2]) -> (usize, usize) {
        let left = mesh.mirror_x();
        // Mirroring flips the sense of rotation: flip the axis back so equal
        // angles move both sides the same way.
        let l = self.add(name[0], group[0], left, anim(-1.0), pivot * Vec3::new(-1.0, 1.0, 1.0), -(axis * Vec3::new(-1.0, 1.0, 1.0)), parent[0]);
        let r = self.add(name[1], group[1], mesh, anim(1.0), pivot, axis, parent[1]);
        (l, r)
    }
    pub fn role(&mut self, i: usize, r: Role) -> &mut Self {
        self.pieces[i].role = r;
        self
    }
}

pub struct Model {
    pub pieces: Vec<Piece>,
    /// every group's pieces merged at rest, for the wreckage
    pub groups: Vec<Mesh>,
    /// a downloaded aircraft's own textures (see fgmodel.rs), drawn from by its M_TEX surfaces
    pub tex: Option<std::sync::Arc<crate::fgmodel::TexSet>>,
    /// the conditions `Vis::When` pieces are drawn under
    pub conds: Vec<Cond>,
    /// pieces drawn somewhere other than their mesh frame says (a downloaded jet's
    /// exhaust plumes: the Hornet's, which the shader lights in its own frame, moved
    /// onto each nozzle), and the transform after their pose
    pub placed: Vec<(usize, Mat4)>,
}

impl Model {
    /// Where piece i is drawn, given its pose.
    pub fn place(&self, i: usize, pose: Mat4) -> Mat4 {
        self.placed.iter().find(|p| p.0 == i).map_or(pose, |p| pose * p.1)
    }

    /// Whether a piece is drawn, given the plane's inputs (`blur`: how far its propeller has blurred, 0..1).
    pub fn shown(&self, p: &Piece, blur: f32, chute_out: bool, input: &impl Fn(Input) -> f32) -> bool {
        match p.vis {
            Vis::Always => true,
            Vis::SlowProp => blur < 1.0,
            Vis::FastProp => blur > 0.0,
            Vis::Chute => chute_out,
            Vis::When(i) => self.conds.get(i as usize).map_or(true, |c| c.eval(input)),
        }
    }
}

/// One of the outside lights, found from its lens: the glow sprites and the
/// landing lights' beam (main.rs) sit where the lenses are modelled.
#[derive(Clone, Copy, Debug)]
pub struct Lamp {
    pub piece: usize,
    /// the lens's middle, body frame at rest
    pub at: Vec3,
    /// which light (`lamp::*`)
    pub id: u32,
}

impl Model {
    /// The outside lights (navigation, tail, strobes, beacons, landing): the
    /// lens vertices of each, gathered into one lamp per lens.
    pub fn lamps(&self) -> Vec<Lamp> {
        let mut acc: Vec<(Lamp, Vec3, f32)> = Vec::new();
        for (pi, p) in self.pieces.iter().enumerate() {
            for v in &p.mesh.v {
                if v.mat[0] != mat::LAMP || v.mat[3] > lamp::LANDING + 0.5 {
                    continue;
                }
                let id = v.mat[3].round() as u32;
                let at = Vec3::from(v.pos);
                match acc.iter_mut().find(|(l, s, n)| l.piece == pi && l.id == id && (*s / *n).distance(at) < 0.6) {
                    Some((_, s, n)) => {
                        *s += at;
                        *n += 1.0;
                    }
                    None => acc.push((Lamp { piece: pi, at, id }, at, 1.0)),
                }
            }
        }
        acc.into_iter().map(|(l, s, n)| Lamp { at: s / n, ..l }).collect()
    }

    /// Finish a model: split the cut pieces and merge the groups.
    pub fn finish(mut pieces: Vec<Piece>) -> Model {
        split_cut(&mut pieces);
        let mut groups = vec![Mesh::default(); N_GROUPS];
        for p in &pieces {
            if !p.glass && p.vis != Vis::Chute {
                groups[p.group].append(&p.mesh);
            }
        }
        Model { pieces, groups, tex: None, conds: Vec::new(), placed: Vec::new() }
    }
    /// Centre and half size of everything solid in it (body frame, at rest).
    /// (Temporal anti-aliasing tells the aircraft's pixels by this box. A NaN
    /// vertex is skipped: glam's min/max would take it and drop everything
    /// before it, and a box that misses the wings smears them in flight.)
    pub fn bounds(&self) -> (Vec3, Vec3) {
        let (lo, hi) = self.pieces.iter().filter(|p| !p.glass && p.vis != Vis::Chute).flat_map(|p| p.mesh.v.iter()).map(|v| Vec3::from(v.pos)).filter(|p| p.is_finite()).fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(lo, hi), p| (lo.min(p), hi.max(p)));
        ((lo + hi) * 0.5, (hi - lo) * 0.5)
    }
}


/// Static pieces that mix surfaces with openings cut in them and plain ones
/// are split, so the plain part can be drawn without discarding fragments
/// (which keeps early depth testing on for the bulk of the interior).
pub fn split_cut(pieces: &mut Vec<Piece>) {
    let n = pieces.len();
    for k in 0..n {
        let p = &mut pieces[k];
        if p.anim != Anim::Fixed || p.glass {
            continue;
        }
        let m = &p.mesh;
        let cut_tri = |t: &[u32]| cut_by(m.v[t[0] as usize].mat[1]);
        let n_cut = m.i.chunks(3).filter(|t| cut_tri(t)).count();
        if n_cut == 0 || n_cut * 3 == m.i.len() {
            continue;
        }
        let mut parts = [Mesh::default(), Mesh::default()];
        for (side, part) in parts.iter_mut().enumerate() {
            let mut remap = vec![u32::MAX; m.v.len()];
            for t in m.i.chunks(3).filter(|t| cut_tri(t) == (side == 0)) {
                for &i in t {
                    if remap[i as usize] == u32::MAX {
                        remap[i as usize] = part.v.len() as u32;
                        part.v.push(m.v[i as usize]);
                    }
                    part.i.push(remap[i as usize]);
                }
            }
        }
        let [cut, solid] = parts;
        let (name, group, occluder, role) = (p.name, p.group, p.occluder, p.role);
        p.mesh = cut;
        pieces.push(Piece { hotspot: None, name, mesh: solid, group, parent: None, pivot: Vec3::ZERO, axis: Vec3::X, anim: Anim::Fixed, role, vis: Vis::Always, glass: false, occluder });
    }
}

// ---------------------------------------------------------------- shared parts

/// A round access plate held by screws.
pub fn inspection_plate(m: &mut Mesh, c: Vec3, n: Vec3, up: Vec3, r: f32, screws: usize) {
    m.disc(&WHITE, c, n, up, r, 28);
    let (x, y) = (up.cross(n).normalize(), up);
    for k in 0..screws {
        let a = k as f32 / screws as f32 * TAU + 0.3;
        m.screw(&SCREW_PAINT, c + (x * a.cos() + y * a.sin()) * (r - 0.011) + n * 0.0003, n, 0.0045, c.x * 9.0 + c.z * 3.0 + k as f32);
    }
}

/// Spars and ribs inside a lifting surface. Never seen while the skin is
/// whole; once a piece is torn off they are what the stump shows.
/// `spars`: chord fractions; `ribs`: (station, from, to chord fraction).
pub fn structure(f: &Foil, m: &mut Mesh, s0: f32, s1: f32, spars: &[f32], ribs: &[(f32, f32, f32)]) {
    let ns = ((s1 - s0) / 0.15).ceil().max(2.0) as usize;
    let inset = |p: Vec3, mid: Vec3| p + (mid - p).normalize_or_zero() * 0.004;
    for &xc in spars {
        // A vertical web with a cap strip along each edge.
        m.patch(&PRIMER, ns, 1, false, |a, bb| {
            let s = s0 + (s1 - s0) * a;
            let (t, b) = (f.pt(s, xc, 1.0, -1.0), f.pt(s, xc, -1.0, -1.0));
            let mid = (t + b) * 0.5;
            (inset(t, mid).lerp(inset(b, mid), bb), [s, bb], 0.0)
        });
        for side in [1.0f32, -1.0] {
            let pts: Vec<Vec3> = (0..=ns).map(|k| { let s = s0 + (s1 - s0) * k as f32 / ns as f32; let p = f.pt(s, xc, side, -1.0); inset(p, f.pt(s, xc, -side, -1.0)) }).collect();
            m.tube(&ALU, &pts, &|_| 0.011, 6, true);
        }
    }
    for &(s, a, bb) in ribs {
        let n = 14;
        let mut ring = Vec::new();
        for j in 0..=2 * n {
            let (side, w) = if j <= n { (1.0, 1.0 - j as f32 / n as f32) } else { (-1.0, (j - n) as f32 / n as f32) };
            let xc = a + (bb - a) * (1.0 - (w * FRAC_PI_2).cos());
            let p = f.pt(s, xc, side, -1.0);
            ring.push(inset(p, f.pt(s, xc, -side, -1.0)));
        }
        ring.pop();
        let k = (f.at)(s);
        let c = f.pt(s, (a + bb) * 0.5, 0.0, -1.0);
        m.fan(&PRIMER, c, &ring, k.cdir.cross(k.up).normalize());
    }
}

// ---------------------------------------------------------------- landing gear

/// Tyre and wheel around axle `axis` through `c`.
pub fn wheel(m: &mut Mesh, c: Vec3, axis: Vec3, r: f32, half_w: f32, rim: f32, brake: bool) {
    let tyre: Vec<[f32; 2]> = [(0.45, -0.80), (0.58, -0.97), (0.74, -1.0), (0.87, -0.92), (0.96, -0.72), (1.0, -0.40), (1.0, 0.40), (0.96, 0.72), (0.87, 0.92), (0.74, 1.0), (0.58, 0.97), (0.45, 0.80)]
        .iter()
        .map(|&(a, b)| [rim + (r - rim) * ((a - 0.45) / 0.55), b * half_w])
        .collect();
    m.lathe(&RUBBER.pat(pat::TREAD), c, axis, &tyre, 48, true);
    let hub = mt(mat::SATIN, 0.55, 0.56, 0.57);
    for s in [-1.0f32, 1.0] {
        let p: Vec<[f32; 2]> = [(rim + 0.008, 0.80), (rim + 0.008, 0.88), (rim - 0.004, 0.86), (rim * 0.8, 0.75), (rim * 0.62, 0.78), (rim * 0.40, 0.88), (rim * 0.22, 0.92), (0.0, 0.92)]
            .iter()
            .map(|&(a, b)| [a, b * half_w * s])
            .collect();
        let mut p = p;
        if s < 0.0 {
            p.reverse();
        }
        m.lathe(&hub, c, axis, &p, 32, false);
        for k in 0..6 {
            let a = k as f32 / 6.0 * TAU;
            let u = axis.any_orthonormal_vector();
            let w = axis.cross(u);
            let q = c + (u * a.cos() + w * a.sin()) * rim * 0.52 + axis * (half_w * 0.83 * s);
            m.bolt(&STEEL, q, axis * s, 0.0045);
        }
    }
    let u = axis.any_orthonormal_vector();
    let q = c + u * rim * 0.72 + axis * half_w * 0.8;
    m.tube(&STEEL, &[q, q + axis * 0.02], &|_| 0.003, 6, true);
    if brake {
        m.lathe(&mt(mat::STEEL, 0.4, 0.36, 0.33).x(2.0), c - axis * (half_w + 0.012), axis, &[[rim * 0.75, 0.0], [rim * 1.18, 0.0], [rim * 1.18, 0.006], [rim * 0.75, 0.006], [rim * 0.75, 0.0]], 40, false);
    }
}

// ---------------------------------------------------------------- ambient occlusion

/// How the occlusion bake samples an airframe.
pub struct AoBake<'a> {
    /// voxel size and how far rays reach (m)
    pub vox: f32,
    pub reach: f32,
    pub rays: usize,
    /// is this body-frame point in an opening (a window, a gear well) of a
    /// surface with this pattern? Light passes through those.
    pub open: &'a (dyn Fn(Vec3, f32) -> bool + Sync),
}

/// Bakes ambient occlusion into every vertex: the airframe (less its glass) is
/// voxelised and a fan of rays is marched from each vertex. Light reaching a
/// cabin only through its windows falls out of this naturally.
pub fn bake_ao(pieces: &mut [Piece], cfg: &AoBake) {
    let vox = cfg.vox;
    // The voxel grid covers everything, with a margin for rays leaving it.
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for p in pieces.iter().filter(|p| !p.glass) {
        for v in &p.mesh.v {
            lo = lo.min(v.pos.into());
            hi = hi.max(v.pos.into());
        }
    }
    let lo = lo - Vec3::splat(vox * 2.0);
    let size = hi - lo + Vec3::splat(vox * 2.0);
    let dim = [(size.x / vox) as usize + 1, (size.y / vox) as usize + 1, (size.z / vox) as usize + 1];
    let mut grid = vec![0u64; (dim[0] * dim[1] * dim[2]) / 64 + 1];
    // coarse occupancy, one flag per CB^3 block, so rays skip empty air in one step.
    const CB: usize = 8;
    let cdim = [dim[0].div_ceil(CB), dim[1].div_ceil(CB), dim[2].div_ceil(CB)];
    let mut coarse = vec![false; cdim[0] * cdim[1] * cdim[2]];
    let cell = |p: Vec3| -> Option<usize> {
        let q = (p - lo) / vox;
        if q.min_element() < 0.0 {
            return None;
        }
        let (x, y, z) = (q.x as usize, q.y as usize, q.z as usize);
        if x >= dim[0] || y >= dim[1] || z >= dim[2] {
            return None;
        }
        Some((z * dim[1] + y) * dim[0] + x)
    };
    for p in pieces.iter().filter(|p| p.occluder && !p.glass && !matches!(p.role, Role::WingStructure(_) | Role::TailStructure)) {
        let m = &p.mesh;
        for t in m.i.chunks(3) {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| Vec3::from(m.v[i as usize].pos));
            let pattern = m.v[t[0] as usize].mat[1];
            let cut = cut_by(pattern);
            let n = (((b - a).length().max((c - a).length()).max((c - b).length())) / (vox * 0.5)).ceil().max(1.0) as usize;
            for i in 0..=n {
                for j in 0..=n - i {
                    let q = a + (b - a) * (i as f32 / n as f32) + (c - a) * (j as f32 / n as f32);
                    if cut && (cfg.open)(q, pattern) {
                        continue;
                    }
                    if let Some(k) = cell(q) {
                        grid[k / 64] |= 1 << (k % 64);
                        let (x, y, z) = (k % dim[0], (k / dim[0]) % dim[1], k / (dim[0] * dim[1]));
                        coarse[((z / CB) * cdim[1] + y / CB) * cdim[0] + x / CB] = true;
                    }
                }
            }
        }
    }
    // Cosine-weighted directions about +Z.
    let rays = cfg.rays;
    let dirs: Vec<Vec3> = (0..rays)
        .map(|k| {
            let u = (k as f32 + 0.5) / rays as f32;
            let r = u.sqrt();
            let a = k as f32 * 2.399963;
            Vec3::new(r * a.cos(), r * a.sin(), (1.0 - u).sqrt())
        })
        .collect();
    let reach = cfg.reach;
    let occluded = |o: Vec3, d: Vec3| -> bool {
        let mut t = vox * 1.2;
        while t < reach {
            let q = (o + d * t - lo) / vox;
            if q.min_element() < 0.0 {
                return false;
            }
            let (x, y, z) = (q.x as usize, q.y as usize, q.z as usize);
            if x >= dim[0] || y >= dim[1] || z >= dim[2] {
                return false;
            }
            let (bx, by, bz) = (x / CB, y / CB, z / CB);
            if !coarse[(bz * cdim[1] + by) * cdim[0] + bx] {
                // empty block, jump to where the ray leaves it.
                let b0 = Vec3::new((bx * CB) as f32, (by * CB) as f32, (bz * CB) as f32);
                let b1 = b0 + Vec3::splat(CB as f32);
                let exit = |qa: f32, da: f32, lo: f32, hi: f32| if da > 1e-6 { (hi - qa) / da } else if da < -1e-6 { (lo - qa) / da } else { f32::MAX };
                let e = exit(q.x, d.x, b0.x, b1.x).min(exit(q.y, d.y, b0.y, b1.y)).min(exit(q.z, d.z, b0.z, b1.z));
                t += e.max(0.0) * vox + vox * 0.05;
                continue;
            }
            let k = (z * dim[1] + y) * dim[0] + x;
            if grid[k / 64] & (1 << (k % 64)) != 0 {
                return true;
            }
            t += vox * 0.7 + t * 0.03;
        }
        false
    };
    let ao_of = |v: &PartVertex| -> f32 {
        let n = Vec3::from(v.nrm).normalize_or_zero();
        if n == Vec3::ZERO {
            return 1.0;
        }
        // The ray set turned a different way at every vertex: the same set
        // everywhere aliases into blotches as the hits flip together.
        let pos = Vec3::from(v.pos);
        let spin = hash(pos.x * 71.3 + pos.y * 113.9 + pos.z * 37.1) * TAU;
        let q = Quat::from_rotation_arc(Vec3::Z, n) * Quat::from_rotation_z(spin);
        let o = pos + n * (vox * 1.1);
        let hit = dirs.iter().filter(|d| occluded(o, q * **d)).count();
        1.0 - hit as f32 / rays as f32
    };
    let threads = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
    for p in pieces.iter_mut() {
        if p.glass {
            continue;
        }
        if matches!(p.role, Role::WingStructure(_) | Role::TailStructure) {
            // Boxed in by the skin: no light reaches it until that is torn open.
            for v in &mut p.mesh.v {
                v.col[3] = 0.6;
            }
            continue;
        }
        let verts = &mut p.mesh.v;
        let chunk = verts.len().div_ceil(threads).max(64);
        std::thread::scope(|s| {
            for part in verts.chunks_mut(chunk) {
                let ao_of = &ao_of;
                s.spawn(move || {
                    for v in part {
                        v.col[3] = ao_of(v);
                    }
                });
            }
        });
        smooth_ao(verts, vox * 1.6, threads);
    }
}

/// Averages each vertex's AO with its neighbours within `r` that face the
/// same way, so the per-vertex ray noise doesn't show as speckle.
fn smooth_ao(verts: &mut [PartVertex], r: f32, threads: usize) {
    use std::collections::HashMap;
    let key = |p: [f32; 3]| ((p[0] / r).floor() as i32, (p[1] / r).floor() as i32, (p[2] / r).floor() as i32);
    let mut cells: HashMap<(i32, i32, i32), Vec<u32>> = HashMap::new();
    for (i, v) in verts.iter().enumerate() {
        cells.entry(key(v.pos)).or_default().push(i as u32);
    }
    let src: &[PartVertex] = verts;
    let out: Vec<f32> = {
        let chunk = src.len().div_ceil(threads).max(256);
        let cells = &cells;
        std::thread::scope(|s| {
            let hs: Vec<_> = (0..src.len())
                .step_by(chunk)
                .map(|a| {
                    s.spawn(move || {
                        (a..(a + chunk).min(src.len()))
                            .map(|i| {
                                let v = &src[i];
                                let (p, n) = (Vec3::from(v.pos), Vec3::from(v.nrm));
                                let (cx, cy, cz) = key(v.pos);
                                let (mut sum, mut wsum) = (0.0f32, 0.0f32);
                                for dx in -1..=1 {
                                    for dy in -1..=1 {
                                        for dz in -1..=1 {
                                            let Some(list) = cells.get(&(cx + dx, cy + dy, cz + dz)) else { continue };
                                            for &j in list {
                                                let o = &src[j as usize];
                                                let d2 = (Vec3::from(o.pos) - p).length_squared();
                                                let nd = Vec3::from(o.nrm).dot(n);
                                                if d2 > r * r || nd < 0.8 {
                                                    continue;
                                                }
                                                let w = (-d2 / (r * r * 0.35)).exp() * (nd - 0.8) * 5.0;
                                                sum += o.col[3] * w;
                                                wsum += w;
                                            }
                                        }
                                    }
                                }
                                if wsum > 0.0 { sum / wsum } else { v.col[3] }
                            })
                            .collect::<Vec<f32>>()
                    })
                })
                .collect();
            hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
        })
    };
    for (v, a) in verts.iter_mut().zip(out) {
        v.col[3] = a;
    }
}
