//! A reader for YASim aircraft files (FlightGear's other flight model, the
//! one most of its collection flies on), made into what the fit in hangar.rs
//! takes: a `jsbsim::Aircraft` with a lift and drag polar, its mass, centre
//! of gravity, inertia and landing gear, and its engines.
//!
//! YASim keeps no tables of coefficients. It builds an aircraft from its
//! geometry (wings, stabilisers, fuselages, engines, gear) and then *solves*
//! for the lift and drag that make it fly as the file says at two
//! conditions: an approach (speed, angle of attack, flaps, gear down) and a
//! cruise (speed, altitude, engine power, or a glide angle). The polar here is
//! solved the same way, as a whole rather than surface by surface:
//!
//! - the lift curve runs through the wing's zero-lift angle (its incidence and
//!   camber), with the slope that carries the approach weight at the approach
//!   angle of attack and speed (the flaps' share taken out), up to the wing's
//!   stall angle, then falls to the post-stall lift its stall `peak` gives;
//! - the induced drag is that of the wing's aspect ratio; the parasite drag is
//!   what is left of the cruise thrust (or of the glide) at the cruise speed;
//! - the flaps are those the flaps lever moves: their lift factor over the
//!   share of the wing they cover;
//! - the empty mass is spread over the airframe (fuselages by their size,
//!   surfaces by their area) around what the file places itself (ballast,
//!   engines), and with the fuel and crew it gives the centre of gravity and
//!   the inertia.
//!
//! The file's frame: metres, x forward, y left, z up; masses in pounds, speeds
//! in knots, altitudes in feet, power in horsepower, thrust in pounds-force.
//! Nothing here comes from YASim's code: the file format is documented on
//! FlightGear's wiki.

use crate::jsbsim::{self, Contact, Polar};
use roxmltree::Node;

const LB: f64 = 0.453_592;
const FT: f64 = 0.3048;
const KT: f64 = 0.514_444;
const G: f64 = 9.806_65;

/// An engine, as the fit sees it.
#[derive(Clone, Debug, PartialEq)]
pub enum Engine {
    /// a propeller on a piston or turbine engine: rated power and the power at
    /// the cruise condition (hp)
    Prop { hp: f64, cruise_hp: f64, turbine: bool },
    /// a jet (or a plain thruster): dry thrust and with afterburner (lbf)
    Jet { thrust: f64, afterburner: f64 },
    /// a lifting rotor (helicopters, autogyros)
    Rotor,
}

#[derive(Clone, Debug, Default)]
struct Flap {
    start: f64,
    end: f64,
    lift: f64,
}

/// A wing or stabiliser (wings and horizontal ones are mirrored left and right).
#[derive(Clone, Debug, Default)]
struct Surface {
    at: [f64; 3],
    length: f64,
    chord: f64,
    taper: f64,
    /// degrees
    sweep: f64,
    dihedral: f64,
    incidence: f64,
    camber: f64,
    idrag: f64,
    stall_aoa: f64,
    stall_width: f64,
    stall_peak: f64,
    /// the flaps the flaps lever moves
    flaps: Vec<Flap>,
    mirrored: bool,
}

impl Surface {
    /// One side's span, across the airflow and seen from above (m).
    fn semispan(&self) -> f64 {
        self.length * self.sweep.to_radians().cos() * self.dihedral.to_radians().cos().abs()
    }
    fn sides(&self) -> f64 {
        if self.mirrored { 2.0 } else { 1.0 }
    }
    /// Its own area, both sides (m²).
    fn area(&self) -> f64 {
        self.sides() * self.length * self.sweep.to_radians().cos() * self.chord * (1.0 + self.taper) / 2.0
    }
    /// Its area with the part the fuselage hides between the roots, as wing areas are given.
    fn ref_area(&self) -> f64 {
        self.sides() * (self.semispan() * self.chord * (1.0 + self.taper) / 2.0 + self.at[1].abs() * self.chord)
    }
    fn span(&self) -> f64 {
        2.0 * (self.at[1].abs() + self.semispan())
    }
    /// The mean aerodynamic chord.
    fn mac(&self) -> f64 {
        let t = self.taper;
        2.0 / 3.0 * self.chord * (1.0 + t + t * t) / (1.0 + t).max(1e-6)
    }
    /// The share of its area between two fractions of its length (a flap's).
    fn share(&self, s: f64, e: f64) -> f64 {
        let (s, e) = (s.clamp(0.0, 1.0), e.clamp(0.0, 1.0));
        let k = 1.0 - self.taper;
        let int = |x: f64| x - k * x * x / 2.0;
        ((int(e) - int(s)) / int(1.0)).max(0.0)
    }
    /// Points along its mid-chord line (file frame) with the share of its area each stands for.
    fn stations(&self) -> Vec<([f64; 3], f64)> {
        let (sw, dh) = (self.sweep.to_radians(), self.dihedral.to_radians());
        let n = 4;
        let mut v = Vec::new();
        for i in 0..n {
            let f = (i as f64 + 0.5) / n as f64;
            let l = f * self.length;
            let w = self.chord * (1.0 - (1.0 - self.taper) * f) / (self.chord * (1.0 + self.taper) / 2.0 * n as f64 * self.sides());
            let (dx, dy, dz) = (-l * sw.sin(), l * sw.cos() * dh.cos(), l * sw.cos() * dh.sin());
            v.push(([self.at[0] + dx, self.at[1] + dy, self.at[2] + dz], w));
            if self.mirrored {
                v.push(([self.at[0] + dx, -(self.at[1] + dy), self.at[2] + dz], w));
            }
        }
        v
    }
}

/// An approach or cruise condition.
#[derive(Clone, Debug, Default)]
struct Cond {
    speed_kt: f64,
    alt_ft: f64,
    aoa_deg: f64,
    /// the fraction of the fuel aboard
    fuel: f64,
    glide_deg: f64,
    /// the flaps lever, and the throttles (as set, if they are)
    flaps: f64,
    throttle: Option<f64>,
    /// payload weights aboard (index of a `<weight>`, lbs)
    solve: Vec<(usize, f64)>,
}

#[derive(Clone, Debug, Default)]
struct Gear {
    at: [f64; 3],
    compression: f64,
    spring: f64,
    damp: f64,
    retract: bool,
    /// degrees it steers (360 when castering)
    steer: f64,
    brake: bool,
    skid: bool,
}

/// A YASim aircraft as its file describes it.
#[derive(Clone, Debug, Default)]
pub struct Airplane {
    pub mass_lbs: f64,
    approach: Cond,
    cruise: Cond,
    cockpit: Option<[f64; 3]>,
    /// each fuselage's ends and width
    fuselages: Vec<([f64; 3], [f64; 3], f64)>,
    /// the wing, then the other main surfaces (mstab: a biplane's lower wing, canards…)
    wing: Option<Surface>,
    mstabs: Vec<Surface>,
    tails: Vec<Surface>,
    pub engines: Vec<Engine>,
    /// what the file places itself (engines with their mass, ballast), lbs
    placed: Vec<([f64; 3], f64)>,
    /// where each jet is (where its exhaust leaves is found on the 3D model)
    jets: Vec<[f64; 3]>,
    gear: Vec<Gear>,
    tanks: Vec<([f64; 3], f64)>,
    weights: Vec<[f64; 3]>,
}

fn num(n: Node, k: &str) -> Option<f64> {
    n.attribute(k).and_then(|v| v.trim().parse::<f64>().ok()).filter(|v| v.is_finite())
}

fn f(n: Node, k: &str, d: f64) -> f64 {
    num(n, k).unwrap_or(d)
}

fn pos(n: Node) -> [f64; 3] {
    [f(n, "x", 0.0), f(n, "y", 0.0), f(n, "z", 0.0)]
}

fn bool_attr(n: Node, k: &str) -> bool {
    n.attribute(k).is_some_and(|v| matches!(v.trim().to_lowercase().as_str(), "1" | "true" | "yes"))
}

/// The controls its `<control-input>`s drive from an axis matching `axis`.
fn driven(n: Node, axis: impl Fn(&str) -> bool) -> Vec<String> {
    n.children().filter(|c| c.has_tag_name("control-input")).filter(|c| c.attribute("axis").is_some_and(|a| axis(a.trim()))).filter_map(|c| c.attribute("control")).map(|c| c.trim().to_uppercase()).collect()
}

fn is_flaps_axis(a: &str) -> bool {
    a.ends_with("/flaps") || a.ends_with("/flaps-cmd") || a.ends_with("flap-cmd-norm")
}

fn surface(n: Node, mirrored: bool) -> Surface {
    let mut s = Surface {
        at: pos(n),
        length: f(n, "length", 0.0),
        chord: f(n, "chord", 0.0),
        taper: f(n, "taper", 1.0).clamp(0.0, 2.0),
        sweep: f(n, "sweep", 0.0).clamp(-80.0, 80.0),
        dihedral: f(n, "dihedral", if mirrored { 0.0 } else { 90.0 }),
        incidence: f(n, "incidence", 0.0),
        camber: f(n, "camber", 0.0),
        idrag: f(n, "idrag", 1.0),
        stall_aoa: 16.0,
        stall_width: 4.0,
        stall_peak: 1.5,
        flaps: Vec::new(),
        mirrored,
    };
    if let Some(st) = n.children().find(|c| c.has_tag_name("stall")) {
        (s.stall_aoa, s.stall_width, s.stall_peak) = (f(st, "aoa", 16.0), f(st, "width", 4.0), f(st, "peak", 1.5));
    }
    let lever = driven(n, is_flaps_axis);
    for c in n.children().filter(|c| c.is_element()) {
        let ctl = match c.tag_name().name() {
            "flap0" => "FLAP0",
            "flap1" => "FLAP1",
            "slat" => "SLAT",
            _ => continue,
        };
        if lever.iter().any(|l| l == ctl) {
            s.flaps.push(Flap { start: f(c, "start", 0.0), end: f(c, "end", 1.0), lift: f(c, "lift", 1.0) });
        }
    }
    s
}

fn cond(n: Node, fuel: f64) -> Cond {
    let mut c = Cond { speed_kt: f(n, "speed", 0.0), alt_ft: f(n, "alt", 0.0), aoa_deg: f(n, "aoa", 0.0), fuel: f(n, "fuel", fuel).clamp(0.0, 1.0), glide_deg: f(n, "glide-angle", 0.0), ..Default::default() };
    let mut thr = Vec::new();
    for s in n.children().filter(|s| s.has_tag_name("control-setting")) {
        let (axis, v) = (s.attribute("axis").unwrap_or("").trim(), f(s, "value", 0.0));
        if is_flaps_axis(axis) {
            c.flaps = v.clamp(0.0, 1.0);
        } else if axis.contains("throttle") {
            thr.push(v.clamp(0.0, 1.0));
        }
    }
    if !thr.is_empty() {
        c.throttle = Some(thr.iter().sum::<f64>() / thr.len() as f64);
    }
    for w in n.children().filter(|w| w.has_tag_name("solve-weight")) {
        c.solve.push((f(w, "idx", 0.0).max(0.0) as usize, f(w, "weight", 0.0).max(0.0)));
    }
    c
}

/// Air density (kg/m³) at an altitude (ft) in the standard atmosphere.
fn density(alt_ft: f64) -> f64 {
    let h = (alt_ft * FT).clamp(-500.0, 30_000.0);
    if h < 11_000.0 { 1.225 * (1.0 - 2.255_77e-5 * h).powf(4.255_88) } else { 0.363_92 * (-(h - 11_000.0) / 6341.6).exp() }
}

impl Airplane {
    pub fn parse(xml: &str) -> Result<Airplane, String> {
        let doc = roxmltree::Document::parse(xml).map_err(|e| e.to_string())?;
        let root = doc.root_element();
        if !root.has_tag_name("airplane") {
            return Err("not a YASim aircraft (no <airplane>)".into());
        }
        let mut a = Airplane { mass_lbs: f(root, "mass", 0.0), ..Default::default() };
        for n in root.children().filter(|c| c.is_element()) {
            match n.tag_name().name() {
                "approach" if a.approach.speed_kt == 0.0 => a.approach = cond(n, 0.2),
                "cruise" if a.cruise.speed_kt == 0.0 => a.cruise = cond(n, 0.5),
                "cockpit" => a.cockpit = Some(pos(n)),
                "fuselage" => a.fuselages.push(([f(n, "ax", 0.0), f(n, "ay", 0.0), f(n, "az", 0.0)], [f(n, "bx", 0.0), f(n, "by", 0.0), f(n, "bz", 0.0)], f(n, "width", 1.0))),
                "wing" if a.wing.is_none() => a.wing = Some(surface(n, true)),
                "wing" | "mstab" => a.mstabs.push(surface(n, true)),
                "hstab" => a.tails.push(surface(n, true)),
                "vstab" => a.tails.push(surface(n, false)),
                "jet" => {
                    let t = f(n, "thrust", 0.0);
                    a.engines.push(Engine::Jet { thrust: t, afterburner: f(n, "afterburner", 0.0).max(t) });
                    a.placed.push((pos(n), f(n, "mass", 0.0)));
                    a.jets.push(pos(n));
                }
                "thruster" => a.engines.push(Engine::Jet { thrust: f(n, "thrust", 0.0), afterburner: 0.0 }),
                "rotor" => a.engines.push(Engine::Rotor),
                "propeller" => {
                    a.placed.push((pos(n), f(n, "mass", 0.0)));
                    let Some(e) = n.children().find(|c| c.has_tag_name("piston-engine") || c.has_tag_name("turbine-engine")) else { continue };
                    let hp = f(e, "eng-power", 0.0);
                    let cruise_hp = num(n, "cruise-power").unwrap_or(hp * 0.75);
                    a.engines.push(Engine::Prop { hp, cruise_hp, turbine: e.has_tag_name("turbine-engine") });
                }
                "gear" => {
                    let steer = driven(n, |_| true).iter().any(|c| c == "STEER");
                    a.gear.push(Gear {
                        at: pos(n),
                        compression: f(n, "compression", 1.0).max(0.02),
                        spring: f(n, "spring", 1.0).max(0.05),
                        damp: f(n, "damp", 1.0).max(0.05),
                        retract: f(n, "retract-time", 0.0) > 0.0 || driven(n, |_| true).iter().any(|c| c == "EXTEND"),
                        steer: if bool_attr(n, "castering") { 360.0 } else if steer { 60.0 } else { 0.0 },
                        brake: driven(n, |_| true).iter().any(|c| c == "BRAKE"),
                        skid: bool_attr(n, "skid"),
                    });
                }
                "tank" => a.tanks.push((pos(n), f(n, "capacity", 0.0).max(0.0))),
                "ballast" => a.placed.push((pos(n), f(n, "mass", 0.0))),
                "weight" => a.weights.push(pos(n)),
                _ => {}
            }
        }
        if a.mass_lbs <= 0.0 {
            return Err("its file gives no mass".into());
        }
        if a.approach.speed_kt <= 0.0 || a.cruise.speed_kt <= 0.0 {
            return Err("its file gives no approach or cruise".into());
        }
        Ok(a)
    }

    /// The surfaces that carry it: the wing, and any other as big as a third
    /// of it (a biplane's lower wing, a tandem wing; not canards).
    fn lifting(&self) -> Vec<&Surface> {
        let Some(w) = &self.wing else { return Vec::new() };
        std::iter::once(w).chain(self.mstabs.iter().filter(|s| s.area() >= 0.33 * w.area())).collect()
    }

    fn fuel_cap(&self) -> f64 {
        self.tanks.iter().map(|t| t.1).sum()
    }

    /// The weight (N) at a condition: empty, its fuel and its payload.
    fn weight(&self, c: &Cond) -> f64 {
        (self.mass_lbs + c.fuel * self.fuel_cap() + c.solve.iter().map(|s| s.1).sum::<f64>()) * LB * G
    }

    /// Its masses as points (file frame, lbs): the airframe's spread over its
    /// parts, what the file places, the fuel at half and the approach payload.
    fn masses(&self) -> Vec<([f64; 3], f64)> {
        let placed: f64 = self.placed.iter().map(|p| p.1.max(0.0)).sum();
        let structure = (self.mass_lbs - placed).max(0.25 * self.mass_lbs);
        // (the fuselages by their side area, the surfaces by theirs)
        let mut shares: Vec<([f64; 3], f64)> = Vec::new();
        for (a, b, w) in &self.fuselages {
            let len = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
            let n = 6;
            for i in 0..n {
                let t = (i as f64 + 0.5) / n as f64;
                shares.push(([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t], len * w / n as f64));
            }
        }
        for s in self.wing.iter().chain(&self.mstabs).chain(&self.tails) {
            let area = s.area();
            for (p, w) in s.stations() {
                shares.push((p, area * w));
            }
        }
        let total: f64 = shares.iter().map(|s| s.1).sum();
        let mut out: Vec<([f64; 3], f64)> = if total > 0.0 { shares.into_iter().map(|(p, w)| (p, structure * w / total)).collect() } else { vec![([0.0; 3], structure)] };
        out.extend(self.placed.iter().filter(|p| p.1 > 0.0).copied());
        out.extend(self.tanks.iter().map(|(p, c)| (*p, 0.5 * c)));
        out.extend(self.approach.solve.iter().filter_map(|(i, w)| self.weights.get(*i).map(|p| (*p, *w))));
        out
    }

    /// The centre of gravity (file frame), kept where the gear can carry it:
    /// ahead of the main wheels with a nose wheel, behind them with a tail wheel.
    fn cg(&self, masses: &[([f64; 3], f64)]) -> [f64; 3] {
        let m: f64 = masses.iter().map(|p| p.1).sum::<f64>().max(1e-9);
        let mut cg = [0.0; 3];
        for (p, w) in masses {
            for k in 0..3 {
                cg[k] += p[k] * w / m;
            }
        }
        let wheels: Vec<&Gear> = self.gear.iter().filter(|g| !g.skid).collect();
        let sides: Vec<f64> = wheels.iter().filter(|g| g.at[1].abs() >= 0.3).map(|g| g.at[0]).collect();
        let centre: Vec<f64> = wheels.iter().filter(|g| g.at[1].abs() < 0.3).map(|g| g.at[0]).collect();
        if !sides.is_empty() {
            let main = sides.iter().sum::<f64>() / sides.len() as f64;
            let nose = centre.iter().copied().fold(f64::MIN, f64::max);
            let tail = centre.iter().copied().fold(f64::MAX, f64::min);
            if nose > main + 0.3 {
                let wb = nose - main;
                cg[0] = cg[0].clamp(main + 0.05 * wb, main + 0.4 * wb);
            } else if tail < main - 0.3 {
                let wb = main - tail;
                cg[0] = cg[0].clamp(main - 0.3 * wb, main - 0.04 * wb);
            }
        }
        cg
    }

    /// Its polar, wing area (m²), span (m) and mean chord (m), as YASim would solve them.
    pub fn polar(&self) -> Result<(Polar, f64, f64, f64), String> {
        let lifting = self.lifting();
        let Some(w) = lifting.first().copied() else { return Err("its file gives no wing".into()) };
        let area: f64 = lifting.iter().map(|s| s.ref_area()).sum();
        let span = lifting.iter().map(|s| s.span()).fold(0.0, f64::max);
        if area <= 0.1 || span <= 0.3 {
            return Err("its file gives no wing".into());
        }
        let mac = lifting.iter().map(|s| s.mac() * s.ref_area()).sum::<f64>() / area;
        let ar = span * span / area;
        let tan_sw = w.sweep.to_radians().tan();
        // A finite swept wing's lift slope (per radian), for when the approach doesn't give one.
        let slope_th = 2.0 * std::f64::consts::PI * ar / (2.0 + (4.0 + ar * ar * (1.0 + tan_sw * tan_sw)).sqrt());
        // The flaps: their lift factor over the share of the lifting area they cover.
        let flap_gain: f64 = lifting.iter().map(|s| s.flaps.iter().map(|fl| (fl.lift - 1.0).max(0.0) * s.share(fl.start, fl.end)).sum::<f64>() * s.ref_area()).sum::<f64>() / area;

        // Approach: lift carries the weight at its angle of attack.
        let ap = &self.approach;
        let q_ap = 0.5 * density(ap.alt_ft) * (ap.speed_kt * KT).powi(2);
        let cl_ap = self.weight(ap) / (q_ap * area) * ap.glide_deg.to_radians().cos();
        let cl_ap_clean = cl_ap / (1.0 + flap_gain * ap.flaps);
        // The wing lifts from its zero-lift angle: its incidence, and its camber (the share of its peak lift at zero).
        let stall_body = w.stall_aoa - w.incidence;
        let zero = -w.incidence - w.camber.clamp(0.0, 0.5) * w.stall_aoa;
        let span_deg = ap.aoa_deg - zero;
        let slope = if span_deg > 1.5 { cl_ap_clean / span_deg.to_radians() } else { slope_th }.clamp(2.0, 6.5);
        let cl0 = (cl_ap_clean - slope * ap.aoa_deg.to_radians()).clamp(-0.3, 1.0);
        let mut stall = stall_body.clamp(9.0, 30.0);
        let cl_at = |a: f64| cl0 + slope * a.to_radians();
        if cl_at(stall) > 3.0 {
            stall = ((3.0 - cl0) / slope).to_degrees();
        } else if cl_at(stall) < 0.8 {
            stall = ((0.8 - cl0) / slope).to_degrees().min(30.0);
        }
        let cl_max = cl_at(stall);
        let width = w.stall_width.clamp(1.0, 15.0);
        let post = cl_max / w.stall_peak.clamp(1.05, 3.0);
        let curve = |a: f64| -> f64 {
            if a <= stall {
                cl_at(a.max(-stall))
            } else if a <= 45.0 {
                let t = ((a - stall) / width).min(1.0);
                cl_max + (post - cl_max) * t * t * (3.0 - 2.0 * t)
            } else {
                post * ((90.0 - a) / 45.0).max(0.0)
            }
        };
        let mut alphas: Vec<f64> = (-40..=180).map(|k| k as f64 * 0.5).collect();
        alphas.push(stall);
        alphas.sort_by(|a, b| a.total_cmp(b));
        alphas.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        let lift: Vec<(f64, f64)> = alphas.iter().map(|&a| (a, curve(a))).collect();
        let flap_cl = cl_ap_clean.clamp(0.3, cl_max) * flap_gain;

        // Cruise: the engines' thrust (or the glide) meets the drag.
        let cr = &self.cruise;
        let (rho, v) = (density(cr.alt_ft), cr.speed_kt * KT);
        let q_cr = 0.5 * rho * v * v;
        let cl_cr = self.weight(cr) / (q_cr * area);
        let thr = cr.throttle.unwrap_or(1.0);
        let thrust: f64 = self
            .engines
            .iter()
            .map(|e| match e {
                Engine::Prop { cruise_hp, .. } => 0.85 * cruise_hp * 745.7 / v.max(10.0),
                Engine::Jet { thrust, .. } => thrust * 4.448_22 * thr * rho / 1.225,
                Engine::Rotor => 0.0,
            })
            .sum();
        let drag = if cr.glide_deg > 0.0 || thrust <= 0.0 { self.weight(cr) * cr.glide_deg.clamp(1.0, 20.0).to_radians().tan() } else { thrust };
        let k = w.idrag.clamp(0.3, 3.0) / (std::f64::consts::PI * 0.8 * ar);
        let cd0 = (drag / (q_cr * area) - k * cl_cr * cl_cr).clamp(0.012, 0.12);
        let retracts = self.gear.iter().any(|g| g.retract);
        let polar = Polar { lift, flap_cl, cd_flap: 0.08 * flap_cl, cd0, k, cd_gear: if retracts { (0.2 * cd0).clamp(0.004, 0.025) } else { 0.0 } };
        Ok((polar, area, span, mac))
    }

    /// As what the fit reads: a `jsbsim::Aircraft` with its polar, mass,
    /// balance, inertia, eye point and landing gear (JSBSim's structural frame).
    pub fn to_jsbsim(&self, name: &str) -> Result<jsbsim::Aircraft, String> {
        let (polar, area, span, mac) = self.polar()?;
        let masses = self.masses();
        let cg = self.cg(&masses);
        // (file frame to JSBSim's: x aft, y right, z up, in feet)
        let st = |p: [f64; 3]| [-p[0] / FT, -p[1] / FT, p[2] / FT];
        let mut a = jsbsim::Aircraft::default();
        (a.name, a.source) = (name.to_string(), "YASim");
        a.wing_area_ft2 = area / (FT * FT);
        a.span_ft = span / FT;
        a.chord_ft = mac / FT;
        a.empty_lbs = self.mass_lbs;
        a.points_lbs = self.approach.solve.iter().map(|s| s.1).sum();
        a.fuel_cap_lbs = self.fuel_cap();
        a.fuel_lbs = 0.5 * a.fuel_cap_lbs;
        a.weight_lbs = a.empty_lbs + a.points_lbs + a.fuel_cap_lbs;
        // Inertia about the centre of gravity (slug ft²): roll, pitch, yaw.
        let mut i = [0.0; 3];
        for (p, w) in &masses {
            let (x, y, z) = (p[0] - cg[0], p[1] - cg[1], p[2] - cg[2]);
            let kg = w * LB;
            i[0] += kg * (y * y + z * z);
            i[1] += kg * (x * x + z * z);
            i[2] += kg * (x * x + y * y);
        }
        a.inertia = i.map(|v| v / 1.355_82);
        a.cg = Some(st(cg));
        a.vrp = Some([0.0; 3]);
        a.eye = self.cockpit.map(st);
        a.jets = self.jets.iter().map(|p| st(*p)).collect();
        a.contacts = self.contacts(cg, &masses);
        a.set_polar(&polar);
        Ok(a)
    }

    /// The gear as JSBSim contacts: wheels below the centre of gravity, the
    /// rest (skids, bumpers on top) as structure; at most `MAX_GEAR` wheels,
    /// the nearest merged when there are more. YASim's springs are factors of
    /// one that sinks a third of its travel under its share of the weight.
    fn contacts(&self, cg: [f64; 3], masses: &[([f64; 3], f64)]) -> Vec<Contact> {
        let kg: f64 = masses.iter().map(|p| p.1).sum::<f64>() * LB;
        let mut wheels: Vec<(Gear, usize)> = self.gear.iter().filter(|g| !g.skid && g.at[2] < cg[2]).map(|g| (g.clone(), 1)).collect();
        while wheels.len() > crate::aircraft::MAX_GEAR {
            let mut best = (0, 1, f64::MAX);
            for i in 0..wheels.len() {
                for j in i + 1..wheels.len() {
                    let (a, b) = (&wheels[i].0.at, &wheels[j].0.at);
                    let d = (a[0] - b[0]).hypot(a[1] - b[1]);
                    if d < best.2 {
                        best = (i, j, d);
                    }
                }
            }
            let (b, nb) = wheels.remove(best.1);
            let (a, na) = &mut wheels[best.0];
            let t = nb as f64 / (*na + nb) as f64;
            for k in 0..3 {
                a.at[k] += (b.at[k] - a.at[k]) * t;
            }
            a.at[2] = a.at[2].min(b.at[2]);
            a.spring += b.spring;
            a.damp = a.damp.max(b.damp);
            (a.retract, a.brake) = (a.retract || b.retract, a.brake || b.brake);
            a.steer = a.steer.max(b.steer);
            *na += nb;
        }
        let n = wheels.len().max(1) as f64;
        let st = |p: [f64; 3]| [-p[0] / FT, -p[1] / FT, p[2] / FT];
        let mut out: Vec<Contact> = wheels
            .iter()
            .map(|(g, _)| {
                let load = kg * G / n;
                let k = g.spring * 3.0 * load / g.compression;
                let c = g.damp * 0.5 * 2.0 * (k * load / G).sqrt();
                Contact { bogey: true, at: st(g.at), spring: k / 14.5939, damping: c / 14.5939, max_steer: g.steer, brakes: g.brake, retractable: g.retract }
            })
            .collect();
        for g in self.gear.iter().filter(|g| g.skid || g.at[2] >= cg[2]) {
            out.push(Contact { bogey: false, at: st(g.at), spring: 0.0, damping: 0.0, max_steer: 0.0, brakes: false, retractable: false });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A made-up light twin: a wing with flaps, a tail, two propellers, tricycle gear.
    const TWIN: &str = r#"<?xml version="1.0"?>
<airplane mass="3000">
  <approach speed="70" aoa="6" fuel="0.2">
    <control-setting axis="/controls/engines/engine[0]/throttle" value="0.3"/>
    <control-setting axis="/controls/flight/flaps" value="1.0"/>
    <solve-weight idx="0" weight="180"/>
  </approach>
  <cruise speed="170" alt="8000" fuel="0.5">
    <control-setting axis="/controls/engines/engine[0]/throttle" value="0.75"/>
    <solve-weight idx="0" weight="180"/>
  </cruise>
  <cockpit x="1.5" y="0.3" z="0.6"/>
  <fuselage ax="3.5" ay="0" az="0" bx="-5" by="0" bz="0.2" width="1.3" taper="0.4" midpoint="0.3"/>
  <wing x="0" y="0.6" z="-0.4" length="5.5" chord="1.8" taper="0.6" incidence="2" camber="0.05" dihedral="5">
    <stall aoa="16" width="4" peak="1.5"/>
    <flap0 start="0.05" end="0.6" lift="1.6" drag="1.8"/>
    <flap1 start="0.6" end="1" lift="1.3" drag="1.2"/>
    <control-input axis="/controls/flight/flaps" control="FLAP0"/>
    <control-input axis="/controls/flight/aileron" control="FLAP1" split="true"/>
  </wing>
  <hstab x="-4.5" y="0" z="0.3" length="1.8" chord="1.0" taper="0.7"><stall aoa="16" width="4" peak="1.5"/></hstab>
  <vstab x="-4.4" y="0" z="0.4" length="1.6" chord="1.2" taper="0.5" sweep="30"/>
  <propeller x="0.8" y="2.0" z="-0.2" mass="400" radius="0.95" cruise-speed="170" cruise-rpm="2400" cruise-power="220" cruise-alt="8000">
    <piston-engine eng-power="300" eng-rpm="2700"/>
  </propeller>
  <propeller x="0.8" y="-2.0" z="-0.2" mass="400" radius="0.95" cruise-speed="170" cruise-rpm="2400" cruise-power="220" cruise-alt="8000">
    <piston-engine eng-power="300" eng-rpm="2700"/>
  </propeller>
  <gear x="2.8" y="0" z="-1.4" compression="0.3" retract-time="5">
    <control-input axis="/controls/flight/rudder" control="STEER"/>
    <control-input axis="/controls/gear/gear-down" control="EXTEND"/>
  </gear>
  <gear x="-0.4" y="1.6" z="-1.4" compression="0.25" retract-time="5">
    <control-input axis="/controls/gear/brake-left" control="BRAKE"/>
  </gear>
  <gear x="-0.4" y="-1.6" z="-1.4" compression="0.25" retract-time="5">
    <control-input axis="/controls/gear/brake-right" control="BRAKE"/>
  </gear>
  <gear x="-5" y="0" z="1.5" compression="0.1"/>
  <tank x="0" y="2" z="-0.3" capacity="400"/>
  <tank x="0" y="-2" z="-0.3" capacity="400"/>
  <ballast x="2" y="0" z="0" mass="300"/>
  <weight x="1.5" y="0" z="0" mass-prop="/sim/weight[0]/weight-lb"/>
</airplane>"#;

    #[test]
    fn reads_the_file() {
        let a = Airplane::parse(TWIN).unwrap();
        assert_eq!(a.mass_lbs, 3000.0);
        assert_eq!(a.engines, vec![Engine::Prop { hp: 300.0, cruise_hp: 220.0, turbine: false }; 2]);
        assert_eq!((a.approach.flaps, a.approach.throttle, a.approach.fuel), (1.0, Some(0.3), 0.2));
        assert_eq!(a.cruise.fuel, 0.5);
        let w = a.wing.as_ref().unwrap();
        assert_eq!(w.flaps.len(), 1, "only the flap the lever moves");
        assert_eq!(a.gear.len(), 4);
        assert!(a.gear[0].steer > 0.0 && a.gear[0].retract && !a.gear[0].brake);
        assert!(a.gear[1].brake);
        assert!(Airplane::parse("<fdm_config/>").is_err());
    }

    #[test]
    fn the_polar_flies_the_approach_and_the_cruise() {
        let a = Airplane::parse(TWIN).unwrap();
        let (p, area, span, _) = a.polar().unwrap();
        let cl = |alpha: f64| jsbsim_interp(&p.lift, alpha);
        // At the approach: its weight carried at 6° with full flaps, at 70 kt.
        let q = 0.5 * 1.225 * (70.0 * KT).powi(2);
        let need = a.weight(&a.approach) / (q * area);
        assert!((cl(6.0) + p.flap_cl - need).abs() < 0.02, "{} + {} vs {need}", cl(6.0), p.flap_cl);
        // At the cruise: the drag is the thrust of 2 × 220 hp at 85%.
        let (rho, v) = (density(8000.0), 170.0 * KT);
        let q = 0.5 * rho * v * v;
        let clc = a.weight(&a.cruise) / (q * area);
        let thrust = 2.0 * 0.85 * 220.0 * 745.7 / v;
        assert!(((p.cd0 + p.k * clc * clc) * q * area - thrust).abs() < 1.0);
        // Its stall is the wing's, past its incidence, and lift falls after it.
        let peak = p.lift.iter().fold((0.0, f64::MIN), |m, x| if x.1 > m.1 { *x } else { m });
        assert!((peak.0 - 14.0).abs() < 0.01, "stall at {}", peak.0);
        assert!(cl(25.0) < peak.1 * 0.8);
        assert!((span - 2.0 * (0.6 + 5.5 * 5f64.to_radians().cos())).abs() < 1e-9);
    }

    fn jsbsim_interp(t: &[(f64, f64)], x: f64) -> f64 {
        let i = t.windows(2).position(|w| x >= w[0].0 && x <= w[1].0).unwrap();
        let (a, b) = (t[i], t[i + 1]);
        a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)
    }

    #[test]
    fn mass_balance_and_gear() {
        let a = Airplane::parse(TWIN).unwrap();
        let j = a.to_jsbsim("twin").unwrap();
        assert_eq!(j.source, "YASim");
        // Between the main wheels and the nose wheel, nearer the mains.
        let cg = j.cg.unwrap();
        assert!(cg[0] < 0.4 / FT && cg[0] > -2.8 / FT, "cg {cg:?}");
        assert!(j.inertia.iter().all(|v| *v > 0.0) && j.inertia[2] > j.inertia[1]);
        // Three wheels (in JSBSim's frame: x aft, y right), the bumper on top as structure.
        let wheels: Vec<_> = j.contacts.iter().filter(|c| c.bogey).collect();
        assert_eq!(wheels.len(), 3);
        assert!((wheels[0].at[0] + 2.8 / FT).abs() < 1e-9 && wheels[0].max_steer > 0.0);
        assert!((wheels[1].at[1] + 1.6 / FT).abs() < 1e-9 && wheels[1].brakes && wheels[1].retractable);
        assert_eq!(j.contacts.iter().filter(|c| !c.bogey).count(), 1);
        // It reads as a JSBSim aircraft would: lift at the approach.
        let mut s = j.state(70.0, 6.0, 0.0, 0.002_377, [0.0; 3], 30.0, 30.0, 0.0, 0.0, 0.0);
        let c = j.coefficients(&mut s);
        assert!(c[0] > 1.0 && c[1] > 0.03, "{c:?}");
    }

    #[test]
    fn many_wheels_are_merged() {
        let mut xml = String::from(r#"<airplane mass="500000"><approach speed="140" aoa="5"/><cruise speed="480" alt="35000"/>
            <wing x="0" y="3" z="0" length="30" chord="10" taper="0.3" sweep="30"/>
            <jet x="0" y="10" z="0" thrust="60000"/>"#);
        for k in 0..4 {
            xml += &format!(r#"<gear x="{}" y="0" z="-4" compression="0.5"><control-input axis="/controls/flight/rudder" control="STEER"/></gear>"#, 20 - k / 2);
        }
        for (x, y) in [(-1.0, 3.0), (-1.0, -3.0), (-3.0, 3.0), (-3.0, -3.0), (-1.0, 5.0), (-1.0, -5.0), (-3.0, 5.0), (-3.0, -5.0)] {
            xml += &format!(r#"<gear x="{x}" y="{y}" z="-4" compression="0.5"/>"#);
        }
        xml += "</airplane>";
        let j = Airplane::parse(&xml).unwrap().to_jsbsim("big").unwrap();
        let wheels: Vec<_> = j.contacts.iter().filter(|c| c.bogey).collect();
        assert_eq!(wheels.len(), crate::aircraft::MAX_GEAR);
        // the nose wheels stay one steering leg, the mains stay either side
        assert_eq!(wheels.iter().filter(|c| c.max_steer > 0.0).count(), 1);
        assert!(wheels.iter().any(|c| c.at[1] > 0.0) && wheels.iter().any(|c| c.at[1] < 0.0));
    }
}
