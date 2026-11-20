//! FlightGear 3D models: an aircraft's own exterior (and cockpit), as its
//! authors built it, turned into one of our models.
//!
//! A FlightGear aircraft describes its looks in a model file (XML) that
//! names an AC3D mesh (`.ac`), places other models (gear, propellers,
//! instruments) with offsets, and animates named objects from properties
//! (`surface-positions/elevator-pos-norm`, `gear/gear[1]/position-norm`,
//! `engines/engine[0]/rpm`...). We read the meshes and their textures, turn
//! each animation we can drive into a piece hinged or sliding the same way
//! (see `airframe::Anim`), evaluate the others at rest, and place the whole
//! thing in our body frame from the JSBSim file's reference points: the
//! model's origin sits on the visual reference point (VRP), ours on the
//! centre of gravity.
//!
//! Frames: AC3D files are y-up; FlightGear's model frame is x aft, y right,
//! z up (an AC point (x, y, z) is (x, -z, y) there); ours is +X right,
//! +Y up, -Z forward (FlightGear (x, y, z) is (y, z, x) here).

use crate::airframe::{self, Anim, CmpOp, Cond, Curve, Drive, Input, Mesh, Model, Motion, Piece, Role, Vis};
use crate::render::PartVertex;
use glam::{Mat3, Mat4, Vec3};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Material id (vertex `mat.x`) of a textured surface: colour from the
/// aircraft's texture array (layer in `mat.w`, < 0 untextured), times the vertex colour.
pub const M_TEX: f32 = 18.0;
/// Its pattern (vertex `mat.y`): drawn with the pipelines that can discard (cut-out textures).
pub const P_TEX: f32 = 24.0;

/// The textures of a model: square layers of `size`, RGBA (sRGB).
pub struct TexSet {
    pub size: u32,
    pub layers: Vec<Vec<u8>>,
}

/// What it takes to build a FlightGear aircraft's model.
#[derive(Clone, Debug)]
pub struct Ext {
    /// the aircraft's folder (what its files call `Aircraft/<fg_dir>/`)
    pub dir: PathBuf,
    pub fg_dir: String,
    /// the model file (XML, or an .ac on its own)
    pub model: PathBuf,
    /// which livery it wears
    pub paint: Paint,
    /// where the model's origin (the VRP) is in our body frame
    pub origin: Vec3,
    /// a jet's engines (body frame): an exhaust plume goes on the nozzle behind each
    pub jets: Vec<Vec3>,
    /// our steering's full angle (rad), and each leg's oleo travel (m), for scaling the drives
    pub steer_max: f32,
    pub travel: Vec<f32>,
    /// span (m), for sorting its parts into the breakable groups
    pub span: f32,
}

/// An aircraft's paint, as its `-set.xml` gives it.
///
/// FlightGear paints a model with `material` animations: named objects take
/// the texture a property holds (`sim/model/livery/texture-fuselage`). A Nasal
/// script fills those properties from a livery file (`Models/Liveries/DD.xml`),
/// the one `sim/model/livery/file` names. We run no Nasal, so we read that
/// file ourselves; the set file's own texture properties come under it.
#[derive(Clone, Debug, Default)]
pub struct Paint {
    /// the default livery file's name, without `.xml` ("DD")
    pub livery: Option<String>,
    /// texture properties the set file sets ("sim/model/livery/texture" → "Liveries/side1.nasa.png")
    pub props: Vec<(String, String)>,
}

impl Paint {
    /// From a set file's properties, flattened ("sim/model/livery/file" → "DD").
    pub fn from_set(m: &HashMap<String, String>) -> Paint {
        let livery = m.get("sim/model/livery/file").map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
        let mut props: Vec<(String, String)> = m
            .iter()
            .filter(|(k, v)| k.starts_with("sim/model/") && k.rsplit('/').next().is_some_and(|l| l.starts_with("texture")) && !v.trim().is_empty())
            .map(|(k, v)| (k.clone(), v.trim().to_string()))
            .collect();
        props.sort();
        Paint { livery, props }
    }
}

/// A property file's leaf values, by path ("sim/model/livery/texture-wing1").
fn flat_props(n: roxmltree::Node, prefix: &str, out: &mut HashMap<String, String>) {
    for c in n.children().filter(|c| c.is_element()) {
        let key = if prefix.is_empty() { c.tag_name().name().to_string() } else { format!("{prefix}/{}", c.tag_name().name()) };
        if c.children().any(|x| x.is_element()) {
            flat_props(c, &key, out);
        } else if let Some(t) = c.text().map(str::trim).filter(|t| !t.is_empty()) {
            out.entry(key).or_insert_with(|| t.to_string());
        }
    }
}

/// The folder a model's Nasal hands to FlightGear's livery code
/// (`aircraft.livery_update.new("Aircraft/p51d/Models/Liveries", 10)`), as written.
fn livery_dir_in(nasal: &str) -> Option<&str> {
    for call in ["livery_update.new(", "livery.init("] {
        if let Some(i) = nasal.find(call) {
            let rest = &nasal[i + call.len()..];
            let a = rest.find('"')? + 1;
            let b = a + rest[a..].find('"')?;
            return Some(&rest[a..b]);
        }
    }
    None
}

/// The textures a `material` animation can put on its objects, best first:
/// what its property holds, then its own default.
fn material_textures(a: roxmltree::Node, props: &HashMap<String, String>) -> Vec<String> {
    let mut out = Vec::new();
    if let Some(tp) = txt(a, "texture-prop") {
        let key = match txt(a, "property-base") {
            Some(b) => format!("{}/{}", b.trim_matches('/'), tp.trim_matches('/')),
            None => tp.trim_matches('/').to_string(),
        };
        if let Some(v) = props.get(&key) {
            out.push(v.clone());
        }
    }
    if let Some(t) = txt(a, "texture").filter(|t| !t.is_empty()) {
        out.push(t.to_string());
    }
    out
}

// ------------------------------------------------------------------ AC3D

#[derive(Clone)]
struct AcMat {
    rgb: Vec3,
    emis: Vec3,
    shi: f32,
    trans: f32,
}

struct AcObj {
    name: String,
    depth: usize,
    texture: Option<String>,
    texrep: [f32; 2],
    texoff: [f32; 2],
    crease: f32,
    /// vertices in the file's root frame
    verts: Vec<Vec3>,
    /// polygons: (material, flags, [(vertex, u, v)])
    surfs: Vec<(usize, u32, Vec<(usize, f32, f32)>)>,
    /// index of the parent object (None at the top)
    parent: Option<usize>,
}

struct Ac {
    mats: Vec<AcMat>,
    objs: Vec<AcObj>,
}

fn parse_ac(text: &str) -> Result<Ac, String> {
    let mut lines = text.lines().peekable();
    let head = lines.next().unwrap_or("");
    if !head.starts_with("AC3D") {
        return Err("not an AC3D file".into());
    }
    let mut mats = Vec::new();
    let mut objs: Vec<AcObj> = Vec::new();
    // (object index, kids still to come, its transform)
    let mut stack: Vec<(usize, usize, Mat4)> = Vec::new();
    let f = |t: &[&str], i: usize| t.get(i).and_then(|s| s.parse::<f32>().ok()).unwrap_or(0.0);
    let quoted = |l: &str| -> String {
        let a = l.find('"').map(|i| i + 1).unwrap_or(0);
        let b = l.rfind('"').filter(|b| *b >= a).unwrap_or(l.len());
        l[a..b].to_string()
    };
    while let Some(line) = lines.next() {
        let t: Vec<&str> = line.split_whitespace().collect();
        match t.first().copied() {
            Some("MATERIAL") => {
                // MATERIAL "name" rgb r g b amb r g b emis r g b spec r g b shi n trans t
                let rest = &line[line.rfind('"').map(|i| i + 1).unwrap_or(0)..];
                let w: Vec<&str> = rest.split_whitespace().collect();
                let at = |k: &str| w.iter().position(|x| *x == k);
                let v3 = |k: &str| at(k).map(|i| Vec3::new(f(&w, i + 1), f(&w, i + 2), f(&w, i + 3))).unwrap_or(Vec3::ONE);
                mats.push(AcMat { rgb: v3("rgb"), emis: at("emis").map(|i| Vec3::new(f(&w, i + 1), f(&w, i + 2), f(&w, i + 3))).unwrap_or(Vec3::ZERO), shi: at("shi").map(|i| f(&w, i + 1)).unwrap_or(32.0), trans: at("trans").map(|i| f(&w, i + 1)).unwrap_or(0.0) });
            }
            Some("OBJECT") => {
                // Pop the parents whose kids are all read.
                while let Some(top) = stack.last() {
                    if top.1 == 0 {
                        stack.pop();
                    } else {
                        break;
                    }
                }
                let parent = stack.last().map(|s| s.0);
                let parent_m = stack.last().map(|s| s.2).unwrap_or(Mat4::IDENTITY);
                if let Some(top) = stack.last_mut() {
                    top.1 -= 1;
                }
                let mut o = AcObj { name: String::new(), depth: stack.len(), texture: None, texrep: [1.0, 1.0], texoff: [0.0, 0.0], crease: 61.0, verts: Vec::new(), surfs: Vec::new(), parent };
                let mut loc = Vec3::ZERO;
                let mut rot = Mat3::IDENTITY;
                let mut local: Vec<Vec3> = Vec::new();
                let mut kids = 0usize;
                while let Some(l) = lines.next() {
                    let w: Vec<&str> = l.split_whitespace().collect();
                    match w.first().copied() {
                        Some("name") => o.name = quoted(l),
                        Some("data") => {
                            // n characters follow, possibly over several lines
                            let mut n = w.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(0);
                            while n > 0 {
                                let Some(d) = lines.next() else { break };
                                n = n.saturating_sub(d.len() + 1);
                            }
                        }
                        Some("texture") => o.texture = Some(quoted(l)),
                        Some("texrep") => o.texrep = [f(&w, 1), f(&w, 2)],
                        Some("texoff") => o.texoff = [f(&w, 1), f(&w, 2)],
                        Some("crease") => o.crease = f(&w, 1),
                        Some("loc") => loc = Vec3::new(f(&w, 1), f(&w, 2), f(&w, 3)),
                        Some("rot") => rot = Mat3::from_cols_array(&[f(&w, 1), f(&w, 2), f(&w, 3), f(&w, 4), f(&w, 5), f(&w, 6), f(&w, 7), f(&w, 8), f(&w, 9)]),
                        Some("numvert") => {
                            let n = w.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(0);
                            for _ in 0..n {
                                let v: Vec<&str> = lines.next().unwrap_or("").split_whitespace().collect();
                                local.push(Vec3::new(f(&v, 0), f(&v, 1), f(&v, 2)));
                            }
                        }
                        Some("numsurf") => {
                            let n = w.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(0);
                            for _ in 0..n {
                                let mut flags = 0u32;
                                let mut mat = 0usize;
                                let mut refs = Vec::new();
                                while let Some(s) = lines.next() {
                                    let sw: Vec<&str> = s.split_whitespace().collect();
                                    match sw.first().copied() {
                                        Some("SURF") => flags = u32::from_str_radix(sw.get(1).unwrap_or(&"0").trim_start_matches("0x"), 16).unwrap_or(0),
                                        Some("mat") => mat = sw.get(1).and_then(|s| s.parse().ok()).unwrap_or(0),
                                        Some("refs") => {
                                            let k = sw.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(0);
                                            for _ in 0..k {
                                                let r: Vec<&str> = lines.next().unwrap_or("").split_whitespace().collect();
                                                refs.push((r.first().and_then(|s| s.parse().ok()).unwrap_or(0), f(&r, 1), f(&r, 2)));
                                            }
                                            break;
                                        }
                                        _ => {}
                                    }
                                }
                                o.surfs.push((mat, flags, refs));
                            }
                        }
                        Some("kids") => {
                            kids = w.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
                            break;
                        }
                        _ => {}
                    }
                }
                // AC3D: an object's vertices are relative to its location (and rotation), its location to its parent's.
                let m = parent_m * Mat4::from_translation(loc) * Mat4::from_mat3(rot.transpose());
                o.verts = local.iter().map(|v| m.transform_point3(*v)).collect();
                objs.push(o);
                stack.push((objs.len() - 1, kids, m));
            }
            _ => {}
        }
    }
    Ok(Ac { mats, objs })
}

// ------------------------------------------------------------------ textures

/// The largest SGI image we decode (8192 x 8192): past any aircraft texture.
const SGI_MAX_PIXELS: usize = 8192 * 8192;

/// An SGI image (.rgb, .rgba, .sgi, .bw), as FlightGear's older models use.
fn read_sgi(d: &[u8]) -> Option<image::RgbaImage> {
    if d.len() < 512 || u16::from_be_bytes([d[0], d[1]]) != 474 {
        return None;
    }
    let rle = d[2] == 1;
    if d[3] != 1 {
        return None; // only 8 bits a channel
    }
    let (w, h, zs) = (u16::from_be_bytes([d[6], d[7]]) as usize, u16::from_be_bytes([d[8], d[9]]) as usize, u16::from_be_bytes([d[10], d[11]]).max(1) as usize);
    // (the header's own word for its size: up to 65535 x 65535 x 65535 bytes to allocate)
    if w == 0 || h == 0 || zs > 4 || w * h > SGI_MAX_PIXELS {
        return None;
    }
    let mut ch = vec![vec![0u8; w * h]; zs];
    if rle {
        let tab = |i: usize| -> usize { let o = 512 + i * 4; u32::from_be_bytes([d[o], d[o + 1], d[o + 2], d[o + 3]]) as usize };
        if d.len() < 512 + h * zs * 8 {
            return None;
        }
        for z in 0..zs {
            for y in 0..h {
                let mut p = tab(y + z * h);
                let row = &mut ch[z][y * w..(y + 1) * w];
                let mut x = 0;
                while p < d.len() && x < w {
                    let b = d[p];
                    p += 1;
                    let n = (b & 0x7f) as usize;
                    if n == 0 {
                        break;
                    }
                    if b & 0x80 != 0 {
                        for _ in 0..n {
                            if x < w && p < d.len() {
                                row[x] = d[p];
                            }
                            x += 1;
                            p += 1;
                        }
                    } else {
                        let v = *d.get(p)?;
                        p += 1;
                        for _ in 0..n {
                            if x < w {
                                row[x] = v;
                            }
                            x += 1;
                        }
                    }
                }
            }
        }
    } else {
        for z in 0..zs {
            let o = 512 + z * w * h;
            ch[z].copy_from_slice(d.get(o..o + w * h)?);
        }
    }
    // (rows run bottom to top)
    let mut img = image::RgbaImage::new(w as u32, h as u32);
    for y in 0..h {
        for x in 0..w {
            let i = (h - 1 - y) * w + x;
            let px = match zs {
                1 => [ch[0][i], ch[0][i], ch[0][i], 255],
                2 => [ch[0][i], ch[0][i], ch[0][i], ch[1][i]],
                3 => [ch[0][i], ch[1][i], ch[2][i], 255],
                _ => [ch[0][i], ch[1][i], ch[2][i], ch[3][i]],
            };
            img.put_pixel(x as u32, y as u32, image::Rgba(px));
        }
    }
    Some(img)
}

fn read_texture(p: &Path) -> Option<image::RgbaImage> {
    let d = std::fs::read(p).ok()?;
    read_sgi(&d).or_else(|| image::load_from_memory(&d).ok().map(|i| i.to_rgba8()))
}

// ------------------------------------------------------------------ model files

#[derive(Clone, Copy, Debug)]
enum Kind {
    Rotate,
    Translate,
    Select,
}

/// An animation, in the root model frame (FlightGear axes).
#[derive(Clone, Debug)]
struct AnimDef {
    kind: Kind,
    center: Vec3,
    axis: Vec3,
    /// what drives it, if we have it, and its value then: `offset + gain * curve(input)` (degrees or metres)
    drive: Option<Drive>,
    /// its value at rest when nothing of ours drives it
    rest: f32,
    /// a select: shown at rest, or with the propeller turning fast / slowly
    vis: Option<Vis>,
    shown: bool,
    prop: bool,
    gear_leg: Option<u8>,
    surface: Option<Role>,
    /// a select on something we model (the gear, flaps...): checked as the plane flies
    cond: Option<Cond>,
}

/// One mesh object of the whole model, its vertices in the root model frame.
struct Obj {
    ac: usize,
    idx: usize,
    /// (nesting level of the model file, AC depth, order) of each animation acting on it
    chain: Vec<((usize, f32, usize), usize)>,
}

struct Loader<'a> {
    ext: &'a Ext,
    acs: Vec<(Ac, PathBuf, Mat4)>,
    objs: Vec<Obj>,
    anims: Vec<AnimDef>,
    order: usize,
    warn: Vec<String>,
    /// the livery's texture properties (its file's, over the set file's)
    paint: HashMap<String, String>,
    /// objects a `material` animation gives another texture: the file it is in
    tex_over: HashMap<usize, PathBuf>,
}

fn num(n: roxmltree::Node, k: &str) -> Option<f32> {
    n.children().find(|c| c.has_tag_name(k)).and_then(|c| c.text()).and_then(|t| t.trim().parse().ok())
}
fn txt<'a>(n: roxmltree::Node<'a, 'a>, k: &str) -> Option<&'a str> {
    n.children().find(|c| c.has_tag_name(k)).and_then(|c| c.text()).map(str::trim)
}

/// A FlightGear property, as one of our drive inputs and what one unit of it is in ours.
fn input_of(p: &str, ext: &Ext) -> Option<(Input, f32)> {
    let p = p.trim().trim_start_matches('/');
    let idx = |s: &str| -> u8 { s.find('[').and_then(|i| s[i + 1..].split(']').next()).and_then(|n| n.parse().ok()).unwrap_or(0) };
    let leg = |s: &str| idx(s.split('/').nth(1).unwrap_or(""));
    Some(match p {
        "surface-positions/elevator-pos-norm" => (Input::Elevator, 1.0),
        "surface-positions/rudder-pos-norm" => (Input::Rudder, 1.0),
        "surface-positions/left-aileron-pos-norm" => (Input::Aileron(-1.0), 1.0),
        "surface-positions/right-aileron-pos-norm" => (Input::Aileron(1.0), 1.0),
        "surface-positions/aileron-pos-norm" => (Input::Roll, 1.0),
        "surface-positions/flap-pos-norm" | "surface-positions/left-flap-pos-norm" | "surface-positions/right-flap-pos-norm" => (Input::Flaps, 1.0),
        "surface-positions/slat-pos-norm" => (Input::Slats, 1.0),
        "surface-positions/speedbrake-pos-norm" | "surface-positions/spoiler-pos-norm" => (Input::Spoiler(1.0), 1.0),
        "surface-positions/left-spoiler-pos-norm" => (Input::Spoiler(-1.0), 1.0),
        "surface-positions/right-spoiler-pos-norm" => (Input::Spoiler(1.0), 1.0),
        "controls/flight/elevator" => (Input::Pitch, 1.0),
        "controls/flight/aileron" => (Input::Roll, 1.0),
        "controls/flight/rudder" => (Input::Yaw, 1.0),
        // (one throttle moves every engine's: a twin's right nozzle reads engine[1])
        _ if p.starts_with("controls/engines/engine") && p.ends_with("/throttle") => (Input::Throttle, 1.0),
        "gear/gear/position-norm" => (Input::Gear, 1.0),
        _ if p.starts_with("gear/gear") && p.ends_with("/position-norm") => (Input::Gear, 1.0),
        _ if p.starts_with("controls/gear/gear-down") => (Input::Gear, 1.0),
        _ if p.starts_with("gear/gear") && p.ends_with("/steering-norm") => (Input::Steer, 1.0 / ext.steer_max.max(1e-3)),
        _ if p.starts_with("gear/gear") && p.ends_with("/compression-norm") => {
            let n = leg(p);
            (Input::Compress(n), 1.0 / ext.travel.get(n as usize).copied().unwrap_or(0.2).max(0.01))
        }
        _ if p.starts_with("gear/gear") && p.ends_with("/compression-m") => (Input::Compress(leg(p)), 1.0),
        _ if p.starts_with("gear/gear") && p.ends_with("/compression-ft") => (Input::Compress(leg(p)), 1.0 / 0.3048),
        _ if p.starts_with("gear/gear") && (p.ends_with("/rollspeed-ms") || p.ends_with("/wow")) => (Input::Wheel(leg(p)), 1.0),
        _ if p.starts_with("engines/engine") && p.ends_with("/reverser-pos-norm") => (Input::Reverser, 1.0),
        // A jet's fan speed (percent) moves its nozzle petals; a `spin` on it turns the fan instead (see `anim`).
        _ if p.starts_with("engines/engine") && (p.ends_with("/n1") || p.ends_with("/n2")) => (Input::N1, 1.0),
        _ if p.starts_with("engines/engine") && (p.ends_with("/rpm") || p.contains("propeller")) => (Input::Prop, 1.0),
        _ => return None,
    })
}

/// A condition, with every property at rest (zero, false), and whether it turns on how fast the engine runs.
fn condition(n: roxmltree::Node) -> (bool, Option<bool>) {
    let mut fast = None;
    let v = cond_eval(n, &mut fast);
    (v, fast)
}
fn cond_eval(n: roxmltree::Node, fast: &mut Option<bool>) -> bool {
    let kids: Vec<_> = n.children().filter(|c| c.is_element()).collect();
    match n.tag_name().name() {
        "condition" | "and" => kids.iter().all(|c| cond_eval(*c, fast)),
        "or" => kids.iter().any(|c| cond_eval(*c, fast)),
        "not" => !kids.iter().all(|c| cond_eval(*c, fast)),
        op @ ("equals" | "not-equals" | "greater-than" | "greater-than-equals" | "less-than" | "less-than-equals") => {
            let props: Vec<&str> = kids.iter().filter(|c| c.has_tag_name("property")).filter_map(|c| c.text()).collect();
            let val = kids.iter().find(|c| c.has_tag_name("value")).and_then(|c| c.text()).map(str::trim).unwrap_or("0");
            let x = 0.0f64;
            let y: f64 = if props.len() > 1 { 0.0 } else { val.parse().unwrap_or(if val.eq_ignore_ascii_case("true") { 1.0 } else { 0.0 }) };
            if props.iter().any(|p| p.contains("rpm") || p.contains("/n1") || p.contains("propeller")) {
                // (a blur shown above some speed, even below another, is a fast one)
                *fast = Some(fast.unwrap_or(false) || op.starts_with("greater"));
            }
            match op {
                "equals" => (x - y).abs() < 1e-9,
                "not-equals" => (x - y).abs() >= 1e-9,
                "greater-than" => x > y,
                "greater-than-equals" => x >= y,
                "less-than" => x < y,
                _ => x <= y,
            }
        }
        "property" => false,
        _ => true,
    }
}

/// Each jet's nozzle exit, found on the model: the aftmost skin around the
/// engine's line (its position fore and aft in the flight model is where its
/// mass is, not where its exhaust leaves), and that nozzle's radius.
fn nozzles(pieces: &[Piece], jets: &[Vec3]) -> Vec<(Vec3, f32)> {
    let mut out = Vec::new();
    for j in jets {
        let near = |p: Vec3| (p.x - j.x).hypot(p.y - j.y);
        let skin = || pieces.iter().filter(|p| !p.glass).flat_map(|p| p.mesh.v.iter()).map(|v| Vec3::from(v.pos)).filter(|p| p.is_finite() && p.z > j.z - 3.0);
        let Some(z) = skin().filter(|p| near(*p) < 0.5).map(|p| p.z).reduce(f32::max) else { continue };
        // its radius: how far out the skin within a few centimetres of the exit reaches
        let r = skin().filter(|p| p.z > z - 0.15 && near(*p) < 0.8).map(near).fold(0.0f32, f32::max);
        out.push((Vec3::new(j.x, j.y, z), if r > 0.05 { r } else { crate::aircraft::fa18::PLUME_RADIUS }));
    }
    out
}

/// A select's condition, if it turns on something we model (gear, flaps, speedbrake, throttle, fan speed):
/// those parts checked as the plane flies, every other property held at rest.
fn live_condition(n: roxmltree::Node, ext: &Ext) -> Option<Cond> {
    let mut live = false;
    let c = live_eval(n, ext, &mut live);
    live.then_some(c)
}
fn live_input(p: &str, ext: &Ext) -> Option<(Input, f32)> {
    // (angles that wrap, the propeller's and the wheels', mean nothing compared with a value)
    input_of(p, ext).filter(|(i, _)| !matches!(i, Input::Prop | Input::Wheel(_)))
}
fn live_eval(n: roxmltree::Node, ext: &Ext, live: &mut bool) -> Cond {
    let kids: Vec<_> = n.children().filter(|c| c.is_element()).collect();
    match n.tag_name().name() {
        "condition" | "and" => Cond::All(kids.iter().map(|c| live_eval(*c, ext, live)).collect()),
        "or" => Cond::Any(kids.iter().map(|c| live_eval(*c, ext, live)).collect()),
        "not" => Cond::Not(Box::new(Cond::All(kids.iter().map(|c| live_eval(*c, ext, live)).collect()))),
        op @ ("equals" | "not-equals" | "greater-than" | "greater-than-equals" | "less-than" | "less-than-equals") => {
            let props: Vec<&str> = kids.iter().filter(|c| c.has_tag_name("property")).filter_map(|c| c.text()).collect();
            if let [p] = props[..] {
                if let Some((input, unit)) = live_input(p, ext) {
                    let val = kids.iter().find(|c| c.has_tag_name("value")).and_then(|c| c.text()).map(str::trim).unwrap_or("0");
                    let y: f32 = val.parse().unwrap_or(if val.eq_ignore_ascii_case("true") { 1.0 } else { 0.0 });
                    let op = match op {
                        "equals" => CmpOp::Eq,
                        "not-equals" => CmpOp::Ne,
                        "greater-than" => CmpOp::Gt,
                        "greater-than-equals" => CmpOp::Ge,
                        "less-than" => CmpOp::Lt,
                        _ => CmpOp::Le,
                    };
                    *live = true;
                    return Cond::Cmp(input, unit, op, y);
                }
            }
            Cond::Const(cond_eval(n, &mut None))
        }
        // a bare property: true when it isn't zero
        "property" => match n.text().and_then(|p| live_input(p, ext)) {
            Some((input, unit)) => {
                *live = true;
                Cond::Cmp(input, unit, CmpOp::Ne, 0.0)
            }
            None => Cond::Const(false),
        },
        _ => Cond::Const(true),
    }
}

/// `<offsets>` as a transform (FlightGear axes: pitch about y, roll about x, heading about z).
fn offsets(n: Option<roxmltree::Node>) -> Mat4 {
    let Some(o) = n else { return Mat4::IDENTITY };
    let g = |k: &str| num(o, k).unwrap_or(0.0);
    Mat4::from_translation(Vec3::new(g("x-m"), g("y-m"), g("z-m")))
        * Mat4::from_rotation_z(g("heading-deg").to_radians())
        * Mat4::from_rotation_y(-g("pitch-deg").to_radians())
        * Mat4::from_rotation_x(g("roll-deg").to_radians())
}

/// AC3D frame (y up) to FlightGear's (z up).
const AC_TO_FG: Mat4 = Mat4::from_cols_array(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);

impl<'a> Loader<'a> {
    /// A path as a model file writes it: relative to that file, or from the FlightGear root (`Aircraft/...`).
    fn resolve(&self, base: &Path, p: &str) -> Option<PathBuf> {
        let p = p.trim();
        if let Some(rest) = p.strip_prefix("Aircraft/") {
            let mut it = rest.splitn(2, '/');
            let (dir, sub) = (it.next()?, it.next()?);
            // Only the aircraft's own files: FlightGear's shared ones aren't here.
            if !dir.eq_ignore_ascii_case(&self.ext.fg_dir) {
                return None;
            }
            let q = self.ext.dir.join(sub);
            return q.exists().then_some(q);
        }
        let q = base.join(p);
        q.exists().then_some(q)
    }

    /// A texture a model file names, where it really is: the same name as
    /// .png/.rgb/.jpg/.dds too, then under the model's `Liveries` folder
    /// (where many models keep a default their XML names bare).
    fn find_texture(&self, base: &Path, t: &str) -> Option<PathBuf> {
        let t = t.trim().replace('\\', "/");
        let with = |s: &str| -> Vec<String> {
            let stem = s.rsplit_once('.').filter(|(a, _)| !a.ends_with('/')).map(|(a, _)| a).unwrap_or(s);
            let mut v = vec![s.to_string()];
            v.extend(["png", "rgb", "jpg", "dds"].iter().map(|e| format!("{stem}.{e}")));
            v
        };
        let file = t.rsplit('/').next().unwrap_or(&t).to_string();
        with(&t).iter().find_map(|c| self.resolve(base, c)).or_else(|| with(&format!("Liveries/{file}")).iter().find_map(|c| self.resolve(base, c)))
    }

    /// Reads the default livery file into `paint`, given the root model file's folder and text.
    fn read_livery(&mut self, base: &Path, root: roxmltree::Node) {
        let Some(name) = self.ext.paint.livery.clone() else { return };
        let nasal: String = root.descendants().filter(|n| n.has_tag_name("nasal")).flat_map(|n| n.descendants().filter_map(|d| d.text())).collect();
        let dir = livery_dir_in(&nasal).and_then(|d| self.resolve(base, d)).unwrap_or_else(|| base.join("Liveries"));
        let file = dir.join(format!("{name}.xml"));
        let Ok(text) = std::fs::read_to_string(&file) else {
            self.warn.push(format!("livery {} missing", file.display()));
            return;
        };
        let Ok(doc) = roxmltree::Document::parse(&text) else {
            self.warn.push(format!("livery {}: bad XML", file.display()));
            return;
        };
        let mut m = HashMap::new();
        flat_props(doc.root_element(), "", &mut m);
        log::info!("FlightGear model: livery {} ({} properties)", file.display(), m.len());
        // (the livery's own values over the set file's)
        self.paint.extend(m);
    }

    /// Loads a model file (or a bare .ac) placed by `xf`; returns the objects it brought in.
    fn load(&mut self, path: &Path, xf: Mat4, level: usize) -> Vec<usize> {
        if level > 8 {
            return Vec::new();
        }
        let ext_l = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
        if ext_l == "ac" {
            return self.load_ac(path, xf, level).map(|(r, _)| r).unwrap_or_default();
        }
        let Ok(text) = std::fs::read_to_string(path) else {
            self.warn.push(format!("{}: unreadable", path.display()));
            return Vec::new();
        };
        let Ok(doc) = roxmltree::Document::parse(&text) else {
            self.warn.push(format!("{}: bad XML", path.display()));
            return Vec::new();
        };
        let root = doc.root_element();
        let base = path.parent().unwrap_or(Path::new(".")).to_path_buf();
        let xf = xf * offsets(root.children().find(|c| c.has_tag_name("offsets")));
        if level == 0 {
            self.read_livery(&base, root);
        }
        let mut mine: Vec<usize> = Vec::new();
        // name -> objects, for this file's animations
        let mut names: HashMap<String, Vec<usize>> = HashMap::new();
        let mut depth_of: HashMap<usize, usize> = HashMap::new();
        if let Some(p) = txt(root, "path") {
            if let Some(q) = self.resolve(&base, p) {
                if let Some((objs, by_name)) = self.load_ac(&q, xf, level) {
                    mine.extend(&objs);
                    for (k, v) in by_name {
                        names.entry(k).or_default().extend(v);
                    }
                    for o in objs {
                        let ob = &self.objs[o];
                        depth_of.insert(o, self.acs[ob.ac].0.objs[ob.idx].depth);
                    }
                }
            }
        }
        for m in root.children().filter(|c| c.has_tag_name("model")) {
            let Some(p) = txt(m, "path") else { continue };
            let pl = p.to_ascii_lowercase();
            // Light halos, ground equipment, effects and walkers: not the aircraft.
            if ["light", "effects", "tug", "immat", "walker", "human", "procedural", "pushback", "chock", "cone", "ground", "flag", "smoke", "contrail", "vapour", "vapor", "fire"].iter().any(|k| pl.contains(k)) {
                continue;
            }
            let Some(q) = self.resolve(&base, p) else { continue };
            let sub = self.load(&q, xf * offsets(m.children().find(|c| c.has_tag_name("offsets"))), level + 1);
            if let Some(n) = txt(m, "name") {
                names.entry(n.to_string()).or_default().extend(&sub);
            }
            for o in &sub {
                depth_of.insert(*o, 0);
            }
            mine.extend(sub);
        }
        // Its animations, in order.
        for a in root.children().filter(|c| c.has_tag_name("animation")) {
            let targets: Vec<&str> = a.children().filter(|c| c.has_tag_name("object-name")).filter_map(|c| c.text()).map(str::trim).collect();
            let mut objs: Vec<usize> = targets.iter().flat_map(|t| names.get(*t).cloned().unwrap_or_default()).collect();
            objs.sort_unstable();
            objs.dedup();
            let ty = txt(a, "type").unwrap_or("");
            if ty == "material" {
                // Paint: the first of its textures that is there, on its objects (outer files over inner).
                if let Some(p) = material_textures(a, &self.paint).iter().find_map(|t| self.find_texture(&base, t)) {
                    for &o in &objs {
                        self.tex_over.insert(o, p.clone());
                    }
                }
                if let Some(n) = txt(a, "name") {
                    names.entry(n.to_string()).or_default().extend(objs.clone());
                }
                continue;
            }
            let def = self.anim(a, ty, xf);
            if let Some(n) = txt(a, "name") {
                // later animations may name the group this one made
                names.entry(n.to_string()).or_default().extend(objs.clone());
            }
            let Some(def) = def else { continue };
            let id = self.anims.len();
            self.anims.push(def);
            self.order += 1;
            // (deeper objects' own animations act inside their parents')
            let d = targets.iter().filter_map(|t| names.get(*t)).flatten().filter_map(|o| depth_of.get(o)).min().copied().unwrap_or(0);
            for o in objs {
                self.objs[o].chain.push(((level, d as f32, self.order), id));
            }
        }
        mine
    }

    /// An animation we use (rotations, slides, selects), else None.
    fn anim(&mut self, a: roxmltree::Node, ty: &str, xf: Mat4) -> Option<AnimDef> {
        let kind = match ty {
            "rotate" | "spin" => Kind::Rotate,
            "translate" => Kind::Translate,
            "select" => Kind::Select,
            _ => return None,
        };
        let mut d = AnimDef { kind, center: Vec3::ZERO, axis: Vec3::X, drive: None, rest: 0.0, vis: None, shown: true, prop: false, gear_leg: None, surface: None, cond: None };
        if let Kind::Select = kind {
            let c = a.children().find(|c| c.has_tag_name("condition"))?;
            let (shown, fast) = condition(c);
            d.shown = shown;
            if let Some(f) = fast {
                d.vis = Some(if f { Vis::FastProp } else { Vis::SlowProp });
                d.shown = true;
                d.prop = true;
            } else if let Some(lc) = live_condition(c, self.ext) {
                d.cond = Some(lc);
                d.shown = true;
            }
            return Some(d);
        }
        // Centre and axis: <center> and <axis>, or the axis as two points (whose middle is the centre).
        let mut center = Vec3::ZERO;
        let mut axis = Vec3::X;
        if let Some(c) = a.children().find(|c| c.has_tag_name("center")) {
            center = Vec3::new(num(c, "x-m").unwrap_or(0.0), num(c, "y-m").unwrap_or(0.0), num(c, "z-m").unwrap_or(0.0));
        }
        if let Some(ax) = a.children().find(|c| c.has_tag_name("axis")) {
            if num(ax, "x1-m").is_some() || num(ax, "x2-m").is_some() {
                let p1 = Vec3::new(num(ax, "x1-m").unwrap_or(0.0), num(ax, "y1-m").unwrap_or(0.0), num(ax, "z1-m").unwrap_or(0.0));
                let p2 = Vec3::new(num(ax, "x2-m").unwrap_or(0.0), num(ax, "y2-m").unwrap_or(0.0), num(ax, "z2-m").unwrap_or(0.0));
                axis = p2 - p1;
                if a.children().all(|c| !c.has_tag_name("center")) {
                    center = (p1 + p2) * 0.5;
                }
            } else {
                axis = Vec3::new(num(ax, "x").unwrap_or(0.0), num(ax, "y").unwrap_or(0.0), num(ax, "z").unwrap_or(0.0));
            }
        }
        if axis.length_squared() < 1e-12 {
            return None;
        }
        d.center = xf.transform_point3(center);
        d.axis = xf.transform_vector3(axis).normalize();
        let deg = matches!(kind, Kind::Rotate);
        let factor = num(a, "factor").unwrap_or(1.0);
        let offset = if deg { num(a, "offset-deg") } else { num(a, "offset-m") }.unwrap_or(0.0);
        let mut table: Vec<(f32, f32)> = a
            .children()
            .find(|c| c.has_tag_name("interpolation"))
            .map(|t| t.children().filter(|e| e.has_tag_name("entry")).filter_map(|e| Some((num(e, "ind")?, num(e, "dep")?))).collect())
            .unwrap_or_default();
        // FlightGear keeps a table in order of its input, however it was written (the B-1B's runs 100 down to 75)
        table.sort_by(|p, q| p.0.total_cmp(&q.0));
        let prop = txt(a, "property").unwrap_or("");
        let spin = ty == "spin";
        let at_rest = |x: f32| -> f32 {
            if table.len() >= 2 {
                let mut y = table[0].1;
                for w in table.windows(2) {
                    if x >= w[0].0 && x <= w[1].0 {
                        let t = if w[1].0 > w[0].0 { (x - w[0].0) / (w[1].0 - w[0].0) } else { 0.0 };
                        return w[0].1 + (w[1].1 - w[0].1) * t;
                    }
                    y = w[1].1;
                }
                if x < table[0].0 { table[0].1 } else { y }
            } else {
                offset + factor * x
            }
        };
        let pl = prop.to_ascii_lowercase();
        if pl.contains("gear/gear") {
            d.gear_leg = pl.split("gear/gear").nth(1).map(|s| s.trim_start_matches('[').split(']').next().unwrap_or("0").parse().unwrap_or(0));
        }
        if pl.contains("rpm") || pl.contains("propeller") || pl.contains("/n1") {
            d.prop = spin || pl.contains("rpm");
        }
        d.surface = if pl.contains("left-aileron") { Some(Role::Aileron(0)) } else if pl.contains("right-aileron") { Some(Role::Aileron(1)) } else if pl.contains("elevator-pos") { Some(Role::Elevator) } else if pl.contains("rudder-pos") { Some(Role::Rudder) } else { None };
        d.rest = if spin { 0.0 } else { at_rest(0.0) };
        let k = if deg { std::f32::consts::PI / 180.0 } else { 1.0 };
        if let Some((input, unit)) = input_of(prop, self.ext) {
            // a fan spun by its speed turns with the engine's fan angle
            let input = if spin && input == Input::N1 { Input::Prop } else { input };
            if spin {
                // A turning propeller or wheel: our angle, in the direction it turns.
                d.drive = Some(Drive::new(input, factor.signum()));
            } else if table.len() >= 2 {
                // A table: a ramp from where it starts changing to where it stops, or up and back down.
                let (x0, x1) = (table[0].0, table[table.len() - 1].0);
                let (y0, y1) = (table[0].1, table[table.len() - 1].1);
                let (pk_i, pk) = table.iter().enumerate().max_by(|a, b| (a.1 .1 - y0).abs().total_cmp(&(b.1 .1 - y0).abs())).map(|(i, e)| (i, e.1)).unwrap();
                let s = |x: f32| x / unit;
                if (y1 - y0).abs() < 1e-4 && (pk - y0).abs() > 1e-4 {
                    let a0 = table[..=pk_i].iter().rev().find(|e| (e.1 - y0).abs() < 1e-4).map(|e| e.0).unwrap_or(x0);
                    let d1 = table[pk_i..].iter().find(|e| (e.1 - y0).abs() < 1e-4).map(|e| e.0).unwrap_or(x1);
                    let pkx = table[pk_i].0;
                    d.drive = Some(Drive::new(input, (pk - y0) * k).pulse(s(a0), s(pkx), s(pkx), s(d1)).offset(y0 * k));
                } else {
                    let first = table.iter().position(|e| (e.1 - y0).abs() > 1e-4).map(|i| table[i - 1].0).unwrap_or(x0);
                    let last = table.iter().rposition(|e| (e.1 - y1).abs() > 1e-4).map(|i| table[i + 1].0).unwrap_or(x1);
                    // (two entries: straight between them and held beyond, as FlightGear does)
                    let mut dr = Drive::new(input, (y1 - y0) * k).offset(y0 * k);
                    dr.curve = if table.len() == 2 { Curve::Ramp(s(x0), s(x1)) } else { Curve::Smooth(s(first), s(last)) };
                    d.drive = Some(dr);
                }
            } else {
                // offset + factor x, x in FlightGear's units = our value * unit... per our unit:
                d.drive = Some(Drive::new(input, factor * unit * k).offset(offset * k));
            }
            // (FlightGear keeps angles inside min..max; most models don't depend on it)
        }
        Some(d)
    }

    fn load_ac(&mut self, path: &Path, xf: Mat4, _level: usize) -> Option<(Vec<usize>, HashMap<String, Vec<usize>>)> {
        let text = match std::fs::read(path) {
            Ok(b) => String::from_utf8_lossy(&b).into_owned(),
            Err(_) => {
                self.warn.push(format!("{}: missing", path.display()));
                return None;
            }
        };
        let ac = match parse_ac(&text) {
            Ok(a) => a,
            Err(e) => {
                self.warn.push(format!("{}: {e}", path.display()));
                return None;
            }
        };
        let ai = self.acs.len();
        let mut ids = Vec::new();
        let mut by_name: HashMap<String, Vec<usize>> = HashMap::new();
        let first = self.objs.len();
        for (i, o) in ac.objs.iter().enumerate() {
            self.objs.push(Obj { ac: ai, idx: i, chain: Vec::new() });
            ids.push(first + i);
            // A name stands for its object and everything under it.
            let mut p = Some(i);
            while let Some(k) = p {
                let n = &ac.objs[k].name;
                if !n.is_empty() {
                    by_name.entry(n.clone()).or_default().push(first + i);
                }
                p = ac.objs[k].parent;
            }
            let _ = o;
        }
        self.acs.push((ac, path.parent().unwrap_or(Path::new(".")).to_path_buf(), xf * AC_TO_FG));
        Some((ids, by_name))
    }
}

/// FlightGear model frame to our body frame (without the origin shift).
fn fg_to_body(v: Vec3) -> Vec3 {
    Vec3::new(v.y, v.z, v.x)
}

fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
}

impl Ext {
    /// Reads the model and builds ours from it.
    pub fn build(&self) -> Result<Model, String> {
        let t0 = std::time::Instant::now();
        let paint: HashMap<String, String> = self.paint.props.iter().cloned().collect();
        let mut ld = Loader { ext: self, acs: Vec::new(), objs: Vec::new(), anims: Vec::new(), order: 0, warn: Vec::new(), paint, tex_over: HashMap::new() };
        ld.load(&self.model, Mat4::IDENTITY, 0);
        if ld.objs.is_empty() {
            return Err(format!("its model {} has nothing in it", self.model.display()));
        }
        let to_body = |v: Vec3| fg_to_body(v) + self.origin;

        // Each object's chain of animations, innermost first.
        for o in &mut ld.objs {
            o.chain.sort_by(|a, b| b.0 .0.cmp(&a.0 .0).then(b.0 .1.total_cmp(&a.0 .1)).then(a.0 .2.cmp(&b.0 .2)));
        }

        // Textures: every one the meshes use, in one array.
        let mut tex_ids: HashMap<PathBuf, i32> = HashMap::new();
        let mut images: Vec<image::RgbaImage> = Vec::new();
        // An object's texture: the livery's (a material animation's), else its mesh's own.
        let tex_of = |i: usize| -> Option<PathBuf> {
            if let Some(p) = ld.tex_over.get(&i) {
                return Some(p.clone());
            }
            let o = &ld.objs[i];
            let (ac, base, _) = &ld.acs[o.ac];
            ac.objs[o.idx].texture.as_ref().filter(|t| !t.is_empty()).map(|t| base.join(t))
        };
        for i in 0..ld.objs.len() {
            let Some(p) = tex_of(i) else { continue };
            if tex_ids.contains_key(&p) {
                continue;
            }
            // (a missing .png is often there as .rgb, and the other way round)
            let found = [p.clone(), p.with_extension("png"), p.with_extension("rgb"), p.with_extension("jpg"), p.with_extension("dds")].into_iter().find_map(|q| read_texture(&q));
            match found {
                Some(img) => {
                    tex_ids.insert(p, images.len() as i32);
                    images.push(img);
                }
                None => {
                    ld.warn.push(format!("texture {} unreadable", p.display()));
                    tex_ids.insert(p, -1);
                }
            }
        }
        let size = match images.len() {
            0 => 4,
            1..=3 => 2048,
            4..=12 => 1024,
            _ => 512,
        };
        let layers: Vec<Vec<u8>> = images.iter().map(|i| image::imageops::resize(i, size, size, image::imageops::FilterType::Triangle).into_raw()).collect();

        // Pieces: one per distinct chain of moving animations (outermost first)
        // and breakable group; the static ones are baked into the vertices.
        let mut pieces: Vec<Piece> = Vec::new();
        let mut node_of: HashMap<(Vec<usize>, usize), usize> = HashMap::new();
        // the selects checked as it flies (`Vis::When`)
        let mut conds: Vec<Cond> = Vec::new();
        let span = self.span.max(1.0);
        let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
        for o in &ld.objs {
            let (ac, _, xf) = &ld.acs[o.ac];
            for v in &ac.objs[o.idx].verts {
                let b = to_body(xf.transform_point3(*v));
                lo = lo.min(b);
                hi = hi.max(b);
            }
        }
        let len = (hi.z - lo.z).max(1.0);
        let mut n_tris = 0usize;
        for (oi, o) in ld.objs.iter().enumerate() {
            let (ac, _, xf) = &ld.acs[o.ac];
            let a = &ac.objs[o.idx];
            if a.surfs.is_empty() {
                continue;
            }
            // Hidden at rest (a select that is off): left out; a propeller select: its visibility.
            let mut vis = Vis::Always;
            let mut live: Vec<Cond> = Vec::new();
            let mut hidden = false;
            let mut prop = false;
            let mut leg: Option<u8> = None;
            let mut surface: Option<Role> = None;
            let mut moving: Vec<usize> = Vec::new();
            let mut bake = Mat4::IDENTITY;
            for &(_, id) in &o.chain {
                let d = &ld.anims[id];
                prop |= d.prop;
                leg = leg.or(d.gear_leg);
                surface = surface.or(d.surface);
                match d.kind {
                    Kind::Select => {
                        if let Some(v) = d.vis {
                            vis = v;
                        } else if let Some(c) = &d.cond {
                            live.push(c.clone());
                        } else if !d.shown {
                            hidden = true;
                        }
                    }
                    _ if d.drive.is_some() => moving.push(id),
                    _ if d.rest.abs() > 1e-6 && moving.is_empty() => {
                        // a static transform inside every moving one: into the vertices
                        bake = anim_matrix(d, d.rest, false) * bake;
                    }
                    _ if d.rest.abs() > 1e-6 => moving.push(id),
                    _ => {}
                }
            }
            if hidden {
                continue;
            }
            if vis == Vis::Always && !live.is_empty() {
                let c = if live.len() == 1 { live.pop().unwrap() } else { Cond::All(live) };
                let i = conds.iter().position(|x| *x == c).unwrap_or_else(|| {
                    conds.push(c);
                    conds.len() - 1
                });
                vis = Vis::When(i.min(u16::MAX as usize) as u16);
            }
            let name = a.name.to_ascii_lowercase();
            if name.contains("shadow") || name.contains("halo") || name.contains("light-cone") || name == "lightcone" {
                continue;
            }
            // Vertices in our body frame.
            let fgv: Vec<Vec3> = a.verts.iter().map(|v| bake.transform_point3(xf.transform_point3(*v))).collect();
            let bv: Vec<Vec3> = fgv.iter().map(|v| to_body(*v)).collect();
            let (c_lo, c_hi) = bv.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(l, h), v| (l.min(*v), h.max(*v)));
            let c = (c_lo + c_hi) * 0.5;
            let group = if prop || matches!(vis, Vis::SlowProp | Vis::FastProp) {
                airframe::G_PROP
            } else if leg.is_some() {
                airframe::G_GEAR
            } else if c.x.abs() > 0.16 * span && c.z < hi.z - 0.22 * len {
                if c.x < 0.0 { airframe::G_LWING } else { airframe::G_RWING }
            } else if c.z > hi.z - 0.22 * len {
                airframe::G_TAIL
            } else {
                airframe::G_FUSE
            };
            let tex = tex_of(oi).and_then(|p| tex_ids.get(&p).copied()).unwrap_or(-1);
            // Glass: a see-through material, or a texture with see-through parts on a propeller blur.
            let mut solid = Mesh::default();
            let mut glass = Mesh::default();
            // Smooth normals per vertex (for the smooth-shaded surfaces), kept within the crease angle.
            let mut acc = vec![Vec3::ZERO; bv.len()];
            let face_n = |refs: &[(usize, f32, f32)]| -> Vec3 {
                let mut n = Vec3::ZERO;
                for i in 0..refs.len() {
                    let (p, q) = (bv.get(refs[i].0).copied().unwrap_or_default(), bv.get(refs[(i + 1) % refs.len()].0).copied().unwrap_or_default());
                    n += Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
                }
                n.normalize_or_zero()
            };
            for (_, flags, refs) in &a.surfs {
                if flags & 0xf != 0 || refs.len() < 3 {
                    continue;
                }
                if flags & 0x10 != 0 {
                    let n = face_n(refs);
                    for r in refs {
                        if let Some(x) = acc.get_mut(r.0) {
                            *x += n;
                        }
                    }
                }
            }
            let cos_crease = a.crease.clamp(0.0, 180.0).to_radians().cos();
            for (mi, flags, refs) in &a.surfs {
                if flags & 0xf != 0 || refs.len() < 3 || refs.iter().any(|r| r.0 >= bv.len()) {
                    continue;
                }
                let m = ac.mats.get(*mi).cloned().unwrap_or(AcMat { rgb: Vec3::ONE, emis: Vec3::ZERO, shi: 32.0, trans: 0.0 });
                let fnrm = face_n(refs);
                let see_through = m.trans > 0.15 || vis == Vis::FastProp;
                let mesh = if see_through { &mut glass } else { &mut solid };
                let col = Vec3::new(srgb_to_linear(m.rgb.x), srgb_to_linear(m.rgb.y), srgb_to_linear(m.rgb.z));
                let emis = m.emis.max_element();
                let rough = (1.0 - (m.shi / 128.0).clamp(0.0, 1.0) * 0.75).clamp(0.2, 1.0);
                let base = mesh.v.len() as u32;
                for r in refs {
                    let p = bv[r.0];
                    let mut n = fnrm;
                    if flags & 0x10 != 0 {
                        let s = acc[r.0].normalize_or_zero();
                        if s.dot(fnrm) >= cos_crease {
                            n = s;
                        }
                    }
                    let uv = [r.1 * a.texrep[0] + a.texoff[0], 1.0 - (r.2 * a.texrep[1] + a.texoff[1])];
                    // (glass: its opacity in the alpha, where the solid surfaces keep their occlusion)
                    let a = if see_through { (1.0 - m.trans).clamp(0.02, 1.0) } else { 1.0 };
                    mesh.v.push(PartVertex { pos: p.into(), nrm: n.into(), col: [col.x, col.y, col.z, a], uv, mat: [M_TEX, P_TEX, rough + if emis > 0.3 { 2.0 } else { 0.0 }, tex as f32] });
                }
                for k in 1..refs.len() as u32 - 1 {
                    mesh.i.extend([base, base + k, base + k + 1]);
                    n_tris += 1;
                }
            }
            // Its piece: the chain of moving animations, outermost first.
            let mut parent: Option<usize> = None;
            let mut key: Vec<usize> = Vec::new();
            for &id in moving.iter().rev() {
                key.push(id);
                let k = (key.clone(), group);
                parent = Some(match node_of.get(&k) {
                    Some(i) => *i,
                    None => {
                        let d = &ld.anims[id];
                        let (anim, pivot, axis) = to_anim(d, self.origin);
                        let role = if d.prop { Role::Prop } else if let Some(l) = d.gear_leg { Role::Gear(l.min(crate::aircraft::MAX_GEAR as u8 - 1)) } else { d.surface.unwrap_or(Role::Other) };
                        pieces.push(Piece { hotspot: None, name: "fg", mesh: Mesh::default(), group, parent, pivot, axis, anim, role, vis: Vis::Always, glass: false, occluder: true });
                        node_of.insert(k, pieces.len() - 1);
                        pieces.len() - 1
                    }
                });
            }
            let role = if prop || matches!(vis, Vis::SlowProp | Vis::FastProp) { Role::Prop } else if let Some(l) = leg { Role::Gear(l.min(crate::aircraft::MAX_GEAR as u8 - 1)) } else { surface.unwrap_or(Role::Other) };
            for (mesh, is_glass) in [(solid, false), (glass, true)] {
                if mesh.i.is_empty() {
                    continue;
                }
                // Merge into a piece with the same parent, group, glass and visibility.
                match pieces.iter().position(|p| p.name == "fg mesh" && p.parent == parent && p.group == group && p.glass == is_glass && p.vis == vis && p.role == role) {
                    Some(i) => pieces[i].mesh.append(&mesh),
                    None => pieces.push(Piece { hotspot: None, name: "fg mesh", mesh, group, parent, pivot: Vec3::ZERO, axis: Vec3::X, anim: Anim::Fixed, role, vis, glass: is_glass, occluder: !is_glass }),
                }
            }
        }
        // Parents must come before their children: they do (a node is made before anything under it).
        for w in ld.warn.iter().take(12) {
            log::warn!("FlightGear model: {w}");
        }
        log::info!("FlightGear model {}: {} pieces, {} triangles, {} textures at {size}², read in {:.0} ms", self.model.display(), pieces.len(), n_tris, layers.len(), t0.elapsed().as_secs_f64() * 1000.0);
        // A jet's exhaust: the Hornet's plume on each of its nozzles.
        let plumes = nozzles(&pieces, &self.jets);
        let mut placed = Vec::new();
        if !plumes.is_empty() {
            let shells = crate::aircraft::fa18::plume();
            for (at, r) in &plumes {
                let s = (r / crate::aircraft::fa18::PLUME_RADIUS).clamp(0.7, 1.6);
                // (a left engine takes the Hornet's left plume, leaving its nozzle on the other side)
                let left = at.x < -0.05;
                let n = crate::aircraft::fa18::PLUME_NOZZLE * if left { Vec3::new(-1.0, 1.0, 1.0) } else { Vec3::ONE };
                let xf = Mat4::from_translation(*at) * Mat4::from_scale(Vec3::splat(s)) * Mat4::from_translation(-n);
                pieces.push(Piece { hotspot: None, name: "fg plume", mesh: shells[left as usize].clone(), group: airframe::G_TAIL, parent: None, pivot: Vec3::ZERO, axis: Vec3::X, anim: Anim::Fixed, role: Role::Other, vis: Vis::Always, glass: true, occluder: false });
                placed.push((pieces.len() - 1, xf));
            }
            log::info!("FlightGear model: exhaust plumes on {} nozzles: {}", plumes.len(), plumes.iter().map(|(a, r)| format!("({:.2}, {:.2}, {:.2}) r {r:.2}", a.x, a.y, a.z)).collect::<Vec<_>>().join(", "));
        }
        let mut model = Model::finish(pieces);
        model.conds = conds;
        model.placed = placed;
        model.tex = Some(std::sync::Arc::new(TexSet { size, layers }));
        Ok(model)
    }
}

/// The transform of an animation at a value (degrees or metres), FlightGear frame.
fn anim_matrix(d: &AnimDef, v: f32, _: bool) -> Mat4 {
    match d.kind {
        Kind::Rotate => Mat4::from_translation(d.center) * Mat4::from_axis_angle(d.axis, ROT_SIGN * v.to_radians()) * Mat4::from_translation(-d.center),
        Kind::Translate => Mat4::from_translation(d.axis * v),
        Kind::Select => Mat4::IDENTITY,
    }
}

/// FlightGear turns a rotate animation by the right-hand rule about its axis
/// (the SR22T's right flap: axis +y, factor +35.5, trailing edge down).
const ROT_SIGN: f32 = 1.0;

/// An animation as one of our moving pieces: (how it moves, pivot, axis), body frame.
fn to_anim(d: &AnimDef, origin: Vec3) -> (Anim, Vec3, Vec3) {
    let pivot = fg_to_body(d.center) + origin;
    let axis = fg_to_body(d.axis);
    let dr = d.drive.unwrap_or(Drive { input: Input::Gear, curve: Curve::Lin, gain: 0.0, offset: d.rest.to_radians() });
    match d.kind {
        Kind::Rotate => {
            // (our drives are in radians already, except a rest angle)
            let dr = if d.drive.is_none() { Drive { offset: d.rest.to_radians(), ..dr } } else { dr };
            let dr = Drive { gain: dr.gain * ROT_SIGN, offset: dr.offset * ROT_SIGN, ..dr };
            (Anim::Move(Motion { turn: [dr, Drive::NONE], slide: Drive::NONE, dir: Vec3::ZERO }), pivot, axis)
        }
        Kind::Translate => {
            let dr = if d.drive.is_none() { Drive { offset: d.rest, ..dr } } else { dr };
            (Anim::Move(Motion { turn: [Drive::NONE, Drive::NONE], slide: dr, dir: axis }), pivot, Vec3::X)
        }
        Kind::Select => (Anim::Fixed, pivot, Vec3::X),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sgi_header_cannot_ask_for_gigabytes() {
        // 65535 x 65535, 65535 channels, 8 bits: a 512-byte file asking for terabytes.
        let mut d = vec![0u8; 512];
        d[..4].copy_from_slice(&[0x01, 0xda, 0, 1]);
        d[6..12].copy_from_slice(&[0xff; 6]);
        assert!(read_sgi(&d).is_none());
    }

    #[test]
    fn ac_hierarchy_and_frames() {
        let ac = "AC3Db\nMATERIAL \"m\" rgb 1 0.5 0.25  amb 0.2 0.2 0.2  emis 0 0 0  spec 0 0 0  shi 10  trans 0\nOBJECT world\nkids 1\nOBJECT poly\nname \"wing\"\nloc 1 0 0\nnumvert 3\n0 0 0\n0 1 0\n0 0 1\nnumsurf 1\nSURF 0x10\nmat 0\nrefs 3\n0 0 0\n1 1 0\n2 0 1\nkids 0\n";
        let a = parse_ac(ac).unwrap();
        assert_eq!(a.objs.len(), 2);
        assert_eq!(a.objs[1].name, "wing");
        assert_eq!(a.objs[1].verts[1], Vec3::new(1.0, 1.0, 0.0));
        assert_eq!(a.objs[1].parent, Some(0));
        // AC y up -> FlightGear z up; AC z -> FlightGear -y
        assert!((AC_TO_FG.transform_point3(Vec3::new(1.0, 2.0, 3.0)) - Vec3::new(1.0, -3.0, 2.0)).length() < 1e-6);
        // FlightGear x aft, y right, z up -> ours x right, y up, z aft
        assert_eq!(fg_to_body(Vec3::new(1.0, 2.0, 3.0)), Vec3::new(2.0, 3.0, 1.0));
    }

    #[test]
    fn sgi_verbatim() {
        let mut d = vec![0u8; 512];
        d[0..2].copy_from_slice(&474u16.to_be_bytes());
        d[3] = 1;
        d[4..6].copy_from_slice(&2u16.to_be_bytes());
        d[6..8].copy_from_slice(&2u16.to_be_bytes());
        d[8..10].copy_from_slice(&1u16.to_be_bytes());
        d[10..12].copy_from_slice(&3u16.to_be_bytes());
        d.extend([10, 20, 30, 40, 50, 60]);
        let i = read_sgi(&d).unwrap();
        assert_eq!(i.get_pixel(0, 0).0, [10, 30, 50, 255]);
    }

    #[test]
    fn livery_folder_from_nasal() {
        assert_eq!(livery_dir_in("var l = aircraft.livery_update.new(\"Aircraft/p51d/Models/Liveries\", 10);"), Some("Aircraft/p51d/Models/Liveries"));
        assert_eq!(livery_dir_in("aircraft.livery.init(\"Aircraft/c310/Models/Liveries\");"), Some("Aircraft/c310/Models/Liveries"));
        assert_eq!(livery_dir_in("print(\"hello\");"), None);
    }

    #[test]
    fn paint_from_set_and_material_order() {
        let m: HashMap<String, String> = [("sim/model/livery/file", "DD"), ("sim/model/livery/texture", "Liveries/side1.nasa.png"), ("sim/model/path", "Models/x.xml")].into_iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
        let p = Paint::from_set(&m);
        assert_eq!(p.livery.as_deref(), Some("DD"));
        assert_eq!(p.props, vec![("sim/model/livery/texture".to_string(), "Liveries/side1.nasa.png".to_string())]);
        // the property's texture comes before the animation's own default
        let doc = roxmltree::Document::parse("<animation><type>material</type><property-base>/sim/model/livery</property-base><texture-prop>texture</texture-prop><texture>side1.rgb</texture></animation>").unwrap();
        let props: HashMap<String, String> = p.props.into_iter().collect();
        assert_eq!(material_textures(doc.root_element(), &props), vec!["Liveries/side1.nasa.png".to_string(), "side1.rgb".to_string()]);
        assert_eq!(material_textures(doc.root_element(), &HashMap::new()), vec!["side1.rgb".to_string()]);
    }

    #[test]
    fn default_livery_paints_its_objects() {
        // A tiny aircraft: a bare-metal mesh, and a livery file the set names.
        let dir = std::env::temp_dir().join(format!("fg-livery-test-{}", std::process::id()));
        let models = dir.join("Models");
        std::fs::create_dir_all(models.join("Liveries")).unwrap();
        let ac = "AC3Db\nMATERIAL \"m\" rgb 1 1 1  amb 0 0 0  emis 0 0 0  spec 0 0 0  shi 10  trans 0\nOBJECT world\nkids 2\nOBJECT poly\nname \"fuselage\"\ntexture \"bare.png\"\nnumvert 3\n0 0 0\n0 1 0\n0 0 1\nnumsurf 1\nSURF 0x10\nmat 0\nrefs 3\n0 0 0\n1 1 0\n2 0 1\nkids 0\nOBJECT poly\nname \"canopy\"\ntexture \"bare.png\"\nnumvert 3\n0 0 0\n0 1 0\n0 0 1\nnumsurf 1\nSURF 0x10\nmat 0\nrefs 3\n0 0 0\n1 1 0\n2 0 1\nkids 0\n";
        std::fs::write(models.join("x.ac"), ac).unwrap();
        std::fs::write(models.join("x.xml"), "<PropertyList><path>x.ac</path><nasal><load>var l = aircraft.livery_update.new(\"Aircraft/xx/Models/Liveries\", 10);</load></nasal><animation><type>material</type><object-name>fuselage</object-name><property-base>sim/model/livery</property-base><texture-prop>texture-fuselage</texture-prop><texture>DDfuselage.png</texture></animation></PropertyList>").unwrap();
        std::fs::write(models.join("Liveries").join("DD.xml"), "<PropertyList><sim><model><livery><texture-fuselage>Liveries/DDfuselage.png</texture-fuselage></livery></model></sim></PropertyList>").unwrap();
        std::fs::write(models.join("Liveries").join("DDfuselage.png"), b"png").unwrap();
        let ext = Ext { dir: dir.clone(), fg_dir: "xx".into(), model: models.join("x.xml"), paint: Paint { livery: Some("DD".into()), props: Vec::new() }, origin: Vec3::ZERO, steer_max: 1.0, travel: Vec::new(), span: 10.0, jets: Vec::new() };
        let mut ld = Loader { ext: &ext, acs: Vec::new(), objs: Vec::new(), anims: Vec::new(), order: 0, warn: Vec::new(), paint: HashMap::new(), tex_over: HashMap::new() };
        ld.load(&ext.model, Mat4::IDENTITY, 0);
        let painted: Vec<(String, PathBuf)> = ld.tex_over.iter().map(|(o, p)| (ld.acs[ld.objs[*o].ac].0.objs[ld.objs[*o].idx].name.clone(), p.clone())).collect();
        assert_eq!(painted, vec![("fuselage".to_string(), models.join("Liveries/DDfuselage.png"))], "warnings: {:?}", ld.warn);
        // No livery named: the animation's bare default is found under Liveries/ all the same.
        let ext2 = Ext { paint: Paint::default(), ..ext.clone() };
        let mut ld2 = Loader { ext: &ext2, acs: Vec::new(), objs: Vec::new(), anims: Vec::new(), order: 0, warn: Vec::new(), paint: HashMap::new(), tex_over: HashMap::new() };
        ld2.load(&ext2.model, Mat4::IDENTITY, 0);
        assert_eq!(ld2.tex_over.values().cloned().collect::<Vec<_>>(), vec![models.join("Liveries/DDfuselage.png")]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn nozzle_petals_follow_fan_speed_and_every_throttle() {
        let ext = Ext { dir: PathBuf::new(), fg_dir: String::new(), model: PathBuf::new(), paint: Paint::default(), origin: Vec3::ZERO, steer_max: 1.0, travel: Vec::new(), span: 10.0, jets: Vec::new() };
        let mut ld = Loader { ext: &ext, acs: Vec::new(), objs: Vec::new(), anims: Vec::new(), order: 0, warn: Vec::new(), paint: HashMap::new(), tex_over: HashMap::new() };
        let deg = |dr: Drive, x: f32| dr.eval(x).to_degrees();
        // The B-1B's petal, its table written from 100 down to 75 (percent N2): 17 deg below 75, 2 at full.
        let b1 = "<animation><type>rotate</type><object-name>p</object-name><property>engines/engine[0]/n2</property><interpolation><entry><ind>100.0</ind><dep>2.0</dep></entry><entry><ind>75.0</ind><dep>17.0</dep></entry></interpolation><axis><x>0</x><y>1</y><z>0</z></axis></animation>";
        let doc = roxmltree::Document::parse(b1).unwrap();
        let dr = ld.anim(doc.root_element(), "rotate", Mat4::IDENTITY).unwrap().drive.unwrap();
        assert_eq!(dr.input, Input::N1);
        for (n, want) in [(0.0, 17.0), (75.0, 17.0), (87.5, 9.5), (100.0, 2.0), (110.0, 2.0)] {
            assert!((deg(dr, n) - want).abs() < 1e-3, "at {n}% N2: {} deg, want {want}", deg(dr, n));
        }
        // A fan spun by its speed still turns with the fan angle.
        let fan = "<animation><type>spin</type><object-name>f</object-name><property>engines/engine[1]/n1</property><factor>-1</factor><axis><x>1</x><y>0</y><z>0</z></axis></animation>";
        let doc = roxmltree::Document::parse(fan).unwrap();
        assert_eq!(ld.anim(doc.root_element(), "spin", Mat4::IDENTITY).unwrap().drive.unwrap().input, Input::Prop);
        // The MiG-29's right nozzle reads the second engine's throttle.
        let mig = "<animation><type>rotate</type><object-name>n</object-name><property>controls/engines/engine[1]/throttle</property><interpolation><entry><ind>0</ind><dep>15</dep></entry><entry><ind>0.4</ind><dep>15</dep></entry><entry><ind>0.8</ind><dep>10</dep></entry><entry><ind>1</ind><dep>0</dep></entry></interpolation><axis><x>0</x><y>0</y><z>-1</z></axis></animation>";
        let doc = roxmltree::Document::parse(mig).unwrap();
        let dr = ld.anim(doc.root_element(), "rotate", Mat4::IDENTITY).unwrap().drive.unwrap();
        assert_eq!(dr.input, Input::Throttle);
        assert!((deg(dr, 0.0) - 15.0).abs() < 1e-3 && deg(dr, 1.0).abs() < 1e-3);
    }

    #[test]
    fn plumes_go_on_each_nozzle() {
        // A nozzle: a ring 0.45 m across around x = 1.5, from z = 3 back to its lip at z = 5,
        // and a tail fin out at x = 3 reaching further aft (not the nozzle).
        let mut m = Mesh::default();
        for k in 0..16 {
            let a = k as f32 / 16.0 * std::f32::consts::TAU;
            for z in [3.0, 5.0] {
                m.v.push(PartVertex { pos: [1.5 + 0.45 * a.cos(), -0.2 + 0.45 * a.sin(), z], ..Default::default() });
            }
        }
        m.v.push(PartVertex { pos: [3.0, 1.0, 7.0], ..Default::default() });
        let piece = Piece { hotspot: None, name: "fg mesh", mesh: m, group: airframe::G_TAIL, parent: None, pivot: Vec3::ZERO, axis: Vec3::X, anim: Anim::Fixed, role: Role::Other, vis: Vis::Always, glass: false, occluder: true };
        // (the flight model's jet sits mid-engine, a little off the nozzle's line)
        let n = nozzles(&[piece], &[Vec3::new(1.45, -0.15, 2.0)]);
        assert_eq!(n.len(), 1);
        let (at, r) = n[0];
        assert!((at.z - 5.0).abs() < 1e-4 && (at.x - 1.45).abs() < 1e-4, "{at:?}");
        assert!(r > 0.4 && r < 0.6, "radius {r}");
        // The Hornet's plume it borrows: one engine's, leaving the nozzle the shader expects.
        let [shell, other] = crate::aircraft::fa18::plume();
        // (the sheath, core and discs of each engine: 820 + 580 + 240 triangles, no haze)
        assert_eq!((shell.i.len() / 3, other.i.len() / 3), (1640, 1640));
        let xr = |m: &Mesh| m.v.iter().map(|v| v.pos[0]).fold((f32::MAX, f32::MIN), |(a, b), x| (a.min(x), b.max(x)));
        let ((r0, r1), (l0, l1)) = (xr(&shell), xr(&other));
        assert!(r0 > 0.0 && l1 < 0.0 && (r0 + l1).abs() < 0.01 && (r1 + l0).abs() < 0.01, "right {r0}..{r1}, left {l0}..{l1}");
        let zmin = shell.v.iter().map(|v| v.pos[2]).fold(f32::MAX, f32::min);
        assert!((zmin - crate::aircraft::fa18::PLUME_NOZZLE.z).abs() < 0.3, "front at {zmin}");
    }

    #[test]
    fn gear_selects_are_checked_as_it_flies() {
        let ext = Ext { dir: PathBuf::new(), fg_dir: String::new(), model: PathBuf::new(), paint: Paint::default(), origin: Vec3::ZERO, steer_max: 1.0, travel: Vec::new(), span: 10.0, jets: Vec::new() };
        let mut ld = Loader { ext: &ext, acs: Vec::new(), objs: Vec::new(), anims: Vec::new(), order: 0, warn: Vec::new(), paint: HashMap::new(), tex_over: HashMap::new() };
        let gear = |g: f32| move |i: Input| if i == Input::Gear { g } else { 0.0 };
        // The MiG-29's wheels: shown once the gear is out (hidden at rest, so they used to be left out altogether).
        let mig = "<animation><type>select</type><object-name>wheel.L</object-name><condition><greater-than><property>gear/gear[1]/position-norm</property><value>0.0</value></greater-than></condition></animation>";
        let doc = roxmltree::Document::parse(mig).unwrap();
        let d = ld.anim(doc.root_element(), "select", Mat4::IDENTITY).unwrap();
        assert!(d.shown && d.vis.is_none());
        let c = d.cond.expect("a live condition");
        assert!(!c.eval(&gear(0.0)) && c.eval(&gear(0.3)) && c.eval(&gear(1.0)));
        // A closed door, unless a switch we don't model is on: the switch stays at rest (off), the gear is live.
        let door = "<animation><type>select</type><object-name>door</object-name><condition><and><equals><property>/gear/gear[0]/position-norm</property><value>0</value></equals><not><property>controls/electric/battery-switch</property></not></and></condition></animation>";
        let doc = roxmltree::Document::parse(door).unwrap();
        let c = ld.anim(doc.root_element(), "select", Mat4::IDENTITY).unwrap().cond.expect("a live condition");
        assert!(c.eval(&gear(0.0)) && !c.eval(&gear(1.0)));
        // Nothing we model (a cockpit light switch): decided at rest, as before.
        let sw = "<animation><type>select</type><object-name>l</object-name><condition><property>controls/electric/battery-switch</property></condition></animation>";
        let doc = roxmltree::Document::parse(sw).unwrap();
        let d = ld.anim(doc.root_element(), "select", Mat4::IDENTITY).unwrap();
        assert!(d.cond.is_none() && !d.shown);
    }

    #[test]
    fn rest_conditions() {
        let doc = roxmltree::Document::parse("<condition><greater-than><property>engines/engine[0]/rpm</property><value>400</value></greater-than></condition>").unwrap();
        assert_eq!(condition(doc.root_element()), (false, Some(true)));
        let doc = roxmltree::Document::parse("<condition><not><property>sim/model/damaged</property></not></condition>").unwrap();
        assert!(condition(doc.root_element()).0);
    }
}
