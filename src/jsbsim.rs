//! A reader for JSBSim aircraft files (the XML flight models FlightGear and
//! JSBSim use), and a comparison of one against one of ours.
//!
//! Only what the aerodynamics need is read: the metrics, the mass and
//! inertia, the control surfaces' travel (from the flight control system),
//! and the aerodynamic functions, which are evaluated as JSBSim does: each
//! axis (LIFT, DRAG, SIDE in pounds; ROLL, PITCH, YAW in foot-pounds) is the
//! sum of its functions, and a function is a tree of operations (product,
//! sum, tables of one to three variables…) over properties (`aero/alpha-rad`,
//! `fcs/flap-pos-deg`…). The state is given as those properties; any the
//! file asks for and the state lacks reads 0, and is listed.
//!
//! Nothing here comes from JSBSim's code: the file format is documented in
//! the JSBSim reference manual. Aircraft files are read from wherever the
//! player keeps them; none ship with the game.
//!
//! `fablegame --jsbsim-compare path/to/aircraft.xml [--aircraft c172]` prints
//! the comparison (see `compare`).

use std::collections::{BTreeSet, HashMap};
use std::path::{Path, PathBuf};

/// An operation of an aerodynamic function.
#[derive(Clone, Debug)]
enum Func {
    Value(f64),
    /// a property, negated when written "-name"
    Prop(String, bool),
    Product(Vec<Func>),
    Sum(Vec<Func>),
    Difference(Vec<Func>),
    Quotient(Vec<Func>),
    Pow(Vec<Func>),
    Abs(Box<Func>),
    Sin(Box<Func>),
    Cos(Box<Func>),
    Tan(Box<Func>),
    Sqrt(Box<Func>),
    Min(Vec<Func>),
    Max(Vec<Func>),
    /// lt, le, gt, ge, eq, nt: 1 when true
    Cmp(&'static str, Vec<Func>),
    IfThen(Vec<Func>),
    /// the other operations, by their name
    Op(&'static str, Vec<Func>),
    Table(Table),
}

/// A table of one, two or three variables; out of its range it holds the end values.
#[derive(Clone, Debug)]
struct Table {
    /// row, column and table variables (name, negated)
    vars: Vec<(String, bool)>,
    rows: Vec<f64>,
    cols: Vec<f64>,
    /// [table][row][column] (one table, one column for a 1D one)
    data: Vec<Vec<Vec<f64>>>,
    /// the third variable's break points, one per table
    breaks: Vec<f64>,
}

/// The six aerodynamic axes.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash, PartialOrd, Ord)]
pub enum Axis {
    Lift,
    Drag,
    Side,
    Roll,
    Pitch,
    Yaw,
}

impl Axis {
    fn parse(s: &str) -> Option<Axis> {
        Some(match s.trim().to_uppercase().as_str() {
            "LIFT" => Axis::Lift,
            "DRAG" => Axis::Drag,
            "SIDE" => Axis::Side,
            "ROLL" => Axis::Roll,
            "PITCH" => Axis::Pitch,
            "YAW" => Axis::Yaw,
            _ => return None,
        })
    }
}

/// The properties of a state, and the names asked for that it lacked.
#[derive(Default, Debug)]
pub struct Props {
    pub values: HashMap<String, f64>,
    pub missing: BTreeSet<String>,
}

impl Props {
    pub fn set(&mut self, k: &str, v: f64) {
        self.values.insert(k.to_string(), v);
    }

    fn get(&mut self, k: &str) -> f64 {
        match self.values.get(k) {
            Some(v) => *v,
            None => {
                self.missing.insert(k.to_string());
                0.0
            }
        }
    }
}

/// An aircraft as its JSBSim file describes it (imperial units, as in the file).
#[derive(Debug)]
pub struct Aircraft {
    pub name: String,
    /// the flight model it was read from: "JSBSim", or "YASim" for one made from a YASim file (yasim.rs)
    pub source: &'static str,
    pub wing_area_ft2: f64,
    pub span_ft: f64,
    pub chord_ft: f64,
    /// roll, pitch, yaw (slug ft²)
    pub inertia: [f64; 3],
    /// empty, with the point masses (crew…) and the fuel
    pub weight_lbs: f64,
    /// control surface travel (radians) by output property ("fcs/elevator-pos-rad")
    pub travel: HashMap<String, (f64, f64)>,
    /// the empty weight, the point masses (crew, passengers, baggage), the
    /// fuel as loaded and with the tanks full (lbs)
    pub empty_lbs: f64,
    pub points_lbs: f64,
    pub fuel_lbs: f64,
    pub fuel_cap_lbs: f64,
    /// full flap travel (degrees), from the flaps' kinematic or scale component
    pub flaps_max_deg: Option<f64>,
    /// each engine: its file and its thruster's (names without ".xml")
    pub engines: Vec<(String, Option<String>)>,
    /// the file header: author, description, the license it states (name, URL), notes
    pub author: String,
    pub description: String,
    pub license: Option<(String, String)>,
    pub notes: String,
    /// helper functions, evaluated first (in order) into properties
    functions: Vec<(String, Func)>,
    axes: Vec<(Axis, Vec<(String, Func)>)>,
    /// geometry, structural frame (x aft, y right, z up; feet): the empty
    /// centre of gravity, the visual reference point (where a 3D model's
    /// origin goes), the pilot's eye
    pub cg: Option<[f64; 3]>,
    pub vrp: Option<[f64; 3]>,
    pub eye: Option<[f64; 3]>,
    /// where each engine's thrust acts (its thruster's location, or a YASim jet's position)
    pub jets: Vec<[f64; 3]>,
    /// the ground reactions, in file order (a 3D model's gear\[n\] is the n-th)
    pub contacts: Vec<Contact>,
}

/// A lift and drag polar for an aircraft read from another flight model
/// (see yasim.rs), given to an `Aircraft` as its aerodynamics.
#[derive(Clone, Debug)]
pub struct Polar {
    /// the clean lift curve: (angle of attack in degrees, CL), increasing in angle
    pub lift: Vec<(f64, f64)>,
    /// lift added by full flaps (at every angle), and their drag
    pub flap_cl: f64,
    pub cd_flap: f64,
    /// parasite drag, induced drag factor (CD = cd0 + k CL²), drag of the gear down
    pub cd0: f64,
    pub k: f64,
    pub cd_gear: f64,
}

impl Default for Aircraft {
    fn default() -> Aircraft {
        Aircraft { name: String::new(), source: "JSBSim", wing_area_ft2: 0.0, span_ft: 0.0, chord_ft: 0.0, inertia: [0.0; 3], weight_lbs: 0.0, travel: HashMap::new(), empty_lbs: 0.0, points_lbs: 0.0, fuel_lbs: 0.0, fuel_cap_lbs: 0.0, flaps_max_deg: None, engines: Vec::new(), author: String::new(), description: String::new(), license: None, notes: String::new(), functions: Vec::new(), axes: Vec::new(), cg: None, vrp: None, eye: None, jets: Vec::new(), contacts: Vec::new() }
    }
}

/// A point that touches the ground (structural frame, feet; lbs/ft and lbs/ft/s).
#[derive(Clone, Debug)]
pub struct Contact {
    /// a wheel (BOGEY), or part of the airframe (STRUCTURE)
    pub bogey: bool,
    pub at: [f64; 3],
    pub spring: f64,
    pub damping: f64,
    /// how far it steers (degrees; 360 = castering)
    pub max_steer: f64,
    pub brakes: bool,
    pub retractable: bool,
}

/// A `<location>` in feet.
fn location(n: roxmltree::Node) -> Option<[f64; 3]> {
    let k = match n.attribute("unit").unwrap_or("IN").to_uppercase().as_str() {
        "M" => 1.0 / 0.3048,
        "FT" => 1.0,
        _ => 1.0 / 12.0,
    };
    let c = |a: &str| child(n, a).and_then(|v| num(text(v))).unwrap_or(0.0) * k;
    Some([c("x"), c("y"), c("z")])
}

/// Forces (lbs) and moments (ft·lbs) on the six axes.
#[derive(Copy, Clone, Debug, Default)]
pub struct Forces {
    pub lift: f64,
    pub drag: f64,
    pub side: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

// ------------------------------------------------------------------ reading

fn num(s: &str) -> Option<f64> {
    s.trim().parse().ok()
}

/// Text on one line, its runs of white space as single spaces.
fn squash(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

fn text<'a>(n: roxmltree::Node<'a, 'a>) -> &'a str {
    n.text().unwrap_or("").trim()
}

fn child<'a>(n: roxmltree::Node<'a, 'a>, name: &str) -> Option<roxmltree::Node<'a, 'a>> {
    n.children().find(|c| c.has_tag_name(name))
}

/// A length, area, mass or inertia in the file's unit, to feet, square feet,
/// pounds or slug ft².
fn imperial(n: roxmltree::Node) -> Option<f64> {
    let v = num(text(n))?;
    Some(match n.attribute("unit").unwrap_or("").to_uppercase().as_str() {
        "M" => v / 0.3048,
        "IN" => v / 12.0,
        "M2" => v / 0.092_903,
        "KG" => v / 0.453_592,
        "KG*M2" => v / 1.355_82,
        _ => v,
    })
}

fn prop_ref(s: &str) -> (String, bool) {
    let s = s.trim();
    match s.strip_prefix('-') {
        Some(r) => (r.trim().to_string(), true),
        None => (s.to_string(), false),
    }
}

fn parse_func(n: roxmltree::Node) -> Result<Func, String> {
    let args = || -> Result<Vec<Func>, String> { n.children().filter(|c| c.is_element() && !c.has_tag_name("description")).map(parse_func).collect() };
    let one = || -> Result<Box<Func>, String> { Ok(Box::new(args()?.into_iter().next().ok_or_else(|| format!("<{}> without an argument", n.tag_name().name()))?)) };
    Ok(match n.tag_name().name() {
        "value" | "v" => Func::Value(num(text(n)).ok_or_else(|| format!("bad <value> {:?}", text(n)))?),
        "property" | "p" => {
            let (p, neg) = prop_ref(text(n));
            Func::Prop(p, neg)
        }
        "product" => Func::Product(args()?),
        "sum" => Func::Sum(args()?),
        "difference" => Func::Difference(args()?),
        "quotient" => Func::Quotient(args()?),
        "pow" => Func::Pow(args()?),
        "abs" => Func::Abs(one()?),
        "sin" => Func::Sin(one()?),
        "cos" => Func::Cos(one()?),
        "tan" => Func::Tan(one()?),
        "sqrt" => Func::Sqrt(one()?),
        "min" => Func::Min(args()?),
        "max" => Func::Max(args()?),
        "lt" => Func::Cmp("lt", args()?),
        "le" => Func::Cmp("le", args()?),
        "gt" => Func::Cmp("gt", args()?),
        "ge" => Func::Cmp("ge", args()?),
        "eq" => Func::Cmp("eq", args()?),
        "nt" => Func::Cmp("nt", args()?),
        "ifthen" => Func::IfThen(args()?),
        "atan2" => Func::Op("atan2", args()?),
        "atan" | "asin" | "acos" | "exp" | "ln" | "log2" | "log10" | "floor" | "ceil" | "sign" | "toradians" | "todegrees" | "not" | "integer" | "fraction" => Func::Op(n.tag_name().name().to_string().leak(), args()?),
        "and" | "or" | "avg" | "mod" | "fmod" | "switch" | "interpolate1d" => Func::Op(n.tag_name().name().to_string().leak(), args()?),
        // noise and integrators: at rest
        "random" | "urandom" => Func::Value(0.0),
        "table" => Func::Table(parse_table(n)?),
        // A function written inside another: its single operation.
        "function" => *one()?,
        other => return Err(format!("unsupported operation <{other}>")),
    })
}

fn parse_table(n: roxmltree::Node) -> Result<Table, String> {
    let mut vars = vec![(String::new(), false); 3];
    let mut nvars = 0;
    for v in n.children().filter(|c| c.has_tag_name("independentVar")) {
        let k = match v.attribute("lookup").unwrap_or("row") {
            "column" => 1,
            "table" => 2,
            _ => 0,
        };
        vars[k] = prop_ref(text(v));
        nvars = nvars.max(k + 1);
    }
    vars.truncate(nvars.max(1));
    let datas: Vec<roxmltree::Node> = n.children().filter(|c| c.has_tag_name("tableData")).collect();
    if datas.is_empty() {
        return Err("<table> without <tableData>".into());
    }
    let lines = |d: roxmltree::Node| -> Result<Vec<Vec<f64>>, String> {
        // (all its text, around any comments)
        d.children()
            .filter(|c| c.is_text())
            .filter_map(|c| c.text())
            .collect::<Vec<_>>()
            .join("\n")
            .lines()
            .map(|l| l.split_whitespace().map(|t| t.parse::<f64>().map_err(|_| format!("bad number {t:?} in a table"))).collect::<Result<Vec<_>, _>>())
            .filter(|r| !matches!(r, Ok(v) if v.is_empty()))
            .collect()
    };
    let mut t = Table { vars, rows: Vec::new(), cols: Vec::new(), data: Vec::new(), breaks: Vec::new() };
    // (only a table of three variables has several; any extra after the first is left out)
    let n_data = if nvars >= 3 { datas.len() } else { 1 };
    for d in datas.into_iter().take(n_data) {
        let l = lines(d)?;
        if nvars <= 1 {
            // x y pairs (several to a line allowed)
            let flat: Vec<f64> = l.into_iter().flatten().collect();
            if flat.len() < 2 || flat.len() % 2 != 0 {
                return Err("a 1D table needs x y pairs".into());
            }
            t.rows = flat.iter().step_by(2).copied().collect();
            t.data.push(flat.iter().skip(1).step_by(2).map(|v| vec![*v]).collect());
        } else {
            // the first line is the column break points, then a row each: key, values
            let (head, body) = l.split_first().ok_or("an empty 2D table")?;
            t.cols = head.clone();
            t.rows = body.iter().map(|r| r[0]).collect();
            let mut tab = Vec::new();
            for r in body {
                if r.len() != t.cols.len() + 1 {
                    return Err(format!("a table row has {} values for {} columns", r.len() - 1, t.cols.len()));
                }
                tab.push(r[1..].to_vec());
            }
            t.data.push(tab);
            if let Some(b) = d.attribute("breakPoint") {
                t.breaks.push(num(b).ok_or("bad breakPoint")?);
            }
        }
    }
    Ok(t)
}

/// A section, from the file itself or from the file its `file` attribute names.
/// JSBSim looks for it beside the aircraft file, then in its `Systems/`
/// folder. Of the sections, only the ones the aerodynamics need must be
/// found: a missing system (autopilot, electrics…) is left out.
fn section(doc_dir: &Path, n: roxmltree::Node) -> Result<Option<(String, PathBuf)>, String> {
    let Some(f) = n.attribute("file") else { return Ok(None) };
    let mut tried = None;
    for dir in [doc_dir.to_path_buf(), doc_dir.join("Systems")] {
        let mut p = dir.join(f);
        if p.extension().is_none() {
            p.set_extension("xml");
        }
        match std::fs::read_to_string(&p) {
            Ok(s) => return Ok(Some((s, p))),
            Err(e) => tried = tried.or(Some(format!("{}: {e}", p.display()))),
        }
    }
    match n.tag_name().name() {
        "metrics" | "mass_balance" | "aerodynamics" | "propulsion" => Err(tried.unwrap_or_default()),
        _ => Ok(None),
    }
}

impl Aircraft {
    pub fn load(path: &Path) -> Result<Aircraft, String> {
        let s = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&s, path.parent().unwrap_or(Path::new(".")))
    }

    pub fn parse(xml: &str, dir: &Path) -> Result<Aircraft, String> {
        let doc = roxmltree::Document::parse(xml).map_err(|e| e.to_string())?;
        let root = doc.root_element();
        if !root.has_tag_name("fdm_config") {
            return Err("not a JSBSim aircraft (no <fdm_config>)".into());
        }
        let mut a = Aircraft { name: root.attribute("name").unwrap_or("?").to_string(), ..Default::default() };
        for sec in root.children().filter(|c| c.is_element()) {
            // (A section may live in a file of its own.)
            let other = section(dir, sec)?;
            let other_doc = match &other {
                Some((s, p)) => Some(roxmltree::Document::parse(s).map_err(|e| format!("{}: {e}", p.display()))?),
                None => None,
            };
            let n = other_doc.as_ref().map(|d| d.root_element()).unwrap_or(sec);
            match sec.tag_name().name() {
                "fileheader" => {
                    let t = |k: &str| child(n, k).map(|c| squash(text(c))).unwrap_or_default();
                    (a.author, a.description) = (t("author"), t("description"));
                    a.notes = n.children().filter(|c| c.has_tag_name("note")).map(|c| squash(text(c))).collect::<Vec<_>>().join("\n\n");
                    if let Some(l) = child(n, "license") {
                        // as attributes, or as elements
                        let get = |k: &str| l.attribute(k).map(squash).or_else(|| child(l, k).map(|c| squash(text(c)))).unwrap_or_default();
                        let (name, url) = (get("licenseName"), get("licenseURL"));
                        let name = if name.is_empty() { squash(text(l)) } else { name };
                        if !name.is_empty() || !url.is_empty() {
                            a.license = Some((name, url));
                        }
                    }
                }
                "metrics" => {
                    let get = |k: &str| child(n, k).and_then(imperial).unwrap_or(0.0);
                    (a.wing_area_ft2, a.span_ft, a.chord_ft) = (get("wingarea"), get("wingspan"), get("chord"));
                    for l in n.children().filter(|c| c.has_tag_name("location")) {
                        match l.attribute("name").unwrap_or("").to_uppercase().as_str() {
                            "VRP" => a.vrp = location(l),
                            "EYEPOINT" => a.eye = location(l),
                            _ => {}
                        }
                    }
                }
                "mass_balance" => {
                    let get = |k: &str| child(n, k).and_then(imperial).unwrap_or(0.0);
                    a.inertia = [get("ixx"), get("iyy"), get("izz")];
                    a.empty_lbs = get("emptywt");
                    if let Some(l) = n.children().find(|c| c.has_tag_name("location")) {
                        a.cg = location(l);
                    }
                    a.weight_lbs += a.empty_lbs;
                    for pm in n.children().filter(|c| c.has_tag_name("pointmass")) {
                        let w = child(pm, "weight").and_then(imperial).unwrap_or(0.0);
                        a.weight_lbs += w;
                        a.points_lbs += w;
                    }
                }
                "propulsion" => {
                    for tank in n.children().filter(|c| c.has_tag_name("tank")) {
                        let fuel = tank.attribute("type").is_none_or(|t| t.eq_ignore_ascii_case("FUEL"));
                        let have = child(tank, "contents").and_then(imperial).unwrap_or(0.0);
                        a.weight_lbs += have;
                        if fuel {
                            a.fuel_lbs += have;
                            a.fuel_cap_lbs += child(tank, "capacity").and_then(imperial).unwrap_or(have);
                        }
                    }
                    for e in n.children().filter(|c| c.has_tag_name("engine")) {
                        let Some(f) = e.attribute("file") else { continue };
                        let th = child(e, "thruster").and_then(|t| t.attribute("file")).map(str::to_string);
                        if let Some(at) = child(e, "thruster").and_then(|t| child(t, "location")).and_then(location) {
                            a.jets.push(at);
                        }
                        a.engines.push((f.to_string(), th));
                    }
                }
                "ground_reactions" => {
                    for c in n.children().filter(|c| c.has_tag_name("contact")) {
                        let Some(at) = child(c, "location").and_then(location) else { continue };
                        let get = |k: &str| child(c, k).and_then(|v| num(text(v))).unwrap_or(0.0);
                        // (spring and damper in lbs/ft unless stated otherwise)
                        let per_ft = |k: &str| {
                            child(c, k).map(|v| {
                                let x = num(text(v)).unwrap_or(0.0);
                                match v.attribute("unit").unwrap_or("").to_uppercase().as_str() {
                                    "N/M" => x * 0.3048 / 4.448_22,
                                    "N/M/SEC" | "N/M/S" => x * 0.3048 / 4.448_22,
                                    "LBS/IN" => x * 12.0,
                                    "LBS/IN/SEC" => x * 12.0,
                                    _ => x,
                                }
                            })
                            .unwrap_or(0.0)
                        };
                        let brake = child(c, "brake_group").map(|b| text(b).to_uppercase()).unwrap_or_default();
                        a.contacts.push(Contact {
                            bogey: c.attribute("type").unwrap_or("BOGEY").eq_ignore_ascii_case("BOGEY"),
                            at,
                            spring: per_ft("spring_coeff"),
                            damping: per_ft("damping_coeff"),
                            max_steer: get("max_steer"),
                            brakes: !brake.is_empty() && brake != "NONE",
                            retractable: get("retractable") > 0.5,
                        });
                    }
                }
                "flight_control" | "system" | "autopilot" => {
                    // Control surface travel: an aerosurface_scale maps the command onto
                    // its range, times its gain.
                    for c in n.descendants().filter(|c| c.has_tag_name("aerosurface_scale")) {
                        let (Some(out), Some(range)) = (child(c, "output"), child(c, "range")) else { continue };
                        let g = child(c, "gain").and_then(|g| num(text(g))).unwrap_or(1.0);
                        let lim = |k: &str| child(range, k).and_then(|v| num(text(v)));
                        if let (Some(lo), Some(hi)) = (lim("min"), lim("max")) {
                            let (x, y) = (lo * g, hi * g);
                            a.travel.insert(text(out).to_string(), (x.min(y), x.max(y)));
                            if text(out).contains("flap-pos-deg") {
                                a.flaps_max_deg = Some(x.abs().max(y.abs()));
                            }
                        }
                    }
                    // The flaps: a kinematic component running through its settings (degrees).
                    for c in n.descendants().filter(|c| c.has_tag_name("kinematic")) {
                        let Some(out) = child(c, "output") else { continue };
                        if !text(out).contains("flap-pos-deg") {
                            continue;
                        }
                        let most = c.descendants().filter(|p| p.has_tag_name("position")).filter_map(|p| num(text(p))).fold(0.0f64, |m, v| m.max(v.abs()));
                        if most > 0.0 {
                            a.flaps_max_deg = Some(most);
                        }
                    }
                }
                "aerodynamics" => {
                    for c in n.children().filter(|c| c.is_element()) {
                        match c.tag_name().name() {
                            "function" => {
                                let name = c.attribute("name").unwrap_or("").to_string();
                                a.functions.push((name, parse_func(c)?));
                            }
                            "axis" => {
                                let Some(axis) = c.attribute("name").and_then(Axis::parse) else { continue };
                                let mut fs = Vec::new();
                                for f in c.children().filter(|f| f.has_tag_name("function")) {
                                    let name = f.attribute("name").unwrap_or("").to_string();
                                    fs.push((name.clone(), parse_func(f).map_err(|e| format!("{name}: {e}"))?));
                                }
                                a.axes.push((axis, fs));
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
        if a.axes.is_empty() {
            return Err("no <aerodynamics> axes".into());
        }
        Ok(a)
    }

    /// Gives it a polar as its aerodynamics, in place of any it had: CL is
    /// the lift curve plus the flaps' share (`fcs/flap-pos-norm`), CD the
    /// polar's with the flaps' and the gear's drag (`gear/gear-pos-norm`).
    pub fn set_polar(&mut self, p: &Polar) {
        let prop = |k: &str| Func::Prop(k.to_string(), false);
        let curve = Table { vars: vec![("aero/alpha-deg".into(), false)], rows: p.lift.iter().map(|x| x.0).collect(), cols: Vec::new(), data: vec![p.lift.iter().map(|x| vec![x.1]).collect()], breaks: Vec::new() };
        let cl = Func::Sum(vec![Func::Table(curve), Func::Product(vec![prop("fcs/flap-pos-norm"), Func::Value(p.flap_cl)])]);
        let cd = Func::Sum(vec![
            Func::Value(p.cd0),
            Func::Product(vec![Func::Value(p.k), prop("aero/polar/cl"), prop("aero/polar/cl")]),
            Func::Product(vec![prop("fcs/flap-pos-norm"), Func::Value(p.cd_flap)]),
            Func::Product(vec![prop("gear/gear-pos-norm"), Func::Value(p.cd_gear)]),
        ]);
        let force = |c: &str| Func::Product(vec![prop("aero/qbar-psf"), prop("metrics/Sw-sqft"), prop(c)]);
        self.functions = vec![("aero/polar/cl".into(), cl), ("aero/polar/cd".into(), cd)];
        self.axes = vec![(Axis::Lift, vec![("aero/polar/lift".into(), force("aero/polar/cl"))]), (Axis::Drag, vec![("aero/polar/drag".into(), force("aero/polar/cd"))])];
    }

    // ------------------------------------------------------------ evaluating

    /// The forces and moments for a state (see `state`). The helper functions
    /// and each coefficient are left in `p` as properties too.
    pub fn forces(&self, p: &mut Props) -> Forces {
        p.set("metrics/Sw-sqft", self.wing_area_ft2);
        p.set("metrics/bw-ft", self.span_ft);
        p.set("metrics/cbarw-ft", self.chord_ft);
        for (name, f) in &self.functions {
            let v = eval(f, p);
            p.set(name, v);
        }
        let mut out = Forces::default();
        for (axis, fs) in &self.axes {
            let mut total = 0.0;
            for (name, f) in fs {
                let v = eval(f, p);
                p.set(name, v);
                total += v;
            }
            match axis {
                Axis::Lift => out.lift += total,
                Axis::Drag => out.drag += total,
                Axis::Side => out.side += total,
                Axis::Roll => out.roll += total,
                Axis::Pitch => out.pitch += total,
                Axis::Yaw => out.yaw += total,
            }
        }
        out
    }

    /// A steady state as properties: true airspeed (kt), angle of attack and
    /// sideslip (degrees), air density (slug/ft³), body rates (rad/s), the
    /// flaps (degrees) and the control surfaces (radians), high above the ground.
    #[allow(clippy::too_many_arguments)]
    pub fn state(&self, kt: f64, alpha_deg: f64, beta_deg: f64, rho: f64, pqr: [f64; 3], flaps_deg: f64, flaps_max: f64, elevator: f64, aileron: f64, rudder: f64) -> Props {
        let mut p = Props::default();
        let v = kt * 1.687_81;
        let (a, b) = (alpha_deg.to_radians(), beta_deg.to_radians());
        let qbar = 0.5 * rho * v * v;
        for (k, x) in [
            ("velocities/vt-fps", v),
            ("velocities/u-aero-fps", v * a.cos() * b.cos()),
            ("velocities/v-aero-fps", v * b.sin()),
            ("velocities/w-aero-fps", v * a.sin() * b.cos()),
            ("velocities/mach", v / 1116.4),
            ("velocities/p-aero-rad_sec", pqr[0]),
            ("velocities/q-aero-rad_sec", pqr[1]),
            ("velocities/r-aero-rad_sec", pqr[2]),
            ("velocities/p-rad_sec", pqr[0]),
            ("velocities/q-rad_sec", pqr[1]),
            ("velocities/r-rad_sec", pqr[2]),
            ("atmosphere/rho-slugs_ft3", rho),
            ("aero/qbar-psf", qbar),
            ("aero/qbarUW-psf", qbar),
            ("aero/alpha-rad", a),
            ("aero/alpha-wing-rad", a),
            ("aero/qbar-area", qbar * self.wing_area_ft2),
            ("aero/qbarUV-psf", qbar),
            ("aero/alpha-deg", alpha_deg),
            ("aero/mag-alpha-rad", a.abs()),
            ("aero/beta-rad", b),
            ("aero/beta-deg", beta_deg),
            ("aero/mag-beta-rad", b.abs()),
            ("aero/mag-beta-deg", beta_deg.abs()),
            ("aero/alphadot-rad_sec", 0.0),
            ("aero/betadot-rad_sec", 0.0),
            ("aero/bi2vel", self.span_ft / (2.0 * v.max(1.0))),
            ("aero/ci2vel", self.chord_ft / (2.0 * v.max(1.0))),
            ("aero/h_b-mac-ft", 10.0),
            ("aero/h_b-cg-ft", 10.0),
            ("aero/stall-hyst-norm", 0.0),
            ("fcs/flap-pos-deg", flaps_deg),
            ("fcs/flap-pos-norm", if flaps_max > 0.0 { flaps_deg / flaps_max } else { 0.0 }),
            ("fcs/elevator-pos-rad", elevator),
            ("fcs/elevator-pos-deg", elevator.to_degrees()),
            ("fcs/mag-elevator-pos-rad", elevator.abs()),
            ("fcs/left-aileron-pos-rad", aileron),
            ("fcs/right-aileron-pos-rad", -aileron),
            ("fcs/aileron-pos-rad", aileron),
            ("fcs/effective-aileron-pos", aileron),
            ("fcs/rudder-pos-rad", rudder),
            ("fcs/rudder-pos-deg", rudder.to_degrees()),
            ("fcs/speedbrake-pos-norm", 0.0),
            ("fcs/spoiler-pos-norm", 0.0),
            ("gear/gear-pos-norm", 1.0),
            ("propulsion/engine/prop-induced-velocity_fps", 0.0),
        ] {
            p.set(k, x);
        }
        p
    }

    /// The force and moment coefficients for a state: (CL, CD, CY, Cl, Cm, Cn).
    pub fn coefficients(&self, p: &mut Props) -> [f64; 6] {
        let f = self.forces(p);
        let qs = p.values["aero/qbar-psf"] * self.wing_area_ft2;
        let qs = if qs.abs() < 1e-9 { 1.0 } else { qs };
        [f.lift / qs, f.drag / qs, f.side / qs, f.roll / (qs * self.span_ft), f.pitch / (qs * self.chord_ft), f.yaw / (qs * self.span_ft)]
    }
}

fn eval(f: &Func, p: &mut Props) -> f64 {
    let all = |v: &Vec<Func>, p: &mut Props| -> Vec<f64> { v.iter().map(|f| eval(f, p)).collect() };
    match f {
        Func::Value(v) => *v,
        Func::Prop(k, neg) => {
            let v = p.get(k);
            if *neg { -v } else { v }
        }
        Func::Product(v) => all(v, p).iter().product(),
        Func::Sum(v) => all(v, p).iter().sum(),
        Func::Difference(v) => {
            let x = all(v, p);
            x.first().copied().unwrap_or(0.0) - x.iter().skip(1).sum::<f64>()
        }
        Func::Quotient(v) => {
            let x = all(v, p);
            let (a, b) = (x.first().copied().unwrap_or(0.0), x.get(1).copied().unwrap_or(1.0));
            if b == 0.0 { 0.0 } else { a / b }
        }
        Func::Pow(v) => {
            let x = all(v, p);
            x.first().copied().unwrap_or(0.0).powf(x.get(1).copied().unwrap_or(1.0))
        }
        Func::Abs(a) => eval(a, p).abs(),
        Func::Sin(a) => eval(a, p).sin(),
        Func::Cos(a) => eval(a, p).cos(),
        Func::Tan(a) => eval(a, p).tan(),
        Func::Sqrt(a) => eval(a, p).max(0.0).sqrt(),
        Func::Min(v) => all(v, p).into_iter().fold(f64::INFINITY, f64::min),
        Func::Max(v) => all(v, p).into_iter().fold(f64::NEG_INFINITY, f64::max),
        Func::Cmp(op, v) => {
            let x = all(v, p);
            let (a, b) = (x.first().copied().unwrap_or(0.0), x.get(1).copied().unwrap_or(0.0));
            let t = match *op {
                "lt" => a < b,
                "le" => a <= b,
                "gt" => a > b,
                "ge" => a >= b,
                "eq" => a == b,
                _ => a != b,
            };
            if t { 1.0 } else { 0.0 }
        }
        Func::IfThen(v) => {
            let x = all(v, p);
            if x.first().copied().unwrap_or(0.0) != 0.0 { x.get(1).copied().unwrap_or(0.0) } else { x.get(2).copied().unwrap_or(0.0) }
        }
        Func::Op(op, v) => {
            let x = all(v, p);
            let a = x.first().copied().unwrap_or(0.0);
            let b = x.get(1).copied().unwrap_or(0.0);
            match *op {
                "atan2" => a.atan2(b),
                "atan" => a.atan(),
                "asin" => a.clamp(-1.0, 1.0).asin(),
                "acos" => a.clamp(-1.0, 1.0).acos(),
                "exp" => a.exp(),
                "ln" => if a > 0.0 { a.ln() } else { 0.0 },
                "log2" => if a > 0.0 { a.log2() } else { 0.0 },
                "log10" => if a > 0.0 { a.log10() } else { 0.0 },
                "floor" => a.floor(),
                "ceil" => a.ceil(),
                "sign" => if a < 0.0 { -1.0 } else { 1.0 },
                "toradians" => a.to_radians(),
                "todegrees" => a.to_degrees(),
                "not" => if a == 0.0 { 1.0 } else { 0.0 },
                "integer" => a.trunc(),
                "fraction" => a.fract(),
                "and" => if x.iter().all(|v| *v != 0.0) { 1.0 } else { 0.0 },
                "or" => if x.iter().any(|v| *v != 0.0) { 1.0 } else { 0.0 },
                "avg" => if x.is_empty() { 0.0 } else { x.iter().sum::<f64>() / x.len() as f64 },
                "mod" | "fmod" => if b != 0.0 { a % b } else { 0.0 },
                // the first argument picks one of the rest
                "switch" => x.get(1 + a.max(0.0) as usize).copied().unwrap_or(0.0),
                // index, then x y pairs
                "interpolate1d" => {
                    let (xs, ys): (Vec<f64>, Vec<f64>) = x[1.min(x.len())..].chunks(2).filter(|c| c.len() == 2).map(|c| (c[0], c[1])).unzip();
                    interp(&xs, &ys, a)
                }
                _ => 0.0,
            }
        }
        Func::Table(t) => {
            let var = |k: usize, p: &mut Props| -> f64 {
                let (name, neg) = &t.vars[k.min(t.vars.len() - 1)];
                let v = p.get(name);
                if *neg { -v } else { v }
            };
            let r = var(0, p);
            match t.vars.len() {
                1 => interp(&t.rows, &t.data[0].iter().map(|row| row[0]).collect::<Vec<_>>(), r),
                2 => table2(&t.rows, &t.cols, &t.data[0], r, var(1, p)),
                _ => {
                    let (c, z) = (var(1, p), var(2, p));
                    let vals: Vec<f64> = t.data.iter().map(|d| table2(&t.rows, &t.cols, d, r, c)).collect();
                    interp(&t.breaks, &vals, z)
                }
            }
        }
    }
}

/// Linear interpolation in a table, held at its ends.
fn interp(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    // (a table with more break points than values is read as far as both go)
    let n = xs.len().min(ys.len());
    let (xs, ys) = (&xs[..n], &ys[..n]);
    match n {
        0 => 0.0,
        1 => ys[0],
        _ => {
            if x <= xs[0] {
                return ys[0];
            }
            let last = xs.len() - 1;
            if x >= xs[last] {
                return ys[last];
            }
            let i = xs.windows(2).position(|w| x >= w[0] && x <= w[1]).unwrap_or(0);
            let t = (x - xs[i]) / (xs[i + 1] - xs[i]).max(1e-12);
            ys[i] + (ys[i + 1] - ys[i]) * t
        }
    }
}

fn table2(rows: &[f64], cols: &[f64], data: &[Vec<f64>], r: f64, c: f64) -> f64 {
    let by_row: Vec<f64> = data.iter().map(|row| interp(cols, row, c)).collect();
    interp(rows, &by_row, r)
}

// ------------------------------------------------------------------ comparing

/// Least-squares line through points: (intercept, slope).
fn fit(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len() as f64;
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let (mx, my) = (sx / n, sy / n);
    let sxx: f64 = pts.iter().map(|p| (p.0 - mx).powi(2)).sum();
    let sxy: f64 = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
    let b = if sxx > 0.0 { sxy / sxx } else { 0.0 };
    (my - b * mx, b)
}

/// The comparison of a JSBSim aircraft with one of ours, as a text report:
/// geometry and mass, the lift and drag curves clean and with full flaps,
/// stall, best glide, roll rate at full aileron (JSBSim's from its roll
/// derivatives; ours flown), and the values of our `Spec` that would match
/// the JSBSim aerodynamics (as a start for tuning by hand, not a verdict:
/// ours is a handling model, JSBSim's a build-up of coefficients).
pub fn compare(j: &Aircraft, spec: &crate::aircraft::Spec, our_roll_rate: Option<f64>) -> String {
    use std::fmt::Write;
    let mut o = String::new();
    let rho = 0.002_377; // sea level, slug/ft³
    let flaps_max = spec.flaps.last().copied().unwrap_or(0.0);
    let jmass = j.weight_lbs * 0.453_592;
    let jarea = j.wing_area_ft2 * 0.092_903;
    let jspan = j.span_ft * 0.3048;
    let _ = writeln!(o, "JSBSim \"{}\"  vs  {}\n", j.name, spec.name);
    let _ = writeln!(o, "{:<28}{:>12}{:>12}", "", "JSBSim", "ours");
    let _ = writeln!(o, "{:<28}{:>12.1}{:>12.1}", "wing area (m²)", jarea, spec.wing_area);
    let _ = writeln!(o, "{:<28}{:>12.2}{:>12.2}", "span (m)", jspan, spec.b);
    let _ = writeln!(o, "{:<28}{:>12.0}{:>12.0}", "mass (kg)", jmass, spec.mass);
    let si = 1.355_82;
    let _ = writeln!(o, "{:<28}{:>12.0}{:>12.0}", "inertia roll (kg m²)", j.inertia[0] * si, spec.inertia.z);
    let _ = writeln!(o, "{:<28}{:>12.0}{:>12.0}", "inertia pitch (kg m²)", j.inertia[1] * si, spec.inertia.x);
    let _ = writeln!(o, "{:<28}{:>12.0}{:>12.0}", "inertia yaw (kg m²)", j.inertia[2] * si, spec.inertia.y);

    // The curves: JSBSim's at 90 kt, ours (the same formula as in flight, intact, gear down).
    let mut missing = BTreeSet::new();
    let mut jcoef = |alpha: f64, flaps: f64| -> [f64; 6] {
        let mut p = j.state(90.0, alpha, 0.0, rho, [0.0; 3], flaps, flaps_max, 0.0, 0.0, 0.0);
        let c = j.coefficients(&mut p);
        missing.extend(p.missing);
        c
    };
    let ours = |alpha_deg: f64, flaps: f64| -> (f64, f64) {
        let a = alpha_deg.to_radians();
        let flap_cl = spec.flap_cl * flaps;
        let stall_a = spec.stall_alpha - spec.stall_flap * flaps;
        crate::plane::lift_drag(spec, a, 0.0, flap_cl, crate::plane::stall_amount(stall_a, a), flaps, 1.0, 0.0, 1.0)
    };
    let alphas: Vec<f64> = (-3..=12).map(|k| k as f64 * 2.0).collect();
    let _ = writeln!(o, "\nLift and drag (90 kt, sea level, out of ground effect; ours with the gear down)");
    let _ = writeln!(o, "{:>7} {:>9}{:>9} {:>9}{:>9} | {:>9}{:>9} {:>9}{:>9}", "alpha", "CL jsb", "CL ours", "CD jsb", "CD ours", "CLf jsb", "CLf ours", "CDf jsb", "CDf ours");
    let mut rows = Vec::new();
    for &a in &alphas {
        let (c0, c1) = (jcoef(a, 0.0), jcoef(a, flaps_max));
        let (o0, o1) = (ours(a, 0.0), ours(a, 1.0));
        let _ = writeln!(o, "{:>6.0}° {:>9.3}{:>9.3} {:>9.4}{:>9.4} | {:>9.3}{:>9.3} {:>9.4}{:>9.4}", a, c0[0], o0.0, c0[1], o0.1, c1[0], o1.0, c1[1], o1.1);
        rows.push((a, c0, c1, o0, o1));
    }
    let _ = writeln!(o, "(CLf, CDf: flaps at {flaps_max:.0}°)");

    // Stall and best glide.
    let fine: Vec<f64> = (-40..=300).map(|k| k as f64 * 0.1).collect();
    let peak = |f: &dyn Fn(f64) -> f64| fine.iter().map(|&a| (a, f(a))).fold((0.0, f64::MIN), |m, x| if x.1 > m.1 { x } else { m });
    let (ja0, jcl0) = peak(&|a| j.coefficients(&mut j.state(90.0, a, 0.0, rho, [0.0; 3], 0.0, flaps_max, 0.0, 0.0, 0.0))[0]);
    let (ja1, jcl1) = peak(&|a| j.coefficients(&mut j.state(90.0, a, 0.0, rho, [0.0; 3], flaps_max, flaps_max, 0.0, 0.0, 0.0))[0]);
    let (oa0, ocl0) = peak(&|a| ours(a, 0.0).0);
    let (oa1, ocl1) = peak(&|a| ours(a, 1.0).0);
    let vs = |m: f64, s: f64, cl: f64| (2.0 * m * 9.81 / (1.225 * s * cl.max(0.1))).sqrt() * 1.943_84;
    let ld = |f: &dyn Fn(f64) -> (f64, f64)| fine.iter().map(|&a| f(a)).filter(|c| c.0 > 0.05).map(|c| (c.0 / c.1, c.0)).fold((0.0, 0.0), |m, x| if x.0 > m.0 { x } else { m });
    let jld = ld(&|a| {
        let c = j.coefficients(&mut j.state(90.0, a, 0.0, rho, [0.0; 3], 0.0, flaps_max, 0.0, 0.0, 0.0));
        (c[0], c[1])
    });
    let old = ld(&|a| ours(a, 0.0));
    let _ = writeln!(o, "\n{:<28}{:>12}{:>12}", "", "JSBSim", "ours");
    let _ = writeln!(o, "{:<28}{:>11.2} {:>11.2}", "CL max clean", jcl0, ocl0);
    let _ = writeln!(o, "{:<28}{:>11.1}°{:>11.1}°", "  at alpha", ja0, oa0);
    let _ = writeln!(o, "{:<28}{:>11.2} {:>11.2}", "CL max flaps", jcl1, ocl1);
    let _ = writeln!(o, "{:<28}{:>11.1}°{:>11.1}°", "  at alpha", ja1, oa1);
    let _ = writeln!(o, "{:<28}{:>9.0} kt{:>9.0} kt", "stall speed clean", vs(jmass, jarea, jcl0), vs(spec.mass, spec.wing_area, ocl0));
    let _ = writeln!(o, "{:<28}{:>9.0} kt{:>9.0} kt", "stall speed flaps", vs(jmass, jarea, jcl1), vs(spec.mass, spec.wing_area, ocl1));
    let _ = writeln!(o, "{:<28}{:>12.1}{:>12.1}", "best L/D (clean)", jld.0, old.0);
    let _ = writeln!(o, "{:<28}{:>12.2}{:>12.2}", "  at CL", jld.1, old.1);

    // Roll: JSBSim's steady rate at full aileron is where the aileron's moment
    // meets the damping's (Clδa δa = -Clp p b/2V).
    let v = 90.0 * 1.687_81;
    let roll_at = |ail: f64, p_rate: f64| j.forces(&mut j.state(90.0, 2.0, 0.0, rho, [p_rate, 0.0, 0.0], 0.0, flaps_max, 0.0, ail, 0.0)).roll;
    let (lo, hi) = j.travel.get("fcs/left-aileron-pos-rad").or_else(|| j.travel.get("fcs/aileron-pos-rad")).copied().unwrap_or((-0.3, 0.3));
    let damp = roll_at(0.0, 1.0) - roll_at(0.0, 0.0);
    let rate = |ail: f64| if damp.abs() > 1e-9 { -(roll_at(ail, 0.0) - roll_at(0.0, 0.0)) / damp } else { 0.0 };
    let jr = 0.5 * (rate(hi).abs() + rate(lo).abs());
    let _ = writeln!(o, "{:<28}{:>8.0} °/s{:>8} °/s", "roll rate, full aileron 90kt", jr.to_degrees(), our_roll_rate.map(|r| format!("{:.0}", r.to_degrees())).unwrap_or("-".into()));
    let _ = writeln!(o, "  (JSBSim aileron travel {:.0}° / {:.0}°, roll damping Clp·b/2V {:.3} per rad/s)", lo.to_degrees(), hi.to_degrees(), damp / (rho * 0.5 * v * v * j.wing_area_ft2 * j.span_ft));

    // What our Spec would need to match: fitted over the straight part of the lift curve.
    let lin: Vec<_> = rows.iter().filter(|r| r.0 >= -4.0 && r.0 <= 8.0).collect();
    let (cl0, cla) = fit(&lin.iter().map(|r| (r.0.to_radians(), r.1[0])).collect::<Vec<_>>());
    let (cd0, k) = fit(&lin.iter().map(|r| (r.1[0] * r.1[0], r.1[1])).collect::<Vec<_>>());
    let at0 = rows.iter().find(|r| r.0 == 0.0).copied();
    let _ = writeln!(o, "\nOur Spec, and what would match the JSBSim curves (a start for tuning, by hand):");
    let line = |o: &mut String, k: &str, now: f64, want: f64| {
        let _ = writeln!(o, "  {k:<14}{now:>9.3}  ->{want:>9.3}");
    };
    line(&mut o, "cl0", spec.cl0, cl0);
    line(&mut o, "cl_a", spec.cl_a, cla);
    line(&mut o, "cd0 (+gear)", spec.cd0 + spec.cd_gear, cd0);
    line(&mut o, "k_ind", spec.k_ind, k);
    line(&mut o, "stall_alpha", spec.stall_alpha, ja0.to_radians());
    line(&mut o, "stall_flap", spec.stall_flap, (ja0 - ja1).to_radians());
    if let Some(r) = at0 {
        line(&mut o, "flap_cl", spec.flap_cl, r.2[0] - r.1[0]);
        line(&mut o, "cd_flap total", spec.cd_flap[0] + spec.cd_flap[1], r.2[1] - r.1[1]);
    }
    if !missing.is_empty() {
        let _ = writeln!(o, "\nProperties the file reads that were taken as 0: {}", missing.into_iter().collect::<Vec<_>>().join(", "));
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A made-up aircraft with one of each kind of thing.
    const TEST: &str = r#"<?xml version="1.0"?>
<fdm_config name="test" version="2.0">
  <metrics>
    <wingarea unit="M2"> 10 </wingarea>
    <wingspan unit="FT"> 30 </wingspan>
    <chord unit="IN"> 60 </chord>
  </metrics>
  <mass_balance>
    <ixx unit="SLUG*FT2"> 1000 </ixx> <iyy unit="SLUG*FT2"> 1200 </iyy> <izz unit="SLUG*FT2"> 2000 </izz>
    <emptywt unit="LBS"> 1000 </emptywt>
    <pointmass name="pilot"><weight unit="KG"> 90.7184 </weight></pointmass>
  </mass_balance>
  <propulsion><tank type="FUEL"><contents unit="LBS"> 100 </contents></tank></propulsion>
  <flight_control name="fcs">
    <channel name="roll">
      <aerosurface_scale name="ail"><input>fcs/roll-cmd-norm</input><gain>0.01745</gain>
        <range><min>-20</min><max>15</max></range><output>fcs/left-aileron-pos-rad</output></aerosurface_scale>
    </channel>
  </flight_control>
  <aerodynamics>
    <function name="aero/function/k">
      <table><independentVar>aero/h_b-mac-ft</independentVar>
        <tableData> 0 0.5   1 1.0 </tableData></table>
    </function>
    <axis name="LIFT">
      <function name="aero/coefficient/CLa">
        <description>lift</description>
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property> <property>aero/function/k</property>
          <table>
            <independentVar lookup="row">aero/alpha-rad</independentVar>
            <independentVar lookup="column">fcs/flap-pos-deg</independentVar>
            <tableData>
                     0     30
              -0.1  -0.2   0.1
               0.0   0.3   0.6
               0.2   1.3   1.6
               0.3   1.0   1.3
            </tableData>
          </table>
        </product>
      </function>
    </axis>
    <axis name="DRAG">
      <function name="aero/coefficient/CD">
        <product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>
          <sum><value>0.03</value><product><value>0.05</value><property>aero/function/CL2</property></product></sum>
        </product>
      </function>
    </axis>
    <axis name="ROLL">
      <function name="aero/coefficient/Clda">
        <product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/bw-ft</property>
          <property>fcs/left-aileron-pos-rad</property><value>0.2</value></product>
      </function>
      <function name="aero/coefficient/Clp">
        <product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/bw-ft</property>
          <property>aero/bi2vel</property><property>velocities/p-aero-rad_sec</property><value>-0.5</value></product>
      </function>
    </axis>
    <axis name="PITCH">
      <function name="aero/coefficient/Cm">
        <product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/cbarw-ft</property>
          <difference><value>0.05</value><product><property>-aero/alpha-rad</property><value>-1</value></product></difference>
          <ifthen><gt><property>aero/alpha-rad</property><value>0.25</value></gt><value>2</value><value>1</value></ifthen>
        </product>
      </function>
    </axis>
  </aerodynamics>
</fdm_config>"#;

    #[test]
    fn reads_the_file() {
        let a = Aircraft::parse(TEST, Path::new(".")).unwrap();
        assert!((a.wing_area_ft2 - 107.639).abs() < 0.01);
        assert_eq!((a.span_ft, a.chord_ft), (30.0, 5.0));
        assert!((a.weight_lbs - 1300.0).abs() < 0.01, "{}", a.weight_lbs);
        assert_eq!(a.inertia, [1000.0, 1200.0, 2000.0]);
        let (lo, hi) = a.travel["fcs/left-aileron-pos-rad"];
        assert!((lo + 0.349).abs() < 0.001 && (hi - 0.2618).abs() < 0.001);
    }

    #[test]
    fn evaluates_like_jsbsim() {
        let a = Aircraft::parse(TEST, Path::new(".")).unwrap();
        // 2D table half way between rows and columns, out of ground effect (k held at 1).
        let mut p = a.state(100.0, 0.1f64.to_degrees(), 0.0, 0.002377, [0.0; 3], 15.0, 30.0, 0.0, 0.0, 0.0);
        let c = a.coefficients(&mut p);
        assert!((c[0] - 0.95).abs() < 1e-9, "CL {}", c[0]);
        // CL2 is not defined: read as 0, and listed.
        assert!((c[1] - 0.03).abs() < 1e-9);
        assert!(p.missing.contains("aero/function/CL2"));
        // Pitch: (0.05 - (-alpha)(-1)) = 0.05 - 0.1, times 1 below 0.25 rad.
        assert!((c[4] + 0.05).abs() < 1e-9, "Cm {}", c[4]);
        // Past the table's end it holds; past 0.25 rad the ifthen doubles the moment.
        let mut p = a.state(100.0, 30.0, 0.0, 0.002377, [0.0; 3], 0.0, 30.0, 0.0, 0.0, 0.0);
        let c = a.coefficients(&mut p);
        assert!((c[0] - 1.0 * 1.0).abs() < 0.05, "CL {}", c[0]);
        assert!((c[4] - (0.05 - 30f64.to_radians()) * 2.0).abs() < 1e-9);
        // Roll: aileron against damping.
        let mut p = a.state(100.0, 0.0, 0.0, 0.002377, [1.0, 0.0, 0.0], 0.0, 30.0, 0.0, 0.1, 0.0);
        let c = a.coefficients(&mut p);
        let bi2v = 30.0 / (2.0 * 100.0 * 1.68781);
        assert!((c[3] - (0.2 * 0.1 - 0.5 * bi2v)).abs() < 1e-9);
    }

    #[test]
    fn three_d_tables_and_errors() {
        let xml = r#"<fdm_config name="t"><metrics><wingarea>100</wingarea><wingspan>10</wingspan><chord>5</chord></metrics>
          <aerodynamics><axis name="LIFT"><function name="f"><table>
            <independentVar lookup="row">aero/alpha-rad</independentVar>
            <independentVar lookup="column">aero/beta-rad</independentVar>
            <independentVar lookup="table">fcs/flap-pos-deg</independentVar>
            <tableData breakPoint="0">   0 1
               0 0 0
               1 1 1 </tableData>
            <tableData breakPoint="10">  0 1
               0 2 2
               1 3 3 </tableData>
          </table></function></axis></aerodynamics></fdm_config>"#;
        let a = Aircraft::parse(xml, Path::new(".")).unwrap();
        let mut p = Props::default();
        for (k, v) in [("aero/alpha-rad", 0.5), ("aero/beta-rad", 0.0), ("fcs/flap-pos-deg", 5.0), ("aero/qbar-psf", 1.0)] {
            p.set(k, v);
        }
        // (0.5 + 2.5) / 2 between the two tables
        assert!((a.forces(&mut p).lift - 1.5).abs() < 1e-9);
        assert!(Aircraft::parse("<other/>", Path::new(".")).is_err());
        assert!(Aircraft::parse(r#"<fdm_config><aerodynamics><axis name="LIFT"><function name="x"><bogus/></function></axis></aerodynamics></fdm_config>"#, Path::new(".")).is_err());
    }

    #[test]
    fn compares() {
        let a = Aircraft::parse(TEST, Path::new(".")).unwrap();
        let r = compare(&a, &crate::aircraft::c172::SPEC, Some(1.0));
        assert!(r.contains("CL max clean") && r.contains("cl_a") && r.contains("roll rate"), "{r}");
    }
}
