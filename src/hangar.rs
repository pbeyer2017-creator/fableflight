//! More aircraft: FlightGear's aircraft collection (FGAddon), browsed,
//! downloaded and flown, each with its own 3D model.
//!
//! The collection is FlightGear's aircraft catalogue (`CATALOG`): one XML
//! file listing every package with its name, authors, size and download
//! mirrors. For the one shown we read its `-set.xml` straight out of its zip
//! on the mirror (range requests: the zip's directory, then just that file;
//! cached on disk) to learn which flight model it uses:
//! the ones on JSBSim we can fly, and for those the JSBSim file and its
//! engines are read too, to show what it flies like here. Downloading takes
//! the package's zip from a mirror and unpacks it (without its sounds, scripts
//! and splash screens) into the player's data folder
//! (`~/.local/share/fablegame/aircraft/<folder>/` on Linux), with a
//! `source.json` saying where it came from and under which license.
//!
//! Flying one: our flight model is a handling model, not JSBSim's build-up
//! of coefficients, so a JSBSim aircraft is *fitted*: its mass, inertia,
//! wing, lift and drag curves (from its own aerodynamic tables, evaluated by
//! jsbsim.rs), flaps, engines and fuel go into a `Spec` built on the closest
//! of our own types (a propeller aircraft on the Cessna, a fighter or
//! anything with afterburners on the Hornet, a big jet on the 747). It is
//! drawn with its own model (fgmodel.rs), sits on its own landing gear (the
//! JSBSim contact points), and keeps the stand-in's cockpit instruments,
//! sounds and damage rules.
//!
//! (Aircraft an earlier version took from the JSBSim repository, which has
//! no 3D models, are no longer loaded: `~/.local/share/fablegame/jsbsim/`.)
//!
//! Nothing from FlightGear ships with the game; the files are fetched only
//! when the player asks for them.

use crate::aircraft::{self, Engine, Origin, Spec};
use crate::{jsbsim, yasim};
use parking_lot::Mutex;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// FlightGear's aircraft catalogue (the 2020 series of packages).
pub const CATALOG: &str = "https://mirrors.ibiblio.org/flightgear/ftp/Aircraft-2020/catalog.xml";
/// FGAddon, the repository behind the packages (linked from the menu; files are read from the packages themselves).
const SVN: &str = "https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft";
/// The collection's license, which covers files that state none (FGAddon takes only GPL-compatible aircraft).
pub const REPO_LICENSE: &str = "GNU GPL v2 or later";
pub const REPO_LICENSE_URL: &str = "https://www.gnu.org/licenses/old-licenses/gpl-2.0.html";

pub fn repo_url() -> String {
    "https://wiki.flightgear.org/FGAddon".into()
}

pub fn folder_url(folder: &str) -> String {
    format!("{SVN}/{folder}/")
}

/// The key a downloaded aircraft answers to (`--aircraft fg:SR22T`).
pub fn key(folder: &str) -> String {
    format!("fg:{}", folder.to_lowercase())
}

/// What a fit makes of an aircraft, for the menu.
#[derive(Clone, Debug)]
pub struct Summary {
    /// "Piston twin", "Jet, 4 engines"…
    pub kind: String,
    pub mass_kg: f64,
    pub span_m: f64,
    /// "1 490 hp", "2 × 79 kN (129 kN with afterburner)"
    pub power: String,
    pub stall_kt: f64,
    pub vmax_kt: f64,
    /// the built-in type it borrows its instruments and sounds from
    pub body: &'static str,
    /// drawn with its own 3D model (else as `body`)
    pub own_model: bool,
    /// the flight model fitted to ours: "JSBSim" or "YASim"
    pub fdm: &'static str,
}

/// Where the license shown for an aircraft comes from.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LicenseFrom {
    /// its file header names one
    Header,
    /// the file's comments carry the license text
    Comment,
    /// the file states none: the collection's covers it
    Repository,
}

#[derive(Clone, Debug)]
pub struct Info {
    pub name: String,
    pub author: String,
    pub description: String,
    pub license: String,
    pub license_url: String,
    pub license_from: LicenseFrom,
    pub notes: String,
    /// what it flies like here, or why it can't be flown
    pub fit: Result<Summary, String>,
}

#[derive(Clone, Debug)]
pub enum State {
    /// not read yet
    Pending,
    Ready(Box<Info>),
    Failed(String),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Install {
    No,
    /// bytes done of bytes to get
    Downloading(u64, u64),
    /// unpacking and reading it
    Unpacking,
    Installed,
    Failed(String),
}

#[derive(Clone, Debug)]
pub struct Entry {
    /// its folder (FlightGear's `Aircraft/<folder>/`), what we know it by
    pub folder: String,
    /// the package id: its main file is `<id>-set.xml`
    pub id: String,
    /// as the catalogue names it, and its authors
    pub name: String,
    pub author: String,
    /// the zip's size, its mirrors, and the package revision (a date)
    pub size: u64,
    pub urls: Vec<String>,
    pub revision: String,
    pub state: State,
    pub install: Install,
}

impl Entry {
    pub fn bytes(&self) -> u64 {
        self.size
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Status {
    Idle,
    Listing,
    Ready,
    Failed(String),
}

pub struct Catalogue {
    pub status: Status,
    pub entries: Vec<Entry>,
}

static CAT: Mutex<Catalogue> = Mutex::new(Catalogue { status: Status::Idle, entries: Vec::new() });

/// The catalogue as it stands (it fills in from worker threads).
pub fn catalogue() -> parking_lot::MutexGuard<'static, Catalogue> {
    CAT.lock()
}

fn agent() -> ureq::Agent {
    ureq::Agent::config_builder().timeout_global(Some(std::time::Duration::from_secs(30))).user_agent("fablegame-flightsim/0.1").build().into()
}

fn get(agent: &ureq::Agent, url: &str) -> Result<Vec<u8>, String> {
    let mut r = agent.get(url).call().map_err(|e| e.to_string())?;
    r.body_mut().with_config().limit(16 << 20).read_to_vec().map_err(|e| e.to_string())
}

fn cache_dir() -> PathBuf {
    dirs::cache_dir().unwrap_or_else(|| PathBuf::from(".")).join("fablegame").join("fgaddon")
}

/// Whether a name from the catalogue can be used as one folder name on disk:
/// no path separators, no drive or stream colon, not hidden, not `.` or `..`.
fn safe_name(s: &str) -> bool {
    !s.is_empty() && !s.starts_with('.') && !s.chars().any(|c| matches!(c, '/' | '\\' | ':') || c.is_control())
}

/// Where downloaded aircraft live, one folder each.
pub fn install_dir() -> PathBuf {
    dirs::data_dir().unwrap_or_else(|| PathBuf::from(".")).join("fablegame").join("aircraft")
}

/// A file of a package (path inside its folder), from the disk cache or read
/// out of the package's zip on the mirror (and cached).
fn svn(agent: &ureq::Agent, folder: &str, rev: &str, path: &str) -> Option<Vec<u8>> {
    let p = scratch(folder, rev).join(path);
    let missing = p.with_extension("404");
    if let Ok(d) = std::fs::read(&p) {
        return Some(d);
    }
    if missing.exists() {
        return None;
    }
    let z = remote_zip(agent, folder)?;
    let d = match z.read(agent, path) {
        Some(d) => d,
        None => {
            if let Some(dir) = missing.parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            let _ = std::fs::write(&missing, b"");
            return None;
        }
    };
    if let Some(dir) = p.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let tmp = p.with_extension("part");
    if std::fs::write(&tmp, &d).is_ok() {
        let _ = std::fs::rename(&tmp, &p);
    }
    Some(d)
}

/// A package's zip on a mirror, read in pieces with range requests: its
/// directory (at the end of the file), then just the files asked for.
struct RemoteZip {
    url: String,
    /// path inside the aircraft's folder -> (local header offset, compressed size, method)
    files: HashMap<String, (u64, u64, u16)>,
}

static ZIPS: Mutex<Vec<(String, Arc<RemoteZip>)>> = Mutex::new(Vec::new());

fn range(agent: &ureq::Agent, url: &str, start: u64, end: u64) -> Option<Vec<u8>> {
    let mut r = agent.get(url).header("Range", &format!("bytes={start}-{}", end.saturating_sub(1))).call().ok()?;
    if r.status().as_u16() != 206 {
        return None; // (a mirror that ignores ranges would send the whole package)
    }
    r.body_mut().with_config().limit(64 << 20).read_to_vec().ok()
}

fn remote_zip(agent: &ureq::Agent, folder: &str) -> Option<Arc<RemoteZip>> {
    if let Some(z) = ZIPS.lock().iter().find(|z| z.0 == folder) {
        return Some(z.1.clone());
    }
    let (urls, size) = {
        let c = CAT.lock();
        let e = c.entries.iter().find(|e| e.folder == folder)?;
        (e.urls.clone(), e.size)
    };
    let z = urls.iter().find_map(|u| read_directory(agent, u, size, folder))?;
    let z = Arc::new(z);
    ZIPS.lock().push((folder.to_string(), z.clone()));
    Some(z)
}

fn read_directory(agent: &ureq::Agent, url: &str, size: u64, folder: &str) -> Option<RemoteZip> {
    if size < 22 {
        return None;
    }
    let u16_at = |b: &[u8], i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
    let u32_at = |b: &[u8], i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    // The end of central directory record, within the last 64 KB (its comment can be that long).
    let tail_start = size.saturating_sub(65_558);
    let tail = range(agent, url, tail_start, size)?;
    let eocd = (0..tail.len().saturating_sub(21)).rev().find(|&i| u32_at(&tail, i) == 0x0605_4b50)?;
    let (cd_size, cd_off) = (u32_at(&tail, eocd + 12) as u64, u32_at(&tail, eocd + 16) as u64);
    // (a directory the record puts past the end of the file is not one to read)
    if cd_off.checked_add(cd_size)? > size {
        return None;
    }
    let cd = if cd_off >= tail_start { tail.get((cd_off - tail_start) as usize..)?.to_vec() } else { range(agent, url, cd_off, cd_off + cd_size)? };
    let mut files = HashMap::new();
    let mut i = 0usize;
    while i + 46 <= cd.len() && u32_at(&cd, i) == 0x0201_4b50 {
        let method = u16_at(&cd, i + 10);
        let csize = u32_at(&cd, i + 20) as u64;
        let (nl, xl, cl) = (u16_at(&cd, i + 28) as usize, u16_at(&cd, i + 30) as usize, u16_at(&cd, i + 32) as usize);
        let off = u32_at(&cd, i + 42) as u64;
        let name = String::from_utf8_lossy(cd.get(i + 46..i + 46 + nl)?).replace('\\', "/");
        // (inside the package's one folder)
        let inner = match name.split_once('/') {
            Some((top, rest)) if top.eq_ignore_ascii_case(folder) => rest.to_string(),
            _ => name.clone(),
        };
        if !inner.is_empty() && !inner.ends_with('/') {
            files.insert(inner, (off, csize, method));
        }
        i += 46 + nl + xl + cl;
    }
    (!files.is_empty()).then(|| RemoteZip { url: url.to_string(), files })
}

impl RemoteZip {
    fn read(&self, agent: &ureq::Agent, path: &str) -> Option<Vec<u8>> {
        let &(off, csize, method) = self.files.get(path).or_else(|| self.files.iter().find(|(k, _)| k.eq_ignore_ascii_case(path)).map(|(_, v)| v))?;
        // The local header (30 bytes, then a name and extra field we learn the length of), then the data.
        let head = range(agent, &self.url, off, off + 30 + 512 + csize)?;
        if head.len() < 30 || u32::from_le_bytes([head[0], head[1], head[2], head[3]]) != 0x0403_4b50 {
            return None;
        }
        let skip = 30 + u16::from_le_bytes([head[26], head[27]]) as usize + u16::from_le_bytes([head[28], head[29]]) as usize;
        let data = if skip + csize as usize <= head.len() { head[skip..skip + csize as usize].to_vec() } else { range(agent, &self.url, off + skip as u64, off + skip as u64 + csize)? };
        match method {
            0 => Some(data),
            8 => {
                use std::io::Read;
                let mut out = Vec::new();
                flate2::read::DeflateDecoder::new(&data[..]).read_to_end(&mut out).ok()?;
                Some(out)
            }
            _ => None,
        }
    }
}

/// The scratch copy of a package's files read so far (as the repository lays them out).
fn scratch(folder: &str, rev: &str) -> PathBuf {
    cache_dir().join("files").join(format!("{folder}@{rev}"))
}

/// Starts reading the collection, once.
pub fn open() {
    {
        let mut c = CAT.lock();
        if matches!(c.status, Status::Listing | Status::Ready) {
            return;
        }
        c.status = Status::Listing;
    }
    std::thread::spawn(|| {
        let agent = agent();
        match list(&agent) {
            Ok(entries) => {
                let installed = installed_folders();
                let mut c = CAT.lock();
                c.entries = entries;
                for e in &mut c.entries {
                    if installed.contains(&e.folder) {
                        e.install = Install::Installed;
                    }
                }
                c.status = Status::Ready;
            }
            Err(e) => {
                CAT.lock().status = Status::Failed(e);
                return;
            }
        }
        // The ones read before (their files are in the cache) come back at once.
        let cached: Vec<String> = CAT.lock().entries.iter().filter(|e| scratch(&e.folder, &e.revision).exists()).map(|e| e.folder.clone()).collect();
        for f in cached {
            want(&f);
        }
    });
}

/// Entries being read now.
static READING: Mutex<Vec<String>> = Mutex::new(Vec::new());

/// Reads an entry (its `-set.xml`, JSBSim file and engines, out of its zip
/// on the mirror), unless it is read or being read: the menu asks for the one
/// shown, rather than reading all 650 packages up front.
pub fn want(folder: &str) {
    {
        let c = CAT.lock();
        if !c.entries.iter().any(|e| e.folder == folder && matches!(e.state, State::Pending)) {
            return;
        }
        let mut r = READING.lock();
        // (a few at a time)
        if r.iter().any(|f| f == folder) || r.len() >= 3 {
            return;
        }
        r.push(folder.to_string());
    }
    let folder = folder.to_string();
    std::thread::spawn(move || {
        let st = match read_entry(&agent(), &folder) {
            Ok(i) => State::Ready(Box::new(i)),
            Err(e) => State::Failed(e),
        };
        if let Some(e) = CAT.lock().entries.iter_mut().find(|e| e.folder == folder) {
            e.state = st;
        }
        READING.lock().retain(|f| *f != folder);
    });
}

/// How many entries are being read.
pub fn reading() -> usize {
    READING.lock().len()
}

/// What its catalogue name says of its flight model: Some(true) JSBSim, Some(false) YASim.
pub fn named_fdm(e: &Entry) -> Option<bool> {
    let n = e.name.to_lowercase();
    if n.contains("jsbsim") || n.contains("jsb)") {
        Some(true)
    } else if n.contains("yasim") {
        Some(false)
    } else {
        None
    }
}

/// The catalogue: an entry for each package (refetched once it is a week old).
fn list(agent: &ureq::Agent) -> Result<Vec<Entry>, String> {
    let cached = cache_dir().join("catalog.xml");
    let fresh = std::fs::metadata(&cached).and_then(|m| m.modified()).ok().and_then(|t| t.elapsed().ok()).is_some_and(|age| age.as_secs() < 7 * 86400);
    let body = match std::fs::read(&cached) {
        Ok(d) if !d.is_empty() && fresh => d,
        old => match get(agent, CATALOG) {
            Ok(d) => {
                let _ = std::fs::create_dir_all(cache_dir());
                let _ = std::fs::write(&cached, &d);
                d
            }
            // (offline: last week's will do)
            Err(e) => old.ok().filter(|d| !d.is_empty()).ok_or_else(|| format!("could not get FlightGear's aircraft catalogue: {e}"))?,
        },
    };
    let text = xml_text(&body);
    let doc = roxmltree::Document::parse(&text).map_err(|e| format!("the catalogue doesn't read: {e}"))?;
    let t = |n: roxmltree::Node, k: &str| n.children().find(|c| c.has_tag_name(k)).and_then(|c| c.text()).map(|s| s.trim().to_string()).unwrap_or_default();
    let mut entries: Vec<Entry> = Vec::new();
    for p in doc.descendants().filter(|n| n.has_tag_name("package")) {
        let (id, folder) = (t(p, "id"), t(p, "dir"));
        if id.is_empty() || folder.is_empty() || entries.iter().any(|e| e.folder == folder) {
            continue;
        }
        // The folder and revision name folders on disk (the install, the file cache): one plain name each.
        let revision = t(p, "revision");
        if !safe_name(&folder) || !(revision.is_empty() || safe_name(&revision)) {
            log::warn!("FlightGear catalogue: skipping {id}, its folder or revision is not a plain name");
            continue;
        }
        let mut urls: Vec<String> = p.children().filter(|c| c.has_tag_name("url")).filter_map(|c| c.text()).map(|s| s.trim().to_string()).collect();
        // ibiblio first (over https: it answers range requests and fast; some other mirrors fail their TLS handshake)
        for u in &mut urls {
            if u.starts_with("http://mirrors.ibiblio.org") {
                *u = u.replacen("http://", "https://", 1);
            }
        }
        urls.sort_by_key(|u| (!u.contains("ibiblio.org"), !u.starts_with("https")));
        entries.push(Entry { folder, name: t(p, "name"), author: t(p, "author"), size: t(p, "file-size-bytes").parse().unwrap_or(0), urls, revision, id, state: State::Pending, install: Install::No });
    }
    if entries.is_empty() {
        return Err("the catalogue lists no aircraft".into());
    }
    entries.sort_by_key(|e| e.name.to_lowercase());
    Ok(entries)
}

/// An XML file as text: UTF-8 (or Latin-1, as many FlightGear files are), without its declaration.
fn xml_text(d: &[u8]) -> String {
    let s = match std::str::from_utf8(d) {
        Ok(s) => s.to_string(),
        Err(_) => d.iter().map(|&b| b as char).collect(),
    };
    let s = s.trim_start_matches('\u{feff}');
    match (s.find("<?xml"), s.find("?>")) {
        (Some(0), Some(e)) => s[e + 2..].to_string(),
        _ => s.to_string(),
    }
}

/// What an aircraft's `-set.xml` says (with the files it includes).
#[derive(Clone, Debug, Default)]
pub struct SetInfo {
    /// "jsb", "yasim"...
    pub fdm: String,
    /// the JSBSim file's name (without .xml)
    pub aero: String,
    /// the model file, as a path inside the aircraft's folder
    pub model: String,
    pub description: String,
    pub author: String,
    /// the pilot's view (view 0): right, up, aft of the model's origin (m)
    pub eye: Option<[f32; 3]>,
    /// its default livery and the texture properties it sets
    pub paint: crate::fgmodel::Paint,
}

/// A path written in one of the aircraft's files, as a path inside its folder
/// (None for FlightGear's shared files).
fn rel_path(from_file: &str, p: &str, folder: &str) -> Option<String> {
    let p = p.trim();
    let joined = if let Some(rest) = p.strip_prefix("Aircraft/") {
        let (dir, sub) = rest.split_once('/')?;
        if !dir.eq_ignore_ascii_case(folder) {
            return None;
        }
        sub.to_string()
    } else {
        match from_file.rfind('/') {
            Some(i) => format!("{}/{p}", &from_file[..i]),
            None => p.to_string(),
        }
    };
    let mut out: Vec<&str> = Vec::new();
    for c in joined.split('/') {
        match c {
            "" | "." => {}
            ".." => {
                out.pop()?;
            }
            c => out.push(c),
        }
    }
    Some(out.join("/"))
}

/// Reads a property file and those it includes into `out` ("sim/aero" → "c172p"); its own values win.
fn flatten(get: &dyn Fn(&str) -> Option<Vec<u8>>, file: &str, prefix: &str, folder: &str, depth: usize, out: &mut HashMap<String, String>) {
    if depth > 6 {
        return;
    }
    let Some(d) = get(file) else { return };
    let text = xml_text(&d);
    let Ok(doc) = roxmltree::Document::parse(&text) else { return };
    let root = doc.root_element();
    let mut includes: Vec<(String, String)> = Vec::new();
    fn walk(n: roxmltree::Node, prefix: &str, out: &mut HashMap<String, String>, inc: &mut Vec<(String, String)>) {
        let mut seen: HashMap<&str, usize> = HashMap::new();
        for c in n.children().filter(|c| c.is_element()) {
            // (repeated elements are numbered, as FlightGear does: `view[100]`; the first, or n="0", has none)
            let tag = c.tag_name().name();
            let count = seen.entry(tag).or_insert(0);
            let idx = c.attribute("n").and_then(|n| n.trim().parse::<usize>().ok()).unwrap_or(*count);
            *count += 1;
            let name = if idx > 0 { format!("{tag}[{idx}]") } else { tag.to_string() };
            let key = if prefix.is_empty() { name } else { format!("{prefix}/{name}") };
            if let Some(i) = c.attribute("include") {
                inc.push((i.to_string(), key.clone()));
            }
            if c.children().any(|x| x.is_element()) {
                walk(c, &key, out, inc);
            } else if let Some(t) = c.text() {
                out.entry(key).or_insert_with(|| t.trim().to_string());
            }
        }
    }
    walk(root, prefix, out, &mut includes);
    if let Some(i) = root.attribute("include") {
        includes.push((i.to_string(), prefix.to_string()));
    }
    for (inc, pre) in includes {
        if let Some(p) = rel_path(file, &inc, folder) {
            flatten(get, &p, &pre, folder, depth + 1, out);
        }
    }
}

fn read_set(get: &dyn Fn(&str) -> Option<Vec<u8>>, id: &str, folder: &str) -> Result<SetInfo, String> {
    let file = format!("{id}-set.xml");
    if get(&file).is_none() {
        return Err(format!("its {file} can't be read"));
    }
    let mut m = HashMap::new();
    flatten(get, &file, "", folder, 0, &mut m);
    let v = |k: &str| m.get(k).cloned().unwrap_or_default();
    let model = rel_path(&file, &v("sim/model/path"), folder).unwrap_or_default();
    let f = |k: &str| m.get(&format!("sim/view/config/{k}")).and_then(|x| x.trim().parse::<f32>().ok());
    let eye = match (f("x-offset-m"), f("y-offset-m"), f("z-offset-m")) {
        (Some(x), Some(y), Some(z)) => Some([x, y, z]),
        _ => None,
    };
    let paint = crate::fgmodel::Paint::from_set(&m);
    Ok(SetInfo { fdm: v("sim/flight-model").to_lowercase(), aero: v("sim/aero"), model, description: v("sim/description"), author: v("sim/author"), eye, paint })
}

/// What the menu shows of an aircraft (before it is downloaded): its
/// `-set.xml` and, for a JSBSim one, its fit.
fn read_entry(agent: &ureq::Agent, folder: &str) -> Result<Info, String> {
    let (id, rev, cat_name, cat_author) = {
        let c = CAT.lock();
        let e = c.entries.iter().find(|e| e.folder == folder).ok_or("gone")?;
        (e.id.clone(), e.revision.clone(), e.name.clone(), e.author.clone())
    };
    let fetch = |p: &str| svn(agent, folder, &rev, p);
    let set = read_set(&fetch, &id, folder)?;
    let author = if cat_author.is_empty() { set.author.clone() } else { cat_author };
    let main = format!("{}.xml", set.aero);
    match set.fdm.as_str() {
        "jsb" => {}
        "yasim" => {
            // One file holds it all.
            let xml = fetch(&main).map(|d| xml_text(&d)).ok_or_else(|| format!("its YASim file {main} can't be read"))?;
            let (j, parts) = read_yasim(&xml, &set, folder);
            let mut i = info(&xml, j, &parts, folder, &cat_name, !set.model.is_empty());
            i.author = author;
            if i.description.is_empty() {
                i.description = set.description;
            }
            return Ok(i);
        }
        f => {
            let what = if f.is_empty() { "its files name no flight model".to_string() } else { format!("it flies on FlightGear's {f} flight model: only JSBSim and YASim aircraft can be flown here") };
            return Ok(Info { name: cat_name, author, description: set.description, license: REPO_LICENSE.into(), license_url: REPO_LICENSE_URL.into(), license_from: LicenseFrom::Repository, notes: String::new(), fit: Err(what) });
        }
    }
    // The JSBSim file and the files it keeps sections in, then its engines.
    let xml = fetch(&main).map(|d| xml_text(&d)).ok_or_else(|| format!("its JSBSim file {main} can't be read"))?;
    for f in file_refs(&xml) {
        for p in [f.clone(), format!("Systems/{f}")] {
            if fetch(&p).is_some() {
                break;
            }
        }
    }
    let dir = scratch(folder, &rev);
    let j = jsbsim::Aircraft::parse(&xml, &dir).map_err(|e| format!("our reader can't read its JSBSim file: {e}"));
    let mut parts = Vec::new();
    if let Ok(j) = &j {
        for (eng, th) in &j.engines {
            let part = |n: &str| fetch(&format!("Engines/{n}.xml")).and_then(|d| read_part(&xml_text(&d)).ok());
            parts.push((part(eng), th.as_deref().and_then(part)));
        }
    }
    let mut i = info(&xml, j, &parts, folder, &cat_name, !set.model.is_empty());
    i.author = author;
    if i.description.is_empty() {
        i.description = set.description;
    }
    Ok(i)
}

/// A YASim aircraft's file, read as what the fit takes (see yasim.rs), with its engines.
fn read_yasim(xml: &str, set: &SetInfo, folder: &str) -> (Result<jsbsim::Aircraft, String>, Vec<(Option<Part>, Option<Part>)>) {
    let y = match yasim::Airplane::parse(xml) {
        Ok(y) => y,
        Err(e) => return (Err(format!("our reader can't read its YASim file: {e}")), Vec::new()),
    };
    if y.engines.contains(&yasim::Engine::Rotor) {
        return (Err("it flies on rotors (a helicopter or an autogyro): rotors can't be flown yet".into()), Vec::new());
    }
    let part = |kind: &str| Part { kind: kind.to_string(), ..Default::default() };
    let parts = y
        .engines
        .iter()
        .map(|e| match *e {
            yasim::Engine::Prop { hp, turbine, .. } => (Some(Part { hp, ..part(if turbine { "turboprop_engine" } else { "piston_engine" }) }), Some(part("propeller"))),
            yasim::Engine::Jet { thrust, afterburner } => (Some(Part { mil: thrust, max: afterburner.max(thrust), augmented: afterburner > thrust, ..part("turbine_engine") }), Some(part("nozzle"))),
            yasim::Engine::Rotor => (Some(part("turbine_engine")), Some(part("rotor"))),
        })
        .collect();
    let j = y.to_jsbsim(folder).map(|mut j| {
        j.author = squash(&set.author);
        j.description = squash(&set.description);
        j
    });
    (j, parts)
}

fn squash(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// The files a JSBSim file keeps sections in (`file="..."`), with .xml.
fn file_refs(xml: &str) -> Vec<String> {
    let mut v = Vec::new();
    let mut rest = xml;
    while let Some(i) = rest.find("file=\"") {
        rest = &rest[i + 6..];
        let Some(e) = rest.find('"') else { break };
        let f = &rest[..e];
        if !f.is_empty() && !f.contains("..") {
            v.push(if f.ends_with(".xml") { f.to_string() } else { format!("{f}.xml") });
        }
        rest = &rest[e..];
    }
    v
}

/// What the menu shows of an aircraft: from its file and, when it reads, its fit.
fn info(xml: &str, j: Result<jsbsim::Aircraft, String>, parts: &[(Option<Part>, Option<Part>)], folder: &str, name: &str, own_model: bool) -> Info {
    let (license, license_url, license_from) = license_of(xml, j.as_ref().ok());
    match j {
        Ok(j) => {
            let fit = fit(&j, parts, folder, name, None).map(|mut f| {
                f.1.own_model = own_model;
                f.1
            });
            Info { name: if name.is_empty() { display_name(&j, folder) } else { name.to_string() }, author: j.author.clone(), description: j.description.clone(), license, license_url, license_from, notes: j.notes.clone(), fit }
        }
        Err(e) => Info { name: if name.is_empty() { folder.to_string() } else { name.to_string() }, author: String::new(), description: String::new(), license, license_url, license_from, notes: String::new(), fit: Err(e) },
    }
}

/// An engine or a thruster file, read for what the fit needs.
#[derive(Clone, Debug, Default)]
struct Part {
    /// the root element: piston_engine, turbine_engine, turboprop_engine,
    /// rocket_engine, electric_engine; propeller, direct, nozzle, rotor
    kind: String,
    /// horsepower (piston maxhp, turboprop maxpower, electric power)
    hp: f64,
    /// dry and wet thrust (lbf), afterburner
    mil: f64,
    max: f64,
    augmented: bool,
}

fn read_part(xml: &str) -> Result<Part, String> {
    let doc = roxmltree::Document::parse(xml).map_err(|e| e.to_string())?;
    let r = doc.root_element();
    let num = |k: &str| r.children().find(|c| c.has_tag_name(k)).and_then(|c| c.text()).and_then(|t| t.trim().parse::<f64>().ok());
    let mut p = Part { kind: r.tag_name().name().to_string(), ..Default::default() };
    p.hp = num("maxhp").or_else(|| num("maxpower")).unwrap_or(0.0);
    if p.kind == "electric_engine" {
        // (watts)
        p.hp = num("power").map(|w| w / 745.7).unwrap_or(0.0);
    }
    p.mil = num("milthrust").unwrap_or(0.0);
    p.max = num("maxthrust").unwrap_or(p.mil).max(p.mil);
    p.augmented = num("augmented").unwrap_or(0.0) > 0.5;
    Ok(p)
}

/// An engine or propeller file's path: the aircraft's own `Engines/`, else the shared `engine/`.
/// The license an aircraft's file states: in its header, else in its comments, else the repository's.
fn license_of(xml: &str, j: Option<&jsbsim::Aircraft>) -> (String, String, LicenseFrom) {
    if let Some((n, u)) = j.and_then(|j| j.license.clone()) {
        let n = if n.is_empty() { u.clone() } else { n };
        return (n, u, LicenseFrom::Header);
    }
    let head: String = xml.chars().take(12_000).collect();
    if head.contains("GNU General Public License") {
        let v = if head.contains("version 2 of the License") { "GNU GPL v2 or later" } else { "GNU GPL" };
        return (v.into(), "https://www.gnu.org/licenses/old-licenses/gpl-2.0.html".into(), LicenseFrom::Comment);
    }
    if head.contains("GNU Lesser General Public License") || head.contains("GNU LGPL") {
        return ("GNU LGPL".into(), "https://www.gnu.org/licenses/lgpl.html".into(), LicenseFrom::Comment);
    }
    (REPO_LICENSE.into(), REPO_LICENSE_URL.into(), LicenseFrom::Repository)
}

/// The name in the file, unless it is a placeholder; the folder's otherwise.
fn display_name(j: &jsbsim::Aircraft, folder: &str) -> String {
    let n = j.name.trim();
    // (a short name the folder adds to, like "c172" in "c172p", says less than the folder)
    let vague = folder.len() > n.len() && folder.to_lowercase().starts_with(&n.to_lowercase());
    if n.is_empty() || n == "?" || n.eq_ignore_ascii_case("none") || n.len() > 40 || vague { folder.to_string() } else { n.to_string() }
}

// ------------------------------------------------------------------ fitting

const LB: f64 = 0.453_592;
const FT: f64 = 0.3048;
const KT: f64 = 0.514_444;
const RHO: f64 = 1.225;

/// The lift and drag curves of a JSBSim aircraft, fitted to our model's terms.
#[derive(Debug)]
struct Aero {
    cl0: f64,
    cl_a: f64,
    cd0: f64,
    k_ind: f64,
    cd_gear: f64,
    cl_max: f64,
    cl_max_flaps: f64,
    stall_alpha: f64,
    stall_flap: f64,
    flap_cl: f64,
    cd_flap: f64,
}

fn least_squares(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len() as f64;
    let (mx, my) = (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n);
    let sxx: f64 = pts.iter().map(|p| (p.0 - mx).powi(2)).sum();
    let sxy: f64 = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
    let b = if sxx > 0.0 { sxy / sxx } else { 0.0 };
    (my - b * mx, b)
}

fn fit_aero(j: &jsbsim::Aircraft, kt: f64) -> Result<Aero, String> {
    let rho = 0.002_377;
    let fmax = j.flaps_max_deg.unwrap_or(30.0);
    let coef = |alpha: f64, flaps: f64, gear: f64| -> [f64; 6] {
        let mut p = j.state(kt, alpha, 0.0, rho, [0.0; 3], flaps, fmax, 0.0, 0.0, 0.0);
        p.set("gear/gear-pos-norm", gear);
        p.set("gear/gear-cmd-norm", gear);
        // (out of ground effect, at the speed of the fit)
        p.set("aero/h_b-mac-ft", 1000.0);
        p.set("aero/h_b-cg-ft", 1000.0);
        p.set("position/h-agl-ft", 5000.0);
        p.set("position/h-sl-ft", 5000.0);
        j.coefficients(&mut p)
    };
    let lin: Vec<(f64, [f64; 6])> = (-2..=4).map(|k| k as f64 * 2.0).map(|a| (a, coef(a, 0.0, 0.0))).collect();
    let (cl0, cl_a) = least_squares(&lin.iter().map(|(a, c)| (a.to_radians(), c[0])).collect::<Vec<_>>());
    let (cd0, k_ind) = least_squares(&lin.iter().map(|(_, c)| (c[0] * c[0], c[1])).collect::<Vec<_>>());
    let fine: Vec<f64> = (-20..=450).map(|k| k as f64 * 0.1).collect();
    let peak = |flaps: f64| fine.iter().map(|&a| (a, coef(a, flaps, 0.0)[0])).fold((0.0, f64::MIN), |m, x| if x.1 > m.1 { x } else { m });
    // A table that never stops climbing (no stall modelled, or a fighter's
    // vortex lift): its stall is taken where a wing of its kind would be.
    let held = |(a, cl): (f64, f64), flaps: f64| if a >= 44.0 { let at = if kt > 120.0 { 30.0 } else { 16.0 } - if flaps > 0.0 { 2.0 } else { 0.0 }; (at, coef(at, flaps, 0.0)[0]) } else { (a, cl) };
    let (a0, clm0) = held(peak(0.0), 0.0);
    let (a1, clm1) = held(peak(fmax), fmax);
    let c0 = coef(0.0, 0.0, 0.0);
    let cf = coef(0.0, fmax, 0.0);
    let cg = coef(0.0, 0.0, 1.0);
    let ar = j.span_ft * j.span_ft / j.wing_area_ft2.max(1e-6);
    let k_ind = if (0.01..0.4).contains(&k_ind) { k_ind } else { 1.0 / (std::f64::consts::PI * 0.75 * ar.max(1.0)) };
    let a = Aero {
        cl0,
        cl_a,
        cd0: cd0.max(0.008),
        k_ind,
        cd_gear: (cg[1] - c0[1]).clamp(0.0, 0.05),
        cl_max: clm0,
        cl_max_flaps: clm1.max(clm0),
        stall_alpha: a0.to_radians(),
        stall_flap: (a0 - a1).max(0.0).to_radians(),
        flap_cl: (cf[0] - c0[0]).max(0.0),
        cd_flap: (cf[1] - c0[1]).max(0.0),
    };
    if !a.cl_a.is_finite() || !(1.0..=9.0).contains(&a.cl_a) {
        return Err(format!("its lift curve doesn't fit ours (slope {:.2} per radian)", a.cl_a));
    }
    if !(0.008..=0.2).contains(&cd0) {
        return Err(format!("its drag doesn't fit ours (CD0 {cd0:.3})"));
    }
    if !(0.5..=3.5).contains(&a.cl_max) || a.stall_alpha < 5f64.to_radians() || a.stall_alpha > 44f64.to_radians() {
        return Err(format!("its stall doesn't fit ours (CL max {:.2} at {:.0}°)", a.cl_max, a.stall_alpha.to_degrees()));
    }
    Ok(a)
}

/// Roughly the most lift one of our types makes (with flaps), and its top speed on its power.
fn our_cl_max(s: &Spec, flaps: bool) -> f64 {
    s.cl0 + s.cl_a * (s.stall_alpha - if flaps { s.stall_flap } else { 0.0 }) + if flaps { s.flap_cl } else { 0.0 }
}

fn stall_speed(mass: f64, area: f64, cl: f64) -> f64 {
    (2.0 * mass * 9.81 / (RHO * area * cl.max(0.2))).sqrt()
}

/// Level flight top speed at sea level: power (W) or thrust (N) against parasite drag.
fn top_speed(prop: bool, power_or_thrust: f64, area: f64, cd0: f64) -> f64 {
    if prop { (2.0 * 0.8 * power_or_thrust / (RHO * area * cd0)).cbrt() } else { (2.0 * 0.85 * power_or_thrust / (RHO * area * cd0)).sqrt().min(320.0) }
}

fn leak(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

/// "12 345"
fn thousands(v: f64) -> String {
    let s = format!("{:.0}", v.round());
    let mut o = String::new();
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            o.push('\u{202f}');
        }
        o.push(c);
    }
    o
}

/// A JSBSim aircraft as one of our types, and what that makes of it.
/// A downloaded FlightGear aircraft's files, for drawing it with its own model.
pub struct FgFiles {
    pub dir: PathBuf,
    pub model: PathBuf,
    pub still: Option<&'static [u8]>,
    /// the pilot's eye from its own view (right, up, aft of the model's origin, m)
    pub eye: Option<[f32; 3]>,
    /// its default livery, from its `-set.xml`
    pub paint: crate::fgmodel::Paint,
}

fn fit(j: &jsbsim::Aircraft, parts: &[(Option<Part>, Option<Part>)], folder: &str, name: &str, model: Option<&FgFiles>) -> Result<(Spec, Summary), String> {
    if j.wing_area_ft2 <= 0.0 || j.span_ft <= 0.0 {
        return Err("its file gives no wing".into());
    }
    if parts.is_empty() {
        return Err("it has no engine: gliders, balloons and the like can't be flown yet".into());
    }
    let n = parts.len();
    let (eng, th): (Vec<_>, Vec<_>) = parts.iter().cloned().unzip();
    let eng: Vec<Part> = eng.into_iter().collect::<Option<_>>().ok_or("one of its engine files is missing")?;
    let th: Vec<Option<Part>> = th;
    let kind = |p: &Part, k: &str| p.kind == k;
    if th.iter().flatten().any(|t| kind(t, "rotor")) {
        return Err("it is a helicopter: rotors can't be flown yet".into());
    }
    if eng.iter().any(|e| kind(e, "rocket_engine")) {
        return Err("it is rocket powered: rockets can't be flown yet".into());
    }
    if eng.iter().any(|e| kind(e, "electric_engine")) {
        return Err("electric engines can't be flown yet".into());
    }
    let prop = th.iter().flatten().any(|t| kind(t, "propeller"));
    let augmented = eng.iter().any(|e| e.augmented && e.max > e.mil * 1.05);
    // Power (W) for a propeller, dry and wet thrust (N) for a jet. A turbine
    // driving a propeller gives its rating as thrust: about 2.2 lbf per shaft horsepower.
    let hp: f64 = eng.iter().map(|e| if e.hp > 0.0 { e.hp } else { e.mil / 2.2 }).sum();
    let mil: f64 = eng.iter().map(|e| e.mil).sum::<f64>() * 4.448_22;
    let wet: f64 = eng.iter().map(|e| e.max).sum::<f64>() * 4.448_22;
    if prop && hp <= 0.0 || !prop && mil <= 0.0 {
        return Err("its engines give no power our model can use".into());
    }
    let mass_half = if j.empty_lbs > 0.0 { (j.empty_lbs + j.points_lbs + 0.5 * j.fuel_cap_lbs) * LB } else { j.weight_lbs * LB };
    if mass_half < 50.0 {
        return Err("its file gives no mass".into());
    }
    let base: &'static Spec = if prop {
        &aircraft::c172::SPEC
    } else if augmented || mass_half < 25_000.0 {
        &aircraft::fa18::SPEC
    } else {
        &aircraft::b747::SPEC
    };
    let a = fit_aero(j, if prop { 90.0 } else { 160.0 })?;
    let area = j.wing_area_ft2 * FT * FT;
    let span = j.span_ft * FT;
    let m = mass_half / base.mass;
    let mut s = base.clone();

    // Mass, balance, fuel.
    s.mass = mass_half;
    s.fuel_kg = if j.fuel_cap_lbs > 0.0 { j.fuel_cap_lbs * LB } else { base.fuel_kg * m };
    s.payload = [base.payload[0] * m, base.payload[1] * m, base.payload[2]];
    s.mac = if j.chord_ft > 0.0 { j.chord_ft * FT } else { area / span };
    s.inertia = if j.inertia.iter().all(|v| *v > 0.0) { glam::DVec3::new(j.inertia[1], j.inertia[2], j.inertia[0]) * 1.355_82 } else { base.inertia * m };
    for g in &mut s.groups {
        g.1 *= m as f32;
    }

    // The wing.
    s.wing_area = area;
    s.b = span;
    (s.cl0, s.cl_a, s.cd0, s.k_ind, s.cd_gear) = (a.cl0, a.cl_a, a.cd0, a.k_ind, if base.gear_rate > 0.0 { a.cd_gear.max(0.004) } else { 0.0 });
    let fs = base.cd_flap[0] + base.cd_flap[1];
    let fk = if fs > 0.0 { a.cd_flap / fs } else { 1.0 };
    s.cd_flap = [base.cd_flap[0] * fk, base.cd_flap[1] * fk];
    s.flap_cl = a.flap_cl;
    s.stall_alpha = a.stall_alpha;
    s.stall_flap = a.stall_flap;
    s.alpha_trim = base.alpha_trim.min(a.stall_alpha * 0.85);
    if !prop && !augmented {
        s.mcrit = base.mcrit;
    }

    // Speeds: ours scaled by how its stall and top speeds compare with the body's.
    let vs = stall_speed(mass_half, area, a.cl_max);
    let vs_f = stall_speed(mass_half, area, a.cl_max_flaps);
    let vs_base = stall_speed(base.mass, base.wing_area, our_cl_max(base, false));
    let vsf_base = stall_speed(base.mass, base.wing_area, our_cl_max(base, true));
    let (base_power, power) = match &base.engine {
        Engine::Piston(p) => (p.power, hp * 745.7),
        Engine::Fan(f) => (f.thrust * f.n as f64, mil),
        Engine::Rocket(_) => unreachable!("a rocket is never a downloaded type's body"),
    };
    let vmax = top_speed(prop, power, area, a.cd0);
    let vmax_base = top_speed(prop, base_power, base.wing_area, base.cd0);
    let r = (vmax / vmax_base).clamp(0.35, 3.0);
    s.vne = (base.vne * r).max(vs * 1.9);
    s.flutter_v = s.vne + (base.flutter_v - base.vne) * r;
    s.cruise = (base.cruise * r).clamp(vs * 1.4, s.vne * 0.85);
    let fr = vs_f / vsf_base;
    s.vfe = Box::leak(base.vfe.iter().map(|v| v * fr).collect::<Vec<_>>().into_boxed_slice());
    s.approach = ((vs_f * 1.3).max(base.approach.0 * fr * 0.9), base.approach.1);
    s.handling.q_ref = base.handling.q_ref * (vs / vs_base).powi(2);

    // Engines.
    let p = power / base_power;
    match &mut s.engine {
        Engine::Piston(pe) => {
            pe.power = power;
            // The propeller takes p times the power at r times the speed, so
            // it gives p / r times the thrust (the same efficiency, not r times it).
            pe.ct0 *= p / r;
            pe.cp0 *= p;
            pe.static_thrust *= p / r;
            pe.fric0 *= p;
            pe.fric1 *= p;
            pe.prop_i *= p.sqrt();
            // a coarser pitch for a faster aircraft: thrust runs out at a higher speed
            pe.j_t *= r;
            pe.j_p *= r;
        }
        Engine::Fan(f) => {
            f.thrust = mil / f.n as f64;
            f.boost = if augmented { wet / mil } else { 1.0 };
        }
        Engine::Rocket(_) => unreachable!("a rocket is never a downloaded type's body"),
    }
    s.fuel_flow = [base.fuel_flow[0] * p, base.fuel_flow[1] * p, if augmented { base.fuel_flow[1] * p * (1.0 + 2.5 * (wet / mil - 1.0)) } else { base.fuel_flow[1] * p }];

    // The gear and the airframe on the ground carry the new mass.
    s.gear = Box::leak(
        base.gear
            .iter()
            .map(|l| {
                let mut l = l.clone();
                (l.k, l.c, l.nf_max, l.side_k, l.roll_k, l.brake_k) = (l.k * m, l.c * m, l.nf_max * m, l.side_k * m, l.roll_k * m, l.brake_k * m);
                l
            })
            .collect::<Vec<_>>()
            .into_boxed_slice(),
    );
    (s.hull.k, s.hull.c, s.hull.nf_max, s.hull.drag) = (base.hull.k * m, base.hull.c * m, base.hull.nf_max * m, base.hull.drag * m);

    // Its own model: placed from the JSBSim reference points, on its own gear.
    if let Some(fg) = model {
        own_geometry(&mut s, base, j, fg, folder, mass_half, m, span);
    }

    // What it is called and where it comes from.
    let name = if name.is_empty() { display_name(j, folder) } else { name.to_string() };
    let kind = match (prop, n) {
        (true, 1) => "Propeller single".to_string(),
        (true, 2) => "Propeller twin".to_string(),
        (true, n) => format!("Propeller, {n} engines"),
        (false, 1) => "Jet".to_string(),
        (false, n) => format!("Jet, {n} engines"),
    };
    let power_s = if prop {
        format!("{} hp", thousands(hp))
    } else if augmented {
        format!("{} kN ({} kN with afterburner)", thousands(mil / 1000.0), thousands(wet / 1000.0))
    } else {
        format!("{} kN", thousands(mil / 1000.0))
    };
    let (license, _, _) = license_of("", Some(j));
    s.name = leak(name.clone());
    s.keys = Box::leak(vec![leak(key(folder))].into_boxed_slice());
    let what = if j.description.is_empty() || j.description.len() > 90 { String::new() } else { format!("{}. ", j.description.trim_end_matches('.')) };
    let drawn = if model.is_some() { "its own FlightGear 3D model".to_string() } else { format!("drawn as the {}", base.name) };
    s.blurb = leak(format!("{what}{kind}, {:.1} t, {power_s}. {} flight model fitted to ours; {drawn}.", mass_half / 1000.0, j.source));
    s.origin = Some(Box::leak(Box::new(Origin { license, author: j.author.clone(), body: base.name })));
    let summary = Summary { kind, mass_kg: mass_half, span_m: span, power: power_s, stall_kt: vs / KT, vmax_kt: vmax.min(s.vne) / KT, body: base.name, own_model: model.is_some(), fdm: j.source };
    Ok((s, summary))
}


/// Gives a fitted aircraft its own model, landing gear and eye point (from
/// the JSBSim file's reference points and contacts), and scales what of the
/// stand-in's geometry is left (ground contacts, camera) to its size.
fn own_geometry(s: &mut Spec, base: &Spec, j: &jsbsim::Aircraft, fg: &FgFiles, folder: &str, mass: f64, m: f64, span: f64) {
    // The stand-in's controls belong to its cockpit, not this imported model.
    s.hotspots = &[];
    let cg = j.cg.unwrap_or([0.0; 3]);
    // structural frame (x aft, y right, z up; feet) to ours (x right, y up, z aft; metres from the CG)
    let body = |p: [f64; 3]| glam::DVec3::new((p[1] - cg[1]) * FT, (p[2] - cg[2]) * FT, (p[0] - cg[0]) * FT);
    let origin = body(j.vrp.unwrap_or([0.0; 3])).as_vec3();
    let k = (span / base.b).clamp(0.2, 8.0);
    if let Some([x, y, z]) = fg.eye {
        s.eye = glam::Vec3::new(x, y, z) + origin;
    } else if let Some(e) = j.eye {
        s.eye = body(e).as_vec3();
    } else {
        s.eye = base.eye * k as f32;
    }
    let sc = |p: [f64; 3]| [p[0] * k, p[1] * k, p[2] * k];
    s.contacts = Box::leak(base.contacts.iter().map(|(p, part)| (sc(*p), *part)).collect::<Vec<_>>().into_boxed_slice());
    s.tail_pts = Box::leak(base.tail_pts.iter().map(|p| sc(*p)).collect::<Vec<_>>().into_boxed_slice());
    s.hard = base.hard.map(|p| p * k);
    s.cam_dist = base.cam_dist * k.clamp(0.4, 4.0);
    s.shadow_r = base.shadow_r * k.clamp(0.4, 4.0) as f32;

    let bogeys: Vec<&jsbsim::Contact> = j.contacts.iter().filter(|c| c.bogey).take(aircraft::MAX_GEAR).collect();
    let mut travel = Vec::new();
    if bogeys.len() >= 2 && !base.gear.is_empty() {
        let nose_t = base.gear.iter().find(|l| l.steer > 0.0).unwrap_or(&base.gear[0]);
        let main_t = base.gear.iter().find(|l| l.steer == 0.0).unwrap_or(&base.gear[base.gear.len() - 1]);
        let n = bogeys.len() as f64;
        let size = m.cbrt();
        let fwd = bogeys.iter().map(|c| body(c.at).z).fold(f64::MAX, f64::min);
        let legs: Vec<aircraft::Leg> = bogeys
            .iter()
            .map(|c| {
                let at = body(c.at);
                let steer = c.max_steer.abs() > 0.5;
                let t = if steer { nose_t } else { main_t };
                let mut l = t.clone();
                l.at = at;
                // JSBSim's spring and damper (lbs/ft, lbs/ft/s), else the stand-in's scaled
                let (kk, cc) = (c.spring * 14.5939, c.damping * 14.5939);
                l.k = if kk > 0.0 { kk } else { t.k * m };
                l.c = if cc > 0.0 { cc } else { t.c * m };
                let load = mass * 9.81 / n;
                l.travel = (t.travel * size).max(2.5 * load / l.k).clamp(0.05, 0.8);
                l.nf_max = (t.nf_max * m).max(l.k * l.travel * 1.5);
                (l.side_k, l.roll_k, l.brake_k) = (t.side_k * m, t.roll_k * m, t.brake_k * m);
                l.wheel_r = (t.wheel_r * size).clamp(0.1, 0.7);
                l.steer = if steer { 1.0 } else { 0.0 };
                l.brakes = c.brakes;
                l.name = if steer && (at.z - fwd).abs() < 0.3 { "Nose" } else if steer { "Tail" } else if at.x < -0.2 { "Left" } else if at.x > 0.2 { "Right" } else { "Centre" };
                travel.push(l.travel as f32);
                l
            })
            .collect();
        s.gear = Box::leak(legs.into_boxed_slice());
        s.gear_rate = if bogeys.iter().any(|c| c.retractable) { base.gear_rate.max(1.0 / 6.0) } else { 0.0 };
    }
    s.ext = Some(Box::leak(Box::new(crate::fgmodel::Ext { dir: fg.dir.clone(), fg_dir: folder.to_string(), model: fg.model.clone(), paint: fg.paint.clone(), origin, steer_max: s.steer_max as f32, travel, span: span as f32 })));
    if let Some(still) = fg.still {
        s.still = still;
    }
}

// ------------------------------------------------------------------ installing

/// What `source.json` in an installed aircraft's folder says.
fn source_json(e: &Entry, info: Option<&Info>, url: &str) -> String {
    let v = serde_json::json!({
        "source": "FlightGear aircraft catalogue (FGAddon)",
        "catalog": CATALOG,
        "id": e.id,
        "folder": e.folder,
        "revision": e.revision,
        "zip": url,
        "url": folder_url(&e.folder),
        "name": e.name,
        "author": if e.author.is_empty() { info.map(|i| i.author.clone()).unwrap_or_default() } else { e.author.clone() },
        "license": info.map(|i| i.license.clone()).unwrap_or_else(|| REPO_LICENSE.into()),
        "license_url": info.map(|i| i.license_url.clone()).unwrap_or_else(|| REPO_LICENSE_URL.into()),
        "repository_license": REPO_LICENSE,
    });
    serde_json::to_string_pretty(&v).unwrap_or_default()
}

fn installed_in(dir: &Path) -> Vec<String> {
    std::fs::read_dir(dir).map(|d| d.flatten().filter(|e| e.path().join("source.json").exists()).filter_map(|e| e.file_name().to_str().map(str::to_string)).collect()).unwrap_or_default()
}

fn installed_folders() -> Vec<String> {
    installed_in(&install_dir())
}

/// Downloads an aircraft (its package) and adds it to the aircraft list.
pub fn install(folder: &str) {
    let folder = folder.to_string();
    let entry = {
        let mut c = CAT.lock();
        let Some(e) = c.entries.iter_mut().find(|e| e.folder == folder) else { return };
        if matches!(e.install, Install::Downloading(..) | Install::Unpacking) {
            return;
        }
        e.install = Install::Downloading(0, e.size);
        e.clone()
    };
    std::thread::spawn(move || {
        let set = |st: Install| {
            if let Some(e) = CAT.lock().entries.iter_mut().find(|e| e.folder == folder) {
                e.install = st;
            }
        };
        match download(&entry, &set) {
            Ok(()) => set(Install::Installed),
            Err(e) => set(Install::Failed(e)),
        }
    });
}

/// Files of a package we leave out: sounds, scripts, splash screens, documents
/// (the XML anywhere stays: property files include each other from every folder).
fn skipped(top: &str, file: &Path) -> bool {
    let ext = file.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    ["wav", "ogg", "mp3", "flac", "nas", "pdf", "odt", "doc", "docx", "blend", "blend1", "xcf", "psd", "svg", "html", "htm"].contains(&ext.as_str())
        || (ext != "xml" && ["sounds", "splash", "splashs", "docs", "doc", "documentation", "tutorials", "help", "gui", "nasal"].contains(&top.to_ascii_lowercase().as_str()))
}

fn download(e: &Entry, set: &dyn Fn(Install)) -> Result<(), String> {
    let agent: ureq::Agent = ureq::Agent::config_builder().timeout_connect(Some(std::time::Duration::from_secs(20))).timeout_recv_body(Some(std::time::Duration::from_secs(60))).user_agent("fablegame-flightsim/0.1").build().into();
    let _ = std::fs::create_dir_all(install_dir());
    let zip_path = install_dir().join(format!(".{}.zip", e.folder));
    let tmp = install_dir().join(format!(".{}.part", e.folder));
    let _ = std::fs::remove_dir_all(&tmp);
    // The zip, from the first mirror that answers.
    let mut last = String::from("it lists no download");
    let mut used = String::new();
    for url in &e.urls {
        match fetch_to(&agent, url, &zip_path, e.size, set) {
            Ok(()) => {
                used = url.clone();
                break;
            }
            Err(err) => last = format!("{url}: {err}"),
        }
    }
    if used.is_empty() {
        let _ = std::fs::remove_file(&zip_path);
        return Err(last);
    }
    set(Install::Unpacking);
    let unpacked = unpack(&zip_path, &tmp, &e.folder);
    let _ = std::fs::remove_file(&zip_path);
    unpacked?;
    let info = match &catalogue().entries.iter().find(|x| x.folder == e.folder).map(|x| x.state.clone()) {
        Some(State::Ready(i)) => Some((**i).clone()),
        _ => None,
    };
    std::fs::write(tmp.join("source.json"), source_json(e, info.as_ref(), &used)).map_err(|err| err.to_string())?;
    let dest = install_dir().join(&e.folder);
    let _ = std::fs::remove_dir_all(&dest);
    std::fs::rename(&tmp, &dest).map_err(|err| err.to_string())?;
    // (one that doesn't load here isn't kept)
    load(&dest).map(|_| ()).inspect_err(|_| {
        let _ = std::fs::remove_dir_all(&dest);
    })
}

/// Streams a download to a file, reporting the bytes as they come.
fn fetch_to(agent: &ureq::Agent, url: &str, to: &Path, size: u64, set: &dyn Fn(Install)) -> Result<(), String> {
    use std::io::{Read, Write};
    let mut r = agent.get(url).call().map_err(|e| e.to_string())?;
    let total = r.body().content_length().unwrap_or(size);
    let mut rd = r.body_mut().with_config().limit(4 << 30).reader();
    let mut f = std::fs::File::create(to).map_err(|e| e.to_string())?;
    let mut buf = vec![0u8; 1 << 20];
    let mut done = 0u64;
    loop {
        let n = rd.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        f.write_all(&buf[..n]).map_err(|e| e.to_string())?;
        done += n as u64;
        set(Install::Downloading(done, total));
    }
    if total > 0 && done < total {
        return Err(format!("cut short at {done} of {total} bytes"));
    }
    Ok(())
}

/// Unpacks a package (its `<folder>/` inside the zip) into `to`, leaving out what we don't use.
fn unpack(zip_path: &Path, to: &Path, folder: &str) -> Result<(), String> {
    let f = std::fs::File::open(zip_path).map_err(|e| e.to_string())?;
    let mut z = zip::ZipArchive::new(f).map_err(|e| format!("the zip doesn't open: {e}"))?;
    let mut n = 0;
    for i in 0..z.len() {
        let mut file = z.by_index(i).map_err(|e| e.to_string())?;
        if file.is_dir() {
            continue;
        }
        let Some(p) = file.enclosed_name() else { continue };
        let mut comps = p.components();
        // (packages hold one folder, named after the aircraft)
        let first = comps.next().and_then(|c| c.as_os_str().to_str()).unwrap_or("").to_string();
        let rest: PathBuf = comps.collect();
        let inner = if first.eq_ignore_ascii_case(folder) { rest } else { p.clone() };
        let top = inner.components().next().and_then(|c| c.as_os_str().to_str()).unwrap_or("").to_string();
        if inner.as_os_str().is_empty() || skipped(if inner.components().count() > 1 { &top } else { "" }, &inner) {
            continue;
        }
        let out = to.join(&inner);
        if let Some(d) = out.parent() {
            std::fs::create_dir_all(d).map_err(|e| e.to_string())?;
        }
        let mut w = std::fs::File::create(&out).map_err(|e| format!("{}: {e}", out.display()))?;
        std::io::copy(&mut file, &mut w).map_err(|e| format!("{}: {e}", out.display()))?;
        n += 1;
    }
    if n == 0 {
        return Err("the zip is empty".into());
    }
    Ok(())
}

/// Its picture for the aircraft window: an exterior preview (else the
/// thumbnail), cropped to the window's shape, as a PNG.
fn still_of(dir: &Path) -> Option<&'static [u8]> {
    let mut previews: Vec<PathBuf> = std::fs::read_dir(dir.join("Previews")).map(|d| d.flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("png") || x.eq_ignore_ascii_case("jpg"))).collect()).unwrap_or_default();
    previews.sort();
    let img = previews.iter().chain([dir.join("thumbnail.jpg")].iter()).find_map(|p| image::open(p).ok())?;
    let (w, h) = (img.width() as f32, img.height() as f32);
    let aspect = 680.0 / 392.0;
    let (cw, ch) = if w / h > aspect { (h * aspect, h) } else { (w, w / aspect) };
    let crop = img.crop_imm(((w - cw) / 2.0) as u32, ((h - ch) / 2.0) as u32, cw as u32, ch as u32).resize_exact(680, 392, image::imageops::FilterType::Triangle);
    let mut out = std::io::Cursor::new(Vec::new());
    crop.to_rgba8().write_to(&mut out, image::ImageFormat::Png).ok()?;
    Some(Box::leak(out.into_inner().into_boxed_slice()))
}

/// Reads an installed aircraft from its folder and adds it to the aircraft list.
pub fn load(dir: &Path) -> Result<&'static Spec, String> {
    let folder = dir.file_name().and_then(|s| s.to_str()).ok_or("bad folder")?.to_string();
    let src: serde_json::Value = std::fs::read(dir.join("source.json")).ok().and_then(|d| serde_json::from_slice(&d).ok()).unwrap_or_default();
    let id = src["id"].as_str().ok_or("its source.json names no package")?;
    let name = src["name"].as_str().unwrap_or("").to_string();
    let get = |p: &str| std::fs::read(dir.join(p)).ok();
    let set = read_set(&get, id, &folder)?;
    let main = dir.join(format!("{}.xml", set.aero));
    let (j, parts) = match set.fdm.as_str() {
        "jsb" => {
            let j = jsbsim::Aircraft::load(&main)?;
            let part = |n: &str| -> Option<Part> { [dir.join("Engines").join(format!("{n}.xml")), dir.join("engine").join(format!("{n}.xml"))].iter().find_map(|p| std::fs::read(p).ok()).and_then(|d| read_part(&xml_text(&d)).ok()) };
            let parts: Vec<_> = j.engines.iter().map(|(e, t)| (part(e), t.as_deref().and_then(part))).collect();
            (j, parts)
        }
        "yasim" => {
            let xml = std::fs::read(&main).map(|d| xml_text(&d)).map_err(|e| format!("{}: {e}", main.display()))?;
            let (j, parts) = read_yasim(&xml, &set, &folder);
            (j?, parts)
        }
        f => return Err(format!("it flies on {}, not JSBSim or YASim", if f.is_empty() { "an unknown flight model" } else { f })),
    };
    let model = if set.model.is_empty() { None } else { Some(dir.join(&set.model)) }.filter(|p| p.exists());
    let fg = model.map(|m| FgFiles { dir: dir.to_path_buf(), model: m, still: still_of(dir), eye: set.eye, paint: set.paint.clone() });
    if fg.is_none() {
        log::warn!("{folder}: no 3D model found ({:?}); drawn as its stand-in", set.model);
    }
    let (mut spec, _) = fit(&j, &parts, &folder, &name, fg.as_ref())?;
    // The license its package states (the JSBSim file's, else the collection's).
    if let (Some(o), Some(l)) = (spec.origin, src["license"].as_str()) {
        let author = src["author"].as_str().map(str::to_string).filter(|a| !a.is_empty()).unwrap_or_else(|| o.author.clone());
        spec.origin = Some(Box::leak(Box::new(Origin { license: l.to_string(), author, body: o.body })));
    }
    Ok(aircraft::register(spec))
}

/// Adds every installed aircraft to the aircraft list (at start-up).
pub fn load_installed() {
    let mut folders = installed_folders();
    folders.sort();
    for f in folders {
        if let Err(e) = load(&install_dir().join(&f)) {
            log::warn!("downloaded aircraft {f}: {e}");
        }
    }
}

/// Deletes an installed aircraft and takes it off the list.
pub fn remove(folder: &str) -> Result<(), String> {
    std::fs::remove_dir_all(install_dir().join(folder)).map_err(|e| e.to_string())?;
    aircraft::unregister(&key(folder));
    if let Some(e) = CAT.lock().entries.iter_mut().find(|e| e.folder == folder) {
        e.install = Install::No;
    }
    Ok(())
}

/// `--fg-install FOLDER...`: downloads aircraft from the command line (for testing).
pub fn install_report(folders: &[String]) -> i32 {
    let agent = agent();
    let entries = match list(&agent) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("{e}");
            return 1;
        }
    };
    CAT.lock().entries = entries;
    let mut bad = 0;
    for f in folders {
        let Some(e) = CAT.lock().entries.iter().find(|e| e.folder.eq_ignore_ascii_case(f) || e.id.eq_ignore_ascii_case(f)).cloned() else {
            eprintln!("{f}: not in the catalogue");
            bad += 1;
            continue;
        };
        let st = match read_entry(&agent, &e.folder) {
            Ok(i) => {
                match &i.fit {
                    Ok(s) => println!("{}: {} · {} · {:.0} kg · stall {:.0} kt · top {:.0} kt · {}", e.folder, i.name, s.kind, s.mass_kg, s.stall_kt, s.vmax_kt, i.license),
                    Err(err) => println!("{}: {} — can't be flown here: {err}", e.folder, i.name),
                }
                State::Ready(Box::new(i))
            }
            Err(err) => State::Failed(err),
        };
        if let Some(x) = CAT.lock().entries.iter_mut().find(|x| x.folder == e.folder) {
            x.state = st;
        }
        let last = std::sync::atomic::AtomicU64::new(0);
        let progress = |s: Install| {
            if let Install::Downloading(d, t) = s {
                let mb = d >> 20;
                if mb >= last.load(std::sync::atomic::Ordering::Relaxed) + 10 {
                    last.store(mb, std::sync::atomic::Ordering::Relaxed);
                    eprintln!("  {mb} / {} MB", t >> 20);
                }
            }
        };
        match download(&e, &progress) {
            Ok(()) => {
                let k = key(&e.folder);
                let s = aircraft::by_key(&k).map(|s| s.blurb).unwrap_or("?");
                println!("{}: installed in {} (--aircraft {k})\n  {s}", e.folder, install_dir().join(&e.folder).display());
            }
            Err(err) => {
                eprintln!("{}: {err}", e.folder);
                bad += 1;
            }
        }
    }
    bad
}

/// `--fg-survey [TEXT]`: reads the catalogue entries (or those whose name
/// contains TEXT), one at a time, and prints what can be flown.
pub fn survey(filter: Option<&str>) -> i32 {
    let agent = agent();
    let entries = match list(&agent) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("{e}");
            return 1;
        }
    };
    CAT.lock().entries = entries.clone();
    for e in &entries {
        let pick = match filter {
            Some(f) => e.name.to_lowercase().contains(&f.to_lowercase()) || e.folder.to_lowercase().contains(&f.to_lowercase()),
            None => true,
        };
        if !pick {
            continue;
        }
        let st = match read_entry(&agent, &e.folder) {
            Ok(i) => State::Ready(Box::new(i)),
            Err(err) => State::Failed(err),
        };
        if let Some(x) = CAT.lock().entries.iter_mut().find(|x| x.folder == e.folder) {
            x.state = st;
        }
        std::thread::sleep(std::time::Duration::from_millis(400));
    }
    let c = CAT.lock();
    let (mut fly, mut no, mut other) = (0, 0, 0);
    for e in &c.entries {
        match &e.state {
            State::Ready(i) => match &i.fit {
                Ok(s) => {
                    fly += 1;
                    println!("OK  {:<16} {:<50} {:<6} {:<22} {:>6.0} kg  stall {:>3.0} kt  top {:>4.0} kt  {:>6.1} MB", e.folder, i.name.chars().take(50).collect::<String>(), s.fdm, s.kind, s.mass_kg, s.stall_kt, s.vmax_kt, e.size as f64 / 1e6);
                }
                Err(w) if w.contains("flight model:") || w.contains("name no flight model") => other += 1,
                Err(w) => {
                    no += 1;
                    println!("NO  {:<16} {w}", e.folder);
                }
            },
            State::Failed(w) => println!("ERR {:<16} {w}", e.folder),
            State::Pending => {}
        }
    }
    println!("{} aircraft ({} read): {fly} can be flown, {no} can't, {other} on other flight models", c.entries.len(), c.entries.iter().filter(|e| !matches!(e.state, State::Pending)).count());
    0
}

/// `--jsbsim-fit FILE...`: what each JSBSim aircraft file becomes here
/// (engines are looked for beside it, in `Engines/`, `engine/` and `../../engine/`).
pub fn fit_report(files: &[String]) -> i32 {
    for f in files {
        let path = Path::new(f);
        let folder = path.file_stem().and_then(|s| s.to_str()).unwrap_or("?");
        let dir = path.parent().unwrap_or(Path::new("."));
        let xml = std::fs::read(path).map(|d| xml_text(&d)).unwrap_or_default();
        if xml.contains("<airplane") {
            let (j, parts) = read_yasim(&xml, &SetInfo::default(), folder);
            let i = info(&xml, j, &parts, folder, "", false);
            match &i.fit {
                Ok(s) => println!("{folder:<18} OK   {:<28} {:<22} {:>7.0} kg {:>5.1} m  {:<34} stall {:>3.0} kt  top {:>4.0} kt  as {}  | YASim", i.name, s.kind, s.mass_kg, s.span_m, s.power, s.stall_kt, s.vmax_kt, s.body),
                Err(e) => println!("{folder:<18} NO   {e}  | YASim"),
            }
            continue;
        }
        let j = jsbsim::Aircraft::load(path).map_err(|e| format!("our reader can't read its JSBSim file: {e}"));
        let part = |name: &str| -> Option<Part> {
            [dir.join("Engines"), dir.join("engine"), dir.join("../../engine")].iter().map(|d| d.join(format!("{name}.xml"))).find_map(|p| std::fs::read_to_string(p).ok()).and_then(|s| read_part(&s).ok())
        };
        let parts: Vec<_> = j.as_ref().map(|j| j.engines.iter().map(|(e, t)| (part(e), t.as_deref().and_then(part))).collect()).unwrap_or_default();
        let i = info(&xml, j, &parts, folder, "", false);
        match &i.fit {
            Ok(s) => println!("{folder:<18} OK   {:<28} {:<22} {:>7.0} kg {:>5.1} m  {:<34} stall {:>3.0} kt  top {:>4.0} kt  as {}  | {}", i.name, s.kind, s.mass_kg, s.span_m, s.power, s.stall_kt, s.vmax_kt, s.body, i.license),
            Err(e) => println!("{folder:<18} NO   {e}  | {}", i.license),
        }
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imported_model_does_not_inherit_stand_in_hotspots() {
        let base = &aircraft::c172::SPEC;
        let mut s = base.clone();
        assert!(!s.hotspots.is_empty());
        let fg = FgFiles { dir: PathBuf::new(), model: PathBuf::from("Models/test.xml"), still: None, eye: None, paint: Default::default() };
        own_geometry(&mut s, base, &jsbsim::Aircraft::default(), &fg, "test", base.mass, 1.0, base.b);
        assert!(s.ext.is_some());
        assert!(s.hotspots.is_empty());
    }

    #[test]
    fn license_from_comments_and_repository() {
        assert_eq!(license_of("<!-- under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License -->", None).2, LicenseFrom::Comment);
        let (n, _, from) = license_of("<fdm_config/>", None);
        assert_eq!((n.as_str(), from), (REPO_LICENSE, LicenseFrom::Repository));
    }

    #[test]
    fn engine_parts() {
        let p = read_part("<turbine_engine name='x'><milthrust>100</milthrust><maxthrust>160</maxthrust><augmented>1</augmented></turbine_engine>").unwrap();
        assert!(p.augmented && p.mil == 100.0 && p.max == 160.0);
        let p = read_part("<piston_engine name='x'><maxhp>160</maxhp></piston_engine>").unwrap();
        assert_eq!((p.kind.as_str(), p.hp), ("piston_engine", 160.0));
    }

    #[test]
    fn thousands_groups() {
        assert_eq!(thousands(1234567.0), "1\u{202f}234\u{202f}567");
        assert_eq!(thousands(980.0), "980");
    }

    #[test]
    fn catalogue_names_stay_inside_their_folder() {
        for ok in ["c172p", "A-10", "Boeing-707", "737-200", "2020.3", "DR400 Regent"] {
            assert!(safe_name(ok), "{ok}");
        }
        for bad in ["", ".", "..", ".hidden", "../x", "a/b", "/etc", "a\\b", "C:", "C:\\Users", "a\nb"] {
            assert!(!safe_name(bad), "{bad:?}");
        }
    }

    #[test]
    fn set_files_and_includes() {
        let files: HashMap<&str, &str> = [
            ("x-set.xml", "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?><PropertyList include=\"x-base.xml\"><sim><flight-model>jsb</flight-model><aero>x-jsb</aero></sim></PropertyList>"),
            ("x-base.xml", "<PropertyList><sim><flight-model>yasim</flight-model><model><path>Aircraft/X/Models/x.xml</path></model></sim></PropertyList>"),
        ]
        .into_iter()
        .collect();
        let get = |p: &str| files.get(p).map(|s| s.as_bytes().to_vec());
        let s = read_set(&get, "x", "X").unwrap();
        assert_eq!((s.fdm.as_str(), s.aero.as_str(), s.model.as_str()), ("jsb", "x-jsb", "Models/x.xml"));
        assert_eq!(rel_path("Models/a.xml", "../Systems/b.xml", "X").as_deref(), Some("Systems/b.xml"));
        assert_eq!(rel_path("a.xml", "Aircraft/Generic/b.xml", "X"), None);
    }

    #[test]
    fn section_files() {
        assert_eq!(file_refs("<system file=\"electrical\"/><system file='x'/><propulsion file=\"Systems/engine.xml\"/>"), vec!["electrical.xml".to_string(), "Systems/engine.xml".to_string()]);
    }
}
