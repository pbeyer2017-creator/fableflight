mod airframe;
mod airfield;
mod airmodels;
mod airports;
mod approach;
mod audio;
mod autopilot;
mod buildings;
mod aircraft;
mod cockpit;
mod copilot;
mod cars;
mod crash;
mod damage;
mod destruct;
mod fetch;
mod frame_pacer;
mod fgmodel;
mod fluid;
mod gputime;
mod geo;
mod globeclouds;
mod hangar;
mod hangar_ui;
mod bind;
mod i18n;
mod jsbsim;
mod live;
mod radar;
mod livewx;
mod mp;
mod mp_proto;
mod nav;
mod settings;
mod settings_ui;
mod systems;
mod gamemode;
mod gauges;
mod gmaps;
mod graphics;
mod tiles3d;
mod impostor;
mod materials;
mod flightui;
mod mainmenu;
mod menu;
mod minimap;
mod padmap;
mod padui;
mod pave;
mod photonight;
mod relight;
mod rain;
mod storm;
mod weather;
mod weapons;
mod mvt;
mod plane;
mod render;
mod roads;
mod skin;
mod sources;
mod traffic;
mod trees;
mod ocean;
mod shore;
mod skyglow;
mod snow;
mod snowmap;
mod snowtrack;
mod world;
mod yasim;

#[derive(Copy, Clone, PartialEq)]
enum Mode {
    Menu,
    Fly,
}

/// What the gamepad's buttons work at the moment, besides flying.
#[derive(Copy, Clone, PartialEq, Eq)]
enum Nav {
    Off,
    Globe,
    Pause,
    Panel,
}

/// the fewest points of height the ui is laid out for.
pub const UI_MIN_H: f32 = 860.0;
/// how long a damage or failure flash stays at full strength, then how long it fades (s).
const FLASH_HOLD: f64 = 1.2;
const FLASH_FADE: f64 = 0.8;

/// What was chosen in the in-flight menu, carried out once the UI is drawn.
#[derive(Copy, Clone)]
enum PauseAct {
    Resume,
    Respawn,
    GoTo,
    FlyTo(f64, f64, &'static str),
    /// start at an airport: which, on final approach (else on the runway)
    Airport(u32, bool),
    Globe,
    Quit,
}

use crash::Crash;
use damage::Tear;
use fablegame::mcp;
use glam::{DMat4, DQuat, DVec2, DVec3, Mat4, Vec2, Vec3};
use i18n::{t, tr};
use padui::Hint;
use plane::{Controls, Plane};
use render::{DrawObj, Frame, Globals, PSim, Renderer};
use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

/// The other players on the start screen's globe (multiplayer): a pink
/// that no city, airport or the chosen spot uses, and that shows on sea and land.
const MP_COLOUR: egui::Color32 = egui::Color32::from_rgb(255, 112, 186);

const SUN_E: f32 = 10.0;
/// Exposure of the start screen's globe seen from orbit (a day-lit planet in
/// black space; flight's daytime exposure is 0.5).
const GLOBE_EXPOSURE: f32 = 0.5;
/// The aircraft's own shadow map: half width (m), how far back toward the sun
/// it starts, and how deep it reaches.
const PSHADOW_R: f32 = 6.2;
const PSHADOW_BACK: f32 = 40.0;
const PSHADOW_DEPTH: f32 = 1600.0;
/// Moonlight relative to sunlight. Far brighter than the real ratio, so a
/// moonlit night reads like a film's night scene instead of a black screen.
const MOON_E: f32 = SUN_E * 0.0035;
/// Default --render-scale on an integrated GPU (Apple silicon and the like):
/// it shares memory bandwidth and power with the CPU, and the full-resolution
/// scene is too much for it. The temporal AA and its sharpening hide most of it.
const INTEGRATED_SCALE: f32 = 0.75;
/// What a 3D tiles detail level costs, as measured over San Francisco (see
/// docs/photorealistic-3d-tiles-plan.md).
fn detail_hint(d: gmaps::Detail) -> &'static str {
    match d {
        gmaps::Detail::Low => t!("Low: softer buildings, 4 km around the aircraft. About 45 MB a minute over a city (a 6 Mbit/s connection keeps up), 250 MB of GPU memory.", "Faible : bâtiments plus flous, 4 km autour de l'avion. Environ 45 Mo par minute au-dessus d'une ville (une connexion à 6 Mbit/s suffit), 250 Mo de mémoire GPU."),
        gmaps::Detail::Medium => t!("Medium: 6 km around the aircraft. About 75 MB a minute over a city (10 Mbit/s), 400 MB of GPU memory.", "Moyen : 6 km autour de l'avion. Environ 75 Mo par minute au-dessus d'une ville (10 Mbit/s), 400 Mo de mémoire GPU."),
        gmaps::Detail::High => t!("High (default): 9 km around the aircraft. About 90 MB a minute over a city (12 Mbit/s), 640 MB of GPU memory.", "Élevé (par défaut) : 9 km autour de l'avion. Environ 90 Mo par minute au-dessus d'une ville (12 Mbit/s), 640 Mo de mémoire GPU."),
        gmaps::Detail::Ultra => t!("Ultra: sharpest close up, 9 km around the aircraft. About 240 MB a minute over a city (32 Mbit/s), 1 GB of GPU memory.", "Ultra : le plus net de près, 9 km autour de l'avion. Environ 240 Mo par minute au-dessus d'une ville (32 Mbit/s), 1 Go de mémoire GPU."),
    }
}

fn attribution_default() -> &'static str {
    t!("Imagery © Esri, Maxar, Earthstar Geographics", "Imagerie © Esri, Maxar, Earthstar Geographics")
}
fn attribution_rest() -> &'static str {
    t!(
        "Elevation: Mapzen / AWS Terrain Tiles  ·  Buildings, roads, land cover & water © OpenStreetMap contributors, OpenFreeMap / © OpenMapTiles  ·  Night lights: NASA Black Marble",
        "Relief : Mapzen / AWS Terrain Tiles  ·  Bâtiments, routes, occupation du sol et eau © contributeurs OpenStreetMap, OpenFreeMap / © OpenMapTiles  ·  Lumières nocturnes : NASA Black Marble"
    )
}

/// Data sizes ("12.3 MB"), with the French units ("12.3 Mo") in French.
fn bytes(n: u64) -> String {
    let s = gmaps::bytes(n);
    if i18n::is_fr() { s.replace(" GB", " Go").replace(" MB", " Mo").replace(" KB", " Ko").replace(" B", " o") } else { s }
}

/// An aircraft's model and its meshes on the GPU.
struct Loaded {
    spec: &'static aircraft::Spec,
    model: airframe::Model,
    piece_mesh: Vec<usize>,
    group_mesh: Vec<usize>,
    piece_cut: Vec<bool>,
    piece_inner: Vec<bool>,
    group_pivots: Vec<(Vec3, f32)>,
    /// what the anti-aliasing takes to be flying along with the camera:
    /// everything within the aircraft's bounds, and a little around them for
    /// the gear, flaps and control surfaces as they move
    plane_box: [[f32; 4]; 2],
    /// its outside lights (glow sprites, landing-light beam)
    lamps: Vec<airframe::Lamp>,
}

impl Loaded {
    fn build(spec: &'static aircraft::Spec, r: &mut Renderer) -> Loaded {
        Self::upload(spec, Self::make(spec), r)
    }

    /// The CPU half (mesh generation): safe to run off the main thread.
    fn make(spec: &'static aircraft::Spec) -> airframe::Model {
        let t0 = Instant::now();
        let model = match spec.ext.map(|e| e.build()) {
            Some(Ok(m)) => m,
            Some(Err(e)) => {
                log::warn!("{}: its own model doesn't load ({e}); drawing the stand-in", spec.name);
                (spec.build)()
            }
            None => (spec.build)(),
        };
        log::info!("{}: {} pieces, {} vertices, built in {:.0} ms", spec.name, model.pieces.len(), model.pieces.iter().map(|p| p.mesh.v.len()).sum::<usize>(), t0.elapsed().as_secs_f64() * 1000.0);
        model
    }

    fn upload(spec: &'static aircraft::Spec, model: airframe::Model, r: &mut Renderer) -> Loaded {
        let piece_mesh: Vec<usize> = model.pieces.iter().map(|p| r.add_mesh(&p.mesh.v, &p.mesh.i)).collect();
        let piece_cut: Vec<bool> = model.pieces.iter().map(|p| p.mesh.v.iter().any(|v| airframe::cut_by(v.mat[1]))).collect();
        // cabin and cockpit pieces, cabin materials only and no outer skin, culled from far outside.
        let inner_pat = |m: f32| matches!(m as u32, 7..=12 | 14 | 15 | 18..=20);
        let outer_pat = |m: f32| matches!(m as u32, 1..=6 | 13 | 16 | 17);
        let piece_inner: Vec<bool> = model.pieces.iter().map(|p| !p.glass && p.mesh.v.iter().any(|v| inner_pat(v.mat[1])) && !p.mesh.v.iter().any(|v| outer_pat(v.mat[1]))).collect();
        let mut group_mesh: Vec<usize> = model.groups.iter().map(|m| r.add_mesh(&m.v, &m.i)).collect();
        // A downloaded aircraft's own textures, for its pieces and its wreckage.
        if let Some(t) = &model.tex {
            let bind = r.texture_set(t);
            for &i in piece_mesh.iter().chain(group_mesh.iter()) {
                r.set_mesh_bind(i, bind.clone());
            }
        }
        for m in crash::shards() {
            group_mesh.push(r.add_mesh(&m.v, &m.i));
        }
        // Each group's wreckage tumbles about the middle of its bounds.
        let group_pivots = model
            .groups
            .iter()
            .map(|m| {
                let (lo, hi) = m.v.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(lo, hi), v| (lo.min(v.pos.into()), hi.max(v.pos.into())));
                if m.v.is_empty() { (Vec3::ZERO, 0.5) } else { ((lo + hi) * 0.5, ((hi - lo).max_element() * 0.35).max(0.3)) }
            })
            .collect();
        let (c, h) = model.bounds();
        let plane_box = [c.extend(0.0).to_array(), (h + Vec3::splat(0.25 * spec.span / 5.5)).extend(0.0).to_array()];
        let lamps = model.lamps();
        Loaded { spec, model, piece_mesh, group_mesh, piece_cut, piece_inner, group_pivots, plane_box, lamps }
    }
}

/// A scripted mouse event (--click, --wheel), for screenshots and checks:
/// when, where the mouse goes first, and what it does (0 button up, 1 down,
/// 100 + n the wheel turned n notches).
#[derive(Clone, Copy, Debug)]
struct MouseScript {
    t: f64,
    at: Option<(f64, f64)>,
    ev: f64,
}

struct Args {
    lat: f64,
    lon: f64,
    alt: Option<f64>,
    /// starting height above the ground, instead of `alt` above sea level
    agl: Option<f64>,
    heading: f64,
    place: Option<String>,
    max_zoom: u8,
    hour: f64,
    clouds: f32,
    /// cumulonimbus storm cell amount 0..1
    storms: Option<f32>,
    /// storm cells placed by hand: distance (km), compass bearing (deg), tower radius (km)
    storm_at: Vec<(f64, f64, f64, Option<weather::CellKind>)>,
    /// --squall-at: (km, bearing, length km) of squall lines
    squall_at: Vec<(f64, f64, f64)>,
    /// km,bearing: where the rain deck's area is centred, from the spawn
    rain_at: Option<(f64, f64)>,
    time_rate: Option<f64>,
    alto: Option<f32>,
    /// wind in knots and the direction it blows from, haze, day of year; open the F3 panel
    wind: Option<(f64, f64)>,
    haze: Option<f32>,
    day: Option<f64>,
    weather_panel: bool,
    /// real weather on for this run (else the saved setting, off when the command line sets the weather)
    live_weather: bool,
    /// print where it is storming now (weather radar) and quit
    find_storms: bool,
    vsync: bool,
    taa: bool,
    /// graphics preset and render resolution for this run (not saved; the
    /// resolution defaults to INTEGRATED_SCALE on a GPU without memory of its
    /// own until the graphics settings are saved)
    graphics: Option<graphics::Level>,
    render_scale: Option<f32>,
    /// log the frame rate once a second (average and slowest frame)
    fps_log: bool,
    /// draw without showing anything (benchmarks and scripted shots with the screen off)
    headless: bool,
    minimap_plan: Option<bool>,
    minimap_heading_up: Option<bool>,
    offline: bool,
    /// --snow auto|off|always: snow on the ground for this run (snow.rs)
    snow: Option<snow::Mode>,
    shots: Vec<(f64, PathBuf, Option<(f64, f64, f64)>)>,
    /// record frames + audio to a directory: (dir, start, seconds), fixed 30 fps steps
    record: Option<(PathBuf, f64, f64)>,
    crash_at: Option<f64>,
    /// from this many seconds on, full throttle with the boost key held (the afterburners), for scripted screenshots and recordings
    boost_at: Option<f64>,
    /// --ap mode@secs: an autopilot button pressed then (ap, hdg, nav, apr, alt, vs), for scripted tests
    ap_script: Vec<(f64, String)>,
    /// a rocket drops its booster after this many seconds, likewise
    stage_at: Option<f64>,
    /// a rocket burns this engine group (0 all) once it is let go, or starts as the ship alone coming back
    engines: Option<usize>,
    ship: bool,
    /// game time stands still for this many wall-clock seconds at the start (tiles stream in)
    hold: f64,
    size: (u32, u32),
    imagery: String,
    /// Google Maps API key (else GOOGLE_MAPS_API_KEY, else the saved one)
    google_key: Option<String>,
    /// switch Google imagery on or off for good from the command line
    google: Option<bool>,
    /// and Google's photorealistic 3D tiles
    photoreal: Option<bool>,
    /// how much of the 3D tiles to stream
    photo_detail: Option<gmaps::Detail>,
    cam: Option<(f64, f64, f64)>,
    /// for scripted screenshots: the mouse at (x, y) window pixels, and the
    /// left button pressed and released at these times (s)
    cursor: Option<(f64, f64)>,
    clicks: Vec<MouseScript>,
    cockpit: bool,
    /// --mp server[,name]: multiplayer on, through this relay
    mp: Option<(String, Option<String>)>,
    /// --mp-list URL: read the server list from there (testing a relay)
    mp_list: Option<String>,
    /// --mp-mirror: a copy of our own aircraft flies 25 m off the left wing as
    /// another player would be drawn (to check the drawing, no network)
    mp_mirror: bool,
    /// cockpit vertical field of view (deg)
    fov: Option<f32>,
    mute: bool,
    /// interface language (else the saved one, else the system's)
    lang: Option<i18n::Lang>,
    /// start on the start menu (default unless a place or scripted run is given):
    /// the main menu, or the screen `menu_screen` names (--menu-view and
    /// --airport open the globe, --settings and --hangar their screens)
    menu: Option<bool>,
    menu_view: Option<(f64, f64, f64)>,
    menu_go: Option<f64>,
    /// start already damaged (see `Damage::apply_spec`), for trying the damage model out
    damage: Option<String>,
    /// open the in-flight menu on this page after this many seconds, for scripted screenshots
    /// (page, settings section when the page is Settings, seconds)
    pause_menu: Option<(menu::Page, Option<settings_ui::Section>, f64)>,
    /// open the flight menu after this many seconds, for scripted screenshots.
    flight_menu: Option<f64>,
    /// `--settings SECTION`: the start screen opens with the settings screen on that section (scripted screenshots)
    settings: Option<settings_ui::Section>,
    /// `--hangar mine|download[:FOLDER]`: the start screen opens with the hangar on that page (scripted screenshots)
    hangar: Option<(hangar_ui::Tab, Option<String>)>,
    /// the start menu's screen to open on (`--menu-screen`)
    menu_screen: Option<menu::Screen>,
    /// a start position was given (--lat, --lon, --place): the globe opens on it
    pos_set: bool,
    /// start at an airport (ICAO or IATA code), on a runway or on approach
    airport: Option<String>,
    runway: Option<String>,
    approach: bool,
    /// which aircraft to fly (`aircraft::by_key`)
    aircraft: Option<String>,
    /// start with the HUD hidden (H), for screenshots
    no_hud: bool,
    /// rain 0..1 (wets the windows), and how wet the glass starts (default: settled in that rain)
    rain: Option<f32>,
    wet: Option<f32>,
    /// airborne start speed (kt) instead of the aircraft's cruise
    speed: Option<f64>,
    /// a weather preset by name (`--weather storm`: thunderstorms all round)
    weather_preset: Option<String>,
    /// lightning strikes forced at given times, for screenshots
    strikes: Vec<storm::Forced>,
    /// fuel to start with (fraction of full tanks), and the engine off on the ground
    fuel: Option<f64>,
    cold: bool,
    /// --arcade: arcade mode for this run
    arcade: bool,
    /// the AI copilot's loopback control service (on unless --no-mcp)
    mcp: bool,
    mcp_port: u16,
}

fn parse_args() -> Args {
    let mut a = Args {
        lat: 37.8085,
        lon: -122.4650,
        alt: None,
        agl: None,
        heading: 95.0,
        place: None,
        max_zoom: 17,
        hour: 16.6,
        clouds: 0.5,
        storms: None,
        storm_at: Vec::new(),
        squall_at: Vec::new(),
        rain_at: None,
        time_rate: None,
        alto: None,
        wind: None,
        haze: None,
        day: None,
        weather_panel: false,
        live_weather: false,
        find_storms: false,
        vsync: true,
        taa: true,
        graphics: None,
        render_scale: None,
        fps_log: false,
        headless: false,
        minimap_plan: None,
        minimap_heading_up: None,
        offline: false,
        snow: None,
        shots: Vec::new(),
        record: None,
        crash_at: None,
        mp: None,
        mp_mirror: false,
        mp_list: None,
        cursor: None,
        clicks: Vec::new(),
        boost_at: None,
        ap_script: Vec::new(),
        stage_at: None,
        engines: None,
        ship: false,
        hold: 0.0,
        size: (1600, 900),
        imagery: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}".into(),
        google_key: std::env::var("GOOGLE_MAPS_API_KEY").ok(),
        google: None,
        photoreal: None,
        photo_detail: None,
        cam: None,
        cockpit: false,
        fov: None,
        mute: false,
        lang: None,
        menu: None,
        menu_view: None,
        menu_go: None,
        damage: None,
        pause_menu: None,
        flight_menu: None,
        settings: None,
        hangar: None,
        menu_screen: None,
        pos_set: false,
        airport: None,
        runway: None,
        approach: false,
        aircraft: None,
        no_hud: false,
        rain: None,
        wet: None,
        speed: None,
        weather_preset: None,
        strikes: Vec::new(),
        fuel: None,
        cold: false,
        arcade: false,
        mcp: true,
        mcp_port: mcp::rpc::DEFAULT_PORT,
    };
    let v: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    let num = |v: &Vec<String>, i: usize| v.get(i + 1).and_then(|s| s.parse::<f64>().ok());
    while i < v.len() {
        let mut used = 2;
        match v[i].as_str() {
            "--minimap" => a.minimap_plan = v.get(i + 1).and_then(|s| match s.as_str() { "plan" => Some(true), "satellite" => Some(false), _ => None }),
            "--minimap-up" => a.minimap_heading_up = v.get(i + 1).and_then(|s| match s.as_str() { "north" => Some(false), "heading" => Some(true), _ => None }),
            "--lat" => {
                a.lat = num(&v, i).unwrap_or(a.lat);
                a.pos_set = true;
                a.menu.get_or_insert(false);
            }
            "--lon" => {
                a.lon = num(&v, i).unwrap_or(a.lon);
                a.pos_set = true;
                a.menu.get_or_insert(false);
            }
            "--menu-go" => a.menu_go = num(&v, i),
            "--menu-view" => {
                // lat,lon,altitude of the menu camera, for scripted screenshots
                let p: Vec<f64> = v.get(i + 1).map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default();
                if p.len() == 3 {
                    a.menu_view = Some((p[0], p[1], p[2]));
                }
            }
            "--menu" => {
                a.menu = Some(true);
                used = 1;
            }
            "--menu-screen" => {
                // the start menu's screen: main (the default), fly (the globe), aircraft, settings
                a.menu_screen = v.get(i + 1).and_then(|s| menu::Screen::parse(s));
                if a.menu_screen.is_none() {
                    eprintln!("--menu-screen: main, fly, aircraft or settings");
                }
                a.menu.get_or_insert(true);
            }
            "--no-menu" => {
                a.menu = Some(false);
                used = 1;
            }
            "--cockpit" => {
                a.cockpit = true;
                used = 1;
            }
            "--fov" => {
                a.fov = v.get(i + 1).and_then(|s| s.parse().ok());
                used = 2;
            }
            "--airport" => {
                a.airport = v.get(i + 1).cloned();
                a.menu.get_or_insert(false);
            }
            "--runway" => a.runway = v.get(i + 1).cloned(),
            "--aircraft" => {
                a.aircraft = v.get(i + 1).cloned();
                if let Some(k) = &a.aircraft {
                    if aircraft::by_key(k).is_none() {
                        eprintln!("unknown aircraft {k} (try {})", aircraft::list().iter().map(|s| s.keys[0]).collect::<Vec<_>>().join(", "));
                    }
                }
            }
            "--approach" => {
                a.approach = true;
                used = 1;
            }
            "--alt" => a.alt = num(&v, i),
            "--agl" => a.agl = num(&v, i),
            "--heading" => a.heading = num(&v, i).unwrap_or(a.heading),
            "--place" => {
                a.place = v.get(i + 1).cloned();
                a.pos_set = true;
                a.menu.get_or_insert(false);
            }
            "--max-zoom" => a.max_zoom = num(&v, i).unwrap_or(17.0).clamp(8.0, 19.0) as u8,
            "--time" => a.hour = num(&v, i).unwrap_or(a.hour),
            "--clouds" => a.clouds = num(&v, i).unwrap_or(0.5) as f32,
            "--storms" => a.storms = num(&v, i).map(|x| x as f32),
            "--storm-at" => {
                // km,bearing[,radius km[,shower|thunder|severe|supercell]]: a convective cell that far
                // from the spawn, a mature cumulonimbus unless a kind says otherwise (repeatable)
                let f: Vec<&str> = v.get(i + 1).map(|s| s.split(',').collect()).unwrap_or_default();
                let p: Vec<f64> = f.iter().take(3).map_while(|x| x.parse().ok()).collect();
                let kind = f.iter().skip(p.len()).find_map(|x| weather::CellKind::parse(x));
                if p.len() >= 2 {
                    let r = p.get(2).copied().unwrap_or(match kind {
                        Some(weather::CellKind::Shower) => 1.8,
                        Some(weather::CellKind::Supercell) => weather::SUPER_R,
                        _ => 6.0,
                    });
                    a.storm_at.push((p[0], p[1], r, kind));
                }
            }
            "--squall-at" => {
                // km,bearing[,length km]: a squall line that far from the spawn, lying
                // across that bearing (the wind aloft blows from there unless --wind says)
                let p: Vec<f64> = v.get(i + 1).map(|s| s.split(',').map_while(|x| x.parse().ok()).collect()).unwrap_or_default();
                if p.len() >= 2 {
                    a.squall_at.push((p[0], p[1], p.get(2).copied().unwrap_or(100.0)));
                }
            }
            "--rain-at" => {
                // km,bearing: centre the rain deck's area (--rain) that far from the spawn
                let p: Vec<f64> = v.get(i + 1).map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default();
                if p.len() >= 2 {
                    a.rain_at = Some((p[0], p[1]));
                }
            }
            "--time-rate" => a.time_rate = num(&v, i),
            "--alto" => a.alto = num(&v, i).map(|x| x as f32),
            "--wind" => {
                // knots,from-degrees
                let p: Vec<f64> = v.get(i + 1).map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default();
                if p.len() == 2 {
                    a.wind = Some((p[0], p[1]));
                }
            }
            "--haze" => a.haze = num(&v, i).map(|h| h as f32),
            "--day" => a.day = num(&v, i),
            "--weather" => {
                // a preset name (clear, fair, hazy, overcast, windy, storm), or on its own: open the weather panel
                match v.get(i + 1).filter(|s| !s.starts_with("--")) {
                    Some(p) => a.weather_preset = Some(p.clone()),
                    None => {
                        a.weather_panel = true;
                        used = 1;
                    }
                }
            }
            "--strike" => {
                // cg|ic,km,bearing@seconds: a lightning strike that far from the aircraft at that game time
                let f: Vec<&str> = v.get(i + 1).map(|s| s.split('@').collect()).unwrap_or_default();
                let p: Vec<&str> = f.first().map(|s| s.split(',').collect()).unwrap_or_default();
                let n = |k: usize| p.get(k).and_then(|x| x.parse::<f64>().ok());
                if let (Some(kind), Some(km), Some(at)) = (p.first(), n(1), f.get(1).and_then(|t| t.parse::<f64>().ok())) {
                    let kind = if kind.eq_ignore_ascii_case("ic") { storm::Kind::Cloud } else { storm::Kind::Ground };
                    a.strikes.push(storm::Forced { at, kind, km, bearing: n(2).unwrap_or(0.0) });
                }
            }
            "--imagery" => a.imagery = v.get(i + 1).cloned().unwrap_or(a.imagery.clone()),
            "--crash-at" => a.crash_at = num(&v, i),
            "--boost-at" => a.boost_at = num(&v, i),
            "--ap" => {
                if let Some((m, t)) = v.get(i + 1).and_then(|s| s.split_once('@')) {
                    a.ap_script.push((t.parse().unwrap_or(0.0), m.to_ascii_lowercase()));
                }
            }
            "--stage-at" => a.stage_at = num(&v, i),
            "--engines" => a.engines = num(&v, i).map(|k| k as usize),
            "--ship" => {
                a.ship = true;
                used = 1;
            }
            "--hold" => a.hold = num(&v, i).unwrap_or(0.0),
            "--damage" => a.damage = v.get(i + 1).cloned(),
            "--flight-menu" => a.flight_menu = Some(num(&v, i).unwrap_or(3.0)),
            "--pause-menu" => {
                // page[@seconds]: open the flight panel (Tab) or the pause menu (Esc) on that page, for scripted screenshots.
                // `settings:SECTION` (or a section's name alone, e.g. `graphics`) opens the settings screen there.
                let f: Vec<&str> = v.get(i + 1).map(|s| s.split('@').collect()).unwrap_or_default();
                let spec = f.first().copied().unwrap_or("");
                let (pg, sec) = spec.split_once(':').map(|(p, s)| (p, settings_ui::Section::parse(s))).unwrap_or((spec, None));
                let page = match (menu::Page::parse(pg), settings_ui::Section::parse(pg)) {
                    (Some(p), _) => Some((p, sec)),
                    (None, Some(s)) => Some((menu::Page::Settings, Some(s))),
                    _ => None,
                };
                if let Some((page, sec)) = page {
                    a.pause_menu = Some((page, sec, f.get(1).and_then(|t| t.parse().ok()).unwrap_or(0.0)));
                }
            }
            "--settings" => {
                // the start menu's settings screen, open on this section
                a.settings = Some(v.get(i + 1).and_then(|s| settings_ui::Section::parse(s)).unwrap_or(settings_ui::Section::General));
                a.menu.get_or_insert(true);
            }
            "--hangar" => {
                let f = v.get(i + 1).cloned().unwrap_or_default();
                let (page, sel) = f.split_once(':').map(|(p, s)| (p.to_string(), Some(s.to_string()))).unwrap_or((f, None));
                a.hangar = Some((if page.starts_with('d') { hangar_ui::Tab::Download } else { hangar_ui::Tab::Mine }, sel));
                a.menu.get_or_insert(true);
            }
            "--width" => a.size.0 = num(&v, i).unwrap_or(1600.0) as u32,
            "--height" => a.size.1 = num(&v, i).unwrap_or(900.0) as u32,
            "--cam" => {
                // yaw,pitch,distance of the chase camera, for scripted screenshots
                let p: Vec<f64> = v.get(i + 1).map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default();
                if p.len() == 3 {
                    a.cam = Some((p[0], p[1], p[2]));
                }
            }
            "--cursor" => {
                // x,y window pixels: the mouse, for scripted screenshots
                let p: Vec<f64> = v.get(i + 1).map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default();
                if p.len() == 2 {
                    a.cursor = Some((p[0], p[1]));
                }
            }
            "--click" | "--wheel" => {
                // --click seconds[,held seconds[,x,y]]: a left click (at --cursor, or x,y)
                // --wheel seconds,notches[,x,y]: the mouse wheel turned
                let p: Vec<f64> = v.get(i + 1).map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default();
                let at = (p.len() >= 4).then(|| (p[2], p[3]));
                if let Some(&t) = p.first() {
                    if v[i] == "--click" {
                        a.clicks.push(MouseScript { t, at, ev: 1.0 });
                        a.clicks.push(MouseScript { t: t + p.get(1).copied().unwrap_or(0.1), at: None, ev: 0.0 });
                    } else {
                        a.clicks.push(MouseScript { t, at, ev: 100.0 + p.get(1).copied().unwrap_or(1.0) });
                    }
                    a.clicks.sort_by(|x, y| x.t.total_cmp(&y.t));
                }
            }
            "--shot" => {
                // path@seconds[@yaw,pitch,distance]
                let f: Vec<&str> = v.get(i + 1).map(|s| s.split('@').collect()).unwrap_or_default();
                if f.len() >= 2 {
                    let c: Vec<f64> = f.get(2).map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default();
                    a.shots.push((f[1].parse().unwrap_or(5.0), PathBuf::from(f[0]), if c.len() == 3 { Some((c[0], c[1], c[2])) } else { None }));
                }
                a.menu.get_or_insert(false);
            }
            "--record" => {
                // dir@start@seconds: every frame steps 1/30 s; PNG frames + audio.wav
                let f: Vec<&str> = v.get(i + 1).map(|s| s.split('@').collect()).unwrap_or_default();
                if f.len() == 3 {
                    a.record = Some((PathBuf::from(f[0]), f[1].parse().unwrap_or(0.0), f[2].parse().unwrap_or(10.0)));
                }
                a.menu.get_or_insert(false);
            }
            "--no-vsync" => {
                a.vsync = false;
                used = 1;
            }
            "--no-taa" => {
                a.taa = false;
                used = 1;
            }
            "--rain" => a.rain = num(&v, i).map(|x| x as f32),
            "--wet" => a.wet = num(&v, i).map(|x| x as f32),
            "--speed" => a.speed = num(&v, i),
            "--graphics" => a.graphics = v.get(i + 1).and_then(|s| graphics::Level::parse(s)).filter(|l| *l != graphics::Level::Off),
            "--render-scale" => a.render_scale = num(&v, i).map(|s| (s as f32).clamp(graphics::SCALE_MIN, graphics::SCALE_MAX)),
            "--headless" => {
                a.headless = true;
                used = 1;
            }
            "--live-weather" => {
                a.live_weather = true;
                used = 1;
            }
            "--find-storms" => {
                a.find_storms = true;
                used = 1;
            }
            "--fps-log" => {
                a.fps_log = true;
                used = 1;
            }
            "--no-hud" => {
                a.no_hud = true;
                used = 1;
            }
            "--fuel" => a.fuel = num(&v, i).map(|f| f.clamp(0.0, 1.0)),
            "--arcade" => {
                a.arcade = true;
                used = 1;
            }
            "--cold" => {
                a.cold = true;
                used = 1;
            }
            "--mcp" => {
                a.mcp = true;
                used = 1;
            }
            "--no-mcp" => {
                a.mcp = false;
                used = 1;
            }
            "--mcp-port" => a.mcp_port = num(&v, i).map(|p| p.clamp(1.0, 65535.0) as u16).unwrap_or(a.mcp_port),
            "--photo-detail" => {
                a.photo_detail = Some(v.get(i + 1).and_then(|s| gmaps::Detail::parse(s)).unwrap_or_else(|| {
                    eprintln!("--photo-detail takes low, medium, high or ultra");
                    std::process::exit(2)
                }));
            }
            "--no-pave" => {
                pave::ON.store(false, std::sync::atomic::Ordering::Relaxed);
                used = 1;
            }
            "--mute" => {
                a.mute = true;
                used = 1;
            }
            "--lang" => {
                a.lang = Some(v.get(i + 1).and_then(|s| i18n::Lang::parse(s)).unwrap_or_else(|| {
                    eprintln!("--lang takes en or fr");
                    std::process::exit(2)
                }));
            }
            "--mp" => {
                if let Some(s) = v.get(i + 1) {
                    let (server, name) = match s.split_once(',') {
                        Some((a, b)) => (a.to_string(), Some(b.to_string())),
                        None => (s.to_string(), None),
                    };
                    a.mp = Some((server, name));
                }
            }
            "--mp-list" => a.mp_list = v.get(i + 1).cloned(),
            "--mp-mirror" => {
                a.mp_mirror = true;
                used = 1;
            }
            "--snow" => a.snow = v.get(i + 1).and_then(|s| snow::Mode::parse(s)),
            "--offline" => {
                a.offline = true;
                used = 1;
            }
            "--google-key" => a.google_key = v.get(i + 1).cloned(),
            "--google" => {
                a.google = Some(true);
                used = 1;
            }
            "--no-google" => {
                a.google = Some(false);
                used = 1;
            }
            "--photoreal" => {
                a.photoreal = Some(true);
                used = 1;
            }
            "--no-photoreal" => {
                a.photoreal = Some(false);
                used = 1;
            }
            "--help" | "-h" => {
                println!("fablegame [--menu | --no-menu] [--menu-screen main|fly|aircraft|settings] [--menu-view LAT,LON,ALT_M] [--hangar mine|download[:FOLDER]] [--place \"city or address\"] [--lat D --lon D] [--alt M | --agl M] [--heading DEG] [--speed KT] [--time HOURS] [--time-rate X] [--day 0..364] [--clouds 0..1] [--storms 0..1] [--alto 0..1] [--wind KT,FROM_DEG] [--haze 0.2..40] [--rain 0..1] [--rain-at KM,BEARING] [--wet 0..1] [--snow auto|off|always] [--weather [clear|fair|hazy|overcast|windy|storm]] [--storm-at KM,BEARING[,RADIUS_KM][,shower|thunder|severe]] [--squall-at KM,BEARING[,LENGTH_KM]] [--strike cg|ic,KM,BEARING@SECONDS] [--flight-menu SECONDS] [--pause-menu flight|places|radio|weather|game|settings[:SECTION]|controls|failures[@SECONDS]] [--settings general|graphics|sound|camera|controls|map] [--airport CODE [--runway 28L] [--approach]] [--max-zoom 8..19] [--cockpit] [--fov DEG] [--no-vsync] [--no-taa] [--graphics low|medium|high|ultra] [--render-scale 0.5..2] [--headless] [--minimap plan|satellite] [--minimap-up north|heading] [--fps-log] [--mute] [--lang en|fr] [--fuel 0..1] [--cold] [--mcp | --no-mcp] [--mcp-port PORT] [--offline] [--imagery URL_TEMPLATE] [--google | --no-google] [--photoreal | --no-photoreal] [--photo-detail low|medium|high|ultra] [--no-pave] [--google-key KEY] [--hold SECONDS] [--boost-at SECONDS] [--stage-at SECONDS] [--engines GROUP] [--ship] [--damage rwing=4.3,laileron=0,rudder=0.5,prop=0.5,engine=0.4,oil=0.6,crack=1,nosegear=0,lsnap=3.4,rfold=3.2,tail=2.0]");
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument {other}");
                used = 1;
            }
        }
        i += used;
    }
    a.shots.sort_by(|x, y| x.0.total_cmp(&y.0));
    a
}

fn url_encode(s: &str) -> String {
    s.bytes().map(|b| if b.is_ascii_alphanumeric() || b"-_.~".contains(&b) { (b as char).to_string() } else { format!("%{b:02X}") }).collect()
}

/// Look a place name up with OpenStreetMap's Nominatim. Returns lat, lon (degrees) and a label.
fn geocode(f: &fetch::Fetcher, q: &str) -> Option<(f64, f64, String)> {
    let url = format!("https://nominatim.openstreetmap.org/search?format=json&limit=1&accept-language={}&q={}", i18n::lang().code(), url_encode(q));
    let j: serde_json::Value = serde_json::from_slice(&f.get(&url).ok()?).ok()?;
    let r = j.get(0)?;
    Some((r["lat"].as_str()?.parse().ok()?, r["lon"].as_str()?.parse().ok()?, r["display_name"].as_str().unwrap_or(q).to_string()))
}

fn ground_elev(src: &sources::Sources, lat: f64, lon: f64) -> f64 {
    let id = geo::TileId::at(lat.to_radians(), lon.to_radians(), 11);
    let (u, v) = id.frac(lat.to_radians(), lon.to_radians());
    src.elev(id).map(|e| e.sample(u, v) as f64).unwrap_or(0.0).max(0.0)
}

/// The loading screen between the start screen and the flight: when it
/// began, the most tiles seen loading at once, how far along it is, since
/// when hardly anything has been loading, when it was done (it then fades),
/// and which tip it shows.
#[derive(Clone, Debug)]
struct Loading {
    t0: f64,
    most: usize,
    frac: f64,
    quiet_since: Option<f64>,
    done: Option<f64>,
    tip: usize,
}
impl Loading {
    /// shown at least this long, at most this long (s), fading out over FADE
    const MIN: f64 = 1.2;
    const MAX: f64 = 25.0;
    const FADE: f64 = 0.6;
    fn new(now: f64) -> Self {
        Loading { t0: now, most: 0, frac: 0.0, quiet_since: None, done: None, tip: (now * 1000.0) as usize }
    }
}

/// Shown on the loading screen, one at a time (translated in i18n.rs).
const LOADING_TIPS: &[&str] = &[
    "C switches between the chase view and the cockpit; drag the mouse to look around.",
    "F4 starts or stops the engine; for a start by hand: Ins battery, Page Up mixture, I magnetos, hold O.",
    "J shows the radio navigation instruments; the Radio page of the panel (Tab) tunes VORs and ILSs.",
    "X sets the altimeter to the QNH before take-off and on the way down.",
    "F and V move the flaps a notch; L the landing gear; B the wheel brakes.",
    "Tab opens the flight panel: instruments, places, weather, fuel and balance.",
    "R starts again in the air over where you are; G goes to any place or address.",
    "The weather and the traffic are real: METAR at the airports, live aircraft around you.",
    "F1 lists every key; joysticks and pedals are set up in Settings, Bindings.",
];

struct Game {
    window: Arc<Window>,
    r: Renderer,
    world: world::World,
    src: Arc<sources::Sources>,
    /// the Google API key as typed in the settings, and whether it is shown
    gkey: String,
    gkey_show: bool,
    /// Google Maps logo, once it has been shown
    gm_logo: Option<egui::TextureHandle>,
    /// the start screen's aircraft pictures (by index into `aircraft::list()`), once shown
    stills: std::collections::HashMap<usize, egui::TextureHandle>,
    /// the aircraft being flown, and its model
    spec: &'static aircraft::Spec,
    model: airframe::Model,
    /// renderer mesh of each model piece, of each breakable group, of the two shard shapes
    piece_mesh: Vec<usize>,
    group_mesh: Vec<usize>,
    /// does the piece have openings cut in it
    piece_cut: Vec<bool>,
    /// is the piece inside the cabin
    piece_inner: Vec<bool>,
    /// each breakable group's centre and size, for the wreckage
    group_pivots: Vec<(Vec3, f32)>,
    /// the models of the other aircraft flown this session, already on the GPU
    hangar: Vec<Loaded>,
    /// aircraft models being built on a background thread (menu selection)
    building: Vec<(&'static aircraft::Spec, std::sync::mpsc::Receiver<airframe::Model>)>,
    plane: Plane,
    /// the F/A-18's missiles: master arm, seeker and the two wingtip shots
    weapons: weapons::Weapons,
    crashed: bool,
    crash: Option<Crash>,
    crash_real: f64,
    minimap: minimap::Minimap,
    egui_ctx: egui::Context,
    egui_state: egui_winit::State,
    keys: HashSet<KeyCode>,
    drag: bool,
    lean_drag: Option<(f64, f64)>,
    last_cursor: Option<(f64, f64)>,
    /// where the mouse is in the window (pixels), whatever it is doing
    cursor: Option<(f64, f64)>,
    /// the mouse pinned by --cursor (scripted screenshots)
    script_cursor: Option<(f64, f64)>,
    /// scripted mouse events (--click, --wheel)
    script_clicks: Vec<MouseScript>,
    /// a cockpit hotspot the left button went down on: which, when, and
    /// whether its `hold` action has taken over (the key turned to START)
    hot_press: Option<(usize, f64, bool)>,
    /// keys held for a moment by the mouse wheel over a hotspot (until when)
    wheel_hold: Vec<(KeyCode, f64)>,
    /// wheel movement not yet a whole notch (trackpads)
    wheel_acc: cockpit::WheelSteps,
    hot_keys: HashSet<KeyCode>,
    cockpit_controls: Vec<(aircraft::Hotspot, Option<usize>)>,
    pit_blocks: Vec<(usize, Vec3, Vec3)>,
    cam_yaw: f64,
    cam_pitch: f64,
    cam_dist: f64,
    cam_mode: u8,
    cam_pos: DVec3,
    cam_dir: DVec3,
    cam_look: DVec3,
    orbit: f64,
    /// how far the crash camera rises to see past buildings in the way (a
    /// fraction of its distance), and when that was last checked
    cam_rise: f64,
    cam_clear_t: f64,
    /// the crash camera's bearing round what it watches
    cam_ang: Option<f64>,
    cam_hold: f64,
    utc: f64,
    doy: f64,
    /// clouds, haze, wind, clock speed (the F3 panel)
    wx: weather::Weather,
    time: f64,
    real: f64,
    last: Instant,
    acc: f64,
    wave_off: DVec2,
    /// the shore distance field's squares (shore.rs)
    shore: shore::Shore,
    /// the camera in the ocean's coordinates (ocean.rs, wraps at its WRAP)
    ocean_off: DVec2,
    prev_cam: DVec3,
    prev_vp: [[f32; 4]; 4],
    /// graphics settings (the pause menu's Graphics page; taa toggles with F2),
    /// and what was last handed to the renderer and world
    gfx: graphics::Graphics,
    gfx_applied: Option<graphics::Graphics>,
    /// where the aircraft was drawn last frame, relative to that frame's camera
    prev_plane: Mat4,
    /// centre and half size of the aircraft in its body frame
    plane_box: [[f32; 4]; 2],
    /// the outside lights of the aircraft being flown
    lamps: Vec<airframe::Lamp>,
    frame: u64,
    fps: f64,
    /// --fps-log: on, start of this second, frames in it, its slowest frame
    fps_log: (bool, Instant, u32, f64),
    paused: bool,
    show_help: bool,
    show_hud: bool,
    /// the hud in the cockpit view.
    hud_cockpit: bool,
    /// arcade mode (settings.rs): the fun features on
    arcade: bool,
    goto_open: bool,
    goto_text: String,
    goto_rx: Option<std::sync::mpsc::Receiver<Option<(f64, f64, f64, String)>>>,
    status: String,
    status_until: f64,
    shots: Vec<(f64, PathBuf, Option<(f64, f64, f64)>)>,
    crash_at: Option<f64>,
    boost_at: Option<f64>,
    stage_at: Option<f64>,
    engines: Option<usize>,
    hold: f64,
    gilrs: Option<gilrs::Gilrs>,
    pad: Controls,
    /// the gamepad last used (the one whose sticks fly the aircraft)
    pad_id: Option<gilrs::GamepadId>,
    /// pads already given the Xbox layout (so a pad it does not take on is not reopened forever)
    pad_fixed: Vec<[u8; 16]>,
    /// the autopilot (autopilot.rs), and when it last let go (its warning)
    ap: autopilot::Autopilot,
    ap_off_t: f64,
    ap_script: Vec<(f64, String)>,
    /// gamepad input for the menus: key events for the next UI pass, a focus
    /// move out of a text box, a d-pad direction held (and when it repeats),
    /// and a request to pick out the screen's first choice
    pad_events: Vec<egui::Event>,
    pad_move: Option<egui::FocusDirection>,
    /// moving on past text boxes: direction, passes since the last move, moves made
    pad_skip: Option<(egui::FocusDirection, u8, u8)>,
    pad_rep: Option<(egui::Key, f64)>,
    pad_home: bool,
    /// Y (North) held on the gamepad, and whether it has been used with
    /// another button since: a Y chord arms or locks; Y alone fires.
    pad_y: bool,
    pad_y_combo: bool,
    /// the pad's flying actions held now, and the key each holds
    pad_held: Vec<(&'static str, Option<KeyCode>)>,
    /// a word from binding the pad (what lost its binding), until when
    pad_note: (String, f64),
    /// the pad was used last, not the mouse: the middle of the cockpit view
    /// picks out the controls (see `pad_aim_at`)
    pad_aim: bool,
    /// the camera look the view starts out at (right stick click goes back to it)
    cam_home: (f64, f64),
    rng: crash::Rng,
    audio: Option<audio::Audio>,
    record: Option<(PathBuf, f64, f64)>,
    rec_audio: Vec<[f32; 2]>,
    rec_n: u32,
    rec_t0: Instant,
    /// sound on (U), and its volume (0..1)
    sound_on: bool,
    volume: f32,
    /// snow lying on the ground (snow.rs, the Graphics settings)
    snow: snow::Mode,
    /// the flight panel (Tab), beside the running game, and the page it shows
    info_open: bool,
    /// the flight menu, a ring of icons on tab.
    radial: flightui::Radial,
    info_page: menu::Page,
    /// the pause menu (Esc): open, the page it shows, and a scripted opening
    pause_open: bool,
    pause_page: menu::Page,
    pause_at: Option<(menu::Page, Option<settings_ui::Section>, f64)>,
    /// when to open the flight menu, for scripted screenshots.
    radial_at: Option<f64>,
    /// the section the settings screen shows (settings_ui.rs)
    settings_sec: settings_ui::Section,
    /// mouse-look speed (1 = default)
    look_sens: f64,
    /// the pilot's head in the cockpit view, the cockpit's vertical field of
    /// view (deg), and whether the head was in use last frame
    head: cockpit::Head,
    cockpit_fov: f32,
    /// the instruments' own state: lags, gyros, the compass card
    gauges: gauges::Gauges,
    /// water on the windows (rain.rs)
    glass: rain::Glass,
    /// lightning, and the rain falling around the camera (storm.rs)
    lightning: storm::Lightning,
    rain_field: storm::RainField,
    /// rain falling where the camera is, 0..1, and how much of it is hail
    cam_rain: f32,
    cam_hail: f32,
    /// snowflakes around the camera (snow.rs, precip.wgsl): their field, how
    /// much of what falls at the camera is snow, the blowing snow there, and
    /// the fresh snow settling where it snows on the ground
    snow_field: storm::RainField,
    cam_snow: f32,
    cam_blow: f32,
    fresh: snow::Fresh,
    fresh_line: f32,
    /// how far the eye has opened up to the gloom under a storm, 0..1
    storm_adapt: f32,
    head_live: bool,
    /// the cockpit warnings sounding, and when each latched one last tripped
    /// (pull up, too low, sink rate, gear) so they do not flicker
    warn: audio::Warn,
    warn_t: [f64; 4],
    /// for the sound: slow-following load factor (jolts stand out from it),
    /// when the last jolt was, last frame's ground contact, vertical speed,
    /// gear and flaps, and metal scraping along the ground this frame
    g_slow: f64,
    last_bump: f64,
    was_ground: bool,
    prev_vs: f64,
    prev_gear: f64,
    prev_flaps: f64,
    quit: bool,
    mode: Mode,
    menu: menu::Menu,
    menu_go: Option<f64>,
    heading: f64,
    destr: destruct::Destruction,
    /// parts that have come off the aircraft and are falling
    sheds: damage::Sheds,
    /// origin of the particle system's local frame, and how far it moved this frame
    fx_anchor: DVec3,
    fx_shift: DVec3,
    smoke_acc: f32,
    /// things that just broke, to flash on the HUD: text, when
    flash: Vec<(String, f64)>,
    /// the airport notice and when it last changed.
    airport_note: (String, f64),
    /// a weapon's fireball for the renderer's light: position, power, when
    weapon_flash: Option<(DVec3, f32, f64)>,
    /// aircraft shot down and still falling: they burn on the way down and
    /// blow up again where they hit the ground
    kills: Vec<Kill>,
    damage_spec: Option<String>,
    /// group wreck meshes have been cut down and need restoring on respawn
    wreck_dirty: bool,
    /// how long the aircraft has lain wrecked (unburnt), and when that was last looked at
    wreck_t: f64,
    wreck_seen: f64,
    start_rx: Option<std::sync::mpsc::Receiver<(f64, f64, f64)>>,
    /// the loading screen, from the start screen into the air (see `Loading`)
    loading: Option<Loading>,
    name_rx: Option<std::sync::mpsc::Receiver<String>>,
    last_click: f64,
    n_cars: usize,
    /// how much city is around (0..1), for the orange skyglow at night
    city_glow: f32,
    skyglow: skyglow::SkyGlow,
    /// today's real snow cover around the camera (snowmap.rs)
    snowmap: snowmap::SnowMap,
    /// dynamic snow tracks and crash trails carved into lying snow (snowtrack.rs)
    snowtrack: snowtrack::SnowTrack,
    /// an airport start being prepared: (airport, the airfield if it could be put together, runway, on approach, keep the local time)
    airport_rx: Option<std::sync::mpsc::Receiver<(u32, Option<Arc<airports::Geo>>, Option<String>, bool, bool)>>,
    /// an airport start asked for on the command line
    pending_airport: Option<(u32, Option<String>, bool)>,
    /// the settings as last saved, the language picked in the game (None: the
    /// system's), and when they were last compared with the live ones
    prefs: settings::Settings,
    /// where the last flight started.
    last_start: Option<settings::LastStart>,
    prefs_lang: Option<i18n::Lang>,
    prefs_t: f64,
    /// real air traffic around the player, and the last frame's camera
    /// (view-projection, position) to put labels on it
    live: live::Live,
    live_view: Option<(Mat4, DVec3)>,
    /// multiplayer (mp.rs): switched on, the relay and our name (settings),
    /// the connection while flying, and the other players this frame
    mp_on: bool,
    mp_server: String,
    mp_name: String,
    mp: Option<mp::Mp>,
    mp_players: Vec<mp::Remote>,
    /// on the start screen: the network's players for the globe (coarse),
    /// how many fly in all, and whether the relay sends a world map at all
    mp_world: Vec<mp_proto::Spot>,
    mp_world_total: u32,
    mp_world_known: bool,
    /// the network's servers (started when first wanted: settings, or "auto")
    mp_dir: Option<mp::Directory>,
    /// the server is typed in (not picked from the list)
    mp_custom: bool,
    /// where the server list is read from, in turn (--mp-list first)
    mp_list: Vec<String>,
    /// the near ones' stand-in aircraft, for posing their models (by player id)
    mp_puppets: std::collections::HashMap<u32, Plane>,
    /// --mp-mirror (see Args)
    mp_mirror: bool,
    /// real weather (sets `wx` while `wx.live` is on)
    livewx: livewx::LiveWx,
    /// the real clouds on the start screen's globe
    gclouds: globeclouds::GlobeClouds,
    /// the start time has been set by hand (the menu's Time sliders, or
    /// --time/--day): the globe then shows that time instead of the real one
    menu_time_set: bool,
    /// the menu's local time and date as last seen (to notice them being moved)
    menu_time_seen: (f64, f64),
    /// joysticks, pedals and moved keys, and what was last saved of them
    binds: bind::Bindings,
    binds_stamp: String,
    /// random failures: on, the mean interval (minutes), when the next one comes
    fail_random: bool,
    fail_mean: f64,
    fail_next: Option<f64>,
    /// NAV 1, the ADF, the OBS and the altimeter setting
    radios: nav::Radios,
    /// waiting on the ground for the terrain to load: where, and until when at most
    hold_spot: Option<(f64, f64, f64)>,
    /// the AI copilot's tool calls, and the screenshots waiting for a frame
    mcp_calls: Option<crossbeam_channel::Receiver<mcp::rpc::Call>>,
    mcp_shots: Vec<copilot::PendingShot>,
    mcp_shot_n: u64,
    /// the copilot's throttle setting, held until the pilot moves the throttle
    mcp_throttle: Option<f64>,
    /// the bound throttle axis's last reading, to notice the pilot moving it
    throttle_axis: Option<f64>,
}

impl Game {
    fn new(window: Arc<Window>, args: &Args) -> anyhow::Result<Self> {
        let prefs = settings::Settings::load();
        // Saved graphics settings; the command line overrides them for this run.
        let mut gfx = graphics::Graphics::load();
        if let Some(l) = args.graphics {
            gfx.apply(l);
        }
        gfx.vsync &= args.vsync;
        gfx.taa &= args.taa && prefs.taa;
        let r = pollster::block_on(Renderer::new(window.clone(), gfx.vsync, std::env::var("FABLE_MSAA").ok().and_then(|v| v.parse().ok()).unwrap_or(gfx.samples())))?;
        if let Some(sc) = args.render_scale {
            gfx.scale = sc;
        } else if r.integrated && !graphics::Graphics::saved() {
            gfx.scale = INTEGRATED_SCALE;
        }
        let cache = dirs::cache_dir().unwrap_or_else(|| PathBuf::from(".")).join("fablegame");
        let mut r = r;
        r.headless = args.headless;
        // Cached scans load offline too; missing ones keep the procedural layer.
        r.load_ground_photos(fetch::Fetcher::new(cache.clone(), args.offline));
        let google = Arc::new(gmaps::Google::load(args.google_key.clone()));
        if let Some(on) = args.google {
            google.set_enabled(on);
        }
        if let Some(on) = args.photoreal {
            google.set_photoreal(on);
        }
        if let Some(d) = args.photo_detail {
            google.set_detail(d);
        }
        let src = Arc::new(sources::Sources::with_google(fetch::Fetcher::new(cache, args.offline), args.imagery.clone(), google));
        let (mut lat, mut lon, mut status) = (args.lat, args.lon, String::new());
        if let Some(p) = &args.place {
            match geocode(&src.fetch, p) {
                Some((la, lo, name)) => {
                    (lat, lon) = (la, lo);
                    status = name;
                }
                None => status = t!("Could not find \"{}\"", "Introuvable : « {} »", p),
            }
        }
        let alt = args.alt.unwrap_or_else(|| ground_elev(&src, lat, lon) + args.agl.unwrap_or(450.0));
        let spec = args.aircraft.as_deref().and_then(aircraft::by_key).unwrap_or(aircraft::BUILTIN[0]);
        let mut plane = Plane::spawn(spec, lat.to_radians(), lon.to_radians(), alt, args.heading.to_radians(), args.speed.map(|kt| kt / 1.94384).unwrap_or(spec.cruise));
        if args.ship {
            plane.rocket_return();
        }
        if let Some(spec) = &args.damage {
            plane.dmg.apply_spec(spec);
        }
        let mut world = world::World::new(sources::Loader::new(src.clone(), 14), args.max_zoom);
        let mut r = r;
        r.set_paint(spec.paint.as_ref());
        let Loaded { spec: _, model, piece_mesh, group_mesh, piece_cut, piece_inner, group_pivots, plane_box, lamps } = Loaded::build(spec, &mut r);
        let tm = trees::build_meshes();
        r.set_tree_meshes(&tm.meshes);
        world.tree_dims = tm.dims;
        let mut solids = cars::build_models();
        solids.push(cars::cube());
        solids.extend((0..3).map(|k| cars::rock(k * 17 + 3)));
        solids.extend(destruct::slab_meshes());
        solids.extend(destruct::rebar_meshes());
        debug_assert_eq!(solids.len() as u32, airmodels::BASE);
        solids.extend(airmodels::build());
        r.set_inst_meshes(&solids);
        let mut menu = menu::Menu::new(lat, lon, args.hour);
        menu.fuel = args.fuel.unwrap_or(menu.fuel);
        menu.cold = args.cold;
        menu.aircraft = aircraft::index_of(spec);
        if let Some((la, lo, al)) = args.menu_view {
            menu.look_at(la, lo, al);
            menu.alt = al;
        }
        // The screen to open on: the main menu unless asked otherwise.
        if args.menu_view.is_some() || args.airport.is_some() {
            menu.screen = menu::Screen::Fly;
        }
        if args.settings.is_some() {
            menu.screen = menu::Screen::Settings;
        }
        if let Some((tab, sel)) = &args.hangar {
            menu.screen = menu::Screen::Aircraft;
            menu.air_tab = *tab;
            menu.air_view = menu.aircraft;
            menu.hangar_sel = sel.clone();
            if *tab == hangar_ui::Tab::Download {
                hangar::open();
            }
        }
        if let Some(sc) = args.menu_screen {
            menu.screen = sc;
            menu.air_view = menu.aircraft;
        }
        // The sun on the globe is the real one: open on the day side (unless
        // a view, a start position or a start time was given).
        let time_given = std::env::args().any(|a| matches!(a.as_str(), "--time" | "--day" | "--time-rate"));
        if args.menu_view.is_none() && !args.pos_set && !time_given {
            menu.lon = menu::day_side_lon(weather::real_clock().0);
            menu.lat = 25.0;
        }
        let mut wx = weather::Weather::new(args.clouds);
        match args.weather_preset.as_deref() {
            Some(p) if p.eq_ignore_ascii_case("storm") || p.eq_ignore_ascii_case("storms") || p.eq_ignore_ascii_case("thunderstorm") => wx.storm_field(plane.pos, args.heading),
            Some(p) => {
                if !wx.preset(p) {
                    eprintln!("unknown weather preset {p}");
                }
            }
            None => {}
        }
        if let Some(s) = args.storms {
            wx.storms = s.clamp(0.0, 1.0);
        }
        if let Some(s) = args.alto {
            wx.alto = s.clamp(0.0, 1.0);
        }
        wx.rain = args.rain.unwrap_or(0.0).clamp(0.0, 1.0);
        wx.rain_set = args.rain.map(|r| r.clamp(0.0, 1.0));
        wx.deck_place = args.rain_at;
        // The rain deck closes in from the start; the windows start as wet as
        // the rain where the aircraft is (none above the deck).
        wx.step_deck(0.0, plane.pos, None);
        wx.rain = wx.deck_rain(plane.pos) as f32;
        let mut lightning = storm::Lightning::new();
        lightning.forced = args.strikes.clone();
        // (As wet as the rain's share: snow does not run down the glass.)
        let start_snow = {
            let (pla, _, palt) = geo::pos_to_lla(plane.pos);
            let day = args.day.unwrap_or_else(|| std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0) / 86400.0 % 365.25);
            snow::snow_share(palt, snow::freezing_guess(args.snow.unwrap_or(prefs.snow), pla.to_degrees(), day)) as f32
        };
        let mut glass = rain::Glass::new(wx.rain * (1.0 - start_snow));
        if let Some(w) = args.wet {
            glass.wet = w.clamp(0.0, 1.0);
        }
        for &(d, b, r, kind) in &args.storm_at {
            wx.place_storm(plane.pos, d, b, r.clamp(1.0, 12.0), kind);
        }
        if let Some(r) = args.time_rate {
            wx.time_rate = r.clamp(0.0, 3600.0);
        }
        if let Some((kt, from)) = args.wind {
            (wx.wind, wx.wind_from) = (kt / 1.94384, from);
        } else if let Some(&(_, b, _)) = args.squall_at.first() {
            // A squall line comes on broadside with the wind: from where it was put.
            (wx.wind, wx.wind_from) = (wx.wind.max(8.0), b.rem_euclid(360.0));
        }
        for &(d, b, l) in &args.squall_at {
            wx.place_squall(plane.pos, d, b, l.clamp(10.0, 400.0));
        }
        if let Some(h) = args.haze {
            wx.haze = h.clamp(0.2, 40.0);
        }
        wx.open = args.weather_panel;
        // Real weather, unless the command line sets the weather itself.
        let scripted = std::env::args().any(|a| matches!(a.as_str(), "--clouds" | "--rain" | "--storms" | "--alto" | "--wind" | "--haze" | "--squall-at"));
        wx.live = (prefs.live_weather && !scripted) || args.live_weather;
        let set_time = std::env::args().any(|a| matches!(a.as_str(), "--time" | "--day" | "--time-rate"));
        wx.real_time = prefs.real_time && !set_time;
        // The spawn speed is through the air: start out moving with the wind,
        // or a headwind starts it off fast (and climbing it away).
        plane.wind = wx.steady(plane.pos, 300.0);
        plane.vel += plane.wind;
        let egui_ctx = egui::Context::default();
        load_fonts(&egui_ctx);
        menu::apply_theme(&egui_ctx);
        let egui_state = egui_winit::State::new(egui_ctx.clone(), egui::ViewportId::ROOT, &window, Some(window.scale_factor() as f32), None, Some(4096));
        let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
        let doy = (secs / 86400.0) % 365.25;
        let cam_pos = plane.pos - plane.forward() * spec.cam_dist;
        let weapons = weapons::Weapons::new(spec, &model.pieces);
        // What the anti-aliasing takes to be flying along with the camera:
        // everything within the aircraft's bounds, and a little around them
        // for the gear, flaps and control surfaces as they move.
        let (cy, cp, cd) = args.cam.unwrap_or((0.0, 0.2, spec.cam_dist));
        let cockpit_controls = aircraft::cockpit_controls(spec, &model.pieces);
        let pit_blocks = cockpit::blockers(&model.pieces);
        let mut g = Self {
            gkey: src.google.key(), gkey_show: false, gm_logo: None, stills: std::collections::HashMap::new(), window, r, world, src, spec, model, piece_mesh, group_mesh, piece_cut, piece_inner, group_pivots, hangar: Vec::new(), building: Vec::new(), crashed: false, crash: None, crash_real: 0.0, egui_ctx, egui_state, keys: HashSet::new(), drag: false, lean_drag: None, last_cursor: None,
            minimap: minimap::Minimap::new(&prefs),
            cam_yaw: cy.to_radians(), cam_pitch: cp, cam_dist: cd, cam_mode: if args.cockpit { 1 } else { prefs.cam_mode }, cam_pos, cam_dir: -plane.forward(), cam_look: plane.pos, orbit: 0.0, cam_rise: 0.0, cam_clear_t: 0.0, cam_ang: None, cam_hold: 0.0,
            utc: (args.hour - lon / 15.0).rem_euclid(24.0), doy: args.day.unwrap_or(doy), wx, time: 0.0, real: 0.0, last: Instant::now(), acc: 0.0, wave_off: DVec2::ZERO, shore: shore::Shore::default(), ocean_off: DVec2::ZERO, prev_cam: cam_pos, prev_vp: [[0.0; 4]; 4], gfx, gfx_applied: None, prev_plane: Mat4::IDENTITY, plane_box, lamps,
            frame: 0, fps: 60.0, fps_log: (args.fps_log, Instant::now(), 0, 0.0), paused: false, show_help: false, show_hud: !args.no_hud && prefs.show_hud, hud_cockpit: prefs.hud_cockpit, arcade: args.arcade || prefs.arcade, goto_open: false, goto_text: String::new(), goto_rx: None,
            status, status_until: 8.0, shots: args.shots.clone(), crash_at: args.crash_at, boost_at: args.boost_at, stage_at: args.stage_at, engines: args.engines, hold: args.hold, gilrs: open_gamepads(), pad: Controls::default(), pad_id: None, pad_fixed: Vec::new(), pad_events: Vec::new(), pad_move: None, pad_skip: None, pad_rep: None, pad_home: false, pad_y: false, pad_y_combo: false, pad_held: Vec::new(), pad_note: (String::new(), 0.0), pad_aim: false, ap: autopilot::Autopilot::default(), ap_off_t: -100.0, ap_script: args.ap_script.clone(), cam_home: (cy.to_radians(), cp), rng: crash::Rng(secs.to_bits() | 1),
            audio: if args.record.is_some() { Some(audio::Audio::offline()) } else if args.mute { None } else { audio::Audio::new() },
            record: args.record.clone(), rec_audio: Vec::new(), rec_n: 0, rec_t0: Instant::now(), sound_on: prefs.sound_on, volume: prefs.volume, snow: args.snow.unwrap_or(prefs.snow), info_open: false, radial: Default::default(), info_page: menu::Page::Flight, pause_open: false, pause_page: menu::Page::Game, pause_at: args.pause_menu, radial_at: args.flight_menu, settings_sec: args.settings.unwrap_or(settings_ui::Section::General), look_sens: prefs.look_sens, head: cockpit::Head::new(cy.to_radians(), cp), cockpit_fov: args.fov.unwrap_or(prefs.cockpit_fov).clamp(30.0, 90.0), head_live: false, warn: audio::Warn::default(), warn_t: [-10.0; 4],
            g_slow: 1.0, last_bump: 0.0, was_ground: false, prev_vs: 0.0, prev_gear: plane.gear, prev_flaps: plane.flaps, quit: false,
            mode: if args.menu.unwrap_or(true) { Mode::Menu } else { Mode::Fly }, menu, menu_go: args.menu_go, heading: args.heading.to_radians(), destr: destruct::Destruction::new(secs.to_bits()),
            sheds: damage::Sheds::default(), fx_anchor: plane.pos, fx_shift: DVec3::ZERO, smoke_acc: 0.0, flash: Vec::new(), airport_note: (String::new(), -100.0), weapon_flash: None, kills: Vec::new(), damage_spec: args.damage.clone(), wreck_dirty: false, wreck_t: 0.0, wreck_seen: 0.0,
            start_rx: None, loading: None, name_rx: None, last_click: -10.0, n_cars: 0, city_glow: 0.0, skyglow: Default::default(), snowmap: Default::default(), snowtrack: Default::default(), gauges: gauges::Gauges::new(&plane, secs.to_bits()), glass, lightning, rain_field: storm::RainField::default(), cam_rain: 0.0, cam_hail: 0.0, snow_field: storm::RainField::default(), cam_snow: start_snow, cam_blow: 0.0, fresh: snow::Fresh::default(), fresh_line: 0.0, storm_adapt: 0.0, plane, weapons,
            airport_rx: None, hold_spot: None,
            prefs_lang: args.lang.or(prefs.lang),
            prefs_t: 0.0,
            live: live::Live::new(prefs.live_traffic),
            mp_on: prefs.multiplayer || args.mp.is_some(),
            mp_server: args.mp.as_ref().map_or(prefs.mp_server.clone(), |m| m.0.clone()),
            mp_name: args.mp.as_ref().and_then(|m| m.1.clone()).unwrap_or(prefs.mp_name.clone()),
            mp: None,
            mp_players: Vec::new(),
            mp_world: Vec::new(),
            mp_world_total: 0,
            mp_world_known: false,
            mp_dir: None,
            mp_custom: false,
            mp_list: args.mp_list.iter().cloned().chain(mp::DIRECTORY_URLS.iter().map(|u| u.to_string())).collect(),
            mp_puppets: std::collections::HashMap::new(),
            mp_mirror: args.mp_mirror,
            live_view: None,
            cursor: args.cursor,
            script_cursor: args.cursor,
            script_clicks: args.clicks.clone(),
            hot_press: None,
            wheel_hold: Vec::new(),
            wheel_acc: cockpit::WheelSteps::default(),
            hot_keys: HashSet::new(),
            cockpit_controls,
            pit_blocks,
            livewx: livewx::LiveWx::new(),
            gclouds: globeclouds::GlobeClouds::default(),
            menu_time_set: set_time,
            menu_time_seen: (f64::NAN, f64::NAN),
            binds_stamp: String::new(),
            binds: bind::Bindings::load(),
            fail_random: false,
            fail_mean: 30.0,
            fail_next: None,
            radios: nav::Radios::default(),
            prefs: prefs.clone(),
            last_start: prefs.last.clone(),
            pending_airport: args.airport.as_deref().and_then(|c| {
                let i = airports::Db::get().find(c);
                if i.is_none() {
                    eprintln!("unknown airport {c}");
                }
                i.map(|i| (i, args.runway.clone(), args.approach))
            }),
            mcp_calls: None,
            mcp_shots: Vec::new(),
            mcp_shot_n: 0,
            mcp_throttle: None,
            throttle_axis: None,
        };
        if args.mcp {
            match mcp::rpc::serve(args.mcp_port) {
                Ok(rx) => g.mcp_calls = Some(rx),
                Err(e) => log::warn!("AI copilot service on 127.0.0.1:{}: {e}", args.mcp_port),
            }
        }
        if let Some(plan) = args.minimap_plan { g.minimap.plan = plan; }
        if let Some(up) = args.minimap_heading_up { g.minimap.heading_up = up; }
        // With --menu, an --airport is chosen on the globe rather than flown from.
        if args.menu == Some(true) {
            if let Some((i, rw, appr)) = g.pending_airport.take() {
                g.choose_airport(i);
                g.menu.runway = rw;
                g.menu.start = if appr { menu::Start::Approach } else { menu::Start::Runway };
            }
        }
        // otherwise open the globe on the last start.
        if g.menu.pick.is_none() && !args.pos_set && args.menu_view.is_none() && args.aircraft.is_none() {
            g.restore_last_start();
        }
        Ok(g)
    }

    /// the hud shows when on, but in the cockpit only when turned on there.
    fn hud_on(&self) -> bool {
        self.show_hud && (self.cam_mode != 1 || self.hud_cockpit)
    }

    /// puts the last flight's start and aircraft back on the globe.
    fn restore_last_start(&mut self) {
        let Some(l) = self.last_start.clone() else { return };
        if let Some(spec) = aircraft::by_key(&l.aircraft) {
            self.menu.aircraft = aircraft::index_of(spec);
            self.menu.air_view = self.menu.aircraft;
        }
        match l.airport.as_deref().and_then(|c| airports::Db::get().find(c)) {
            Some(i) => {
                self.choose_airport(i);
                self.menu.runway = l.runway.clone();
            }
            None => {
                self.menu.pick = Some((l.lat, l.lon));
                self.menu.pick_name = l.name.clone();
                self.menu.airport = None;
            }
        }
        self.menu.start = match l.start.as_str() {
            "approach" => menu::Start::Approach,
            "overhead" => menu::Start::Overhead,
            _ => menu::Start::Runway,
        };
        self.menu.look_at(l.lat, l.lon, 30_000.0);
    }

    /// the place and aircraft for the fly again button.
    pub(crate) fn last_start_label(&self) -> Option<(String, &'static str)> {
        let l = self.last_start.as_ref()?;
        let spec = aircraft::by_key(&l.aircraft)?;
        let place = match (&l.airport, &l.runway, l.start.as_str()) {
            (Some(a), Some(r), "runway") => format!("{a} {r}"),
            (Some(a), _, "approach") => t!("{} approach", "{} approche", a),
            (Some(a), _, _) => a.clone(),
            (None, _, _) if !l.name.is_empty() => l.name.chars().take(32).collect(),
            _ => menu::latlon(l.lat, l.lon),
        };
        Some((place, spec.name))
    }

    /// Fly another aircraft from the next spawn on: its model is built (once
    /// a session) and swapped in.
    /// The next view: outside, the cockpit, and the other cockpit of a type that has two.
    fn next_view(&mut self) {
        self.cancel_cockpit_input();
        self.head.center_seat();
        if self.cam_mode == 1 && !self.head.rear && self.spec.eye2.is_some() {
            self.head.rear = true;
        } else {
            self.head.rear = false;
            self.cam_mode = (self.cam_mode + 1) % 2;
        }
    }

    fn set_aircraft(&mut self, spec: &'static aircraft::Spec) {
        if std::ptr::eq(spec, self.spec) {
            return;
        }
        self.head.rear = false;
        self.head.center_seat();
        self.restore_wreck_meshes();
        self.sheds.items.clear();
        let next = match self.hangar.iter().position(|l| std::ptr::eq(l.spec, spec)) {
            Some(i) => self.hangar.swap_remove(i),
            None => match self.building.iter().position(|(s, _)| std::ptr::eq(*s, spec)) {
                // already building in the background: wait for that rather than start over
                Some(i) => {
                    let (_, rx) = self.building.swap_remove(i);
                    match rx.recv() {
                        Ok(m) => Loaded::upload(spec, m, &mut self.r),
                        Err(_) => Loaded::build(spec, &mut self.r),
                    }
                }
                None => Loaded::build(spec, &mut self.r),
            },
        };
        let old = Loaded {
            spec: self.spec,
            model: std::mem::replace(&mut self.model, next.model),
            piece_mesh: std::mem::replace(&mut self.piece_mesh, next.piece_mesh),
            group_mesh: std::mem::replace(&mut self.group_mesh, next.group_mesh),
            piece_cut: std::mem::replace(&mut self.piece_cut, next.piece_cut),
            piece_inner: std::mem::replace(&mut self.piece_inner, next.piece_inner),
            group_pivots: std::mem::replace(&mut self.group_pivots, next.group_pivots),
            plane_box: std::mem::replace(&mut self.plane_box, next.plane_box),
            lamps: std::mem::replace(&mut self.lamps, next.lamps),
        };
        self.hangar.push(old);
        self.r.set_paint(spec.paint.as_ref());
        self.spec = spec;
        self.cancel_cockpit_input();
        self.cockpit_controls = aircraft::cockpit_controls(spec, &self.model.pieces);
        self.pit_blocks = cockpit::blockers(&self.model.pieces);
        self.weapons.reset(spec, &self.model.pieces);
        self.cam_dist = spec.cam_dist;
        self.menu.aircraft = aircraft::index_of(spec);
    }

    /// Menu selection: swap to `spec` without stalling the frame. A model not
    /// built yet is generated on a worker thread and swapped in once it's done
    /// (only the GPU upload stays on the main thread).
    fn want_aircraft(&mut self, spec: &'static aircraft::Spec) {
        if self.hangar.iter().any(|l| std::ptr::eq(l.spec, spec)) {
            return self.set_aircraft(spec);
        }
        match self.building.iter().position(|(s, _)| std::ptr::eq(*s, spec)) {
            Some(i) => match self.building[i].1.try_recv() {
                Ok(m) => {
                    self.building.swap_remove(i);
                    let l = Loaded::upload(spec, m, &mut self.r);
                    self.hangar.push(l);
                    self.set_aircraft(spec);
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
                Err(_) => {
                    self.building.swap_remove(i);
                    self.set_aircraft(spec);
                }
            },
            None => {
                let (tx, rx) = std::sync::mpsc::channel();
                std::thread::spawn(move || {
                    let _ = tx.send(Loaded::make(spec));
                });
                self.building.push((spec, rx));
            }
        }
    }

    /// Readings for the cockpit instruments and the state of the lights (see plane.wgsl).
    fn instruments(&self, agl: f64) -> [[f32; 4]; 6] {
        let p = &self.plane;
        let (_, lon, _) = geo::pos_to_lla(p.pos);
        let local = (self.utc + lon.to_degrees() / 15.0).rem_euclid(24.0) * 3600.0;
        let t = self.time;
        let strobe = { let s = t.rem_euclid(1.3); if s < 0.045 || (s > 0.16 && s < 0.2) { 1.0 } else { 0.0 } };
        let beacon = (1.0 - ((t * 1.1).rem_euclid(1.0) - 0.15).abs() * 7.0).max(0.0);
        let (mut g, card) = self.gauges.readings(p);
        // (The navigation lights, beacon and strobes run off the bus.)
        let (lit, pw) = if p.sys.powered() { (1000.0, 1.0) } else { (0.0, 0.0) };
        g[0][1] += self.radios.alt_error_ft(self.wx.qnh) as f32;
        if p.spec.id == 9 && p.sys.powered() {
            // The G650's attitude and heading come from its inertial reference
            // units, not the vacuum gyro and DG the gauges model: the aircraft's
            // own, as soon as the bus is powered (the IRS alignment is not simulated).
            let (e, n, u) = geo::enu(p.pos);
            let (f, r, up) = (p.forward(), p.right(), p.up());
            g[1][0] = f.dot(u).clamp(-1.0, 1.0).asin().to_degrees() as f32;
            g[1][1] = (-r.dot(u)).atan2(up.dot(u)).to_degrees() as f32;
            g[0][3] = f.dot(e).atan2(f.dot(n)).to_degrees().rem_euclid(360.0) as f32;
        }
        if self.spec.is_jet() {
            // The glass cockpit's engine and air data (b747/deck747.wgsl).
            let (_, _, h) = geo::pos_to_lla(p.pos);
            let mach = p.mach();
            let oat = 15.0 - 6.5 * h.clamp(0.0, 11000.0) / 1000.0;
            let tat = oat + (oat + 273.15) * 0.2 * mach * mach;
            let x = ((p.n1 - 0.2) / 0.82).clamp(0.0, 1.2);
            let running = if p.running { (p.n1 / 0.15).clamp(0.0, 1.0) } else { 0.0 };
            // (The Hornet's two F404s: hotter and thirstier for their size, 4.9 t of fuel inside.
            // The Phantom's J79 turbojets: about 250 C idling, 650 C at military power.
            // The Raptor's F119s are taken to run like the Hornet's.
            // Concorde's Olympus 593 turbojets: about 350 C idling, 750 C flat out.
            // The Blackbird's J58s: about 400 C idling, 795 C at military power and in afterburner.
            // The G650's BR725s show ITT: taken as about 440 C idling and 870 C at take-off
            // power, an estimate (no published figures found).)
            let (idle_t, rise) = match self.spec.id { 2 | 4 => (400.0, 380.0), 3 => (250.0, 330.0), 6 => (350.0, 400.0), 7 => (80.0, 740.0), 9 => (440.0, 430.0), _ => (440.0, 200.0) };
            let egt = oat + (idle_t - oat + rise * x.powf(1.4)) * running;
            // Fuel flow per engine (kg/h) and the fuel on board (kg), from the engines' burn.
            let n_eng = match &self.spec.engine { aircraft::Engine::Fan(f) => f.n as f64, _ => 1.0 };
            let ff = p.fuel_flow * 3600.0 / n_eng;
            let fuel = p.fuel;
            let _ = x;
            let u = p.pos.normalize();
            let gs = (p.vel - u * p.vel.dot(u)).length() * 1.94384;
            return [
                g[0],
                g[1],
                [p.n1 as f32, egt as f32, ff as f32, fuel as f32],
                [mach as f32, gs as f32, (agl * 3.28084) as f32, tat as f32],
                [p.throttle as f32, local as f32, p.gear as f32, p.flap_deg() as f32],
                [lit + card.rem_euclid(360.0), strobe * pw, beacon as f32 * pw, if (if matches!(self.spec.id, 0 | 9) { p.landing_lights } else { p.gear_down }) && p.sys.powered() { 1.0 } else { 0.0 }],
            ];
        }
        // Row 5's first slot carries the nav lights (on above 1000) and the compass card.
        [
            g[0],
            g[1],
            g[2],
            g[3],
            [g[4][0], local as f32, p.gear as f32, p.flap_deg() as f32],
            [lit + card.rem_euclid(360.0), strobe * pw, beacon as f32 * pw, if (self.spec.id != 0 && p.gear_down || self.spec.id == 0 && p.landing_lights) && p.sys.powered() { 1.0 } else { 0.0 }],
        ]
    }

    /// The flown aircraft's outside lights: a glow sprite on each lens
    /// (lights.wgsl, as the traffic has), and the landing lights' beams
    /// (Globals `spot`, common.wgsl `spot_light`). `p5` is the lights' state
    /// (instruments row 5: nav on, strobe, beacon, landing).
    fn own_lights(&self, base: &Mat4, pose: &[Mat4], p5: [f32; 4], g: &mut render::Globals, out: &mut Vec<airfield::Light>) {
        use airfield::{L_GREEN, L_RED, L_STROBE, L_WHITE};
        use airframe::lamp;
        let dm = &self.plane.dmg;
        let big = self.spec.span > 30.0;
        let nav = if p5[0] >= 999.0 { 1.0 } else { 0.0 };
        let mut beam: [(Vec3, f32); 2] = [(Vec3::ZERO, 0.0); 2];
        let mut beam_dir = Vec3::ZERO;
        let bend = self.plane.wing_bend();
        let mid = {
            let xs: Vec<f32> = self.lamps.iter().filter(|l| l.id == lamp::LANDING as u32 && l.at.x.abs() > 0.5).map(|l| l.at.x).collect();
            xs.iter().sum::<f32>() / xs.len().max(1) as f32
        };
        for l in &self.lamps {
            let p = &self.model.pieces[l.piece];
            if dm.hidden(p) || !dm.tears_for(p).iter().all(|t| t.keeps(l.at)) {
                continue;
            }
            let m = *base * pose[l.piece];
            // (a wingtip light rides on the bent wing, as plane.wgsl bends it)
            let mut b = pose[l.piece].transform_point3(l.at);
            b.y += b.x * b.x * match p.group {
                airframe::G_LWING => bend[0],
                airframe::G_RWING => bend[1],
                _ => 0.0,
            };
            let at = base.transform_point3(b);
            let fwd = m.transform_vector3(Vec3::NEG_Z).normalize();
            let right = m.transform_vector3(Vec3::X).normalize();
            // The navigation lights show only in their sectors: red ahead and
            // out to the left, green to the right, white behind; each fades
            // out a little past 90 degrees from the middle of its sector, so
            // all show from above and below (lights.wgsl).
            let (inten, col, dir, spread) = match l.id as f32 {
                lamp::NAV_RED => (40.0 * nav, L_RED, (fwd - right).normalize(), -0.35),
                lamp::NAV_GREEN => (40.0 * nav, L_GREEN, (fwd + right).normalize(), -0.35),
                lamp::TAIL => (25.0 * nav, L_WHITE, -fwd, -0.35),
                lamp::STROBE => (1600.0 * p5[1], L_STROBE, Vec3::ZERO, 0.0),
                lamp::BEACON => (160.0 * p5[2], L_RED, Vec3::ZERO, 0.0),
                _ => (if big { 6000.0 } else { 3000.0 } * p5[3], L_WHITE, fwd, 0.86),
            };
            if l.id == lamp::LANDING as u32 && p5[3] > 0.0 && l.at.x.abs() > 0.5 {
                let k = (l.at.x > mid) as usize;
                beam[k].0 += at;
                beam[k].1 += 1.0;
                beam_dir = fwd;
            }
            if inten > 0.0 && out.len() < render::MAX_DYN_LIGHTS {
                out.push(airfield::Light { pos: at.into(), inten, dir: dir.into(), spread, col: [col[0], col[1], col[2], 3], fx: [0, 0, 255, 1] });
            }
        }
        // The beams: aimed a little below the aircraft's axis, as they are
        // set to light the runway ahead in the flare; per side about 3e5 cd
        // on an airliner, 1e5 on a light aircraft (Globals units: cd / 1e4 /
        // pi), times 60: MOON_E is far brighter than the real moon, and the
        // beams graze the runway, so true-to-life ones would hardly show
        // against moonlit pavement.
        if beam_dir != Vec3::ZERO {
            let up = (*base * pose[0]).transform_vector3(Vec3::Y).normalize();
            let dir = (beam_dir - up * 0.11).normalize();
            let per_side = if big { 3.0e5 } else { 1.0e5 } / 1.0e4 / std::f32::consts::PI * 60.0;
            let n = beam.iter().filter(|b| b.1 > 0.0).count() as f32;
            for (i, b) in beam.iter().enumerate() {
                if b.1 > 0.0 {
                    let at = b.0 / b.1;
                    g.spot[i] = [at.x, at.y, at.z, per_side * 2.0 / n];
                }
            }
            g.spot[2] = [dir.x, dir.y, dir.z, 14f32.to_radians().cos()];
        }
    }

    fn sun_dir(&self) -> DVec3 {
        let decl = (-23.44f64).to_radians() * (std::f64::consts::TAU * (self.doy + 10.0) / 365.25).cos();
        let lon = (-15.0 * (self.utc - 12.0)).to_radians();
        DVec3::new(decl.cos() * lon.cos(), decl.cos() * lon.sin(), decl.sin())
    }

    /// Whether snow lies on flat ground at `p` (for snow spray vs dirt dust).
    fn snowy_at(&self, p: DVec3) -> bool {
        match self.snow {
            snow::Mode::Off => false,
            snow::Mode::Always => true,
            snow::Mode::Auto => {
                let (la, _, alt) = geo::pos_to_lla(p);
                let Some(base) = snow::snow_line(self.snow, la.to_degrees(), self.doy, self.wx.freezing) else { return false };
                let line = self.snowmap.snow_line_at(p, base, self.doy);
                snow::cover(alt, 1.0, 0.0, line) > 0.35
            }
        }
    }

    /// Direction to the moon: the sun's ecliptic longitude advanced by the
    /// moon's elongation, carried into the same Earth-fixed frame as the sun.
    fn moon_dir(&self) -> (DVec3, f64) {
        use std::f64::consts::{FRAC_PI_2, TAU};
        let eps = 23.44f64.to_radians();
        let radec = |lam: f64| ((eps.cos() * lam.sin()).atan2(lam.cos()), (eps.sin() * lam.sin()).asin());
        let lam_s = TAU * (self.doy + 10.0) / 365.25 - FRAC_PI_2;
        let elong = TAU * self.wx.moon_age / 29.530588;
        let (ra_s, _) = radec(lam_s);
        let (ra_m, dec_m) = radec(lam_s + elong);
        let lon = (-15.0 * (self.utc - 12.0)).to_radians() + ra_m - ra_s;
        let lit = 0.5 * (1.0 - elong.cos());
        (DVec3::new(dec_m.cos() * lon.cos(), dec_m.cos() * lon.sin(), dec_m.sin()), lit)
    }

    fn respawn(&mut self, lat: f64, lon: f64, alt: f64, heading: f64) {
        self.respawn_at(lat, lon, alt, heading, self.spec.cruise);
    }

    /// The aircraft's missiles can be used: it has some, in arcade mode.
    fn missiles_ok(&self) -> bool {
        self.arcade && self.weapons.has_weapons()
    }

    fn missiles_refusal(&self) -> &'static str {
        if self.weapons.has_weapons() {
            t!("Missiles are for arcade mode (Settings › General)", "Les missiles sont réservés au mode arcade (Réglages › Général)")
        } else {
            t!("No missiles on this aircraft", "Pas de missiles sur cet avion")
        }
    }

    /// Says that `what` is arcade mode's.
    fn arcade_only(&mut self, what: &str) {
        self.status = t!("{} is for arcade mode (Settings › General)", "{} : réservé au mode arcade (Réglages › Général)", what);
        self.status_until = self.real + 3.0;
    }

    fn respawn_at(&mut self, lat: f64, lon: f64, alt: f64, heading: f64, speed: f64) {
        self.cancel_cockpit_input();
        self.ap.disengage();
        self.hold_spot = None;
        self.plane.parking_brake = false;
        self.mcp_throttle = None;
        self.plane = Plane::spawn(self.spec, lat, lon, alt, heading, speed);
        // Rain set by hand comes along to a new place (its deck closes in there).
        self.wx.deck_moved(self.plane.pos);
        self.plane.fuel = self.spec.fuel_kg * self.menu.fuel;
        self.plane.payload = self.start_payload();
        // The spawn speed is through the air, not over the ground.
        self.plane.wind = self.wx.steady(self.plane.pos, 300.0);
        self.plane.vel += self.plane.wind;
        if let Some(spec) = &self.damage_spec {
            self.plane.dmg.apply_spec(spec);
        }
        self.restore_wreck_meshes();
        self.flash.clear();
        self.crashed = false;
        self.cam_rise = 0.0;
        self.cam_ang = None;
        self.acc = 0.0;
        self.cam_pos = self.plane.pos - self.plane.forward() * self.spec.cam_dist;
        self.cam_dir = -self.plane.forward();
        self.prev_cam = self.cam_pos;
        self.warn_t = [-10.0; 4];
        (self.g_slow, self.was_ground, self.prev_vs, self.prev_gear, self.prev_flaps) = (1.0, false, 0.0, self.plane.gear, self.plane.flaps);
        self.gauges = gauges::Gauges::new(&self.plane, self.frame ^ 0x9e37);
        self.weapons.reset(self.spec, &self.model.pieces);
        self.weapon_flash = None;
        self.kills.clear();
    }

    /// On the ground, at rest, waiting for the terrain under it to load.
    fn respawn_ground(&mut self, lat: f64, lon: f64, h: f64, heading: f64, pitch: f64) {
        self.respawn_at(lat, lon, h + 10.0, heading, 0.0);
        self.plane = Plane::spawn_ground(self.spec, lat, lon, h, heading, pitch);
        self.plane.fuel = self.spec.fuel_kg * self.menu.fuel;
        self.plane.payload = self.start_payload();
        if self.menu.cold {
            self.plane.cold_and_dark();
        }
        if let Some(spec) = &self.damage_spec {
            self.plane.dmg.apply_spec(spec);
        }
        self.gauges = gauges::Gauges::new(&self.plane, self.frame ^ 0x51ed);
        self.cam_pos = self.plane.pos - self.plane.forward() * self.spec.cam_dist;
        self.cam_dir = -self.plane.forward();
        self.prev_cam = self.cam_pos;
        (self.prev_gear, self.prev_flaps, self.was_ground) = (1.0, 0.0, true);
        self.hold_spot = Some((lat, lon, self.real + 30.0));
    }

    /// Start at an airport: its layout is put together in the background
    /// (see `airport_rx`), then the aircraft appears lined up on the runway
    /// in use, or on a final approach to it.
    /// The loading screen: done once the ground under the aircraft is there and
    /// the tiles it needs have come in (or after `Loading::MAX` seconds).
    fn update_loading(&mut self) {
        let Some(l) = self.loading.as_mut() else { return };
        if let Some(done) = l.done {
            if self.real - done > Loading::FADE {
                self.loading = None;
            }
            return;
        }
        let photo = self.world.photo();
        let n = self.world.stats.1 + self.world.queued() + if photo.active { photo.stats.2 } else { 0 };
        l.most = l.most.max(n);
        let (lat, lon, _) = geo::pos_to_lla(self.plane.pos);
        let ground = self.world.ground(lat, lon).is_some();
        let t = self.real - l.t0;
        // How far along: the tiles come in, and time passes (never going back).
        let tiles = if l.most > 0 { 1.0 - n as f64 / l.most as f64 } else { 0.0 };
        l.frac = l.frac.max((0.92 * tiles + 0.08 * (t / Loading::MAX)).min(if ground { 1.0 } else { 0.9 }));
        // (A few tiles far off are always on their way: "nearly none" is enough.)
        let quiet = ground && n <= 3;
        l.quiet_since = if quiet { Some(l.quiet_since.unwrap_or(self.real)) } else { None };
        if (t > Loading::MIN && l.quiet_since.is_some_and(|q| self.real - q > 0.6)) || t > Loading::MAX {
            self.finish_loading();
        }
    }
    fn finish_loading(&mut self) {
        if let Some(l) = self.loading.as_mut() {
            l.frac = 1.0;
            l.done.get_or_insert(self.real);
        }
    }
    /// The loading screen over the scene (which goes on loading under it):
    /// the aircraft, where it is going and the weather there, a progress bar
    /// and a tip, fading out once done.
    fn loading_overlay(&mut self, ctx: &egui::Context) {
        use egui::{Align2, Color32, FontId};
        let Some(l) = self.loading.clone() else { return };
        let a = match l.done {
            Some(d) => (1.0 - (self.real - d) / Loading::FADE).clamp(0.0, 1.0) as f32,
            None => 1.0,
        };
        let screen = ctx.content_rect();
        let spec = self.spec;
        let tex = self.still(ctx, spec);
        let fade = |c: Color32| c.gamma_multiply(a);
        egui::Area::new(egui::Id::unique("loading")).order(egui::Order::Foreground).fixed_pos(screen.min).show(ctx, |ui| {
            let p = ui.painter();
            p.rect_filled(screen, 0.0, fade(Color32::from_rgb(8, 11, 16)));
            let c = screen.center();
            // The aircraft, as on the start screen.
            let w = (screen.width() * 0.5).min(680.0);
            let pic = egui::Rect::from_center_size(c - egui::vec2(0.0, w * 0.30), egui::vec2(w, w * 392.0 / 680.0));
            egui::Image::new(&tex).tint(Color32::WHITE.gamma_multiply(a)).paint_at(ui, pic);
            let mut y = pic.bottom() + 18.0;
            let mut line = |s: &str, size: f32, col: Color32| {
                p.text(egui::pos2(c.x, y), Align2::CENTER_TOP, s, FontId::proportional(size), fade(col));
                y += size + 10.0;
            };
            line(spec.name, 26.0, Color32::WHITE);
            if !self.status.is_empty() {
                line(&self.status, 15.0, menu::ACCENT);
            }
            let wx = t!("Wind {:03.0}° / {:.0} kt  ·  QNH {:.0} hPa", "Vent {:03.0}° / {:.0} kt  ·  QNH {:.0} hPa", self.wx.wind_from.rem_euclid(360.0), self.wx.wind * 1.94384, self.wx.qnh);
            line(&wx, 14.0, Color32::from_gray(170));
            // Progress.
            let bar = egui::Rect::from_center_size(egui::pos2(c.x, y + 16.0), egui::vec2(w.min(460.0), 6.0));
            p.rect_filled(bar, menu::R_WIDGET, fade(Color32::from_gray(45)));
            let mut done = bar;
            done.set_width(bar.width() * l.frac as f32);
            p.rect_filled(done, menu::R_WIDGET, fade(menu::ACCENT));
            y = bar.bottom() + 12.0;
            let what = if l.done.is_some() { t!("Ready", "Prêt").to_string() } else { t!("Loading the scenery…", "Chargement du décor…").to_string() };
            p.text(egui::pos2(c.x, y), Align2::CENTER_TOP, what, menu::caption(), fade(Color32::from_gray(150)));
            // A tip, and how to skip.
            let tip = tr(LOADING_TIPS[l.tip % LOADING_TIPS.len()]);
            p.text(egui::pos2(c.x, screen.bottom() - 70.0), Align2::CENTER_TOP, tip, menu::body(), fade(Color32::from_gray(200)));
            p.text(egui::pos2(c.x, screen.bottom() - 40.0), Align2::CENTER_TOP, t!("Enter or click: skip", "Entrée ou clic : passer"), menu::caption(), fade(Color32::from_gray(110)));
        });
        // A click skips it too.
        if l.done.is_none() && ctx.input(|i| i.pointer.any_click()) {
            self.finish_loading();
        }
    }

    fn start_airport(&mut self, idx: u32, runway: Option<String>, approach: bool, keep_local: bool) {
        let a = &airports::Db::get().apts[idx as usize];
        self.status = t!("Preparing {} {}…", "Préparation de {} {}…", a.code(), a.name);
        self.status_until = self.real + 60.0;
        let (tx, rx) = std::sync::mpsc::channel();
        let src = self.src.clone();
        std::thread::spawn(move || {
            let g = airports::geo_for(&src, idx).map_err(|e| log::warn!("airport: {e}")).ok();
            let _ = tx.send((idx, g, runway, approach, keep_local));
        });
        self.airport_rx = Some(rx);
        self.menu.starting = true;
    }

    fn airport_ready(&mut self, idx: u32, g: Option<Arc<airports::Geo>>, runway: Option<String>, approach: bool, keep_local: bool) {
        let a = &airports::Db::get().apts[idx as usize];
        let local = if keep_local { (self.utc + geo::pos_to_lla(self.plane.pos).1.to_degrees() / 15.0).rem_euclid(24.0) } else { self.menu.hour };
        self.utc = (local - a.lon / 15.0).rem_euclid(24.0);
        if self.mode == Mode::Menu {
            self.loading = Some(Loading::new(self.real));
        }
        self.mode = Mode::Fly;
        self.menu.starting = false;
        self.status_until = self.real + 12.0;
        let Some(g) = g.filter(|g| !g.runways.is_empty()) else {
            // Nothing on the map to land on: overhead instead.
            self.respawn(a.lat.to_radians(), a.lon.to_radians(), a.elev as f64 + 450.0, self.heading);
            self.status = t!("{} {} (no runway layout on the map)", "{} {} (pas de plan des pistes sur la carte)", a.code(), a.name);
            return;
        };
        let want = runway.as_deref().map(airports::norm_ident);
        let pick = want.as_ref().and_then(|w| g.runways.iter().enumerate().find_map(|(ri, r)| (0..2).find(|e| airports::norm_ident(&r.ident[*e]) == *w).map(|e| (ri, e))));
        let (ri, e) = pick.or_else(|| g.active(self.wx.wind_from as f32, (self.wx.wind * 1.94384) as f32)).unwrap_or((0, 0));
        let r = &g.runways[ri];
        let (thr, dir) = r.end(e);
        let heading = (dir.x as f64).atan2(dir.y as f64);
        if approach {
            let d = if r.len > 1500.0 { 7000.0 } else { 4500.0 };
            let p = thr - dir * d;
            let (lat, lon) = g.lla(p);
            let alt = g.h(thr) as f64 + (d + traffic::aim_dist(r.len)) as f64 * 3f64.to_radians().tan();
            // Approach speed (70 kt in the light single), approach flaps, gear
            // down, trimmed down a 3 degree glide.
            let (v, notch) = self.spec.approach;
            self.respawn_at(lat, lon, alt, heading, v);
            self.plane.configure(notch, true, v * 3f64.to_radians().sin());
            self.status = t!("{} {} · final approach runway {} · {:.1} nm", "{} {} · approche finale piste {} · {:.1} NM", a.code(), a.name, r.ident[e], d / 1852.0);
            // NAV 1 on the runway's ILS, as a crew would have it set up.
            let (la, lo, _) = geo::pos_to_lla(self.plane.pos);
            if let Some((st, _)) = nav::near(la.to_degrees(), lo.to_degrees(), 20_000.0, nav::Kind::Ils).into_iter().find(|(s, _)| s.name == a.ident && s.ident == r.ident[e]) {
                self.radios.tune_nav(st, self.plane.pos);
            }
        } else {
            let start = if e == 0 { r.a } else { r.b } + dir * 25.0;
            let (lat, lon) = g.lla(start);
            // On a sloping runway it stands nose up (or down) with the slope.
            let pitch = ((g.h(start + dir * 3.0) - g.h(start - dir * 3.0)) / 6.0).atan() as f64;
            self.respawn_ground(lat, lon, g.h(start) as f64, heading, pitch);
            self.status = t!("{} {} · runway {} · {:.0} m × {:.0} m", "{} {} · piste {} · {:.0} m × {:.0} m", a.code(), a.name, r.ident[e], r.len, r.width);
        }
    }


    /// Multiplayer, once a frame: connected while it is switched on (to the
    /// relay and under the name in the settings). In flight our aircraft is
    /// sent and the others near us taken in; in the menus nothing of ours
    /// goes out and the whole network's players come in for the globe.
    fn update_mp(&mut self, in_menu: bool) {
        if self.mp_mirror {
            self.mp_players = if in_menu { Vec::new() } else { vec![self.mirror_player()] };
            return;
        }
        // "auto": the server that answers fastest, kept once chosen.
        let auto = self.mp_auto();
        let target = if !self.mp_on {
            None
        } else if auto {
            match &self.mp {
                Some(m) if m.auto => Some(m.server.clone()),
                _ => self.mp_directory().best().map(|s| s.address),
            }
        } else {
            Some(self.mp_server.trim().to_string()).filter(|s| !s.is_empty())
        };
        let stale = self.mp.as_ref().is_some_and(|m| Some(&m.server) != target.as_ref() || m.auto != auto || m.name != mp_proto::clean_name(&self.mp_name));
        if target.is_none() || stale {
            self.mp = None;
            self.mp_players.clear();
            self.mp_world.clear();
            (self.mp_world_total, self.mp_world_known) = (0, false);
        }
        let Some(target) = target else { return };
        if in_menu {
            // (the server list names the servers in the menus, and tells how many fly on each)
            self.mp_directory();
        }
        let name = self.mp_name.clone();
        let m = self.mp.get_or_insert_with(|| {
            let mut m = mp::Mp::start(&target, &name);
            m.auto = auto;
            m
        });
        m.set_watching(in_menu);
        if in_menu {
            self.mp_players.clear();
            (self.mp_world, self.mp_world_total, self.mp_world_known) = m.world();
            return;
        }
        self.mp_world.clear();
        let p = &self.plane;
        let q = p.rot.as_quat();
        let byte = |x: f64| (x.clamp(0.0, 1.0) * 255.0).round() as u8;
        let axis = |x: f64| (x.clamp(-1.0, 1.0) * 127.0).round() as i8;
        let pw = p.sys.powered();
        let mut flags = 0;
        if pw {
            flags |= mp_proto::F_NAV | mp_proto::F_STROBE | mp_proto::F_BEACON;
        }
        // (The G650's landing lights follow their switches alone.)
        if pw && (if p.spec.id == 9 { p.landing_lights } else { p.gear_down }) {
            flags |= mp_proto::F_LANDING;
        }
        if p.on_ground {
            flags |= mp_proto::F_GROUND;
        }
        if self.crashed {
            flags |= mp_proto::F_CRASHED;
        }
        m.set_state(mp_proto::PlayerState {
            seq: 0,
            aircraft: self.spec.id as u8,
            pos: p.pos.to_array(),
            rot: [q.x, q.y, q.z, q.w],
            vel: p.vel.as_vec3().to_array(),
            flaps: byte(p.flaps),
            gear: byte(p.gear),
            throttle: byte(p.throttle),
            canopy: 0,
            pitch: axis(p.stick.x),
            roll: axis(p.stick.z),
            yaw: axis(p.stick.y),
            flags,
        });
        self.mp_players = m.players();
    }

    /// --mp-mirror: our own aircraft as another player, 25 m off the left wing.
    fn mirror_player(&self) -> mp::Remote {
        let p = &self.plane;
        let q = p.rot.as_quat();
        let byte = |x: f64| (x.clamp(0.0, 1.0) * 255.0).round() as u8;
        let axis = |x: f64| (x.clamp(-1.0, 1.0) * 127.0).round() as i8;
        let pos = p.pos - p.right() * 25.0;
        let flags = mp_proto::F_NAV | mp_proto::F_STROBE | mp_proto::F_BEACON | if p.on_ground { mp_proto::F_GROUND } else { 0 };
        let state = mp_proto::PlayerState { seq: 0, aircraft: self.spec.id as u8, pos: pos.to_array(), rot: [q.x, q.y, q.z, q.w], vel: p.vel.as_vec3().to_array(), flaps: byte(p.flaps), gear: byte(p.gear), throttle: byte(p.throttle), canopy: 0, pitch: axis(p.stick.x), roll: axis(p.stick.z), yaw: axis(p.stick.y), flags };
        mp::Remote { id: 999, name: "Mirror".into(), aircraft: state.aircraft, pos, rot: p.rot, vel: p.vel, gear: p.gear as f32, flags, state }
    }

    /// Whether `spec`'s model is ready to draw (it is flown, or waits in the
    /// hangar); if not, it is built in the background, as for the menu.
    fn fleet_ready(&mut self, spec: &'static aircraft::Spec) -> bool {
        if std::ptr::eq(spec, self.spec) || self.hangar.iter().any(|l| std::ptr::eq(l.spec, spec)) {
            return true;
        }
        match self.building.iter().position(|(s, _)| std::ptr::eq(*s, spec)) {
            Some(i) => match self.building[i].1.try_recv() {
                Ok(m) => {
                    self.building.swap_remove(i);
                    let l = Loaded::upload(spec, m, &mut self.r);
                    self.hangar.push(l);
                    true
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => false,
                Err(_) => {
                    self.building.swap_remove(i);
                    false
                }
            },
            None => {
                let (tx, rx) = std::sync::mpsc::channel();
                std::thread::spawn(move || {
                    let _ = tx.send(Loaded::make(spec));
                });
                self.building.push((spec, rx));
                false
            }
        }
    }

    /// The other players near enough to be worth it, drawn with their
    /// aircraft's own model: a stand-in `Plane` takes their controls, flaps,
    /// gear and throttle, and poses the pieces as our own aircraft's are.
    /// Returns which players were drawn so (the rest stay traffic models).
    fn draw_mp_planes(&mut self, cam: DVec3, dt: f64, objs: &mut Vec<DrawObj>) -> Vec<bool> {
        const FULL_DIST: f64 = 4000.0;
        let players = std::mem::take(&mut self.mp_players);
        let mut full = vec![false; players.len()];
        self.mp_puppets.retain(|id, _| players.iter().any(|p| p.id == *id));
        for (k, rp) in players.iter().enumerate() {
            if (rp.pos - cam).length() > FULL_DIST || rp.flags & mp_proto::F_CRASHED != 0 {
                continue;
            }
            let Some(spec) = aircraft::BUILTIN.iter().copied().find(|s| s.id == rp.aircraft as u32) else { continue };
            if spec.rocket().is_some() || !self.fleet_ready(spec) {
                continue;
            }
            let pup = self.mp_puppets.entry(rp.id).or_insert_with(|| Plane::spawn(spec, 0.0, 0.0, 1000.0, 0.0, 50.0));
            if !std::ptr::eq(pup.spec, spec) {
                *pup = Plane::spawn(spec, 0.0, 0.0, 1000.0, 0.0, 50.0);
            }
            let st = &rp.state;
            (pup.pos, pup.rot, pup.vel) = (rp.pos, rp.rot, rp.vel);
            pup.flaps = st.flaps as f64 / 255.0;
            pup.gear = st.gear as f64 / 255.0;
            pup.gear_down = st.gear > 127;
            pup.throttle = st.throttle as f64 / 255.0;
            pup.stick = DVec3::new(st.pitch as f64, st.yaw as f64, st.roll as f64) / 127.0;
            pup.on_ground = st.flags & mp_proto::F_GROUND != 0;
            // The propeller: idling on the ground, faster with the throttle.
            pup.prop_rate = 75.0 + 200.0 * pup.throttle;
            pup.prop = (pup.prop + pup.prop_rate * dt).rem_euclid(std::f64::consts::TAU * 1000.0);
            pup.t += dt;
            let (model, mesh, cut) = if std::ptr::eq(spec, self.spec) {
                (&self.model, &self.piece_mesh, &self.piece_cut)
            } else {
                let l = self.hangar.iter().find(|l| std::ptr::eq(l.spec, spec)).unwrap();
                (&l.model, &l.piece_mesh, &l.piece_cut)
            };
            // (Room for all of it, or it stays a traffic model: never half an aircraft.)
            if objs.len() + model.pieces.len() + 48 > render::MAX_OBJS as usize {
                continue;
            }
            let pose = pup.pose(&model.pieces);
            let base = Mat4::from_rotation_translation(rp.rot.as_quat(), (rp.pos - cam).as_vec3());
            let blur = ((pup.prop_rate - 30.0) / 25.0).clamp(0.0, 1.0) as f32;
            // Its lights (plane.wgsl reads them from the instruments' last row).
            let t = self.time + rp.id as f64 * 0.37;
            let strobe = { let s = t.rem_euclid(1.3); if s < 0.045 || (s > 0.16 && s < 0.2) { 1.0 } else { 0.0 } };
            let beacon = (1.0 - ((t * 1.1).rem_euclid(1.0) - 0.15).abs() * 7.0).max(0.0) as f32;
            let on = |f: u8| (st.flags & f != 0) as u8 as f32;
            let mut params = [[0.0f32; 4]; 6];
            params[5] = [1000.0 * on(mp_proto::F_NAV), strobe * on(mp_proto::F_STROBE), beacon * on(mp_proto::F_BEACON), on(mp_proto::F_LANDING)];
            for (i, p) in model.pieces.iter().enumerate() {
                let hide = !model.shown(p, blur, false, &|x| pup.input(x));
                if hide {
                    continue;
                }
                objs.push(DrawObj { mesh: mesh[i], model: base * pose[i], tint: [blur, 0.0, 0.0, 0.0], glow: [0.0, spec.id as f32, 0.0, 0.0], params, cast: 0b11, glass: p.glass, cut: cut[i], ..DrawObj::DEFAULT });
            }
            full[k] = true;
        }
        self.mp_players = players;
        full
    }

    fn mp_auto(&self) -> bool {
        self.mp_server.trim().is_empty() || self.mp_server.trim().eq_ignore_ascii_case("auto")
    }
    /// The network's server list (read in the background from the first call on).
    fn mp_directory(&mut self) -> &mp::Directory {
        let (src, url) = (self.src.clone(), self.mp_list.clone());
        let cache = dirs::config_dir().map(|d| d.join("fablegame")).inspect(|d| {
            let _ = std::fs::create_dir_all(d);
        }).map(|d| d.join("mp_servers.json"));
        self.mp_dir.get_or_insert_with(|| mp::Directory::start(src, url, cache))
    }
    /// A server's name in the list, by its address (else the address itself).
    fn mp_server_name(&self, address: &str) -> String {
        let a = mp::with_port(address);
        self.mp_dir.as_ref().and_then(|d| d.servers().into_iter().find(|s| s.address == a).map(|s| s.name)).unwrap_or_else(|| address.to_string())
    }

    /// The cockpit view's ray through (x, y) window pixels, in the aircraft's
    /// body frame (from the eye, unit length), with the pose and the hotspots
    /// where they are now: in the cockpit view, flying, nothing open.
    fn cockpit_ray(&self, x: f64, y: f64) -> Option<(Vec3, Vec3, Vec<Mat4>, Vec<aircraft::Hotspot>)> {
        if self.mode != Mode::Fly || self.cam_mode != 1 || self.pause_open || self.crashed || self.cockpit_controls.is_empty() {
            return None;
        }
        let (vp, cam) = self.live_view?;
        let s = self.window.inner_size();
        let (w, h) = (s.width.max(1) as f64, s.height.max(1) as f64);
        let ndc = glam::Vec4::new((x / w * 2.0 - 1.0) as f32, (1.0 - y / h * 2.0) as f32, 0.5, 1.0);
        let p = vp.inverse() * ndc;
        let d = (p.truncate() / p.w).as_dvec3().normalize();
        let inv = self.plane.rot.inverse();
        let o = (inv * (cam - self.plane.model_pos())).as_vec3();
        let d = (inv * d).as_vec3();
        let pose = self.plane.pose(&self.model.pieces);
        let hs = self.cockpit_hotspots(&pose);
        Some((o, d, pose, hs))
    }
    /// The cockpit control under the mouse at (x, y) window pixels (an index
    /// into the loaded cockpit controls): in the cockpit view, flying, nothing open.
    fn hotspot_at(&self, x: f64, y: f64) -> Option<usize> {
        let (o, d, pose, hs) = self.cockpit_ray(x, y)?;
        let i = aircraft::pick(&hs, o, d)?;
        let depth = (Vec3::from(hs[i].at) - o).dot(d) - hs[i].r - 0.005;
        (!cockpit::occluded(&self.model.pieces, &pose, &self.pit_blocks, o, d, depth)).then_some(i)
    }
    /// Looking about the cockpit with the pad: the control the middle of the
    /// view is on, if any, and how near the nearest one in sight is (1 on
    /// it, falling to 0 at `AIM_NEAR` off it). None: not aiming with the pad.
    fn pad_aim_at(&self) -> Option<(Option<usize>, f32)> {
        /// how far off a control (radians, past its edge) the aim dot shows
        const AIM_NEAR: f32 = 0.14;
        if !self.pad_aim || self.info_open {
            return None;
        }
        let s = self.window.inner_size();
        let (o, d, pose, hs) = self.cockpit_ray(s.width as f64 * 0.5, s.height as f64 * 0.5)?;
        let on = aircraft::pick(&hs, o, d).filter(|&i| {
            let depth = (Vec3::from(hs[i].at) - o).dot(d) - hs[i].r - 0.005;
            !cockpit::occluded(&self.model.pieces, &pose, &self.pit_blocks, o, d, depth)
        });
        if on.is_some() {
            return Some((on, 1.0));
        }
        // The angle from the middle of the view to each control's edge: the
        // nearest few checked for being in sight (not behind the panel).
        let mut near: Vec<(f32, usize)> = hs.iter().enumerate().filter_map(|(i, h)| {
            let c = Vec3::from(h.at) - o;
            let l = c.length();
            (l > h.r && c.dot(d) > 0.0).then(|| (d.angle_between(c) - (h.r / l).asin(), i))
        }).filter(|&(a, _)| a < AIM_NEAR).collect();
        near.sort_by(|a, b| a.0.total_cmp(&b.0));
        let seen = near.iter().take(4).find(|&&(_, i)| {
            let c = Vec3::from(hs[i].at) - o;
            let l = c.length();
            !cockpit::occluded(&self.model.pieces, &pose, &self.pit_blocks, o, c / l, l - hs[i].r - 0.005)
        });
        Some((None, seen.map_or(0.0, |&(a, _)| 1.0 - (a / AIM_NEAR).clamp(0.0, 1.0))))
    }
    fn hotspot_under_mouse(&self) -> Option<usize> {
        if self.drag || self.lean_drag.is_some() {
            return None;
        }
        self.cursor.and_then(|(x, y)| self.hotspot_at(x, y))
    }
    /// The key an action stands for (its default: what `key` understands).
    fn action_key(id: &str) -> Option<KeyCode> {
        bind::ACTIONS.iter().find(|a| a.id == id).map(|a| a.key)
    }
    /// The left button going down (or up) on a cockpit hotspot: a click does
    /// what the action's key does; a hotspot with a `hold` action waits to
    /// see whether it is a click or held (see `mouse_holds`).
    fn hot_click(&mut self, down: bool) {
        if down {
            let Some(i) = self.hotspot_under_mouse() else { return };
            let h = self.cockpit_controls[i].0;
            if h.hold.is_none() {
                if let Some(k) = h.click.and_then(Self::action_key) {
                    self.key(k);
                } else if let Some(id) = h.click { self.cockpit_action(id); }
            }
            self.hot_press = Some((i, self.real, false));
        } else if let Some((i, _, held)) = self.hot_press.take() {
            let Some(h) = self.cockpit_controls.get(i).map(|e| e.0) else { return };
            if held {
                if let Some(k) = h.hold.and_then(Self::action_key) {
                    self.hot_keys.remove(&k);
                }
            } else if h.hold.is_some() {
                if let Some(k) = h.click.and_then(Self::action_key) {
                    self.key(k);
                }
            }
        }
    }
    /// The mouse wheel over a cockpit hotspot: each notch presses its "up"
    /// or "down" action once and holds it a moment. False: no hotspot there.
    fn hot_wheel(&mut self, d: f64) -> bool {
        let Some(i) = self.hotspot_under_mouse() else { return false };
        let Some((up, down)) = self.cockpit_controls[i].0.wheel else { return false };
        let steps = self.wheel_acc.steps(i, d);
        for _ in 0..steps.unsigned_abs() {
            let id = if steps > 0 { up } else { down };
            if let Some(k) = Self::action_key(id) {
                if let Some(opposite) = Self::action_key(if steps > 0 { down } else { up }) {
                    self.hot_keys.remove(&opposite);
                    self.wheel_hold.retain(|(q, _)| *q != opposite);
                }
                self.key(k);
                self.hot_keys.insert(k);
                let until = self.real + 0.12;
                match self.wheel_hold.iter_mut().find(|(q, _)| *q == k) {
                    Some(e) => e.1 = e.1.max(self.real) + 0.12,
                    None => self.wheel_hold.push((k, until)),
                }
            } else { self.cockpit_action(id); }
        }
        true
    }
    /// What the mouse holds: a hotspot's `hold` action once the button has
    /// been down a moment, and the keys the wheel holds until their time is up.
    fn mouse_holds(&mut self) {
        while let Some(&e) = self.script_clicks.first() {
            if self.real < e.t {
                break;
            }
            self.script_clicks.remove(0);
            if let Some(p) = e.at {
                (self.cursor, self.script_cursor) = (Some(p), Some(p));
            }
            if e.ev >= 50.0 {
                self.hot_wheel(e.ev - 100.0);
            } else {
                self.hot_click(e.ev > 0.5);
            }
        }
        if let Some((i, t0, false)) = self.hot_press {
            if let Some(k) = self.cockpit_controls.get(i).and_then(|e| e.0.hold).and_then(Self::action_key) {
                if self.real - t0 > 0.35 {
                    self.key(k);
                    self.hot_keys.insert(k);
                    self.hot_press = Some((i, t0, true));
                }
            }
        }
        let now = self.real;
        let mut done = Vec::new();
        self.wheel_hold.retain(|&(k, until)| {
            if until > now {
                return true;
            }
            done.push(k);
            false
        });
        for k in done {
            self.hot_keys.remove(&k);
        }
    }
    /// The state a cockpit control is in, for the label beside the mouse.
    fn hot_state(&self, id: &str) -> Option<String> {
        let p = &self.plane;
        let on_off = |b: bool| if b { "ON" } else { "OFF" };
        Some(match id {
            "master" => on_off(p.sys.elec.master).into(),
            "ignition" if self.spec.is_jet() => format!("{}{}", if p.ignition { "RUN" } else { "CUTOFF" }, if p.starter_on || p.cranking { " · START" } else { "" }),
            "ignition" => format!("{}{}", on_off(p.ignition), if p.starter_on || p.cranking { " · START" } else { "" }),
            "fuel_control_l" | "fuel_control_r" | "battery_l" | "battery_r" => {
                let fuel = id.starts_with("fuel");
                let on = p.pair_on(!fuel as usize, id.ends_with("_r") as usize);
                if fuel {
                    format!("{}{}", if on { "RUN" } else { "CUTOFF" }, if p.ignition && (p.starter_on || p.cranking) { " · START" } else { "" })
                } else { on_off(on).into() }
            }
            "starter" => if p.starter_on || p.cranking { "START" } else if p.running { "RUNNING" } else { "OFF" }.into(),
            "speedbrake" if self.spec.chute.is_none() => if p.speedbrake > 0.0 { t!("extended", "sortis") } else { t!("retracted", "rentrés") }.into(),
            "ap" => on_off(self.ap.on).into(),
            "ap_hdg" | "ap_hdg_right" => format!("HDG {:03.0}°", self.ap.hdg.rem_euclid(360.0)),
            "ap_alt" | "ap_alt_up" => format!("ALT {:.0} ft", self.ap.alt * 3.28084),
            "ap_nav" | "ap_apr" => {
                let m = if id == "ap_nav" { autopilot::Lat::Nav } else { autopilot::Lat::Apr };
                if !self.ap.on || self.ap.lat != m { "OFF".into() } else if self.ap.nav_cap { "ON".into() } else { t!("armed", "armé").into() }
            }
            "ap_vs" | "ap_vs_up" => format!("VS {:+.0} ft/min{}", self.ap.vs * 196.85, if self.ap.on && self.ap.vert == autopilot::Vert::Vs { "" } else { " (off)" }),
            "ap_at" | "ap_spd_up" => format!("A/T {} · {:.0} kt", on_off(self.ap.at), self.ap.spd * 1.943_84),
            "parking_brake" => on_off(p.parking_brake).into(),
            "mixture_rich" => format!("{:.0} %", p.mixture * 100.0),
            "throttle_up" => format!("{:.0} %", p.throttle * 100.0),
            "flaps_up" if self.spec.lever.is_some() => self.spec.lever.as_ref()?.notch(p.flap_set).into(),
            "speedbrake" if self.spec.chute.is_some() => if p.chute_spent { "JETTISONED" } else if p.speedbrake > 0.0 { "DEPLOYED" } else { "PACKED" }.into(),
            "flaps_up" => format!("{:.0}°", self.spec.flaps.get(p.flap_set as usize).copied().unwrap_or(0.0)),
            "trim_down" => format!("{:+.1}°", p.trim.to_degrees()),
            "obs_right" => format!("OBS {:03.0}°", self.radios.obs),
            "gear" => if p.gear_down { t!("down", "sorti") } else { t!("up", "rentré") }.into(),
            "prop_up" => format!("{:.0} RPM", 1800.0 + 900.0 * p.propeller),
            "cowl_open" => format!("{:.0} %", p.cowl_flaps * 100.0),
            "landing_lights" => on_off(p.landing_lights).into(),
            "fuel_valve" | "fuel_next" => match p.fuel_selector {
                plane::FuelSelector::Center if std::ptr::eq(self.spec, &aircraft::dr400::SPEC) => "MAIN",
                plane::FuelSelector::Center => "BOTH",
                plane::FuelSelector::Right => "RIGHT",
                plane::FuelSelector::Left => "LEFT",
                plane::FuelSelector::Off => "OFF",
            }.into(),
            "nav_next" => self.radios.nav.as_ref().map(|s| format!("{} {}", s.freq_text(), s.label())).unwrap_or_else(|| "OFF".into()),
            "adf_next" => self.radios.adf.as_ref().map(|s| format!("{} {}", s.freq_text(), s.label())).unwrap_or_else(|| "OFF".into()),
            "baro_qnh" => format!("{:.0} hPa", self.radios.setting(self.wx.qnh)),
            _ => return None,
        })
    }
    /// The cockpit control under the mouse: a ring round it, what it is, how
    /// it is set, and the pointing hand.
    ///
    /// Looking about with the pad it is the middle of the view that points:
    /// a faint dot there once a control is near it, filling in on one, and
    /// beside it the pad buttons that work the control.
    fn hotspot_overlay(&self, ctx: &egui::Context) {
        use egui::{Align2, Color32, FontId};
        let screen = ctx.content_rect();
        let aim = self.pad_aim_at();
        if let Some((on, near)) = aim {
            if near > 0.0 {
                let painter = ctx.layer_painter(egui::LayerId::background());
                let c = screen.center();
                // (Faint at the edge of the range, firmer as it closes in.)
                let a = 0.35 + 0.45 * near;
                if on.is_some() {
                    painter.circle_filled(c, 5.0, Color32::from_rgba_unmultiplied(255, 210, 90, 220));
                    painter.circle_stroke(c, 5.0, egui::Stroke::new(1.0, Color32::from_black_alpha(140)));
                } else {
                    painter.circle_filled(c, 4.5, Color32::from_white_alpha((a * 255.0) as u8));
                    painter.circle_stroke(c, 4.5, egui::Stroke::new(1.0, Color32::from_black_alpha((a * 160.0) as u8)));
                }
            }
        }
        let Some(i) = (match aim { Some((on, _)) => on, None => self.hotspot_under_mouse() }) else { return };
        let Some((vp, cam)) = self.live_view else { return };
        if aim.is_none() {
            ctx.set_cursor_icon(egui::CursorIcon::PointingHand);
        }
        let h = self.cockpit_hotspots(&self.plane.pose(&self.model.pieces))[i];
        let to_screen = |b: Vec3| -> Option<egui::Pos2> {
            let w = self.plane.model_pos() + self.plane.rot * b.as_dvec3();
            let c = vp * (w - cam).as_vec3().extend(1.0);
            (c.w > 0.0).then(|| egui::pos2(screen.left() + (c.x / c.w * 0.5 + 0.5) * screen.width(), screen.top() + (0.5 - c.y / c.w * 0.5) * screen.height()))
        };
        let at = Vec3::from(h.at);
        let (Some(c), Some(e)) = (to_screen(at), to_screen(at + Vec3::Y * h.r)) else { return };
        let painter = ctx.layer_painter(egui::LayerId::background());
        let ring = Color32::from_rgba_unmultiplied(255, 210, 90, 200);
        painter.circle_stroke(c, (e - c).length().max(6.0), egui::Stroke::new(1.5, ring));
        let label = |id: &'static str| match id {
            "flaps_up" if self.spec.lever.is_some() => t!("Raise nose / visor", "Relever nez / visière"),
            "flaps_down" if self.spec.lever.is_some() => t!("Lower nose / visor", "Abaisser nez / visière"),
            "speedbrake" if self.spec.chute.is_some() => t!("Drag chute", "Parachute-frein"),
            _ => bind::ACTIONS.iter().find(|a| a.id == id).map(|a| a.label()).unwrap_or_else(|| aircraft::c172::cabin::action_label(id)),
        };
        if aim.is_some() {
            // The pad: each action with the buttons that do it (none: not on the pad).
            let mut rows: Vec<(String, String)> = Vec::new();
            let mut add = |id: &'static str, extra: &str| rows.push((padmap::spec_of(id), format!("{}{extra}", label(id))));
            if let Some(id) = h.click { add(id, ""); }
            if let Some(id) = h.hold { add(id, &if h.click.is_none() { format!(" ({})", t!("hold", "maintenir")) } else { String::new() }); }
            if let Some((up, down)) = h.wheel { add(up, ""); add(down, ""); }
            rows.dedup_by(|a, b| a.0 == b.0 && a.1 == b.1);
            if let Some(st) = h.click.or(h.wheel.map(|w| w.0)).and_then(|id| self.hot_state(id)) {
                rows.push((String::new(), st));
            }
            let p = ctx.layer_painter(egui::LayerId::background());
            let x0 = screen.center().x + 18.0;
            let y0 = screen.center().y + 16.0;
            let gw = rows.iter().map(|r| if r.0.is_empty() { 0.0 } else { padui::spec_width(&p, &r.0, 18.0) + 6.0 }).fold(0.0, f32::max);
            for (k, (spec, text)) in rows.iter().enumerate() {
                let y = y0 + 22.0 * k as f32;
                if !spec.is_empty() {
                    padui::paint_spec(&p, x0, y, spec, 18.0, Color32::WHITE);
                }
                let pos = egui::pos2(x0 + gw, y);
                p.text(pos + egui::vec2(1.0, 1.0), Align2::LEFT_CENTER, text, FontId::proportional(13.5), Color32::from_black_alpha(200));
                p.text(pos, Align2::LEFT_CENTER, text, FontId::proportional(13.5), Color32::WHITE);
            }
            return;
        }
        let mut lines: Vec<String> = Vec::new();
        if let Some(id) = h.click {
            let hold = h.hold.map(|k| t!("  (hold: {})", "  (maintenir : {})", label(k))).unwrap_or_default();
            lines.push(format!("{}{hold}", label(id)));
        }
        if h.click.is_none() {
            if let Some(id) = h.hold { lines.push(format!("{} ({})", label(id), t!("hold", "maintenir"))); }
        }
        if let Some((up, down)) = h.wheel {
            lines.push(format!("{} {} / {}", t!("Wheel:", "Molette :"), label(up), label(down)));
        }
        if let Some(s) = h.click.or(h.wheel.map(|w| w.0)).and_then(|id| self.hot_state(id)) {
            lines.push(s);
        }
        let mouse = self.cursor.map(|(x, y)| egui::pos2(x as f32 / ctx.pixels_per_point(), y as f32 / ctx.pixels_per_point())).unwrap_or(c);
        // (On the right of the screen the label goes left of the mouse, so it stays on it.)
        let left = mouse.x > screen.center().x;
        let (align, dx) = if left { (Align2::RIGHT_TOP, -16.0) } else { (Align2::LEFT_TOP, 16.0) };
        for (k, s) in lines.iter().enumerate() {
            let pos = mouse + egui::vec2(dx, 6.0 + 16.0 * k as f32);
            painter.text(pos + egui::vec2(1.0, 1.0), align, s, FontId::proportional(13.5), Color32::from_black_alpha(200));
            painter.text(pos, align, s, FontId::proportional(13.5), Color32::WHITE);
        }
    }

    fn controls(&mut self, dt: f64) -> Controls {
        self.mouse_holds();
        let k = |c: KeyCode| self.keys.contains(&c) || self.hot_keys.contains(&c);
        let ax = |pos: bool, neg: bool| (pos as i32 - neg as i32) as f64;
        let mut c = Controls {
            pitch: ax(k(KeyCode::KeyS) || k(KeyCode::ArrowDown), k(KeyCode::KeyW) || k(KeyCode::ArrowUp)),
            roll: ax(k(KeyCode::KeyD) || k(KeyCode::ArrowRight), k(KeyCode::KeyA) || k(KeyCode::ArrowLeft)),
            yaw: ax(k(KeyCode::KeyE), k(KeyCode::KeyQ)),
            throttle: ax(k(KeyCode::ShiftLeft) || k(KeyCode::ShiftRight), k(KeyCode::ControlLeft) || k(KeyCode::ControlRight)),
            brake: k(KeyCode::KeyB),
            brake_left: if self.hot_press.is_some_and(|(i, _, _)| self.cockpit_controls[i].0.hold == Some("brake_left")) { 1.0 } else { 0.0 },
            brake_right: if self.hot_press.is_some_and(|(i, _, _)| self.cockpit_controls[i].0.hold == Some("brake_right")) { 1.0 } else { 0.0 },
            // (The game's own boost is arcade mode's; real afterburners always work.)
            boost: k(KeyCode::Space) && (self.arcade || self.spec.real_boost()),
            reverse: k(KeyCode::KeyZ),
            throttle_set: None,
            starter: k(KeyCode::KeyO),
        };
        let mix = ax(k(KeyCode::PageUp), k(KeyCode::PageDown));
        if mix != 0.0 && !self.spec.is_jet() {
            self.plane.mixture = (self.plane.mixture + mix * 0.5 * dt).clamp(0.0, 1.0);
        }
        let obs = ax(k(KeyCode::BracketRight), k(KeyCode::BracketLeft));
        if obs != 0.0 {
            self.radios.obs = (self.radios.obs + obs * 20.0 * dt).rem_euclid(360.0);
        }
        let trim = ax(k(KeyCode::Numpad1), k(KeyCode::Numpad7));
        if trim != 0.0 && self.mode == Mode::Fly && !self.pause_open && !self.crashed {
            self.plane.trim = (self.plane.trim + trim * 0.035 * dt).clamp(-0.05, 0.24);
        }
        self.gamepad(dt, &mut c);
        self.joystick(&mut c);
        self.autopilot(dt, &mut c);
        c.brake |= self.plane.parking_brake;
        if let Some(t) = self.crash_at {
            if self.real > t && !self.crashed {
                c.pitch = -1.0;
                c.throttle = 1.0;
            }
        }
        if self.boost_at.is_some_and(|t| self.real > t) {
            c.throttle = 1.0;
            c.boost = true;
        }
        if !self.plane.hold && self.engines.is_some() {
            self.plane.lit = self.engines.take().unwrap_or(0);
        }
        if self.stage_at.is_some_and(|t| self.real > t) {
            self.stage_at = None;
            self.plane.stage_sep();
        }
        c
    }

    /// An Xbox-style gamepad. Its flying layout can be moved about in
    /// Settings › Controls › Gamepad (padmap.rs); it starts as Microsoft
    /// Flight Simulator (2020) sets one up by default:
    ///
    /// | input | does |
    /// |---|---|
    /// | left stick | pitch and roll |
    /// | LT / RT | rudder left / right |
    /// | A / B (hold) | throttle up / down |
    /// | X (hold) | wheel brakes |
    /// | Y + B | parking brake on / off |
    /// | Y + d-pad up / down | elevator trim nose down / up |
    /// | LB / RB | flaps up / down a notch |
    /// | left stick click | landing gear |
    /// | right stick | look around |
    /// | right stick click | reset the view |
    /// | d-pad | quick views: ahead, panel, left, right |
    /// | View | cockpit / chase camera |
    /// | Menu | pause menu |
    /// | Y + View | flight panel |
    ///
    /// Y with another button reaches what the layout above leaves out:
    ///
    /// | input | does |
    /// |---|---|
    /// | Y + A (hold) | afterburner / boost |
    /// | Y + X (hold) | thrust reversers |
    /// | Y + LB | speedbrake |
    /// | Y + RB | autopilot on / off (a rocket: the ship lands itself) |
    /// | Y + left stick click | start / stop the engine |
    /// | Y + right stick click | hide / show the HUD |
    /// | Y + Menu | canopy open / shut |
    /// | Y + d-pad left / right | master arm, lock a target (a rocket: which engines burn, drop the booster) |
    /// | Y alone, let go | fire at a locked target |
    ///
    /// Those are the defaults; the menu buttons below are fixed, and so is
    /// Menu (pause) and A after a crash.
    ///
    /// In the menus the d-pad (and the left stick, off the globe) moves, A
    /// presses, B steps back and the right stick scrolls; the flight panel
    /// takes the d-pad, A, B, the bumpers and the right stick while it is
    /// open. A skips the loading screen, and after a crash starts again.
    fn gamepad(&mut self, dt: f64, c: &mut Controls) {
        use gilrs::{Axis, Button, EventType};
        padmap::publish(&self.binds.pad);
        let Some(g) = self.gilrs.as_mut() else { return };
        let (mut pressed, mut released, mut relayout) = (Vec::new(), Vec::new(), false);
        // A joystick capture (Settings › Controls › Buttons, Axes) takes every device but the pads.
        let stick_capture = self.binds.capture.as_ref().is_some_and(|c| !c.from_pad() && !matches!(c, bind::Capture::Key(_)));
        while let Some(ev) = g.next_event() {
            // Every device's axes and buttons go to the bindings (joysticks,
            // pedals, throttles); one with something bound is not the gamepad.
            // A pad gilrs knows the layout of is left out unless it has been
            // bound already, so binding a joystick button cannot take it by mistake.
            let (uuid, name, pad_like) = {
                let p = g.gamepad(ev.id);
                (p.uuid(), p.name().to_string(), p.mapping_source() == gilrs::MappingSource::SdlMappings)
            };
            let bound = self.binds.uses(&uuid);
            if bound || !pad_like {
                match ev.event {
                    EventType::AxisChanged(_, v, code) => self.binds.axis_event(uuid, &name, code.into_u32(), v),
                    EventType::ButtonPressed(_, code) => self.binds.button_event(uuid, &name, code.into_u32(), true),
                    EventType::ButtonReleased(_, code) => self.binds.button_event(uuid, &name, code.into_u32(), false),
                    _ => {}
                }
            }
            if bound || (stick_capture && !pad_like) {
                continue;
            }
            match ev.event {
                EventType::Connected => relayout = true,
                EventType::ButtonPressed(b, _) => {
                    self.pad_id = Some(ev.id);
                    pressed.push(b);
                }
                EventType::ButtonReleased(b, _) => released.push(b),
                EventType::AxisChanged(..) | EventType::ButtonChanged(..) => {
                    if self.pad_id.is_none() {
                        self.pad_id = Some(ev.id);
                    }
                }
                EventType::Disconnected if self.pad_id == Some(ev.id) => self.pad_id = None,
                _ => {}
            }
        }
        // A device a preset knows by name, seen for the first time: put it on.
        let seen: Vec<(bind::Uuid, String)> = g.gamepads().map(|(_, p)| (p.uuid(), p.name().to_string())).collect();
        for (uuid, name) in seen {
            if let Some(p) = self.binds.auto_preset(uuid, &name) {
                self.status = t!("{}: preset \"{}\" applied (see Bindings)", "{} : préréglage « {} » appliqué (voir Assignations)", name, p);
                self.status_until = self.real + 8.0;
            }
        }
        // A pad plugged in that gilrs has no layout for: open it again with one.
        let unfixed: Vec<[u8; 16]> = g.gamepads().filter(|(_, p)| needs_xbox_layout(p) && !self.pad_fixed.contains(&p.uuid())).map(|(_, p)| p.uuid()).collect();
        if relayout && !unfixed.is_empty() {
            self.pad_fixed.extend(unfixed);
            self.gilrs = open_gamepads();
            self.pad_id = None;
            return;
        }
        let binds = &self.binds;
        let free = |p: &gilrs::Gamepad| !binds.uses(&p.uuid());
        let id = self.pad_id.filter(|id| g.connected_gamepad(*id).is_some_and(|p| free(&p))).or_else(|| g.gamepads().find(|(_, p)| free(p)).map(|(id, _)| id));
        let Some(pad) = id.and_then(|id| g.connected_gamepad(id)) else {
            self.pad = Controls::default();
            padui::set_active(false);
            self.pad_aim = false;
            self.pad_rep = None;
            (self.pad_y, self.pad_y_combo) = (false, false);
            self.pad_release_all();
            if self.binds.capture.as_ref().is_some_and(|c| c.from_pad()) {
                self.binds.capture = None;
            }
            return;
        };
        padui::set_active(true);
        // Binding from the pad (Settings › Controls › Gamepad): its buttons
        // and sticks go to that alone.
        if let Some(cap) = self.binds.capture.clone().filter(|c| c.from_pad()) {
            use padmap::{Caught, PadBtn};
            let trig = |b: Button, z: Axis| match (pad.button_data(b), pad.axis_data(z)) {
                (Some(d), _) => d.value(),
                (None, Some(a)) => ((a.value() + 1.0) * 0.5).clamp(0.0, 1.0),
                _ => 0.0,
            };
            let sticks = [pad.value(Axis::LeftStickX), pad.value(Axis::LeftStickY), pad.value(Axis::RightStickX), pad.value(Axis::RightStickY), trig(Button::RightTrigger2, Axis::RightZ) - trig(Button::LeftTrigger2, Axis::LeftZ)];
            let y_held = pad.is_pressed(Button::North);
            let now = self.real;
            let c = &mut self.binds.pad_catch;
            let mut got = Caught::Wait;
            let evs = pressed.iter().map(|b| (*b, true)).chain(released.iter().map(|b| (*b, false)));
            for (b, down) in evs {
                if let Some(pb) = PadBtn::from_gilrs(b) {
                    match c.button(pb, down, y_held || (down && pb == PadBtn::Y), now) {
                        Caught::Wait => {}
                        r => got = r,
                    }
                }
            }
            if got == Caught::Wait {
                got = c.tick(now);
            }
            if let (Caught::Wait, bind::Capture::PadAxis(_)) = (&got, &cap) {
                got = c.sticks(sticks);
            }
            let lost = match (got, cap) {
                (Caught::Wait, _) => None,
                (Caught::Cancel, _) => Some(Vec::new()),
                (Caught::Refused(_), _) => {
                    self.pad_note = (t!("Menu pauses: it cannot be given anything else.", "Menu met en pause : on ne peut rien lui donner d'autre.").into(), self.real + 6.0);
                    None
                }
                (Caught::Button(ch), bind::Capture::Pad(id)) => Some(self.binds.pad.set(id, ch)),
                (Caught::Axis(src), bind::Capture::PadAxis(a)) => Some(self.binds.pad.set_axis(a, Some(src))),
                // (a stick moved while waiting for a button, or the other way round)
                _ => None,
            };
            if let Some(lost) = lost {
                self.binds.capture = None;
                if !lost.is_empty() {
                    self.pad_note = (t!("No longer bound: {}", "Plus assigné : {}", lost.join(", ")), self.real + 8.0);
                }
            }
            self.pad = Controls::default();
            self.pad_rep = None;
            (self.pad_y, self.pad_y_combo) = (false, false);
            padui::set_scroll(0.0);
            return;
        }
        // Where the pad's buttons go: the globe, the pause menu, the flight
        // panel (which then has the d-pad, A, B and the bumpers), or the aircraft.
        let nav = if self.mode == Mode::Menu {
            Nav::Globe
        } else if self.pause_open {
            Nav::Pause
        } else if self.info_open || self.radial.open {
            Nav::Panel
        } else {
            Nav::Off
        };
        // A small dead zone, the rest of the travel rescaled so there is no step out of it.
        let dz = |v: f32| {
            let v = v as f64;
            let d = 0.12;
            if v.abs() < d { 0.0 } else { v.signum() * (v.abs() - d) / (1.0 - d) }
        };
        // The analogue triggers; on a pad left without a layout they come through as
        // bare Z axes, -1 at rest to +1 squeezed.
        let trig = |b: Button, z: Axis| match (pad.button_data(b), pad.axis_data(z)) {
            (Some(d), _) => d.value() as f64,
            (None, Some(a)) => ((a.value() as f64 + 1.0) * 0.5).clamp(0.0, 1.0),
            _ => 0.0,
        };
        let modifier = pad.is_pressed(Button::North);
        let flying = self.mode == Mode::Fly && !self.pause_open;
        // The globe itself is only on the start screen's Fly page; the main
        // menu, aircraft and settings pages cover it.
        let globe = nav == Nav::Globe && self.menu.screen == menu::Screen::Fly;
        // The flying axes, wherever Settings › Controls › Gamepad has put them.
        let map = &self.binds.pad;
        let stick = |s: padmap::Stick| match s {
            padmap::Stick::LX => dz(pad.value(Axis::LeftStickX)),
            padmap::Stick::LY => dz(pad.value(Axis::LeftStickY)),
            padmap::Stick::RX => dz(pad.value(Axis::RightStickX)),
            padmap::Stick::RY => dz(pad.value(Axis::RightStickY)),
            padmap::Stick::Trig => (trig(Button::RightTrigger2, Axis::RightZ) - trig(Button::LeftTrigger2, Axis::LeftZ)).clamp(-1.0, 1.0),
        };
        let axis = |a: padmap::PadAxis| map.axis(a).map(|s| stick(s.stick) * s.sign as f64).unwrap_or(0.0);
        self.pad = Controls { pitch: axis(padmap::PadAxis::Pitch), roll: axis(padmap::PadAxis::Roll), yaw: axis(padmap::PadAxis::Rudder), throttle: axis(padmap::PadAxis::Throttle), ..Controls::default() };
        let (look_x, look_y) = (axis(padmap::PadAxis::LookX), axis(padmap::PadAxis::LookY));
        // The buttons held now, for the flying actions held with them.
        let down: Vec<padmap::PadBtn> = padmap::PadBtn::ALL.into_iter().filter(|b| pad.is_pressed(b.gilrs())).collect();
        let ry = dz(pad.value(Axis::RightStickY));

        // Moving about the menus: the d-pad (and the left stick too, except
        // where it turns the globe or flies), repeating while held.
        let dirs = [(Button::DPadUp, egui::Key::ArrowUp), (Button::DPadDown, egui::Key::ArrowDown), (Button::DPadLeft, egui::Key::ArrowLeft), (Button::DPadRight, egui::Key::ArrowRight)];
        let mut held = dirs.iter().find(|(b, _)| pad.is_pressed(*b)).map(|d| d.1);
        let (lx, ly) = (pad.value(Axis::LeftStickX), pad.value(Axis::LeftStickY));
        let stick_nav = nav == Nav::Pause || (nav == Nav::Globe && !globe);
        if held.is_none() && stick_nav && lx.abs().max(ly.abs()) > 0.6 {
            held = Some(if ly.abs() > lx.abs() { if ly > 0.0 { egui::Key::ArrowUp } else { egui::Key::ArrowDown } } else if lx > 0.0 { egui::Key::ArrowRight } else { egui::Key::ArrowLeft });
        }
        let tap = pressed.iter().find_map(|b| dirs.iter().find(|d| d.0 == *b).map(|d| d.1));
        let zoom = trig(Button::RightTrigger2, Axis::RightZ) - trig(Button::LeftTrigger2, Axis::LeftZ);
        let mut nav_keys = Vec::new();
        if nav == Nav::Off {
            self.pad_rep = None;
        } else {
            match (held, self.pad_rep) {
                (Some(k), Some((pk, t))) if pk == k => {
                    if self.real >= t {
                        nav_keys.push(k);
                        self.pad_rep = Some((k, (t + 0.085).max(self.real)));
                    }
                }
                (Some(k), _) => {
                    nav_keys.push(k);
                    self.pad_rep = Some((k, self.real + 0.4));
                }
                (None, _) => {
                    self.pad_rep = None;
                    nav_keys.extend(tap);
                }
            }
        }
        for k in nav_keys {
            self.pad_key(k);
        }
        // The right stick scrolls the menus and the flight panel: squared,
        // for slow reading near the centre, fast at full tilt.
        padui::set_scroll(if nav == Nav::Off { 0.0 } else { (ry * ry.abs() * 1600.0 * dt) as f32 });
        if globe {
            // Left stick turns the globe (squared, for a fine touch), the triggers zoom.
            let h = self.window.inner_size().height.max(1) as f64;
            let sq = |v: f64| v * v.abs();
            let (sx, sy) = (sq(dz(lx)), sq(dz(ly)));
            if sx != 0.0 || sy != 0.0 {
                let k = 0.8 * h * dt;
                self.menu.drag(-sx * k, sy * k, h, 60f64.to_radians());
            }
            if zoom.abs() > 0.05 {
                self.menu.zoom(zoom * 5.0 * dt);
            }
        }

        c.pitch = (c.pitch + self.pad.pitch).clamp(-1.0, 1.0);
        c.roll = (c.roll + self.pad.roll).clamp(-1.0, 1.0);
        c.yaw = (c.yaw + self.pad.yaw).clamp(-1.0, 1.0);
        c.throttle = (c.throttle + self.pad.throttle).clamp(-1.0, 1.0);

        // (With the flight panel open the right stick scrolls it instead.)
        if flying && nav != Nav::Panel {
            // Right stick: look around, as fast as a quick head turn at full deflection.
            // Pushed right the view swings right, in the cockpit and outside.
            let rate = 2.2 * self.look_sens * dt;
            let side = if self.cam_mode == 1 { 1.0 } else { -1.0 };
            self.cam_yaw += look_x * rate * side;
            self.cam_pitch = (self.cam_pitch - look_y * rate * 0.8).clamp(-1.2, 1.4);
        }
        // Looking or flying with the pad: the middle of the view aims at the cockpit's controls.
        if look_x != 0.0 || look_y != 0.0 || self.pad.pitch != 0.0 || self.pad.roll != 0.0 || !pressed.is_empty() {
            self.pad_aim = true;
        }

        // Y held before this frame, or pressed in it: what the buttons pressed now mean.
        let y_now = self.pad_y || pressed.contains(&Button::North);
        for &b in &pressed {
            self.pad_button(nav, b, modifier);
        }
        self.pad_fly_buttons(nav, &down, &pressed, y_now);
        // Y on its own, let go without having been used with another button.
        if pressed.contains(&Button::North) {
            self.pad_y = true;
            self.pad_y_combo = false;
        }
        if self.pad_y && pressed.iter().any(|b| *b != Button::North) {
            self.pad_y_combo = true;
        }
        if released.contains(&Button::North) {
            if self.pad_y && !self.pad_y_combo && nav == Nav::Off && self.mode == Mode::Fly && !self.pause_open {
                if let Some(id) = self.binds.pad.action_at(padmap::Y_ALONE) {
                    // A tap of Y fires only at a locked target: no complaint otherwise.
                    let quiet = id == "fire_missile" && (self.crashed || !self.missiles_ok() || self.weapons.lock < 1.0);
                    if !quiet {
                        self.pad_do(id, false);
                    }
                }
            }
            self.pad_y = false;
            self.pad_y_combo = false;
        }
    }

    /// The pad's flying buttons, as Settings › Controls › Gamepad has them:
    /// each action pressed with its button and held while it is (its key
    /// held for the controls that read keys, like the throttle and brakes).
    /// `down`: the buttons held now; `pressed`: those pressed this frame
    /// (a tap quicker than a frame still counts); `y`: Y held for them.
    fn pad_fly_buttons(&mut self, nav: Nav, down: &[padmap::PadBtn], pressed: &[gilrs::Button], y: bool) {
        use padmap::{Chord, PadBtn};
        let mut now: Vec<&'static str> = Vec::new();
        let mut taps: Vec<&'static str> = Vec::new();
        if matches!(nav, Nav::Off | Nav::Panel) && self.mode == Mode::Fly && !self.pause_open {
            let map = &self.binds.pad;
            // The flight panel keeps A, B, the d-pad and the bumpers while it is open.
            let panel = |c: Chord| nav == Nav::Panel && !c.y && matches!(c.btn, PadBtn::A | PadBtn::B | PadBtn::LB | PadBtn::RB | PadBtn::Up | PadBtn::Down | PadBtn::Left | PadBtn::Right);
            // While Y is held a button with a Y binding does that, not its own.
            let live = |c: Chord, held: bool| held && c != padmap::Y_ALONE && !panel(c) && if c.y { y } else { !(y && map.has_y(c.btn)) };
            for (id, c) in map.bound() {
                if live(c, down.contains(&c.btn)) {
                    now.push(id);
                } else if live(c, pressed.contains(&c.btn.gilrs())) {
                    taps.push(id);
                }
            }
            // A after a crash flies again (pad_button), and skips the loading screen.
            let skip_a = self.crashed || self.loading.as_ref().is_some_and(|l| l.done.is_none());
            if skip_a {
                let a = map.action_at(padmap::Chord { btn: PadBtn::A, y: false });
                now.retain(|id| Some(*id) != a);
                taps.retain(|id| Some(*id) != a);
            }
        }
        let gone: Vec<_> = self.pad_held.iter().filter(|(id, _)| !now.contains(id)).copied().collect();
        for (_, k) in gone {
            if let Some(k) = k {
                self.keys.remove(&k);
            }
        }
        self.pad_held.retain(|(id, _)| now.contains(id));
        for id in now {
            match self.pad_held.iter().find(|(h, _)| *h == id) {
                // (put back if the window losing focus let go of the keys)
                Some((_, Some(k))) => {
                    self.keys.insert(*k);
                }
                Some(_) => {}
                None => {
                    let k = self.pad_do(id, true);
                    self.pad_held.push((id, k));
                }
            }
        }
        for id in taps {
            self.pad_do(id, false);
        }
    }

    /// A pad action pressed: what its key does (returned, to be let go of
    /// later when `held`), or one of the pad's views.
    fn pad_do(&mut self, id: &'static str, held: bool) -> Option<KeyCode> {
        use padmap::{Does, View};
        match padmap::does(id, self.spec.rocket().is_some())? {
            Does::View(v) => {
                // Quick views. Outside, left and right look at that side of the aircraft.
                let (home_y, home_p) = self.cam_home;
                let side = if self.cam_mode == 1 { 1.0 } else { -1.0 };
                (self.cam_yaw, self.cam_pitch) = match v {
                    View::Ahead => (home_y, home_p),
                    View::Down if self.cam_mode == 1 => (home_y, 0.55),
                    View::Down => (home_y + std::f64::consts::PI, home_p),
                    View::Left => (home_y - side * std::f64::consts::FRAC_PI_2, home_p.min(0.15)),
                    View::Right => (home_y + side * std::f64::consts::FRAC_PI_2, home_p.min(0.15)),
                    View::Reset => {
                        self.head.zoom = 1.0;
                        (home_y, home_p)
                    }
                };
                None
            }
            Does::Key(KeyCode::Tab) => {
                self.flight_menu_key();
                None
            }
            Does::Key(k) => {
                self.key(k);
                if held {
                    self.keys.insert(k);
                    return Some(k);
                }
                None
            }
        }
    }

    /// Lets go of every key the pad's buttons were holding.
    fn pad_release_all(&mut self) {
        for (_, k) in std::mem::take(&mut self.pad_held) {
            if let Some(k) = k {
                self.keys.remove(&k);
            }
        }
    }

    /// Bound joystick, pedal and throttle axes, and bound buttons (which do
    /// what their key does).
    fn joystick(&mut self, c: &mut Controls) {
        use bind::AxisAct as A;
        let b = &self.binds;
        if let Some(v) = b.axis(A::Pitch) {
            c.pitch = (c.pitch + v).clamp(-1.0, 1.0);
        }
        if let Some(v) = b.axis(A::Roll) {
            c.roll = (c.roll + v).clamp(-1.0, 1.0);
        }
        if let Some(v) = b.axis(A::Yaw) {
            c.yaw = (c.yaw + v).clamp(-1.0, 1.0);
        }
        // A throttle lever sets the power, unless the keys are moving it.
        if let Some(t) = b.axis(A::Throttle) {
            // The copilot's setting holds until the pilot moves the throttle;
            // any key, gamepad or lever input takes the power back.
            if c.throttle != 0.0 || self.mcp_throttle.is_some_and(|_| self.throttle_axis.is_some_and(|last| (t - last).abs() > 0.02)) {
                self.mcp_throttle = None;
            }
            self.throttle_axis = Some(t);
            if let Some(t) = self.mcp_throttle {
                c.throttle_set = Some(t);
            } else if c.throttle == 0.0 {
                c.throttle_set = Some(t);
            }
        } else {
            self.throttle_axis = None;
        }
        c.brake_left = c.brake_left.max(b.axis(A::BrakeLeft).unwrap_or(0.0).clamp(0.0, 1.0));
        c.brake_right = c.brake_right.max(b.axis(A::BrakeRight).unwrap_or(0.0).clamp(0.0, 1.0));
        for (id, down) in self.binds.take_edges() {
            let Some(a) = bind::ACTIONS.iter().find(|a| a.id == id) else { continue };
            if down {
                // (Esc on the globe would quit the game.)
                if !(a.key == KeyCode::Escape && self.mode != Mode::Fly) {
                    self.key(a.key);
                }
                self.keys.insert(a.key);
            } else {
                self.keys.remove(&a.key);
            }
        }
    }

    /// The payload chosen on the start screen (kg), or the aircraft's usual.
    fn start_payload(&self) -> f64 {
        let [usual, most, _] = self.spec.payload;
        self.menu.payload.map(|f| f * most).unwrap_or(usual)
    }

    /// The pause menu's failures: each one by hand, or one now and then at random.
    fn failures_page(&mut self, ui: &mut egui::Ui) {
        use egui::RichText;
        use menu::{DIM, TEXT};
        menu::heading(ui, t!("FAILURES", "PANNES"));
        ui.label(RichText::new(t!("They act as the real ones do: on the engine, the electrics, the gyros, the airspeed and altimeter, the fuel.", "Elles agissent comme les vraies : sur le moteur, l'électricité, les gyroscopes, l'anémomètre et l'altimètre, le carburant.")).font(menu::caption()).color(DIM));
        for i in 0..systems::Failures::N {
            let mut on = self.plane.sys.fail.get(i);
            if ui.checkbox(&mut on, RichText::new(systems::Failures::label(i)).color(TEXT)).changed() {
                self.plane.sys.fail.set(i, on);
            }
        }
        if ui.button(t!("Repair everything", "Tout réparer")).clicked() {
            self.plane.sys.fail = systems::Failures::default();
        }
        ui.add_space(6.0);
        menu::heading(ui, t!("AT RANDOM", "AU HASARD"));
        if ui.checkbox(&mut self.fail_random, t!("A failure now and then, without warning", "Une panne de temps en temps, sans prévenir")).changed() {
            self.fail_next = None;
        }
        ui.add_enabled_ui(self.fail_random, |ui| {
            ui.horizontal(|ui| {
                ui.label(t!("About every", "En moyenne toutes les"));
                ui.add(egui::Slider::new(&mut self.fail_mean, 5.0..=120.0).logarithmic(true).custom_formatter(|v, _| format!("{v:.0} min")));
            });
        });
    }

    /// Random failures: one at a time, at intervals drawn around the mean.
    fn random_failures(&mut self) {
        if !self.fail_random || self.mode != Mode::Fly || self.crashed || self.pause_open {
            if !self.fail_random {
                self.fail_next = None;
            }
            return;
        }
        let mean = self.fail_mean * 60.0;
        let next = *self.fail_next.get_or_insert_with(|| self.real + -(1.0 - self.rng.f() as f64).max(1e-6).ln() * mean);
        if self.real < next {
            return;
        }
        self.fail_next = None;
        let free: Vec<usize> = (0..systems::Failures::N).filter(|&i| !self.plane.sys.fail.get(i)).collect();
        if free.is_empty() {
            return;
        }
        let i = free[((self.rng.f() * free.len() as f32) as usize).min(free.len() - 1)];
        self.plane.sys.fail.set(i, true);
        self.flash.push((t!("FAILURE: {}", "PANNE : {}", systems::Failures::label(i).to_uppercase()), self.real));
    }

    /// The flight panel's radios: NAV 1 (VOR or ILS) and the ADF tuned from
    /// the stations around, the OBS, and the altimeter setting.
    /// The autopilot flies, if engaged: its stick and throttle replace the
    /// pilot's (whose small inputs add to them; a firm one disconnects it).
    fn autopilot(&mut self, dt: f64, c: &mut Controls) {
        // (Scripted presses: --ap.)
        while let Some(k) = self.ap_script.iter().position(|(t, _)| self.real >= *t) {
            let (_, m) = self.ap_script.remove(k);
            match m.as_str() {
                "ap" => self.ap_toggle(),
                "hdg" => self.ap_lat(autopilot::Lat::Hdg),
                "nav" => self.ap_lat(autopilot::Lat::Nav),
                "apr" => self.ap_lat(autopilot::Lat::Apr),
                "alt" => self.ap_vert(autopilot::Vert::Alt),
                "vs" => self.ap_vert(autopilot::Vert::Vs),
                "at" => self.ap.at = !self.ap.at,
                _ => {}
            }
        }
        if !self.ap.on || dt <= 0.0 || self.mode != Mode::Fly || self.pause_open {
            return;
        }
        if self.crashed {
            self.ap_disconnect(None);
            return;
        }
        let s = autopilot::State::of(&self.plane);
        let r = {
            let nav = self.radios.nav.as_ref().map(|st| autopilot::Nav { st, reading: nav::nav_reading(st, self.radios.obs, self.plane.pos), obs: self.radios.obs });
            self.ap.step(&s, &self.plane, nav.as_ref(), (c.pitch, c.roll), dt)
        };
        match r {
            Ok(o) => {
                c.pitch = o.pitch;
                c.roll = o.roll;
                if let Some(t) = o.throttle_set {
                    // The throttle keys take the power back from the autothrottle.
                    if c.throttle != 0.0 {
                        self.ap.at = false;
                    } else {
                        c.throttle_set = Some(t);
                    }
                }
            }
            Err(why) => self.ap_disconnect(Some(why)),
        }
    }

    fn ap_disconnect(&mut self, why: Option<autopilot::Off>) {
        use autopilot::Off;
        self.ap.disengage();
        self.ap_off_t = self.real;
        log::info!("autopilot off: {why:?}");
        self.status = match why {
            Some(Off::Pilot) | None => t!("Autopilot off", "Pilote automatique coupé").into(),
            Some(Off::Ground) => t!("Autopilot off: on the ground", "Pilote automatique coupé : au sol").into(),
            Some(Off::Stall) => t!("Autopilot off: stall", "Pilote automatique coupé : décrochage").into(),
            Some(Off::Attitude) => t!("Autopilot off: unusual attitude", "Pilote automatique coupé : assiette inhabituelle").into(),
        };
        self.status_until = self.real + 4.0;
    }

    /// F5: the autopilot on (holding the heading and altitude of the
    /// moment, unless modes were picked beforehand) or off.
    fn ap_toggle(&mut self) {
        if self.ap.on {
            self.ap_disconnect(None);
            return;
        }
        if self.plane.on_ground || self.crashed || self.spec.rocket().is_some() {
            self.status = t!("The autopilot works in flight", "Le pilote automatique ne fonctionne qu'en vol").into();
            self.status_until = self.real + 3.0;
            return;
        }
        let s = autopilot::State::of(&self.plane);
        if self.ap.alt == 0.0 {
            self.ap.sync(&s);
        }
        if self.ap.lat == autopilot::Lat::Roll {
            self.ap.set_lat(autopilot::Lat::Hdg, &s);
            self.ap.hdg = s.hdg.round();
        }
        if self.ap.vert == autopilot::Vert::Pitch {
            self.ap.set_vert(autopilot::Vert::Alt, &s);
        }
        self.ap.engage(&s);
        log::info!("autopilot on: {:?} {:?}", self.ap.lat, self.ap.vert);
        self.status = t!("Autopilot on", "Pilote automatique engagé").into();
        self.status_until = self.real + 2.0;
    }

    /// A lateral mode button: picks it (and engages the autopilot), or, pressed again, back to HDG.
    fn ap_lat(&mut self, m: autopilot::Lat) {
        let s = autopilot::State::of(&self.plane);
        let m = if self.ap.on && self.ap.lat == m && m != autopilot::Lat::Hdg { autopilot::Lat::Hdg } else { m };
        // (Off, nothing set yet: the heading and altitude of the moment.)
        if !self.ap.on && self.ap.alt == 0.0 {
            self.ap.sync(&s);
        }
        // NAV and APR stay armed, flying the heading bug, until NAV 1 has the course.
        if matches!(m, autopilot::Lat::Nav | autopilot::Lat::Apr) && self.radios.nav.is_none() {
            self.status = t!("Armed: tune NAV 1 (Radio page)", "Armé : accordez la NAV 1 (page Radios)").into();
            self.status_until = self.real + 3.0;
        }
        self.ap.set_lat(m, &s);
        if !self.ap.on {
            self.ap_toggle();
        }
    }

    fn ap_vert(&mut self, m: autopilot::Vert) {
        let s = autopilot::State::of(&self.plane);
        if !self.ap.on && self.ap.alt == 0.0 {
            self.ap.sync(&s);
        }
        if m == autopilot::Vert::Vs && self.ap.vert == autopilot::Vert::Vs {
            self.ap.set_vert(autopilot::Vert::Alt, &s);
        } else {
            self.ap.set_vert(m, &s);
        }
        if !self.ap.on {
            self.ap_toggle();
        }
    }

    /// The autothrottle on or off (holding the airspeed of the moment if none is set).
    fn ap_at_toggle(&mut self) {
        self.ap.at = !self.ap.at;
        if self.ap.at && self.ap.spd <= 0.0 {
            self.ap.spd = autopilot::State::of(&self.plane).ias;
        }
    }
    /// The vertical speed the autopilot flies in VS, up or down 100 ft/min.
    fn ap_vs_step(&mut self, up: bool) {
        self.ap.vs = ((self.ap.vs * 196.85 / 100.0).round() * 100.0 + if up { 100.0 } else { -100.0 }).clamp(-4000.0, 4000.0) / 196.85;
    }
    /// The autothrottle's speed, up or down 5 kt.
    fn ap_spd_step(&mut self, up: bool) {
        self.ap.spd = ((self.ap.spd * 1.943_84).round() + if up { 5.0 } else { -5.0 }).max(40.0) / 1.943_84;
    }

    /// The flight panel's Autopilot page: the mode buttons and the settings.
    fn autopilot_page(&mut self, ui: &mut egui::Ui) {
        use autopilot::{Lat, Vert};
        use egui::RichText;
        use menu::{ACCENT, DIM};
        let s = autopilot::State::of(&self.plane);
        let (on, armed) = self.ap.annunciation();
        let green = egui::Color32::from_rgb(120, 230, 140);
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new(if self.ap.on { "AP" } else { "AP OFF" }).monospace().size(14.0).strong().color(if self.ap.on { green } else { DIM }));
            if self.ap.on {
                for m in &on {
                    ui.label(RichText::new(m).monospace().size(14.0).color(green));
                }
                for m in &armed {
                    ui.label(RichText::new(m).monospace().size(14.0).color(DIM));
                }
            }
        });
        ui.add_space(4.0);
        let btn = |ui: &mut egui::Ui, label: &str, lit: bool| ui.add(egui::Button::new(RichText::new(label).monospace().size(13.0).color(if lit { egui::Color32::BLACK } else { egui::Color32::WHITE })).fill(if lit { green } else { egui::Color32::from_gray(45) }).min_size(egui::vec2(52.0, 26.0))).clicked();
        let mut lat = None;
        let mut vert = None;
        let mut toggle = false;
        let mut at = false;
        ui.horizontal(|ui| {
            toggle = btn(ui, "AP", self.ap.on);
            for (m, n) in [(Lat::Hdg, "HDG"), (Lat::Nav, "NAV"), (Lat::Apr, "APR")] {
                if btn(ui, n, self.ap.on && self.ap.lat == m) {
                    lat = Some(m);
                }
            }
        });
        ui.horizontal(|ui| {
            for (m, n) in [(Vert::Alt, "ALT"), (Vert::Vs, "VS")] {
                if btn(ui, n, self.ap.on && (self.ap.vert == m || (m == Vert::Alt && self.ap.vert == Vert::Gs && false))) {
                    vert = Some(m);
                }
            }
            if btn(ui, "AT", self.ap.at) {
                at = true;
            }
        });
        if at {
            self.ap_at_toggle();
        }
        if toggle {
            self.ap_toggle();
        }
        if let Some(m) = lat {
            self.ap_lat(m);
        }
        if let Some(m) = vert {
            self.ap_vert(m);
        }
        ui.add_space(6.0);
        // The settings: −/+ and SYNC to the aircraft's own.
        let row = |ui: &mut egui::Ui, name: &str, value: String, minus: &mut bool, plus: &mut bool, sync: &mut bool| {
            ui.horizontal(|ui| {
                ui.add_sized([44.0, 22.0], egui::Label::new(RichText::new(name).monospace().color(DIM)));
                *minus = hold_repeat(&ui.add_sized([28.0, 22.0], egui::Button::new("−")));
                ui.add_sized([86.0, 22.0], egui::Label::new(RichText::new(value).monospace().size(14.0).color(ACCENT)));
                *plus = hold_repeat(&ui.add_sized([28.0, 22.0], egui::Button::new("+")));
                *sync = ui.small_button("SYNC").clicked();
            });
        };
        let (mut m, mut p, mut y) = (false, false, false);
        row(ui, "HDG", format!("{:03.0}°", self.ap.hdg.rem_euclid(360.0)), &mut m, &mut p, &mut y);
        if m || p {
            self.ap.hdg = (self.ap.hdg + if p { 5.0 } else { -5.0 }).rem_euclid(360.0);
        }
        if y {
            self.ap.hdg = s.hdg.round();
        }
        let (mut m, mut p, mut y) = (false, false, false);
        row(ui, "ALT", format!("{:.0} ft", self.ap.alt * 3.28084), &mut m, &mut p, &mut y);
        if m || p {
            self.ap.alt = (((self.ap.alt * 3.28084 / 100.0).round() * 100.0 + if p { 100.0 } else { -100.0 }).max(0.0)) / 3.28084;
        }
        if y {
            self.ap.alt = (s.alt * 3.28084 / 100.0).round() * 100.0 / 3.28084;
        }
        let (mut m, mut p, mut y) = (false, false, false);
        row(ui, "VS", format!("{:+.0} fpm", self.ap.vs * 196.85), &mut m, &mut p, &mut y);
        if m || p {
            self.ap_vs_step(p);
        }
        if y {
            self.ap.vs = (s.vs * 196.85 / 100.0).round() * 100.0 / 196.85;
        }
        let (mut m, mut p, mut y) = (false, false, false);
        row(ui, "SPD", format!("{:.0} kt", self.ap.spd * 1.943_84), &mut m, &mut p, &mut y);
        if m || p {
            self.ap_spd_step(p);
        }
        if y {
            self.ap.spd = s.ias;
        }
        ui.add_space(6.0);
        ui.label(RichText::new(t!(
            "F5 autopilot on/off · 7 HDG · 8 NAV · 9 APR (ILS) · 0 ALT · − / = heading bug · ; / ' altitude. NAV and APR follow NAV 1 (Radio page). A firm push on the controls disconnects it.",
            "F5 pilote automatique · 7 HDG · 8 NAV · 9 APR (ILS) · 0 ALT · − / = consigne de cap · ; / ' altitude. NAV et APR suivent la NAV 1 (page Radios). Une action franche sur les commandes le déconnecte."
        )).size(11.5).color(DIM));
    }

    fn radio_page(&mut self, ui: &mut egui::Ui) {
        use egui::RichText;
        use flightui::{chip, kv, list_row, pane_section};
        use menu::{ACCENT, TEXT};
        use nav::Kind;
        let pos = self.plane.pos;
        let (la, lo, h) = geo::pos_to_lla(pos);
        let (lat, lon) = (la.to_degrees(), lo.to_degrees());
        let note = |ui: &mut egui::Ui, s: &str| {
            ui.add(egui::Label::new(RichText::new(s).font(menu::caption()).color(flightui::SOFT)).wrap());
        };
        ui.add_space(8.0);
        let w = ui.available_width();
        ui.allocate_ui_with_layout(egui::vec2(w, 26.0), egui::Layout::left_to_right(egui::Align::Center), |ui| {
            ui.set_min_width(w);
            ui.label(RichText::new(t!("Instruments on screen (J)", "Instruments à l'écran (J)")).font(menu::caption()).color(flightui::SOFT));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                menu::switch(ui, &mut self.radios.show);
            });
        });

        pane_section(ui, "NAV 1  ·  VOR / ILS");
        if let Some(st) = self.radios.nav.clone() {
            let r = nav::nav_reading(&st, self.radios.obs, pos);
            let state = if !r.live {
                t!("no signal", "pas de signal").to_string()
            } else {
                let mut s = match r.to {
                    Some(true) => "TO".to_string(),
                    Some(false) => "FROM".to_string(),
                    None => "LOC".to_string(),
                };
                if r.gs.is_some() {
                    s += "  G/S";
                }
                if let Some(d) = r.dme {
                    s += &format!("  DME {d:.1} NM");
                }
                s
            };
            let mut off = false;
            list_row(ui, &st.freq_text(), &st.label(), &state, |ui| {
                off = chip(ui, t!("Off", "Éteindre"), false, false).clicked();
            });
            if off {
                self.radios.nav = None;
            }
            if st.kind == Kind::Ils {
                kv(ui, t!("Course", "Axe"), &format!("{:03.0}°", st.course), ACCENT);
            } else {
                kv(ui, "OBS", &format!("{:03.0}°", self.radios.obs), ACCENT);
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    for (txt, d) in [("−10", -10.0), ("−1", -1.0), ("+1", 1.0), ("+10", 10.0)] {
                        if chip(ui, txt, true, false).clicked() {
                            self.radios.obs = (self.radios.obs.round() + d).rem_euclid(360.0);
                        }
                    }
                    if chip(ui, t!("Direct to", "Direct"), false, false).clicked() {
                        self.radios.obs = nav::bearing(pos, st.pos()).round();
                    }
                });
            }
        } else {
            note(ui, t!("Not tuned: pick a station below.", "Non réglée : choisissez une station ci-dessous."));
        }
        let mut list = nav::near(lat, lon, 200_000.0, Kind::Vor);
        list.truncate(8);
        let mut ils = nav::near(lat, lon, 40_000.0, Kind::Ils);
        ils.truncate(8);
        list.extend(ils);
        list.sort_by(|a, b| a.1.total_cmp(&b.1));
        let mut tune = None;
        ui.add_space(6.0);
        for (st, d) in list.iter().take(10) {
            let on = self.radios.nav.as_ref() == Some(st);
            list_row(ui, &st.freq_text(), &st.label(), &format!("{:.0} km", d / 1000.0), |ui| {
                if chip(ui, if on { t!("Tuned", "Réglée") } else { t!("Tune", "Régler") }, false, on).clicked() && !on {
                    tune = Some(st.clone());
                }
            });
        }
        if let Some(st) = tune {
            self.radios.tune_nav(st, pos);
        }

        pane_section(ui, "ADF  ·  NDB");
        match self.radios.adf.clone() {
            Some(st) => {
                let mut off = false;
                list_row(ui, &st.freq_text(), &st.label(), "", |ui| {
                    off = chip(ui, t!("Off", "Éteindre"), false, false).clicked();
                });
                if off {
                    self.radios.adf = None;
                }
            }
            None => note(ui, t!("Not tuned.", "Non réglé.")),
        }
        let mut tune = None;
        for (st, d) in nav::near(lat, lon, 150_000.0, Kind::Ndb).iter().take(6) {
            let on = self.radios.adf.as_ref() == Some(st);
            list_row(ui, &st.freq_text(), &st.label(), &format!("{:.0} km", d / 1000.0), |ui| {
                if chip(ui, if on { t!("Tuned", "Réglé") } else { t!("Tune", "Régler") }, false, on).clicked() && !on {
                    tune = Some(st.clone());
                }
            });
        }
        if let Some(st) = tune {
            self.radios.tune_adf(st);
        }

        pane_section(ui, t!("Altimeter", "Altimètre"));
        let qnh = self.wx.qnh;
        let src = if self.wx.live && !self.wx.live_station.is_empty() { self.wx.live_station.clone() } else { t!("standard", "standard").into() };
        kv(ui, "QNH", &format!("{qnh:.0} hPa  ({src})"), TEXT);
        let set = self.radios.setting(qnh);
        kv(ui, t!("Setting", "Calage"), &format!("{set:.0} hPa{}", if self.radios.baro.is_none() { t!("  on QNH", "  sur QNH") } else { "" }), TEXT);
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            if chip(ui, "−1", true, false).clicked() {
                self.radios.baro = Some(set.round() - 1.0);
            }
            if chip(ui, "+1", true, false).clicked() {
                self.radios.baro = Some(set.round() + 1.0);
            }
            if chip(ui, "QNH", true, self.radios.baro.is_none()).on_hover_text("X").clicked() {
                self.radios.baro = None;
            }
            if chip(ui, "STD 1013", true, self.radios.baro == Some(1013.25)).clicked() {
                self.radios.baro = Some(1013.25);
            }
        });
        let true_ft = h * 3.28084;
        ui.add_space(4.0);
        kv(ui, t!("Altimeter reads", "L'altimètre indique"), &format!("{:.0} ft", true_ft + self.radios.alt_error_ft(qnh)), TEXT);
        kv(ui, t!("True altitude", "Altitude vraie"), &format!("{true_ft:.0} ft"), TEXT);
    }

    fn cockpit_globals(&self, agl: f64) -> [[f32; 4]; 12] {
        let p = &self.plane;
        let (lat, lon, h) = geo::pos_to_lla(p.pos);
        let (east, north, up) = geo::enu(p.pos);
        let gs = (p.vel - up * p.vel.dot(up)).length() * 1.94384;
        let hdg = p.forward().dot(east).atan2(p.forward().dot(north));
        let mut data = [[0.0; 4]; 12];
        data[0] = [(self.radios.setting(self.wx.qnh) * 0.029529983 * 100.0) as f32,
            ((h * 3.28084 + (1013.25 - self.wx.qnh) * 27.3) / 100.0) as f32,
            gs as f32, (p.spec.fuel_kg / 0.72 / 3.785 / 2.0) as f32];
        let adf = self.radios.adf.as_ref().and_then(|s| nav::adf_bearing(s, p.pos, hdg.to_degrees())).filter(|_| p.sys.powered());
        data[1] = [self.radios.nav.as_ref().map(|s| s.freq_khz as f32 / 1000.0).unwrap_or(0.0),
            self.radios.adf.as_ref().map(|s| s.freq_khz as f32).unwrap_or(0.0), adf.unwrap_or(0.0) as f32, if adf.is_some() { 1.0 } else { 0.0 }];
        data[2] = [self.gauges.cylinder_head_f(), ((p.trim + 0.05) / 0.29) as f32, if p.sys.powered() { 1.0 } else { 0.0 }, p.engine_hours as f32];
        let plot = |pos: DVec3| { let v = pos - p.pos; [v.dot(east) as f32 / 18520.0, v.dot(north) as f32 / 18520.0, 1.0, 0.0] };
        if let Some(st) = &self.radios.nav { data[3] = plot(st.pos()); }
        for (dst, (st, _)) in data[4..8].iter_mut().zip(nav::near(lat.to_degrees(), lon.to_degrees(), 18520.0, nav::Kind::Vor).iter().take(4)) { *dst = plot(st.pos()); }
        // Rows 8-11: what the glass cockpits show of the autopilot and the
        // systems (common.wgsl `Globals::cockpit`, deck650.wgsl).
        {
            use autopilot::{Lat, Vert};
            let ap = &self.ap;
            let bit = |b: bool, v: u32| if b { v } else { 0 };
            let bits = bit(ap.on, 1) | bit(ap.at, 2)
                | match ap.lat { Lat::Roll => 0, Lat::Hdg => 4, Lat::Nav => 8, Lat::Apr => 16 }
                | match ap.vert { Vert::Pitch => 0, Vert::Alt => 32, Vert::Vs => 64, Vert::Gs => 128 }
                | bit(ap.nav_cap, 256) | bit(ap.gs_cap, 512) | bit(ap.alt_cap, 1024) | bit(ap.slow, 2048) | bit(ap.fast, 4096);
            data[8] = [bits as f32, ap.hdg.rem_euclid(360.0) as f32, (ap.alt * 3.28084) as f32, (ap.spd * 1.943_84) as f32];
            data[9] = [(ap.vs * 196.85) as f32, p.trim.to_degrees() as f32, p.speedbrake as f32, p.spoilers as f32];
            let flags = bit(p.running, 1) | bit(p.starter_on, 2) | bit(p.ignition, 4) | bit(p.sys.powered(), 8) | bit(p.on_ground, 16) | bit(p.parking_brake, 32);
            data[10] = [p.reverser as f32, flags as f32, p.toe_brake[0].max(p.toe_brake[1]) as f32, (p.alpha / p.stall_alpha().max(0.01)) as f32];
            // The cabin: held near the field's height on the ground and low,
            // scheduled to about 4,850 ft at 51,000 ft (an estimate, not the
            // real pressurisation schedule). The speed trend: the along-path
            // acceleration (the accelerometers' specific force less gravity)
            // carried 6 s ahead.
            let alt_ft = h * 3.28084;
            let agl_ft = agl * 3.28084;
            let k = ((agl_ft - 500.0) / 3000.0).clamp(0.0, 1.0);
            let cabin = alt_ft + (alt_ft * 0.095 - alt_ft).min(0.0) * k;
            let air = p.vel - p.wind;
            let acc = p.rot * p.sf - up * 9.81;
            let trend = if air.length() > 1.0 { acc.dot(air.normalize()) * 6.0 * 1.943_84 } else { 0.0 };
            data[11] = [p.sys.elec.volts as f32, p.sys.elec.amps as f32, cabin as f32, trend as f32];
        }
        data
    }

    fn cancel_cockpit_input(&mut self) {
        self.hot_press = None;
        self.lean_drag = None;
        self.hot_keys.clear();
        self.wheel_hold.clear();
        self.wheel_acc = cockpit::WheelSteps::default();
    }

    fn cockpit_hotspots(&self, pose: &[Mat4]) -> Vec<aircraft::Hotspot> {
        aircraft::animated_hotspots(&self.cockpit_controls, pose)
    }

    fn cockpit_action(&mut self, id: &str) {
        match id {
            "prop_up" => self.plane.propeller = (self.plane.propeller + 0.05).min(1.0),
            "prop_down" => self.plane.propeller = (self.plane.propeller - 0.05).max(0.0),
            "cowl_open" => self.plane.cowl_flaps = (self.plane.cowl_flaps + 0.1).min(1.0),
            "cowl_close" => self.plane.cowl_flaps = (self.plane.cowl_flaps - 0.1).max(0.0),
            "landing_lights" => self.plane.landing_lights = !self.plane.landing_lights,
            "ap_vs" if !self.crashed => self.ap_vert(autopilot::Vert::Vs),
            "ap_at" => self.ap_at_toggle(),
            "ap_vs_up" | "ap_vs_down" => self.ap_vs_step(id == "ap_vs_up"),
            "ap_spd_up" | "ap_spd_down" => self.ap_spd_step(id == "ap_spd_up"),
            "fuel_control_l" | "fuel_control_r" | "battery_l" | "battery_r" => self.pair_switch(id),
            "fuel_valve" | "fuel_next" | "fuel_prev" => {
                self.plane.fuel_selector = self.plane.fuel_selector.turn(id != "fuel_prev");
            }
            "nav_next" | "nav_prev" | "adf_next" | "adf_prev" => {
                let adf = id.starts_with("adf");
                let (lat, lon, _) = geo::pos_to_lla(self.plane.pos);
                let mut list = nav::near(lat.to_degrees(), lon.to_degrees(), if adf { 150_000.0 } else { 200_000.0 }, if adf { nav::Kind::Ndb } else { nav::Kind::Vor });
                if !adf { list.extend(nav::near(lat.to_degrees(), lon.to_degrees(), 40_000.0, nav::Kind::Ils)); }
                list.sort_by(|a,b| a.1.total_cmp(&b.1));
                let current = if adf { &self.radios.adf } else { &self.radios.nav };
                let slot = current.as_ref().and_then(|s| list.iter().position(|(q, _)| q == s)).map(|i| i + 1).unwrap_or(0);
                let next = (slot as isize + if id.ends_with("next") { 1 } else { -1 }).rem_euclid(list.len() as isize + 1) as usize;
                let station = next.checked_sub(1).map(|i| list[i].0.clone());
                if adf { self.radios.adf = station; } else if let Some(st) = station { let show = self.radios.show; self.radios.tune_nav(st, self.plane.pos); self.radios.show = show; } else { self.radios.nav = None; }
            }
            _ => {}
        }
    }

    /// One of a twin jet's L/R fuel control or battery switches clicked: it
    /// goes to the other position. The shared state (`Plane::ignition`,
    /// `sys.elec.master`) is on while either switch of the pair is, and is
    /// switched with the action's key (`ignition`, `master`), so the mouse
    /// does what the key does.
    fn pair_switch(&mut self, id: &str) {
        let fuel = id.starts_with("fuel");
        let (k, w) = (id.ends_with("_r") as usize, !fuel as usize);
        let shared = |p: &plane::Plane| if fuel { p.ignition } else { p.sys.elec.master };
        let on = shared(&self.plane);
        let want = [!self.plane.pair_on(w, k), self.plane.pair_on(w, 1 - k)];
        if want[0] != on && !(on && want[1]) {
            // Off to on, or the last switch of the pair off: the key.
            if let Some(key) = Self::action_key(if fuel { "ignition" } else { "master" }) {
                self.key(key);
            }
            if shared(&self.plane) == on { return; }
        }
        // Which switch is off while its partner holds the state on (both on, or both off: neither).
        let mut pair = [false; 2];
        if want[0] != want[1] {
            pair[k] = !want[0];
            pair[1 - k] = !want[1];
        }
        self.plane.pair_off[w] = pair;
    }

    /// NAV 1 as the cockpit's course deviation indicators show it (Globals::nav).
    fn nav_globals(&self) -> [f32; 4] {
        let obs = self.radios.obs as f32;
        let Some(st) = self.radios.nav.as_ref().filter(|_| self.plane.sys.powered()) else {
            return [0.0, 0.0, obs, 0.0];
        };
        let r = nav::nav_reading(st, self.radios.obs, self.plane.pos);
        if !r.live {
            return [0.0, 0.0, obs, 0.0];
        }
        let mut flags = 1.0;
        match r.to {
            Some(true) => flags += 2.0,
            Some(false) => flags += 4.0,
            None => {}
        }
        if r.gs.is_some() {
            flags += 8.0;
        }
        [r.cdi as f32, r.gs.unwrap_or(0.0) as f32, obs, flags]
    }

    /// the radio instruments at the bottom centre, false when none are tuned.
    fn nav_overlay(&self, ctx: &egui::Context, hdg: f64) -> bool {
        use egui::{Align2, Color32, Stroke};
        let pos = self.plane.pos;
        // (Without power the receivers are dead: OFF flags.)
        let pw = self.plane.sys.powered();
        let nav = self.radios.nav.as_ref().map(|s| {
            let mut r = nav::nav_reading(s, self.radios.obs, pos);
            if !pw {
                (r.live, r.gs, r.dme) = (false, None, None);
            }
            (s, r)
        });
        let adf = self.radios.adf.as_ref().map(|s| (s, nav::adf_bearing(s, pos, hdg).filter(|_| pw)));
        if nav.is_none() && adf.is_none() {
            return false;
        }
        let white = Color32::from_rgba_unmultiplied(255, 255, 255, 235);
        let dim = Color32::from_rgba_unmultiplied(255, 255, 255, 140);
        let green = Color32::from_rgb(110, 235, 140);
        let amber = Color32::from_rgb(255, 196, 64);
        let red = Color32::from_rgb(255, 90, 60);
        let mono = |s: f32| menu::medium(s + 1.0);
        egui::Area::new(egui::Id::unique("nav_inst")).anchor(Align2::CENTER_BOTTOM, [0.0, -14.0]).interactable(false).show(ctx, |ui| {
            flightui::frosted(ui, flightui::glass().inner_margin(10.0), |ui| {
                ui.horizontal(|ui| {
                    if let Some((st, r)) = &nav {
                        let (rect, _) = ui.allocate_exact_size(egui::vec2(220.0, 128.0), egui::Sense::hover());
                        let p = ui.painter();
                        p.text(rect.left_top(), Align2::LEFT_TOP, format!("NAV1 {} {}", st.ident, st.freq_text()), mono(12.0), white);
                        if let Some(d) = r.dme.filter(|_| r.live) {
                            p.text(rect.right_top(), Align2::RIGHT_TOP, format!("DME {d:.1}"), mono(12.0), white);
                        }
                        // The course needle over five dots each side.
                        let c = egui::pos2(rect.left() + 95.0, rect.top() + 66.0);
                        for k in -5..=5 {
                            if k != 0 {
                                p.circle_stroke(c + egui::vec2(k as f32 * 15.0, 0.0), 2.5, Stroke::new(1.2, dim));
                            }
                        }
                        p.circle_stroke(c, 5.0, Stroke::new(1.2, dim));
                        if r.live {
                            let x = c.x + r.cdi as f32 * 75.0;
                            p.line_segment([egui::pos2(x, c.y - 36.0), egui::pos2(x, c.y + 36.0)], Stroke::new(3.0, green));
                        } else {
                            p.text(c + egui::vec2(0.0, -22.0), Align2::CENTER_CENTER, "OFF", mono(14.0), red);
                        }
                        // TO / FROM, and the course.
                        let course = if st.kind == nav::Kind::Ils { st.course } else { self.radios.obs };
                        let flag = match (r.live, r.to) {
                            (true, Some(true)) => "▲ TO",
                            (true, Some(false)) => "▼ FROM",
                            (true, None) => "LOC",
                            _ => "",
                        };
                        p.text(c + egui::vec2(0.0, 48.0), Align2::CENTER_CENTER, flag, mono(11.0), white);
                        p.text(egui::pos2(rect.left(), rect.bottom()), Align2::LEFT_BOTTOM, format!("CRS {course:03.0}"), mono(12.0), amber);
                        // The glide slope on the right, on an ILS.
                        if st.kind == nav::Kind::Ils {
                            let gx = rect.right() - 12.0;
                            for k in -2..=2 {
                                p.circle_stroke(egui::pos2(gx, c.y + k as f32 * 16.0), if k == 0 { 4.0 } else { 2.5 }, Stroke::new(1.2, dim));
                            }
                            p.text(egui::pos2(gx, rect.bottom()), Align2::CENTER_BOTTOM, "GS", mono(10.0), dim);
                            match r.gs {
                                Some(g) => {
                                    let y = c.y - g as f32 * 32.0;
                                    let d = 6.0;
                                    p.add(egui::Shape::convex_polygon(vec![egui::pos2(gx, y - d), egui::pos2(gx + d, y), egui::pos2(gx, y + d), egui::pos2(gx - d, y)], green, Stroke::NONE));
                                }
                                None => {
                                    p.text(egui::pos2(gx, c.y - 44.0), Align2::CENTER_CENTER, "OFF", mono(10.0), red);
                                }
                            }
                        }
                    }
                    if let Some((st, b)) = &adf {
                        let (rect, _) = ui.allocate_exact_size(egui::vec2(110.0, 128.0), egui::Sense::hover());
                        let p = ui.painter();
                        let c = rect.center() + egui::vec2(0.0, -6.0);
                        let rad = 44.0;
                        p.circle_stroke(c, rad, Stroke::new(1.2, dim));
                        for k in 0..12 {
                            let a = (k as f32 * 30.0).to_radians();
                            let v = egui::vec2(a.sin(), -a.cos());
                            p.line_segment([c + v * (rad - if k % 3 == 0 { 8.0 } else { 4.0 }), c + v * rad], Stroke::new(1.2, dim));
                        }
                        p.text(c + egui::vec2(0.0, -rad + 14.0), Align2::CENTER_CENTER, "▲", mono(9.0), dim);
                        match b {
                            Some(b) => {
                                let a = (*b as f32).to_radians();
                                let v = egui::vec2(a.sin(), -a.cos());
                                p.line_segment([c - v * (rad - 10.0), c + v * (rad - 6.0)], Stroke::new(2.5, amber));
                                p.circle_filled(c + v * (rad - 6.0), 3.5, amber);
                            }
                            None => {
                                p.text(c, Align2::CENTER_CENTER, "OFF", mono(13.0), red);
                            }
                        }
                        p.text(egui::pos2(c.x, rect.bottom()), Align2::CENTER_BOTTOM, format!("ADF {} {}", st.ident, st.freq_text()), mono(11.0), white);
                    }
                });
            });
        });
        true
    }

    /// A gamepad button pressed, for whatever it works at the moment.
    fn pad_button(&mut self, nav: Nav, b: gilrs::Button, modifier: bool) {
        use gilrs::Button;
        match (nav, b) {
            // The start menu. Everywhere A presses what is picked out and B
            // steps back towards the main menu. On the globe Menu takes off
            // and X starts at the crosshair; on the settings and aircraft
            // screens the bumpers turn the sections or the aircraft.
            (Nav::Globe, Button::South) => self.pad_key(egui::Key::Enter),
            (Nav::Globe, Button::East) => self.menu_back(),
            (Nav::Globe, Button::Start) if self.menu.screen == menu::Screen::Fly => self.begin(),
            (Nav::Globe, Button::West) if self.menu.screen == menu::Screen::Fly => self.pad_pick_centre(),
            (Nav::Globe, Button::LeftTrigger | Button::RightTrigger) if self.menu.screen == menu::Screen::Settings => {
                self.settings_sec = self.settings_sec.cycle(b == Button::RightTrigger);
                self.pad_home = true;
            }
            (Nav::Globe, Button::LeftTrigger | Button::RightTrigger) if self.menu.screen == menu::Screen::Aircraft => {
                let n = aircraft::list().len();
                self.menu.air_view = (self.menu.air_view + if b == Button::RightTrigger { 1 } else { n - 1 }) % n;
                self.menu.air_tab = hangar_ui::Tab::Mine;
                self.pad_home = true;
            }
            (Nav::Globe, _) => {}
            // The pause menu: A presses, B steps back out of a page (then resumes), Menu resumes,
            // the bumpers turn the settings screen's sections.
            (Nav::Pause, Button::East) if self.pause_page != menu::Page::Game => {
                self.pause_page = menu::Page::Game;
                self.pad_home = true;
            }
            (Nav::Pause, Button::Start | Button::East) => self.set_pause_menu(false),
            (Nav::Pause, Button::South) => self.pad_key(egui::Key::Enter),
            (Nav::Pause, Button::LeftTrigger | Button::RightTrigger) if self.pause_page == menu::Page::Settings => {
                self.settings_sec = self.settings_sec.cycle(b == Button::RightTrigger);
                self.pad_home = true;
            }
            (Nav::Pause, _) => {}
            // The flight panel, open beside the flying: the same, and B closes it.
            // (Its other buttons, and Y with any, still fly: pad_fly_buttons.)
            (Nav::Panel, Button::South) if !modifier => self.pad_key(egui::Key::Enter),
            (Nav::Panel, Button::East) if !modifier && self.radial.open => self.radial.open = false,
            (Nav::Panel, Button::East) if !modifier => self.info_open = false,
            (Nav::Panel, Button::LeftTrigger | Button::RightTrigger) if !modifier => {
                self.info_page = menu::Page::cycle(&menu::Page::INFO, self.info_page, b == Button::RightTrigger);
                self.pad_home = true;
            }
            // A skips the loading screen (the scenery goes on loading), as Enter does.
            (_, Button::South) if self.loading.as_ref().is_some_and(|l| l.done.is_none()) => self.key(KeyCode::Enter),
            (_, Button::South) if self.crashed => self.respawn_here(),
            // Menu pauses, unless Y + Menu is given something to do.
            (_, Button::Start) if !(modifier && self.binds.pad.has_y(padmap::PadBtn::Menu)) => self.set_pause_menu(true),
            // Everything else that flies is in Settings › Controls › Gamepad (pad_fly_buttons).
            _ => {}
        }
    }

    /// A key for the menus from the gamepad (d-pad arrows, A as Enter). With
    /// nothing picked out yet it picks out the screen's first choice instead;
    /// in a text box the arrows move on rather than along the text.
    fn pad_key(&mut self, key: egui::Key) {
        if self.egui_ctx.memory(|m| m.focused()).is_none() {
            self.pad_home = true;
            return;
        }
        let dir = match key {
            egui::Key::ArrowUp => Some(egui::FocusDirection::Up),
            egui::Key::ArrowDown => Some(egui::FocusDirection::Down),
            egui::Key::ArrowLeft => Some(egui::FocusDirection::Left),
            egui::Key::ArrowRight => Some(egui::FocusDirection::Right),
            _ => None,
        };
        if let Some(d) = dir {
            self.pad_skip = Some((d, 0, 0));
            if self.egui_ctx.text_edit_focused() {
                self.pad_move = Some(d);
                return;
            }
        }
        for pressed in [true, false] {
            self.pad_events.push(egui::Event::Key { key, physical_key: None, pressed, repeat: false, modifiers: egui::Modifiers::NONE });
        }
    }

    /// The globe: start at the airport or the spot under the crosshair.
    fn pad_pick_centre(&mut self) {
        let size = self.window.inner_size();
        let (w, h) = (size.width.max(1) as f64, size.height.max(1) as f64);
        let near = self.menu.marker_near(w * 0.5, h * 0.5, 28.0 * self.egui_ctx.pixels_per_point() as f64);
        if let Some(i) = near {
            self.choose_airport(i);
        } else if let Some((lat, lon)) = self.menu.pick_at(w * 0.5, h * 0.5, w, h, 60f64.to_radians()) {
            self.pick(lat, lon);
        }
    }

    fn key(&mut self, code: KeyCode) {
        if self.mode == Mode::Menu {
            use menu::Screen;
            let enter = matches!(code, KeyCode::Enter | KeyCode::NumpadEnter);
            match self.menu.screen {
                _ if code == KeyCode::Escape => self.menu_back(),
                Screen::Main if enter => {
                    self.menu.screen = Screen::Fly;
                    self.pad_home = true;
                }
                Screen::Fly if enter => self.begin(),
                Screen::Aircraft if enter && self.menu.air_tab == hangar_ui::Tab::Mine => {
                    self.menu.aircraft = self.menu.air_view;
                    self.menu.screen = self.menu.air_back;
                    self.pad_home = true;
                }
                _ if code == KeyCode::F12 => self.shots.insert(0, (0.0, PathBuf::from(format!("screenshot-{}.png", self.frame)), None)),
                _ => {}
            }
            return;
        }
        // Enter, Space or Esc skip the loading screen (the scenery goes on loading).
        if self.loading.as_ref().is_some_and(|l| l.done.is_none()) {
            if matches!(code, KeyCode::Enter | KeyCode::NumpadEnter | KeyCode::Space | KeyCode::Escape) {
                self.finish_loading();
            }
            return;
        }
        if self.pause_open {
            // The pause menu has the keyboard (and the game is stopped) until it closes.
            match code {
                // Esc steps back: out of a page (the settings, the key list, the failures), then out of the menu.
                KeyCode::Escape if self.pause_page != menu::Page::Game => {
                    self.pause_page = menu::Page::Game;
                    self.pad_home = true;
                }
                KeyCode::Escape => self.set_pause_menu(false),
                KeyCode::F12 => self.shots.insert(0, (0.0, PathBuf::from(format!("screenshot-{}.png", self.frame)), None)),
                _ => {}
            }
            return;
        }
        match code {
            KeyCode::Tab => self.flight_menu_key(),
            KeyCode::Escape if self.radial.open => self.radial.open = false,
            // Esc closes a pop-up window first; otherwise it pauses.
            KeyCode::Escape if self.show_help => self.show_help = false,
            KeyCode::Escape if self.wx.open => self.wx.open = false,
            KeyCode::Escape => self.set_pause_menu(true),
            KeyCode::KeyR if self.arcade || self.crashed => self.respawn_here(),
            KeyCode::KeyR => self.arcade_only(t!("Respawning in flight", "La réapparition en vol")),
            KeyCode::KeyC => self.next_view(),
            KeyCode::KeyF if !self.crashed => self.plane.set_flaps(1),
            KeyCode::KeyV if !self.crashed => self.plane.set_flaps(-1),
            KeyCode::KeyL if !self.crashed => self.plane.toggle_gear(),
            KeyCode::Slash if !self.crashed => self.plane.toggle_speedbrake(),
            KeyCode::Digit6 if !self.crashed && self.model.pieces.iter().any(|p| p.anim.uses(airframe::Input::Canopy)) => {
                self.plane.toggle_canopy();
                self.status = if self.plane.canopy_open { t!("Canopy open", "Verrière ouverte") } else { t!("Canopy shut", "Verrière fermée") }.into();
                self.status_until = self.real + 3.0;
            }
            KeyCode::Digit1 if !self.crashed => {
                self.plane.next_engines();
                if let Some((name, lit, _)) = self.plane.engines() {
                    self.status = t!("Engines: {} ({} burning)", "Moteurs : {} ({} allumés)", name, lit);
                    self.status_until = self.real + 3.0;
                }
            }
            KeyCode::Backspace if !self.crashed => self.plane.stage_sep(),
            KeyCode::Digit2 if !self.crashed => self.plane.toggle_autoland(),
            KeyCode::Digit3 if !self.crashed => {
                if !self.missiles_ok() {
                    self.status = self.missiles_refusal().into();
                } else {
                    let on = self.weapons.toggle_arm();
                    self.status = if on { t!("Master arm on", "Armement maître sur ON") } else { t!("Master arm off", "Armement maître sur OFF") }.into();
                }
                self.status_until = self.real + 3.0;
            }
            KeyCode::Digit4 if !self.crashed => {
                if !self.missiles_ok() {
                    self.status = self.missiles_refusal().into();
                } else {
                    self.weapons.request_cycle();
                }
                self.status_until = self.real + 3.0;
            }
            KeyCode::Digit5 if !self.crashed => {
                if !self.missiles_ok() {
                    self.status = self.missiles_refusal().into();
                } else {
                    self.weapons.request_fire();
                }
                self.status_until = self.real + 3.0;
            }
            KeyCode::KeyG => {
                self.goto_open = true;
                self.goto_text.clear();
            }
            KeyCode::KeyT | KeyCode::KeyY | KeyCode::KeyN | KeyCode::KeyM if !self.arcade => self.arcade_only(t!("The weather and time keys", "Les touches météo et heure")),
            KeyCode::KeyT | KeyCode::KeyY => {
                // Setting the time by hand leaves real time.
                self.wx.real_time = false;
                self.utc = (self.utc + if code == KeyCode::KeyT { 0.5 } else { -0.5 }).rem_euclid(24.0);
            }
            KeyCode::KeyN => self.wx.clouds = (self.wx.clouds + 0.1).min(1.0),
            KeyCode::KeyM => self.wx.clouds = (self.wx.clouds - 0.1).max(0.0),
            KeyCode::F3 => self.wx.open = !self.wx.open,
            KeyCode::KeyP => self.paused = !self.paused,
            // h toggles the hud of the current view.
            KeyCode::KeyH if self.cam_mode == 1 => self.hud_cockpit = !self.hud_cockpit,
            KeyCode::KeyH => self.show_hud = !self.show_hud,
            KeyCode::Backquote => self.minimap.visible = !self.minimap.visible,
            KeyCode::KeyJ => self.radios.show = !self.radios.show,
            KeyCode::Insert => {
                self.plane.pair_off[1] = [false; 2];
                let m = &mut self.plane.sys.elec.master;
                *m = !*m;
                self.status = if *m { t!("Battery master on", "Batterie (master) sur ON") } else { t!("Battery master off", "Batterie (master) sur OFF") }.into();
                self.status_until = self.real + 3.0;
            }
            KeyCode::KeyI => {
                self.plane.pair_off[0] = [false; 2];
                self.plane.ignition = !self.plane.ignition;
                let on = self.plane.ignition;
                self.status = if self.spec.is_jet() {
                    if on { t!("Fuel control switches: RUN", "Robinets carburant moteurs : RUN") } else { t!("Fuel control switches: CUTOFF", "Robinets carburant moteurs : CUTOFF") }
                } else if on {
                    t!("Magnetos on", "Magnétos sur BOTH")
                } else {
                    t!("Magnetos off", "Magnétos coupés")
                }
                .into();
                self.status_until = self.real + 3.0;
            }
            KeyCode::KeyO if self.spec.is_jet() => {
                self.plane.starter_on = true;
                self.status = t!("Starters engaged", "Démarreurs engagés").into();
                self.status_until = self.real + 3.0;
            }
            KeyCode::F4 => {
                let on = self.plane.running || self.plane.starter_on;
                if on { self.plane.stop_engine() } else { self.plane.start_engine() }
                self.status = if on { t!("Engine shut down", "Moteur coupé") } else { t!("Starting the engine…", "Démarrage du moteur…") }.into();
                self.status_until = self.real + 3.0;
            }
            KeyCode::KeyX => {
                self.radios.baro = None;
                self.status = t!("Altimeter set to the QNH, {:.0} hPa", "Altimètre calé au QNH, {:.0} hPa", self.wx.qnh);
                self.status_until = self.real + 3.0;
            }
            KeyCode::Period => {
                self.plane.parking_brake = !self.plane.parking_brake;
                self.status = if self.plane.parking_brake { t!("Parking brake set", "Frein de parc serré") } else { t!("Parking brake released", "Frein de parc desserré") }.into();
                self.status_until = self.real + 2.0;
            }
            KeyCode::F5 if !self.crashed => self.ap_toggle(),
            KeyCode::Digit7 if !self.crashed => self.ap_lat(autopilot::Lat::Hdg),
            KeyCode::Digit8 if !self.crashed => self.ap_lat(autopilot::Lat::Nav),
            KeyCode::Digit9 if !self.crashed => self.ap_lat(autopilot::Lat::Apr),
            KeyCode::Digit0 if !self.crashed => self.ap_vert(autopilot::Vert::Alt),
            KeyCode::Minus | KeyCode::Equal => {
                let d = if code == KeyCode::Minus { -5.0 } else { 5.0 };
                self.ap.hdg = (self.ap.hdg + d).rem_euclid(360.0);
                self.status = t!("Heading bug {:03.0}°", "Consigne de cap {:03.0}°", self.ap.hdg);
                self.status_until = self.real + 2.0;
            }
            KeyCode::Semicolon | KeyCode::Quote => {
                let d = if code == KeyCode::Semicolon { -100.0 } else { 100.0 };
                self.ap.alt = (((self.ap.alt * 3.28084 / 100.0).round() * 100.0 + d).max(0.0)) / 3.28084;
                self.status = t!("Altitude preselect {:.0} ft", "Altitude présélectionnée {:.0} ft", self.ap.alt * 3.28084);
                self.status_until = self.real + 2.0;
            }
            KeyCode::KeyK if !self.crashed => {
                self.gauges.sync_dg(&self.plane);
                self.status = t!("Heading indicator set to the compass", "Conservateur de cap recalé sur le compas").into();
                self.status_until = self.real + 2.0;
            }
            KeyCode::KeyU => {
                self.sound_on = !self.sound_on;
                self.status = if self.sound_on { t!("Sound on", "Son activé") } else { t!("Sound off", "Son coupé") }.into();
                self.status_until = self.real + 2.0;
            }
            KeyCode::F1 => self.show_help = !self.show_help,
            KeyCode::F2 => {
                    self.gfx.taa = !self.gfx.taa;
                    self.gfx.save();
                }
            KeyCode::F12 => {
                let p = PathBuf::from(format!("screenshot-{}.png", self.frame));
                self.shots.insert(0, (0.0, p, None));
            }
            _ => {}
        }
    }

    /// Esc or the gamepad's B on the start menu: a step back towards the
    /// main menu (which stays put: Quit is a button of its own).
    fn menu_back(&mut self) {
        use menu::Screen;
        match self.menu.screen {
            Screen::Main => return,
            Screen::Aircraft => self.aircraft_back(),
            Screen::Fly => self.menu.screen = Screen::Main,
            Screen::Settings => self.menu.screen = self.menu.set_back,
        }
        self.pad_home = true;
        self.egui_ctx.memory_mut(|m| {
            if let Some(id) = m.focused() {
                m.surrender_focus(id);
            }
        });
    }

    /// Open or close the pause menu. The game stops while it is open.
    fn set_pause_menu(&mut self, open: bool) {
        if open == self.pause_open || self.mode != Mode::Fly {
            return;
        }
        self.cancel_cockpit_input();
        self.pause_open = open;
        self.radial.open = false;
        self.pause_page = menu::Page::Game;
        self.pad_home = open;
        self.drag = false;
        self.show_help = false;
        // A button or slider left focused would keep the keys from the aircraft.
        self.egui_ctx.memory_mut(|m| {
            if let Some(id) = m.focused() {
                m.surrender_focus(id);
            }
        });
    }

    /// Back to the globe, looking at where we were.
    fn to_globe(&mut self) {
        self.set_pause_menu(false);
        self.info_open = false;
        let (lat, lon, _) = geo::pos_to_lla(self.plane.pos);
        self.menu.look_at(lat.to_degrees(), lon.to_degrees(), 60_000.0);
        self.menu.pick = Some((lat.to_degrees(), lon.to_degrees()));
        self.menu.hour = (self.utc + lon.to_degrees() / 15.0).rem_euclid(24.0);
        self.menu.starting = false;
        self.menu.screen = menu::Screen::Fly;
        self.mode = Mode::Menu;
    }

    /// A fresh aircraft in the air over where we are (or where we crashed), on the same heading.
    fn respawn_here(&mut self) {
        let (lat, lon, h) = geo::pos_to_lla(if self.crashed { self.crash.as_ref().map(|c| c.site).unwrap_or(self.plane.pos) } else { self.plane.pos });
        let g = self.world.ground(lat, lon).map(|g| g.h).unwrap_or(h);
        let hd = self.plane.forward().dot(geo::enu(self.plane.pos).0).atan2(self.plane.forward().dot(geo::enu(self.plane.pos).1));
        self.respawn(lat, lon, g + 420.0, hd);
    }

    /// Fly from a known spot: its ground height is looked up in the background,
    /// then the aircraft appears above it (see `goto_rx`).
    fn fly_to(&mut self, lat: f64, lon: f64, name: &str) {
        self.status = t!("Flying to {}…", "En route vers {}…", i18n::tr_owned(name));
        self.status_until = self.real + 30.0;
        let (tx, rx) = std::sync::mpsc::channel();
        let src = self.src.clone();
        let name = name.to_string();
        std::thread::spawn(move || {
            let _ = tx.send(Some((lat, lon, ground_elev(&src, lat, lon), name)));
        });
        self.goto_rx = Some(rx);
    }

    /// Input for the UI, minus Tab: that opens and closes the menu here, not
    /// egui's keyboard focus (which would then keep keys from the aircraft).
    fn egui_input(&mut self) -> egui::RawInput {
        // shrink the ui to fit short windows instead of letting panels overlap.
        let native = self.window.scale_factor() as f32;
        let h = self.window.inner_size().height as f32;
        let ppp = (h / UI_MIN_H).clamp(0.75, native.max(0.75));
        let zoom = ppp / native;
        if (self.egui_ctx.zoom_factor() - zoom).abs() > 1e-3 {
            self.egui_ctx.set_zoom_factor(zoom);
        }
        let mut raw = self.egui_state.take_egui_input(&self.window);
        if !self.egui_ctx.text_edit_focused() {
            raw.events.retain(|e| !matches!(e, egui::Event::Key { key: egui::Key::Tab, .. }));
        }
        raw.events.append(&mut self.pad_events);
        // The pad works the game whether or not the window has the keyboard,
        // and egui only lets a focused slider take the arrows in a focused window.
        if padui::active() {
            raw.focused = true;
        }
        raw
    }

    /// At the start of a UI pass: a gamepad move out of a text box (the box
    /// itself would keep the arrow keys).
    /// Text boxes are passed over: a pad cannot type, and a slider's number
    /// box would otherwise keep left and right from the slider. (A widget
    /// only shows as a text box the pass after it takes the focus.)
    fn pad_pass(&mut self, ctx: &egui::Context) {
        padui::begin_pass();
        if let Some(d) = self.pad_move.take() {
            ctx.memory_mut(|m| m.move_focus(d));
        }
        if let Some((d, stage, hops)) = self.pad_skip {
            self.pad_skip = if stage < 2 {
                Some((d, stage + 1, hops))
            } else if ctx.text_edit_focused() && hops < 6 {
                ctx.memory_mut(|m| m.move_focus(d));
                Some((d, 1, hops + 1))
            } else {
                None
            };
        }
    }

    /// Whether this is the widget to pick out for the gamepad (once asked for).
    fn pad_home_focus(&mut self, resp: &egui::Response) {
        if self.pad_home && padui::active() {
            resp.request_focus();
            self.pad_home = false;
        }
    }

    /// Menu: choose a spot on the globe and look its name up in the background.
    fn pick(&mut self, lat: f64, lon: f64) {
        self.menu.pick = Some((lat, lon));
        self.menu.airport = None;
        self.menu.runway = None;
        self.menu.pick_name = match menu::Menu::near_city(lat, lon) {
            Some((name, d)) if d < 40_000.0 => tr(name).to_string(),
            Some((name, d)) if d < 400_000.0 => t!("{:.0} km from {}", "À {:.0} km {}", d / 1000.0, if crate::i18n::is_fr() { crate::i18n::de_place(tr(name)) } else { tr(name).to_string() }),
            _ => String::new(),
        };
        let (tx, rx) = std::sync::mpsc::channel();
        let src = self.src.clone();
        std::thread::spawn(move || {
            let url = format!("https://nominatim.openstreetmap.org/reverse?format=json&zoom=12&accept-language={}&lat={lat:.5}&lon={lon:.5}", i18n::lang().code());
            let name = src.fetch.get(&url).ok().and_then(|d| serde_json::from_slice::<serde_json::Value>(&d).ok()).and_then(|j| j["display_name"].as_str().map(String::from));
            if let Some(n) = name {
                let _ = tx.send(n);
            }
        });
        self.name_rx = Some(rx);
    }

    /// Menu: choose an airport to start at.
    fn choose_airport(&mut self, i: u32) {
        let a = &airports::Db::get().apts[i as usize];
        self.menu.look_at(a.lat, a.lon, self.menu.alt.min(25_000.0));
        self.menu.pick = Some((a.lat, a.lon));
        self.menu.pick_name = format!("{} {} ({}, {})", a.code(), a.name, a.city, a.country);
        if self.menu.airport != Some(i) {
            self.menu.runway = None;
        }
        self.menu.airport = Some(i);
        self.name_rx = None;
    }

    /// Menu: take off at the chosen spot (once its ground height is known).
    fn begin(&mut self) {
        let Some((lat, lon)) = self.menu.pick else { return };
        if self.menu.starting {
            return;
        }
        self.set_aircraft(aircraft::nth(self.menu.aircraft));
        // remember the start for next time.
        let db = airports::Db::get();
        self.last_start = Some(settings::LastStart {
            lat,
            lon,
            name: self.menu.pick_name.clone(),
            airport: self.menu.airport.map(|i| db.apts[i as usize].ident.clone()),
            start: match self.menu.start {
                menu::Start::Runway => "runway",
                menu::Start::Approach => "approach",
                menu::Start::Overhead => "overhead",
            }
            .into(),
            runway: self.menu.runway.clone(),
            aircraft: aircraft::nth(self.menu.aircraft).keys.first().copied().unwrap_or("c172").into(),
        });
        self.keep_settings(true);
        if let Some(i) = self.menu.airport.filter(|_| self.menu.start != menu::Start::Overhead) {
            self.start_airport(i, self.menu.runway.clone(), self.menu.start == menu::Start::Approach, false);
            return;
        }
        self.menu.starting = true;
        let (tx, rx) = std::sync::mpsc::channel();
        let src = self.src.clone();
        std::thread::spawn(move || {
            let _ = tx.send((lat, lon, ground_elev(&src, lat, lon)));
        });
        self.start_rx = Some(rx);
    }

    /// Particles live in a frame anchored near the action; move it if the action has moved on.
    fn anchor_fx(&mut self, at: DVec3) {
        let stale = self.crash.as_ref().map(|c| c.time > 90.0).unwrap_or(true);
        if stale && (self.fx_anchor - at).length() > 800.0 {
            // Live particles are carried over into the new frame.
            self.fx_shift += self.fx_anchor - at;
            self.crash = None;
            self.fx_anchor = at;
        }
    }

    /// A weapon-system event: status messages, the launch sound, and a
    /// missile's detonation in the particle system.
    fn weapon_event(&mut self, ev: weapons::Event, femit: &mut Vec<(DVec3, render::Particle)>) {
        use weapons::Refusal;
        match ev {
            weapons::Event::Locked => {
                self.status = t!("Target locked", "Cible verrouillée").into();
                self.status_until = self.real + 2.0;
            }
            weapons::Event::Lost => {}
            weapons::Event::Fired { station } => {
                if let Some(a) = &self.audio {
                    a.event(audio::Event::Missile(1.0));
                }
                let name = self.weapons.station(station).map_or("AIM-9", |s| s.spec.name);
                self.flash.push((t!("MISSILE AWAY  {}", "MISSILE LANCÉ  {}", name), self.real));
            }
            weapons::Event::Refused(why) => {
                let msg = match why {
                    Refusal::NotArmed => t!("MASTER ARM OFF", "ARMEMENT MAÎTRE SUR OFF"),
                    Refusal::NoPower => t!("WEAPONS: NO POWER", "ARMES : PAS D'ALIMENTATION"),
                    Refusal::NoLock => t!("NO LOCK", "PAS DE VERROUILLAGE"),
                    Refusal::NoMissiles => t!("NO MISSILES", "PLUS DE MISSILES"),
                    Refusal::NoWeapons => t!("NO WEAPONS ON THIS AIRCRAFT", "PAS D'ARMES SUR CET AVION"),
                    Refusal::NoTarget => t!("NO TARGET", "PAS DE CIBLE"),
                };
                self.flash.push((msg.into(), self.real));
            }
            weapons::Event::Blast { pos, power, size, target } => {
                let near = 1.0 / (1.0 + ((self.cam_pos - pos).length() as f32 / 500.0).powi(2));
                if let Some(a) = &self.audio {
                    a.boom(power * (0.2 + 0.8 * near));
                }
                // The flash lights the aircraft for a moment and, if the hit
                // is close, gives the airframe a jolt.
                self.weapon_flash = Some((pos, power, self.real));
                if (self.cam_pos - pos).length() < 800.0 {
                    self.plane.dmg.vibe = self.plane.dmg.vibe.max(power * 1.5);
                }
                self.anchor_fx(pos);
                self.blast_fx(pos, power, size, femit);
                if let Some(key) = target {
                    let label = self.weapons.target_view.as_ref().map(|v| v.label.clone()).unwrap_or_default();
                    // Where it was and how it was moving, before it leaves the traffic.
                    let was = match &key {
                        weapons::TargetKey::Live(hex) => self.live.planes.iter().find(|p| &p.hex == hex).map(|p| (p.pos, p.vel)),
                        weapons::TargetKey::Airport { .. } => self.world.air_targets(pos).into_iter().find(|t| t.key == key).map(|t| (t.pos, t.vel)),
                    };
                    let hit = match &key {
                        weapons::TargetKey::Live(hex) => self.live.down(hex),
                        weapons::TargetKey::Airport { .. } => self.world.down_air_target(&key),
                    };
                    if hit {
                        let (at, vel) = was.unwrap_or((pos, DVec3::ZERO));
                        let span = airmodels::TYPES[(size as usize).min(4)].span;
                        self.kill_fx(at, vel, span, femit);
                        if let Some(a) = &self.audio {
                            a.boom((power * 1.5 * (span / 20.0).sqrt()).min(4.0) * (0.2 + 0.8 * near));
                        }
                        self.weapon_flash = Some((at, power * (span / 11.0).sqrt() * 2.0, self.real));
                        self.kills.push(Kill { key, pos: at, vel, span, t: 0.0, acc: 0.0 });
                        let msg = if label.is_empty() { t!("SPLASH", "ABATTU").to_string() } else { t!("SPLASH  {}", "ABATTU  {}", label) };
                        self.flash.push((msg, self.real));
                    }
                }
            }
        }
    }

    /// The fireball and smoke of a missile going off (or a target being hit).
    fn blast_fx(&mut self, at: DVec3, power: f32, size: f32, femit: &mut Vec<(DVec3, render::Particle)>) {
        let up = at.normalize();
        let p = power.max(0.3);
        let k = (0.7 + 0.15 * size).min(1.5);
        let r = &mut self.rng;
        femit.push((at, render::Particle { life: 0.3, size0: 12.0 * p * k, size1: 52.0 * p * k, kind: crash::FLASH, alpha: 2.0 * p, ..Default::default() }));
        // The fireball: overlapping hot blobs, thrown on with the wreckage.
        for _ in 0..30 {
            let d = r.dir();
            femit.push((at, render::Particle {
                vel: (d * r.range(14.0, 50.0) * p.sqrt() + up.as_vec3() * r.range(2.0, 11.0)).into(),
                life: r.range(1.8, 4.0),
                size0: r.range(2.5, 5.5) * p * k,
                size1: r.range(10.0, 24.0) * p * k,
                temp: r.range(2.2, 3.2),
                kind: crash::GAS,
                drag: r.range(1.2, 2.0),
                buoy: r.range(6.0, 12.0),
                seed: r.f(),
                alpha: r.range(0.75, 1.0),
                ..Default::default()
            }));
        }
        // Burning fragments and sparks tearing out through the fireball.
        for _ in 0..40 {
            let d = (r.dir() + up.as_vec3() * 0.6).normalize_or_zero();
            femit.push((at, render::Particle {
                vel: (d * r.range(20.0, 85.0) * p.sqrt()).into(),
                life: r.range(0.6, 2.2),
                size0: r.range(0.06, 0.16),
                size1: 0.03,
                kind: crash::SPARK,
                drag: r.range(0.3, 0.9),
                seed: r.f(),
                alpha: r.range(0.6, 1.0),
                ..Default::default()
            }));
        }
        // The smoke column left behind, and the fire under it, lingering.
        for _ in 0..20 {
            let d = r.dir();
            femit.push((at, render::Particle {
                vel: (d * r.range(3.0, 14.0) * p.sqrt() + up.as_vec3() * r.range(1.0, 6.0)).into(),
                life: r.range(6.0, 12.0),
                size0: r.range(3.0, 6.0) * p * k,
                size1: r.range(14.0, 32.0) * p * k,
                kind: crash::GAS,
                drag: r.range(0.7, 1.3),
                buoy: r.range(2.0, 6.0),
                seed: r.f(),
                alpha: r.range(0.35, 0.6),
                ..Default::default()
            }));
        }
    }

    /// An aircraft blowing up in the air: its fuel going up in one fireball
    /// sized to the airframe (`span`, m), with burning wreckage thrown out
    /// along its line of flight and a pall of black smoke left hanging.
    fn kill_fx(&mut self, at: DVec3, vel: DVec3, span: f32, femit: &mut Vec<(DVec3, render::Particle)>) {
        self.anchor_fx(at);
        let up = at.normalize().as_vec3();
        // Linear size from a light single (11 m span) up to a wide-body.
        let s = (span / 11.0).clamp(0.8, 6.0);
        let carry = vel.as_vec3() * 0.35;
        let r = &mut self.rng;
        femit.push((at, render::Particle { life: 0.45, size0: 18.0 * s, size1: 70.0 * s, kind: crash::FLASH, alpha: 2.5, ..Default::default() }));
        let n = (40.0 + 14.0 * s) as usize;
        for _ in 0..n {
            let d = r.dir();
            let off = d * r.range(0.0, 0.4) * span;
            femit.push((at + off.as_dvec3(), render::Particle {
                vel: (carry + d * r.range(10.0, 32.0) * s.sqrt() + up * r.range(2.0, 8.0)).into(),
                life: r.range(2.5, 5.0) * s.sqrt(),
                size0: r.range(3.0, 6.0) * s,
                size1: r.range(12.0, 26.0) * s,
                temp: r.range(2.4, 3.4),
                kind: crash::GAS,
                drag: r.range(1.0, 1.8),
                buoy: r.range(5.0, 10.0),
                seed: r.f(),
                alpha: r.range(0.8, 1.0),
                ..Default::default()
            }));
        }
        for _ in 0..(50.0 + 20.0 * s) as usize {
            let d = r.dir();
            femit.push((at, render::Particle {
                vel: (vel.as_vec3() * 0.6 + d * r.range(25.0, 110.0) * s.sqrt()).into(),
                life: r.range(1.5, 4.5),
                size0: r.range(0.1, 0.3) * s.sqrt(),
                size1: 0.05,
                kind: crash::SPARK,
                drag: r.range(0.1, 0.5),
                seed: r.f(),
                alpha: r.range(0.7, 1.0),
                ..Default::default()
            }));
        }
        for _ in 0..(20.0 + 8.0 * s) as usize {
            let d = r.dir();
            femit.push((at + (d * r.range(0.0, 0.5) * span).as_dvec3(), render::Particle {
                vel: (carry * 0.4 + d * r.range(2.0, 9.0) * s.sqrt() + up * r.range(0.5, 3.0)).into(),
                life: r.range(10.0, 20.0),
                size0: r.range(4.0, 8.0) * s,
                size1: r.range(20.0, 40.0) * s,
                temp: 0.05,
                kind: crash::GAS,
                drag: r.range(0.8, 1.4),
                buoy: r.range(1.0, 3.0),
                seed: r.f(),
                alpha: r.range(0.45, 0.7),
                ..Default::default()
            }));
        }
    }

    /// Shot-down aircraft on their way down: a fire and smoke trail behind
    /// each, and a second explosion where it hits the ground (or the sea).
    fn step_kills(&mut self, dt: f64, femit: &mut Vec<(DVec3, render::Particle)>) {
        let mut kills = std::mem::take(&mut self.kills);
        kills.retain_mut(|k| {
            k.t += dt;
            match &k.key {
                weapons::TargetKey::Live(hex) => match self.live.planes.iter().find(|p| &p.hex == hex) {
                    // The live feed tumbles it down itself.
                    Some(p) => {
                        k.pos = p.pos;
                        k.vel = p.vel;
                    }
                    None => return false,
                },
                _ => {
                    let down = k.pos.normalize();
                    k.vel = (k.vel - down * 9.81 * dt) * (-dt * 0.15).exp();
                    k.pos += k.vel * dt;
                }
            }
            let (la, lo, h) = geo::pos_to_lla(k.pos);
            let ground = if h < 0.0 { Some(0.0) } else { self.world.ground(la, lo).map(|g| g.h).filter(|g| h < g + 2.0) };
            if let Some(g) = ground {
                let at = geo::lla_to_pos(la, lo, g.max(0.0) + 2.0);
                self.kill_fx(at, DVec3::ZERO, k.span * 0.8, femit);
                if let Some(a) = &self.audio {
                    let near = 1.0 / (1.0 + ((self.cam_pos - at).length() as f32 / 500.0).powi(2));
                    a.boom((1.5 * (k.span / 20.0).sqrt()).min(3.0) * (0.2 + 0.8 * near));
                }
                self.weapon_flash = Some((at, 2.0 * (k.span / 11.0).sqrt(), self.real));
                if let weapons::TargetKey::Live(hex) = &k.key {
                    self.live.remove(hex);
                }
                return false;
            }
            if k.t > 60.0 {
                return false;
            }
            // The trail: a burning fuel fire close behind, black smoke after.
            let s = (k.span / 11.0).clamp(0.8, 6.0);
            let burn = (1.0 - k.t as f32 / 50.0).max(0.3);
            k.acc += dt * 30.0;
            self.anchor_fx(k.pos);
            let r = &mut self.rng;
            while k.acc >= 1.0 {
                k.acc -= 1.0;
                let back = k.pos - k.vel * dt * r.f() as f64;
                femit.push((back, render::Particle {
                    vel: (k.vel.as_vec3() * 0.15 + r.dir() * 2.0 * s.sqrt()).into(),
                    life: r.range(0.6, 1.4),
                    size0: r.range(1.5, 3.0) * s * burn,
                    size1: r.range(4.0, 8.0) * s * burn,
                    temp: r.range(2.0, 3.0),
                    kind: crash::GAS,
                    drag: 2.0,
                    buoy: 4.0,
                    seed: r.f(),
                    alpha: 0.9,
                    ..Default::default()
                }));
                femit.push((back, render::Particle {
                    vel: (k.vel.as_vec3() * 0.05 + r.dir() * 1.5).into(),
                    life: r.range(8.0, 16.0),
                    size0: r.range(2.0, 4.0) * s,
                    size1: r.range(10.0, 22.0) * s,
                    temp: 0.05,
                    kind: crash::GAS,
                    drag: 1.5,
                    buoy: r.range(0.5, 1.5),
                    seed: r.f(),
                    alpha: r.range(0.4, 0.6),
                    ..Default::default()
                }));
            }
            true
        });
        // Anything that went down this step was added while we held the list.
        kills.append(&mut self.kills);
        self.kills = kills;
    }

    /// Knock a building down, with any parts standing on it.
    /// `hit`: the aircraft itself went in (the wall shows its outline).
    /// `energy`: kinetic energy (J) of what hit it. A building only comes down
    /// as a whole when that is enough for its size (see `destruct::resistance`);
    /// less breaks out the part round the impact, and a mere fraction of it
    /// (a blast from the next lot, a lump of rubble) leaves a building alone.
    fn bring_down(&mut self, tile: geo::TileId, idx: usize, at: DVec3, vel: DVec3, energy: f64, hit: Option<destruct::Hitter>, femit: &mut Vec<(DVec3, render::Particle)>) {
        if hit.is_none() {
            if let Some((base, top, area)) = self.world.footprint(tile, idx) {
                let h = (top - base).max(3.0);
                if energy < 0.03 * destruct::resistance(h as f32, (area.max(25.0) as f64 * h) as f32) {
                    return;
                }
            }
        }
        let above = self.world.structure_of(tile, idx);
        let mut parts = Vec::new();
        for (t, i) in std::iter::once((tile, idx)).chain(above) {
            if let Some(w) = self.world.destroy_building(&self.r.queue, t, i) {
                parts.push(w);
            }
        }
        if parts.is_empty() {
            return;
        }
        // Under the 3D tiles the building is the mesh's: it is cut out of the
        // mesh, and its own walls and roof come down on the pieces, which
        // stand as high as it did.
        let rings = self.world.cut_rings(&parts, 2.0);
        let base = parts.iter().map(|w| w.base).fold(f64::MAX, f64::min);
        let photo = self.world.photo_mut().take_building(&rings, base);
        if let Some(src) = &photo {
            for (w, top) in parts.iter_mut().zip(&src.tops) {
                if let Some(t) = *top {
                    w.top = t.max(w.base + 3.0);
                    let top = w.top as f32;
                    w.tiers.retain(|z| z.1 < top - 1.0);
                }
            }
            log::info!("3D tiles: building cut out, {} tiles, {} triangles", src.tiles.len(), src.tiles.iter().map(|t| t.tris.len()).sum::<usize>());
        }
        self.destr.collapse(&parts, at, vel, energy, hit, photo.as_ref(), femit);
    }

    /// The wreck is made of what was left of the aircraft: every group's mesh
    /// is rebuilt without the pieces that had already gone, and cut along its
    /// breaks. Returns the break each group keeps showing.
    fn wreck_meshes(&mut self) -> Vec<[Tear; 2]> {
        let dm = self.plane.dmg;
        let mut out = vec![[Tear::NONE; 2]; airframe::N_GROUPS];
        for g in 0..airframe::N_GROUPS {
            let pieces: Vec<&airframe::Piece> = self.model.pieces.iter().filter(|p| p.group == g && !p.glass).collect();
            let changed = pieces.iter().any(|p| dm.hidden(p) || dm.tears_for(p).iter().any(|t| t.is_some()));
            if !changed {
                continue;
            }
            self.wreck_dirty = true;
            let mut m = airframe::Mesh::default();
            for p in &pieces {
                if dm.hidden(p) {
                    continue;
                }
                let tears = dm.tears_for(p);
                let primary = matches!(p.name, "left wing" | "right wing" | "stabiliser" | "blades");
                if primary {
                    out[g] = tears;
                }
                let base = m.v.len() as u32;
                m.v.extend_from_slice(&p.mesh.v);
                for t in p.mesh.i.chunks(3) {
                    let gone = t.iter().all(|&i| { let q = Vec3::from(p.mesh.v[i as usize].pos); tears.iter().any(|tr| !tr.keeps(q)) });
                    if !gone {
                        m.i.extend(t.iter().map(|i| i + base));
                    }
                }
            }
            if m.i.is_empty() {
                m = self.model.groups[g].clone();
            }
            self.r.replace_mesh(self.group_mesh[g], &m.v, &m.i);
        }
        out
    }
    fn restore_wreck_meshes(&mut self) {
        if self.wreck_dirty {
            for g in 0..airframe::N_GROUPS {
                let m = &self.model.groups[g];
                self.r.replace_mesh(self.group_mesh[g], &m.v, &m.i);
            }
            self.wreck_dirty = false;
        }
    }

    /// The cockpit warnings: stall, terrain ahead, nose dive, too low, sink
    /// rate, overspeed and gear up.
    fn warnings(&mut self) -> audio::Warn {
        let p = &self.plane;
        let mut w = audio::Warn::default();
        // (Nobody is aboard a rocket to be warned.)
        if self.crashed || self.mode == Mode::Menu || p.on_ground || p.spec.rocket().is_some() {
            return w;
        }
        let up = p.pos.normalize();
        let (la, lo, h) = geo::pos_to_lla(p.pos);
        let Some(g) = self.world.ground(la, lo) else { return w };
        let agl = h - g.h;
        if agl < 3.0 {
            return w;
        }
        let vs = p.vel.dot(up);
        let air = p.ias();
        let pitch = p.forward().dot(up).clamp(-1.0, 1.0).asin().to_degrees();
        if air > 8.0 {
            let sa = p.stall_alpha();
            w.stall = smoothstep(sa - 0.07, sa - 0.01, p.alpha.abs()) as f32;
        }
        let over = p.overspeed();
        if over > 0.0 {
            w.overspeed = (over / 13.0).clamp(0.1, 1.0) as f32;
        }
        // Going down nose first, steeply and fast.
        if vs < -10.0 {
            w.dive = smoothstep(-20.0, -55.0, pitch) as f32;
            if pitch < -22.0 || vs < -40.0 {
                w.dive = w.dive.max(0.2);
            }
        }
        // What is along the flight path in the next few seconds: ground, and
        // any building standing on it.
        let (mut tti, mut tower) = (f64::INFINITY, false);
        if p.vel.length() > 15.0 && agl < 1500.0 {
            for t in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0] {
                let (la, lo, h) = geo::pos_to_lla(p.pos + p.vel * t);
                let Some(g) = self.world.ground(la, lo) else { continue };
                let b = if h - g.h < 400.0 { self.world.building(la, lo).filter(|b| !b.bridge && h < b.top + 3.0 && h > b.base - 3.0) } else { None };
                if b.is_some() || h < g.h + 3.0 {
                    (tti, tower) = (t, b.is_some());
                    break;
                }
            }
        }
        // A gentle descent with the gear down is a landing, not a warning
        // (unless it is into a building).
        let landing = p.gear >= 1.0 && vs > -5.0 && !tower;
        let closing = if vs < -1.0 { agl / -vs } else { f64::INFINITY };
        let now = self.real;
        let trip = [
            !landing && (tti <= 5.0 || closing < 5.0),
            !landing && (tti <= 10.0 || (agl < 46.0 && vs < -1.5 && p.gear < 1.0)),
            !landing && agl < 600.0 && vs < -12.0,
            p.sys.powered() && p.gear < 1.0 && (agl < 150.0 && p.throttle < 0.35 || p.spec.id == 0 && p.flap_deg() > 20.0),
        ];
        for (t, on) in self.warn_t.iter_mut().zip(trip) {
            if on {
                *t = now;
            }
        }
        let held = |i: usize| now - self.warn_t[i] < 0.6;
        (w.pull_up, w.too_low, w.sink, w.gear) = (held(0), held(1), held(2), held(3));
        w
    }

    /// Tell the sound what the aircraft and the camera are doing.
    fn update_audio(&mut self, cam: DVec3, fire: f32, in_menu: bool, c: &Controls, dt: f64, scrape: f32) {
        self.warn = if dt > 0.0 { self.warnings() } else { audio::Warn::default() };
        let live = !self.crashed && !in_menu && dt > 0.0;
        if live {
            self.warn.seeker = self.weapons.seeker_tone;
            self.warn.ap_off = self.real - self.ap_off_t < 1.5;
        }
        let p = &self.plane;
        let up = p.pos.normalize();
        let vs = p.vel.dot(up);
        let air = p.airspeed().length();
        // Jolts: quick changes in load stand out from a slow-following copy.
        let dg = p.g_load - self.g_slow;
        self.g_slow += dg * (dt * 8.0).min(1.0);
        let jolt = if live && !p.on_ground { dg.abs() } else { 0.0 };
        let mut events = Vec::new();
        if live {
            if jolt > 0.35 && self.real - self.last_bump > 0.15 {
                self.last_bump = self.real;
                events.push(audio::Event::Bump(((jolt - 0.2) * 1.5) as f32));
            }
            if p.on_ground && !self.was_ground {
                events.push(audio::Event::Touch((-self.prev_vs).max(0.5) as f32));
            }
            if (p.gear - self.prev_gear).abs() < 0.1 {
                if p.gear >= 1.0 && self.prev_gear < 1.0 {
                    events.push(audio::Event::Clunk(1.0));
                } else if p.gear <= 0.0 && self.prev_gear > 0.0 {
                    events.push(audio::Event::Clunk(0.8));
                } else if (self.prev_gear >= 1.0 || self.prev_gear <= 0.0) && p.gear > 0.0 && p.gear < 1.0 {
                    events.push(audio::Event::Clunk(0.4));
                }
            }
        }
        let gear_moving = live && p.gear > 0.0 && p.gear < 1.0 && p.gear != self.prev_gear;
        let flaps_moving = live && p.flaps != self.prev_flaps;
        (self.was_ground, self.prev_vs, self.prev_gear, self.prev_flaps) = (p.on_ground, vs, p.gear, p.flaps);
        let sa = p.stall_alpha();
        let ground_speed = (p.vel - up * vs).length();
        // The sound was tuned on the light single: a jet's airflow is scaled to match.
        let jet = !matches!(p.spec.sound, aircraft::Sound::Piston | aircraft::Sound::Rocket);
        let sk = if jet { 0.45 } else { 1.0 };
        let params = audio::Params {
            live: if live { 1.0 } else { 0.0 },
            master: if self.sound_on { self.volume } else { 0.0 },
            rpm: (p.prop_rate * 60.0 / std::f64::consts::TAU) as f32,
            power: p.power as f32,
            throttle: p.throttle as f32,
            health: p.dmg.engine,
            prop: p.dmg.prop,
            airspeed: (if jet { p.ias() } else { air } * sk) as f32,
            vne: (p.spec.vne * sk) as f32,
            jet: if jet { 1.0 } else { 0.0 },
            n1: p.n1 as f32,
            fighter: match p.spec.sound { aircraft::Sound::Fighter => 1.0, aircraft::Sound::Turbojet => 2.0, _ => 0.0 },
            burner: p.burner as f32,
            four: if matches!(&p.spec.engine, aircraft::Engine::Fan(f) if f.n >= 4) { 1.0 } else { 0.0 },
            rocket: if matches!(p.spec.sound, aircraft::Sound::Rocket) { p.n1 as f32 } else { 0.0 },
            air: (plane::density(geo::pos_to_lla(p.pos).2) / plane::RHO0) as f32,
            g_load: p.g_load as f32,
            rough: p.air.sigma_w as f32,
            jolt: jolt as f32,
            buffet: if p.on_ground { 0.0 } else { (smoothstep(sa - 0.07, sa + 0.03, p.alpha.abs()) * smoothstep(10.0, 35.0, air)) as f32 },
            flutter: p.dmg.flutter,
            gear: p.gear as f32,
            gear_moving: if gear_moving { 1.0 } else { 0.0 },
            flaps: p.flaps as f32,
            flaps_moving: if flaps_moving { 1.0 } else { 0.0 },
            rolling: if p.on_ground { ground_speed as f32 } else { 0.0 },
            braking: if c.brake && p.on_ground { 1.0 } else { 0.0 },
            scrape,
            foliage: p.brushes.min(1.0) as f32,
            cracked: if p.dmg.crack[2] > 0.0 { 1.0 } else { 0.0 },
            cabin: if self.cam_mode == 1 && !self.crashed && !in_menu { 1.0 } else { 0.0 },
            dist: (cam - p.pos).length() as f32,
            fire,
            rain: if in_menu { 0.0 } else { self.cam_rain },
            hail: if in_menu { 0.0 } else { self.cam_hail },
            warn: self.warn,
        };
        if let Some(a) = &self.audio {
            for e in events {
                a.event(e);
            }
            a.set(params);
        }
    }

    /// The settings worth keeping, as they are now.
    fn live_settings(&self) -> settings::Settings {
        settings::Settings { lang: self.prefs_lang, sound_on: self.sound_on, volume: self.volume, cam_mode: self.cam_mode, cockpit_fov: self.cockpit_fov, look_sens: self.look_sens, show_hud: self.show_hud, hud_cockpit: self.hud_cockpit, minimap: self.minimap.visible, landing_guidance: self.minimap.landing_guidance, arcade: self.arcade, minimap_plan: self.minimap.plan, minimap_heading_up: self.minimap.heading_up, minimap_zoom: self.minimap.zoom, minimap_size: self.minimap.size.x, minimap_height: self.minimap.size.y, taa: self.gfx.taa, live_traffic: self.live.enabled(), live_weather: self.wx.live, real_time: self.wx.real_time, snow: self.snow, last: self.last_start.clone(), multiplayer: self.mp_on, mp_server: self.mp_server.clone(), mp_name: self.mp_name.clone() }
    }

    /// Saves the settings when they have changed (looked at once a second,
    /// so dragging a slider does not write the file every frame).
    fn keep_settings(&mut self, now: bool) {
        if std::env::args().any(|a| a == "--headless" || a == "--shot" || a == "--record") { return; }
        if !now && self.real - self.prefs_t < 1.0 {
            return;
        }
        self.prefs_t = self.real;
        let s = self.live_settings();
        if s != self.prefs {
            s.save();
            self.prefs = s;
        }
        let stamp = self.binds.stamp();
        if self.binds_stamp.is_empty() {
            self.binds_stamp = stamp;
        } else if stamp != self.binds_stamp {
            self.binds.save();
            self.binds_stamp = stamp;
        }
    }

    /// English / Français, as selectable buttons.
    fn language_picker(&mut self, ui: &mut egui::Ui) {
        let mut cur = i18n::lang();
        menu::segments(ui, |ui| {
            for l in i18n::Lang::ALL {
                menu::seg(ui, &mut cur, l, l.name());
            }
        });
        if cur != i18n::lang() {
            i18n::set_lang(cur);
            self.prefs_lang = Some(cur);
        }
    }

    fn frame(&mut self) {
        self.keep_settings(false);
        self.random_failures();
        let now = Instant::now();
        let mut dt_real = (now - self.last).as_secs_f64().min(0.05);
        if self.fps_log.0 {
            let l = &mut self.fps_log;
            l.2 += 1;
            l.3 = l.3.max((now - self.last).as_secs_f64());
            let el = (now - l.1).as_secs_f64();
            if el >= 1.0 {
                let n = l.2 as f64;
                let st = std::mem::take(&mut self.r.stats);
                log::info!("{:.1} fps avg ({:.1} ms), slowest frame {:.1} ms | game {:.1}  wait-gpu {:.1}  encode {:.1}  submit {:.1} ms", n / el, el / n * 1000.0, l.3 * 1000.0, st[0] / n * 1000.0, st[1] / n * 1000.0, st[2] / n * 1000.0, st[3] / n * 1000.0);
                (l.1, l.2, l.3) = (now, 0, 0.0);
            }
        }
        if self.record.is_some() {
            // Frozen for a few wall-clock seconds so the city streams in first.
            dt_real = if self.rec_t0.elapsed().as_secs_f64() < 8.0 { 0.0 } else { 1.0 / REC_FPS };
        }
        self.last = now;
        self.real += dt_real;
        self.frame += 1;
        self.pump_mcp();
        if self.record.is_none() {
            self.fps += (1.0 / dt_real.max(1e-4) - self.fps) * 0.05;
        }

        // Results of background lookups.
        if let Some(rx) = &self.goto_rx {
            if let Ok(res) = rx.try_recv() {
                self.goto_rx = None;
                match res {
                    Some((lat, lon, elev, name)) if self.mode == Mode::Menu => {
                        self.menu.look_at(lat, lon, 45_000.0);
                        self.menu.pick = Some((lat, lon));
                        self.menu.pick_name = name;
                        let _ = elev;
                    }
                    Some((lat, lon, elev, name)) => {
                        let local = (self.utc + geo::pos_to_lla(self.plane.pos).1.to_degrees() / 15.0).rem_euclid(24.0);
                        self.utc = (local - lon / 15.0).rem_euclid(24.0);
                        self.respawn(lat.to_radians(), lon.to_radians(), elev + 450.0, 0.0);
                        self.status = name;
                    }
                    None => {
                        self.status = t!("Place not found", "Lieu introuvable").into();
                        self.menu.pick_name = t!("Place not found", "Lieu introuvable").into();
                    }
                }
                self.status_until = self.real + 8.0;
            }
        }
        if let Some(rx) = &self.name_rx {
            if let Ok(name) = rx.try_recv() {
                self.menu.pick_name = name;
                self.name_rx = None;
            }
        }
        if let Some(rx) = &self.start_rx {
            if let Ok((lat, lon, elev)) = rx.try_recv() {
                self.start_rx = None;
                self.utc = (self.menu.hour - lon / 15.0).rem_euclid(24.0);
                self.respawn(lat.to_radians(), lon.to_radians(), elev + self.menu.start_agl, self.heading);
                self.status = self.menu.pick_name.clone();
                self.status_until = self.real + 10.0;
                self.loading = Some(Loading::new(self.real));
                self.mode = Mode::Fly;
                self.menu.starting = false;
            }
        }
        if let Some(rx) = &self.airport_rx {
            if let Ok((idx, g, rw, appr, keep)) = rx.try_recv() {
                self.airport_rx = None;
                self.airport_ready(idx, g, rw, appr, keep);
            }
        }
        if let Some((idx, rw, appr)) = self.pending_airport.take() {
            self.start_airport(idx, rw, appr, false);
        }
        // Released once the ground under the wheels is there in full detail.
        if let Some((lat, lon, until)) = self.hold_spot {
            let fine = self.world.ground_zoom(lat, lon).is_some_and(|z| z >= self.world.max_zoom.min(16));
            if (fine && self.real > until - 28.0) || self.real > until {
                if let Some(gr) = self.world.ground(lat, lon) {
                    let up = self.plane.pos.normalize();
                    self.plane.pos = up * (geo::R + gr.h + self.plane.stand_height());
                }
                self.plane.hold = false;
                self.plane.t = 2.0;
                self.hold_spot = None;
            }
        }
        if let Some(t) = self.menu_go {
            if self.real > t && self.mode == Mode::Menu && !self.menu.starting {
                self.pick(self.menu.lat, self.menu.lon);
                self.begin();
                self.menu_go = None;
            }
        }
        let in_menu = self.mode == Mode::Menu;

        if self.radial_at.is_some_and(|t| self.real >= t && self.mode == Mode::Fly) {
            self.flight_menu_key();
            self.radial_at = None;
        }
        if let Some((page, sec, t)) = self.pause_at {
            if self.real >= t && self.mode == Mode::Fly {
                if page.is_info() {
                    (self.info_open, self.info_page) = (true, page);
                } else {
                    self.set_pause_menu(true);
                    self.pause_page = page;
                    if let Some(s) = sec {
                        self.settings_sec = s;
                    }
                }
                self.pause_at = None;
            }
        }
        if !in_menu {
            self.update_loading();
        }
        let loading = self.loading.as_ref().is_some_and(|l| l.done.is_none());
        let scale = if self.paused || in_menu || self.pause_open || self.real < self.hold || loading { 0.0 } else { 1.0 };
        let dt = dt_real * scale;
        self.time += dt;
        let mut glass_rain = 0.0f32;
        if !in_menu {
            // The clock runs at the panel's rate; past midnight the date turns over.
            let t = self.utc + dt * self.wx.time_rate / 3600.0;
            self.doy = (self.doy + (t / 24.0).floor()).rem_euclid(365.0);
            self.utc = t.rem_euclid(24.0);
            if self.wx.real_time {
                (self.utc, self.doy) = weather::real_clock();
                self.wx.time_rate = 1.0;
            }
            // The air the aircraft flies through, gusts and all.
            let (la, lo, h) = geo::pos_to_lla(self.plane.pos);
            let g = self.world.ground(la, lo);
            let gh = g.as_ref().map(|g| g.h);
            let agl = h - gh.unwrap_or(0.0);
            let water = g.map(|g| g.water).unwrap_or(false);
            self.wx.sun = self.sun_dir().dot(self.plane.pos.normalize());
            // Turbulence, thermals and storm drafts ride on top of the gusting wind.
            let air = self.wx.air(self.plane.pos, agl, self.time, water);
            self.plane.air = air;
            self.plane.wind = self.wx.at(self.plane.pos, agl, self.time) + self.plane.pos.normalize() * air.draft + air.gust;
            // The rain on the windows: only under (or in) a cloud that rains, the
            // rain deck the slider sets or a storm cell.
            self.wx.step_deck(dt, self.plane.pos, gh);
            self.wx.rain = self.wx.rain_at(self.plane.pos, agl, self.time) as f32;
            self.wx.advance(dt, self.plane.pos);
            // (Snow does not wet the glass or streak across it: only the liquid rain share does.)
            let fz = match (self.snow, self.wx.freezing) {
                (snow::Mode::Always, _) | (_, None) => snow::freezing_guess(self.snow, la.to_degrees(), self.doy),
                (_, Some(f)) => f,
            };
            glass_rain = self.wx.rain * (1.0 - snow::snow_share(h, fz) as f32);
            self.glass.step(dt, glass_rain, self.plane.ias() * 1.94384);
        }

        // Physics at a fixed step.
        let c = self.controls(dt_real);
        let mut emit = Vec::new();
        let mut femit: Vec<(DVec3, render::Particle)> = Vec::new();
        let (mut scrape_n, mut steps) = (0usize, 0usize);
        if !self.crashed && !in_menu {
            let weapon_targets = if !self.missiles_ok() {
                Vec::new()
            } else {
                let mut t = self.live.targets();
                t.extend(self.world.air_targets(self.plane.pos));
                t
            };
            self.acc += dt;
            let h = 1.0 / 180.0;
            while self.acc >= h {
                self.acc -= h;
                let imp = self.plane.step(h, &c, &self.world);
                let powered = self.plane.sys.powered();
                for ev in self.weapons.step(h, &self.plane, &weapon_targets, &self.world, powered) {
                    self.weapon_event(ev, &mut femit);
                }
                self.gauges.update(h, &self.plane, self.time);
                steps += 1;
                scrape_n += self.plane.sparks.len();
                // Anything the aircraft ploughed through goes over in its direction of travel.
                if !self.plane.tree_hits.is_empty() {
                    let hits: Vec<_> = self.plane.tree_hits.drain(..).collect();
                    self.anchor_fx(hits[0].pos);
                    for t in hits {
                        self.destr.knock(&mut self.world, &self.r.queue, &t, self.plane.vel, 1.3, 0.0, 0.0);
                    }
                    if let Some(a) = &self.audio {
                        a.event(audio::Event::Crunch((self.plane.vel.length() / 50.0).clamp(0.4, 1.6) as f32));
                    }
                }
                // Parts that came off: let go of them where the aircraft is now.
                if !self.plane.shed.is_empty() {
                    self.anchor_fx(self.plane.pos);
                    let pose = self.plane.pose(&self.model.pieces);
                    let reqs: Vec<_> = self.plane.shed.drain(..).collect();
                    for req in &reqs {
                        self.sheds.spawn(req, &self.plane.dmg_prev, &self.model.pieces, &self.piece_mesh, &self.piece_cut, &pose, self.plane.model_pos(), self.plane.rot, self.plane.vel, &mut self.rng, &mut femit);
                    }
                }
                for n in self.plane.notes.drain(..) {
                    self.flash.push((n, self.real));
                    if let Some(a) = &self.audio {
                        a.event(audio::Event::Snap(1.0));
                    }
                }
                if !self.plane.splashes.is_empty() {
                    self.anchor_fx(self.plane.pos);
                    for (p, v) in self.plane.splashes.drain(..) {
                        if self.rng.f() < 0.25 {
                            let r = &mut self.rng;
                            femit.push((p, render::Particle { vel: (v.as_vec3() + r.dir() * 2.0).into(), life: r.range(1.0, 2.5), size0: r.range(0.15, 0.35), size1: r.range(0.5, 1.2), kind: crash::DROPS, drag: r.range(0.2, 0.8), seed: r.f(), alpha: r.range(0.3, 0.55), ..Default::default() }));
                        }
                    }
                }
                if !self.plane.snow_marks.is_empty() {
                    if self.snow != snow::Mode::Off {
                        for m in self.plane.snow_marks.drain(..) {
                            self.snowtrack.stamp(m);
                        }
                    } else {
                        self.plane.snow_marks.clear();
                    }
                }
                if !self.plane.dirt.is_empty() {
                    self.anchor_fx(self.plane.pos);
                    let up = self.plane.pos.normalize().as_vec3();
                    let field = self.world.airfield_near(self.plane.pos, 3_000.0);
                    let snowy = self.snowy_at(self.plane.pos);
                    for (p, v, amt) in self.plane.dirt.drain(..) {
                        // Paved runways throw up nothing; dirt, grass and fields do.
                        if field.is_some_and(|f| f.geo.on_runway(f.geo.local(p), 2.0).is_some_and(|ri| f.geo.runways[ri].paved)) {
                            continue;
                        }
                        let speed = v.length() as f32;
                        let n = amt * (speed / 8.0).clamp(0.3, 1.5) * 0.2;
                        let r = &mut self.rng;
                        let mut k = n;
                        while k > 0.0 && r.f() < k {
                            k -= 1.0;
                            let big = amt > 1.5;
                            let vel = v.as_vec3() * r.range(0.6, 1.2) + r.dir() * (1.0 + speed * 0.15) + up * r.range(0.5, 2.5);
                            let kind = if snowy && (!big || r.f() < 0.7) { crash::MIST } else { crash::DUST };
                            femit.push((p, render::Particle { vel: vel.into(), life: r.range(1.5, 4.0) * if big { 1.5 } else { 1.0 }, size0: r.range(0.3, 0.7), size1: r.range(2.0, 4.0) * if big { 1.6 } else { 1.0 }, kind, drag: 1.4, buoy: 0.15, seed: r.f(), alpha: r.range(0.45, 0.75), ..Default::default() }));
                        }
                    }
                }
                if !self.plane.sparks.is_empty() {
                    self.anchor_fx(self.plane.pos);
                    let up = self.plane.pos.normalize().as_vec3();
                    for (p, v) in self.plane.sparks.drain(..) {
                        if self.rng.f() < 0.3 {
                            let r = &mut self.rng;
                            femit.push((p, render::Particle { vel: (v.as_vec3() + r.dir() * 3.0 + up * 1.5).into(), life: r.range(0.3, 0.9), size0: 0.05, size1: 0.02, kind: crash::SPARK, drag: 0.6, seed: r.f(), alpha: 0.9, ..Default::default() }));
                        }
                    }
                }
                if let Some(mut imp) = imp {
                    self.r.clear_particles();
                    let tears = self.wreck_meshes();
                    // A house is no match for an airliner: it goes through
                    // it and on, slowed about as much as the house weighs.
                    let size = (self.spec.span / 5.5).max(1.0);
                    let heavy = size > 2.0;
                    // What it brings to a building: its kinetic energy.
                    let e_hit = 0.5 * self.spec.mass * imp.vel.length_squared();
                    let mut through = 0.0f32;
                    if let Some((tile, idx)) = imp.building.filter(|_| heavy) {
                        if let Some((base, top, area)) = self.world.footprint(tile, idx) {
                            let mb = area.max(25.0) as f64 * (top - base).max(3.0) * 120.0 * 0.3;
                            let keep = self.spec.mass / (self.spec.mass + mb);
                            if keep > 0.55 {
                                through = keep as f32;
                                // (the blast stays where the aircraft went up, up the
                                // side of the house; only a low hit is at street level)
                                let (la, lo, alt) = geo::pos_to_lla(imp.pos);
                                let g = self.world.ground(la, lo).map(|g| g.h).unwrap_or(base);
                                if alt - g < 6.0 {
                                    imp.ground_h = g;
                                }
                            }
                        }
                    }
                    let mut crash = Crash::new(&imp, &self.plane, &self.group_mesh, &self.model.groups, &self.group_pivots, &tears, through, self.rng.0 ^ self.frame, self.wx.steady(imp.pos, 40.0));
                    self.r.fluid.latest = None;
                    let (site, power) = (crash.site, crash.power);
                    self.fx_anchor = site;
                    let mut gone = 0.0;
                    // How far the blast reaches beyond a light aircraft's.
                    let reach = if heavy { size.powf(0.6) } else { 1.0 };
                    if let Some((tile, idx)) = imp.building {
                        // The building that was hit comes down, and any shed or small house right beside it.
                        let (la, lo, alt) = geo::pos_to_lla(site);
                        if let Some(g) = self.world.ground(la, lo) {
                            crash.set_lift(alt - g.h);
                        }
                        let d = &self.plane.dmg;
                        let hit = destruct::Hitter { right: self.plane.right(), up: self.plane.up(), span: d.wing, shape: &self.spec.hole, full: self.spec.span, stab: d.stab, fin: d.fin };
                        self.bring_down(tile, idx, imp.pos, imp.vel, e_hit, Some(hit), &mut femit);
                        // (next door only gets the blast and the flying pieces: a hundredth of it)
                        for (t, i) in self.world.buildings_near(site, 9.0 * (power * reach) as f64, if heavy { 700.0 } else { 160.0 }) {
                            self.bring_down(t, i, imp.pos, imp.vel * 0.3, e_hit * 0.01, None, &mut femit);
                        }
                    } else if !imp.water {
                        // Open ground: a bite is taken out of it (a big one
                        // where an airliner came down steeply).
                        let steep = (-imp.vel.dot(site.normalize()) / imp.vel.length().max(1.0)).clamp(0.0, 1.0) as f32;
                        let r = (3.0 + 5.0 * power) * (1.0 + (size.sqrt() - 1.0) * steep);
                        self.world.add_crater(site, r);
                        if self.snow != snow::Mode::Off {
                            let upw = site.normalize();
                            let tang = imp.vel - upw * imp.vel.dot(upw);
                            let along = tang.normalize_or_zero();
                            let skid_len = (r as f64 * (1.8 - 1.1 * steep as f64)).max(0.0);
                            self.snowtrack.stamp(snowtrack::Mark {
                                a: site - along * (r as f64 * 0.5),
                                b: site + along * skid_len,
                                radius: r * 1.35,
                                clear: 1.0,
                                berm: 1.0,
                                rough: 1.0,
                            });
                        }
                        self.destr.eject(site, r, imp.vel, &mut femit);
                        gone = r as f64;
                        for (t, i) in self.world.buildings_near(site, r as f64 + 4.0 * reach as f64, if heavy { 900.0 } else { 400.0 }) {
                            self.bring_down(t, i, imp.pos, imp.vel * 0.3, e_hit * 0.01, None, &mut femit);
                        }
                    }
                    if !imp.water {
                        // A fireball flattens the trees around; without one only those
                        // the wreck itself ploughs into go down.
                        let blast = if crash.ignited { 26.0 + 30.0 * power as f64 } else { 6.0 + 4.0 * power as f64 };
                        self.destr.blast_trees(&mut self.world, &self.r.queue, site, blast * reach as f64, gone, &mut femit);
                    }
                    self.crash = Some(crash);
                    self.crashed = true;
                    self.crash_real = self.real;
                    self.orbit = 0.0;
                    // Watch from back along the way it came in: that air is
                    // clear (it just flew through it), so the camera never
                    // starts inside the building or behind it.
                    {
                        let (e, n, u) = geo::enu(site);
                        let v = imp.vel - u * imp.vel.dot(u);
                        let back = if v.length() > 1.0 { -v.normalize() } else { -self.plane.forward() };
                        let a = back.dot(n).atan2(back.dot(e));
                        self.cam_ang = Some(a);
                        let dist = (70.0 + 34.0 * power as f64) * (self.crash.as_ref().map_or(1.0, |c| c.size) as f64).sqrt() + self.cam_dist;
                        let horiz = e * a.cos() + n * a.sin();
                        self.cam_pos = site + horiz * dist * 0.6 + u * (12.0 + dist * 0.15);
                        self.cam_look = site;
                    }
                    self.cam_clear_t = 0.0;
                    self.cam_rise = 0.0;
                    self.acc = 0.0;
                    break;
                }
            }
        }
        if dt > 0.0 && !self.kills.is_empty() {
            self.step_kills(dt, &mut femit);
        }
        if dt > 0.0 {
            self.destr.update(dt as f32, &self.world, &mut femit);
            // Buildings next door that heavy falling pieces went through.
            let hits: Vec<_> = self.destr.crush.drain(..).collect();
            for (t, i, p, v, e) in hits {
                self.bring_down(t, i, p, v * 0.4, e, None, &mut femit);
                self.destr.booms.push(0.5);
            }
            self.sheds.update(dt as f32, &self.world);
            if !self.sheds.snow_marks.is_empty() {
                if self.snow != snow::Mode::Off {
                    for m in self.sheds.snow_marks.drain(..) {
                        self.snowtrack.stamp(m);
                    }
                } else {
                    self.sheds.snow_marks.clear();
                }
            }
        }
        let booms: Vec<f32> = self.destr.booms.drain(..).collect();
        if !booms.is_empty() {
            let near = self.crash.as_ref().map(|cr| 1.0 / (1.0 + ((self.cam_pos - cr.site).length() as f32 / 400.0).powi(2))).unwrap_or(0.5);
            for b in booms {
                if let Some(a) = &self.audio {
                    a.boom(b * (0.4 + 0.6 * near));
                }
            }
        }
        // Smoke from a damaged engine: oil-white while it still runs, black once it has seized.
        let sm = self.plane.dmg.smoke;
        if sm > 0.0 && dt > 0.0 && !self.crashed && !in_menu {
            self.anchor_fx(self.plane.pos);
            self.smoke_acc += sm * 22.0 * dt as f32;
            let exhaust = self.plane.pos + self.plane.rot * DVec3::new(0.335, -0.75, -2.10);
            let up = self.plane.pos.normalize().as_vec3();
            let fwd = self.plane.forward().as_vec3();
            let pv = self.plane.vel.as_vec3();
            let black = self.plane.dmg.smoke_black;
            while self.smoke_acc >= 1.0 {
                self.smoke_acc -= 1.0;
                let r = &mut self.rng;
                femit.push((exhaust, render::Particle { vel: (pv * 0.3 - fwd * 2.0 + r.dir() * 1.5 + up * 0.5).into(), life: r.range(2.0, 4.5), size0: r.range(0.25, 0.5), size1: r.range(2.5, 5.0), temp: 0.05, kind: if black { crash::GAS } else { crash::MIST }, drag: 1.2, buoy: 0.8, seed: r.f(), alpha: r.range(0.25, 0.45) * sm.clamp(0.4, 1.0), ..Default::default() }));
            }
        }
        let mut fire = 0.0f32;
        let crash_snowy = self.crash.as_ref().is_some_and(|cr| !cr.water && self.snowy_at(cr.site));
        if let Some(cr) = self.crash.as_mut() {
            if dt > 0.0 {
                // Smoke leaving the top of the fire simulation carries on as particles.
                cr.top_out = match (&cr.domain, self.r.fluid.latest) {
                    (Some(dm), Some(st)) if st.out > 1e-3 => {
                        // Just outside the face it leaves through.
                        let f = st.out_c;
                        let edge = |v: f32| if v < 0.08 { -0.01 } else if v > 0.92 { 1.01 } else { v };
                        let fr = Vec3::new(edge(f.x), edge(f.y), if f.z > 0.92 { 1.01 } else { f.z });
                        let at = dm.at_frac(fr) - cr.site;
                        Some((at.as_vec3(), st.out * 1.8 / 110.0, 5.0))
                    }
                    _ => None,
                };
                cr.update(dt, &self.world, crash_snowy, &mut emit);
            }
            if !cr.snow_marks.is_empty() {
                if self.snow != snow::Mode::Off {
                    for m in cr.snow_marks.drain(..) {
                        self.snowtrack.stamp(m);
                    }
                } else {
                    cr.snow_marks.clear();
                }
            }
        }
        // What the wreckage ploughed through on its way comes down.
        let crushed: Vec<_> = self.crash.as_mut().map(|cr| cr.crush.drain(..).collect()).unwrap_or_default();
        for (t, i, p, v, e) in crushed {
            self.bring_down(t, i, p, v * 0.5, e, None, &mut femit);
            self.destr.booms.push(0.6);
            if let Some(a) = &self.audio {
                a.event(audio::Event::Crunch(1.4));
            }
        }
        if let Some(cr) = self.crash.as_mut() {
            if let Some(a) = &self.audio {
                let near = 1.0 / (1.0 + ((self.cam_pos - cr.site).length() as f32 / 250.0).powi(2));
                for p in &cr.booms {
                    a.boom(p * (0.35 + 0.65 * near));
                }
                for p in &cr.splashes {
                    a.event(audio::Event::Splash(p * (0.35 + 0.65 * near)));
                }
                let f = if cr.water { 0.0 } else { (cr.light().min(12.0) / 12.0) * near };
                fire = (f + self.destr.rumble.min(1.0) * near) * scale as f32;
            }
            cr.booms.clear();
            cr.splashes.clear();
        }
        for (pos, mut p) in femit {
            p.pos = (pos - self.fx_anchor).as_vec3().into();
            emit.push(p);
        }
        // Smoke and dust that would start inside the fire simulation's grid go
        // into the grid instead, where they can mix and flow with the rest.
        if let Some(cr) = self.crash.as_mut().filter(|_| dt > 0.0) {
            if let Some(dm) = &cr.domain {
                let off = (self.fx_anchor - cr.site).as_vec3();
                let mut conv = Vec::new();
                emit.retain(|p| {
                    let k = p.kind as i32;
                    let pos = Vec3::from(p.pos) + off;
                    if matches!(k, 0 | 2 | 7) && dm.contains(pos, 2.0) {
                        conv.push((pos, *p));
                        false
                    } else {
                        true
                    }
                });
                let keep = 48usize;
                let scale = (conv.len() as f32 / keep as f32).max(1.0);
                let step = scale.ceil() as usize;
                for (pos, p) in conv.iter().step_by(step.max(1)) {
                    let r = (p.size1 * 0.55).clamp(1.5, 7.0);
                    let amt = p.alpha * 1.2 / r * scale / dt as f32;
                    let k = p.kind as i32;
                    cr.fluid.push(fluid::FluidSrc {
                        pos: *pos,
                        radius: r,
                        vel: Vec3::from(p.vel),
                        radial: 1.0,
                        soot: if k == 0 { amt * 0.5 } else { 0.0 },
                        dust: if k == 0 { 0.0 } else { amt },
                        temp: if k == 0 { p.temp.min(1.2) } else { 0.0 },
                        vel_k: 2.5,
                        temp_k: 4.0,
                        ..Default::default()
                    });
                }
            }
        }
        if !emit.is_empty() {
            self.r.emit(&emit);
        }
        // Missiles leave a trail of smoke (and a little flame on the boost).
        if !in_menu && !self.weapons.missiles.is_empty() {
            let mut trail = Vec::new();
            for m in &self.weapons.missiles {
                let pos = (m.pos - self.fx_anchor).as_vec3();
                let back = (m.rot * DVec3::Z).as_vec3();
                let n = if m.burn > 0.0 { 2 } else { 1 };
                for _ in 0..n {
                    let r = &mut self.rng;
                    let drift = r.dir() * r.range(0.4, 1.6);
                    trail.push(render::Particle {
                        pos: pos.into(),
                        vel: (back * r.range(3.0, 8.0) + drift).into(),
                        life: r.range(1.2, 3.0),
                        size0: if m.burn > 0.0 { 0.18 } else { 0.28 },
                        size1: r.range(1.2, 3.2),
                        temp: if m.burn > 0.0 { 0.15 } else { 0.0 },
                        kind: crash::MIST,
                        drag: 1.0,
                        buoy: 0.4,
                        seed: r.f(),
                        alpha: 0.4,
                        ..Default::default()
                    });
                }
            }
            self.r.emit(&trail);
        }

        // ---- camera
        self.apply_graphics();
        let size = self.window.inner_size();
        let (w, h) = (size.width.max(1) as f32, size.height.max(1) as f32);
        // the size the scene is drawn at (graphics settings: render resolution)
        let (rw, rh) = { let (a, b) = self.r.render_size(); (a as f32, b as f32) };
        let mut fov = 60f32.to_radians();
        let (cam, dir, upv);
        if !(self.cam_mode == 1 && !self.crashed && !in_menu) {
            self.head_live = false;
        }
        if in_menu {
            (cam, dir, upv) = if self.menu.screen == menu::Screen::Fly {
                self.menu.update(dt_real);
                self.menu.camera()
            } else {
                // Off the globe (the main menu, the aircraft and settings
                // screens) the camera turns from the planet to space: level,
                // square to the sun, so neither the planet nor the sun is in
                // view and little but the stars is drawn.
                let (cam, down, _) = self.menu.camera();
                let up = -down;
                let side = up.cross(self.sun_dir());
                let dir = if side.length() > 0.2 { side.normalize() } else { geo::enu(cam).1 };
                (cam, dir, up)
            };
            // The globe's sun is where it is now (the terminator, the night
            // side's lights), unless the start time has been set by hand: then
            // that time at the chosen spot (or under the camera).
            let lon = self.menu.pick.map(|p| p.1).unwrap_or(self.menu.lon);
            if self.wx.real_time {
                // Real time: the start time is the time now at the chosen spot.
                let (utc, doy) = weather::real_clock();
                self.menu.hour = (utc + lon / 15.0).rem_euclid(24.0);
                (self.utc, self.doy) = (utc, doy);
            } else {
                if self.menu_time_seen.0.is_finite() && (self.menu.hour, self.doy) != self.menu_time_seen {
                    self.menu_time_set = true;
                }
                if self.menu_time_set {
                    self.utc = (self.menu.hour - lon / 15.0).rem_euclid(24.0);
                } else {
                    (self.utc, self.doy) = weather::real_clock();
                }
            }
            self.menu_time_seen = (self.menu.hour, self.doy);
            self.cam_pos = cam;
        } else if self.crashed {
            let cr = self.crash.as_ref().unwrap();
            // After an impact up the side of a building, the wreck ends up in the street below.
            // While the fire burns up at the hole (6 s, see `Crash::set_lift`),
            // watch it there; then follow the wreck down to the street.
            let down = { let x = ((cr.time - 5.0) / 4.0).clamp(0.0, 1.0); x * x * (3.0 - 2.0 * x) };
            let street = cr.site - cr.up * (cr.lift * down as f64);
            let mut target = cr.debris.first().filter(|d| !d.sunk).map(|d| d.pos).unwrap_or(street) * 0.5 + street * 0.5 + cr.up * 6.0;
            // (an airliner's fireball is a few hundred metres across: further back)
            let mut dist = (70.0 + 34.0 * cr.power as f64) * (cr.size as f64).sqrt() + self.cam_dist;
            // (into the water there is no fireball to take in: once the
            // spray has come down, closer, to see the wreck afloat)
            if cr.water {
                let settle = ((cr.time - 4.0) / 8.0).clamp(0.0, 1.0);
                let k = settle * settle * (3.0 - 2.0 * settle);
                dist = (38.0 + 14.0 * cr.power as f64) * (cr.size as f64).sqrt() * (1.0 - 0.7 * k) + self.cam_dist * (1.0 - 0.5 * k);
            }
            let (mut centre, mut rise) = (street, 18.0 + dist * 0.24);
            // An airliner's wreck slides a long way: watch the middle of the
            // trail it leaves, and the fireball over it.
            if let Some(w) = cr.wreck_centre() {
                let w = w - cr.up * (w - street).dot(cr.up);
                if cr.water {
                    // Afloat: watch the wreck from low over the water as it
                    // drifts off from the splash.
                    target = w * 0.85 + street * 0.15 + cr.up * (1.5 * (cr.size as f64).sqrt());
                    centre = w * 0.8 + street * 0.2;
                    rise = (6.0 + dist * 0.18) * (cr.size as f64).sqrt().min(2.0);
                } else {
                    target = w * 0.6 + street * 0.4 + cr.up * (6.0 * (cr.size as f64).sqrt());
                    centre = w * 0.5 + street * 0.5;
                }
            }
            // A tower coming down: back off and watch all of it from the side.
            if let Some((mid, hgt)) = self.destr.focus().filter(|f| (f.0 - cr.site).length() < 800.0) {
                target = target * 0.25 + mid * 0.75;
                dist = dist.max(hgt * 1.2 + 60.0 + self.cam_dist);
                centre = mid;
                rise = dist * 0.1;
            }
            self.cam_look += (target - self.cam_look) * (dt_real * 3.0).min(1.0);
            self.orbit += dt_real * 0.1;
            let (e, n, u) = geo::enu(cr.site);
            let back = (self.cam_pos - centre).normalize_or_zero();
            let mut a = self.cam_ang.unwrap_or_else(|| back.dot(n).atan2(back.dot(e))) + dt_real * 0.12;
            let horiz = |a: f64| e * a.cos() + n * a.sin();
            // Keep a clear view: swing round, or rise a little, to see past
            // whatever building stands in the way.
            if self.real - self.cam_clear_t > 0.25 {
                self.cam_clear_t = self.real;
                let look = centre + u * 6.0;
                let clear = |from: DVec3| {
                    !(0..18).any(|k| {
                        let p = from + (look - from) * (k as f64 / 20.0);
                        let (la, lo, h) = geo::pos_to_lla(p);
                        self.world.building(la, lo).is_some_and(|b| h < b.top + 3.0 && h > b.base - 2.0)
                            // (under the 3D tiles the buildings are the mesh, the ground's top)
                            || (self.world.photo().zone_r > 0.0 && self.world.ground(la, lo).is_some_and(|g| h < g.h + 2.0))
                    })
                };
                // Only move when the view is blocked, and then stay a while.
                if !clear(centre + horiz(a) * dist + u * (rise + dist * self.cam_rise)) && self.real > self.cam_hold {
                    let mut best = (0.0, 0.4);
                    'search: for off in [0.0, 0.35, -0.35, 0.7, -0.7, 1.05, -1.05, 1.4, -1.4, 1.75, -1.75, 2.1, -2.1] {
                        for k in [0.0, 0.15, 0.3] {
                            if clear(centre + horiz(a + off) * dist + u * (rise + dist * k)) {
                                best = (off, k);
                                break 'search;
                            }
                        }
                    }
                    a += best.0;
                    self.cam_rise = best.1;
                    self.cam_hold = self.real + 1.5;
                }
            }
            self.cam_ang = Some(a);
            let want = centre + horiz(a) * dist + u * (rise + dist * self.cam_rise);
            self.cam_pos += (want - self.cam_pos) * (dt_real * 1.2).min(1.0);
            cam = self.cam_pos;
            dir = (self.cam_look - cam).normalize();
            upv = u;
        } else if self.cam_mode == 1 {
            if !self.head_live {
                self.head.reset(self.cam_yaw, self.cam_pitch);
                self.head_live = true;
            }
            let up = self.plane.pos.normalize();
            let day = ((up.dot(self.sun_dir()) as f32 + 0.2) / 0.4).clamp(0.0, 1.0);
            let view = self.head.update(self.spec, dt_real, (self.cam_yaw, self.cam_pitch), self.cockpit_fov, self.plane.head, self.plane.rot, up, w / h, day);
            fov = view.fov.to_radians();
            // A rough engine or damage shakes the whole cabin.
            let vibe = self.plane.dmg.vibe.min(2.0) as f64;
            let mut jit = || self.rng.range(-1.0, 1.0) as f64 * vibe;
            let shake = DVec3::new(jit() * 0.006, jit() * 0.006, 0.0);
            let wob = DQuat::from_rotation_y(jit() * 0.004) * DQuat::from_rotation_x(jit() * 0.004);
            let r = self.plane.rot * wob * view.rot;
            cam = self.plane.model_pos() + self.plane.rot * (view.eye + shake);
            dir = r * DVec3::NEG_Z;
            upv = r * DVec3::Y;
            self.cam_pos = cam;
        } else {
            let u = self.plane.pos.normalize();
            // (A rocket points at the sky: its chase camera stays level with it, on the side its back is to, and closes in on the ship once that flies alone.)
            let rocket = self.spec.rocket().is_some();
            let back = if rocket { self.plane.right().cross(u).normalize_or_zero() } else { -self.plane.forward() };
            self.cam_dir = (self.cam_dir + (back - self.cam_dir) * (dt_real * 3.5).min(1.0)).normalize();
            let flat = (self.cam_dir - u * self.cam_dir.dot(u)).normalize_or_zero();
            let tilt = self.cam_dir.dot(u).asin() * 0.6 + self.cam_pitch;
            let d = DQuat::from_axis_angle(u, self.cam_yaw) * (flat * tilt.cos() + u * tilt.sin());
            let vibe = self.plane.dmg.vibe.min(2.0) as f64;
            // (Offsets scale with the aircraft: the camera sits as far back for its size.)
            let sz = self.spec.cam_dist / 16.0;
            let at = self.plane.centre();
            let dist = self.cam_dist * if self.plane.stage > 0 { 0.5 } else { 1.0 };
            cam = at + d * dist + u * (1.5 * sz) + (self.plane.right() * self.rng.range(-1.0, 1.0) as f64 + u * self.rng.range(-1.0, 1.0) as f64) * (vibe * 0.03);
            dir = if rocket { (at - cam).normalize() } else { (self.plane.pos + u * (1.2 * sz) + self.plane.forward() * (6.0 * sz) - cam).normalize() };
            upv = if rocket { u } else { (u + self.plane.up() * 0.25).normalize() };
            self.cam_pos = cam;
            self.cam_look = self.plane.pos;
        }
        // Never let the camera sink into the ground.
        let (clat, clon, calt) = geo::pos_to_lla(cam);
        let ground = self.world.ground(clat, clon).map(|g| g.h);
        let cam = match ground {
            Some(g) if calt < g + 1.5 => cam + cam.normalize() * (g + 1.5 - calt),
            _ => cam,
        };
        let calt = geo::pos_to_lla(cam).2;
        let agl = calt - ground.unwrap_or(0.0);
        // Storms around the camera: the rain falling there, the lightning.
        if in_menu {
            self.cam_rain = 0.0;
            self.cam_hail = 0.0;
            (self.cam_snow, self.cam_blow) = (0.0, 0.0);
        } else {
            self.cam_rain = self.wx.rain_at(cam, agl, self.time) as f32;
            self.cam_hail = if self.cam_rain > 0.0 { storm::local(&self.wx, cam, agl, self.time).hail as f32 } else { 0.0 };
            let world = &self.world;
            let gnd = |p: DVec3| {
                let (la, lo, _) = geo::pos_to_lla(p);
                world.ground(la, lo).map(|g| g.h).unwrap_or(0.0)
            };
            self.lightning.step(dt, self.time, &self.wx, self.plane.pos, cam, &gnd);
            self.rain_field.step(dt, cam, self.wx.at(cam, agl, self.time));
            self.snow_field.step_fall(dt, cam, self.wx.at(cam, agl, self.time), storm::FLAKE_FALL);
            // Snow (snow.rs): what falls is snow above the melting layer under
            // the freezing level (from the real weather, else the season's);
            // where it reaches the ground fresh snow settles; the wind lifts
            // lying snow into a blizzard.
            let (lat_d, gh) = (clat.to_degrees(), ground.unwrap_or(0.0));
            let fz = match (self.snow, self.wx.freezing) {
                (snow::Mode::Always, _) | (_, None) => snow::freezing_guess(self.snow, lat_d, self.doy),
                (_, Some(f)) => f,
            };
            self.cam_snow = snow::snow_share(calt, fz) as f32;
            let at_ground = self.wx.rain_at(geo::lla_to_pos(clat, clon, gh), 0.0, self.time) * snow::snow_share(gh, fz);
            if self.snow == snow::Mode::Off {
                self.fresh = snow::Fresh::default();
            } else {
                self.fresh.step(dt, at_ground);
            }
            self.fresh_line = snow::Fresh::line(gh) as f32;
            let white = snow::snow_line(self.snow, lat_d, self.doy, self.wx.freezing).map_or(0.0, |l| {
                let l = l + (l.min(snow::Fresh::line(gh)) - l) * self.fresh.k;
                snow::cover(gh, 1.0, 0.0, l)
            });
            // (Blown snow fills the lowest hundred metres or so.)
            let low = 1.0 - snow::smoothstep(60.0, 400.0, agl);
            self.cam_blow = (snow::blowing(white, at_ground.min(1.0), self.wx.wind, self.wx.blowing as f64) * low) as f32;
        }
        let scrape = if steps > 0 { (scrape_n as f32 / steps as f32 * 0.5).min(1.0) } else { 0.0 };
        self.update_audio(cam, fire, in_menu, &c, dt, scrape);

        let near = if in_menu { (calt * 0.02).clamp(0.4, 20_000.0) as f32 } else if self.cam_mode == 1 && !self.crashed { 0.03f32 } else { 0.25f32 };
        let proj = perspective_rev(fov, w / h, near);
        let view = look_to(Vec3::ZERO, dir.as_vec3(), upv.as_vec3());
        let vp = proj * view;
        // Temporal anti-aliasing: each frame is drawn a different fraction of
        // a pixel off centre (a Halton sequence), and the renderer blends the
        // frames back together. Everything else works from the centred `vp`.
        let jitter = if self.gfx.taa {
            let halton = |mut i: u64, b: u64| { let (mut f, mut r) = (1.0f32, 0.0f32); while i > 0 { f /= b as f32; r += f * (i % b) as f32; i /= b; } r };
            let k = self.frame % 8 + 1;
            Vec2::new(halton(k, 2) - 0.5, halton(k, 3) - 0.5) * 2.0 / Vec2::new(rw, rh)
        } else {
            Vec2::ZERO
        };
        let vp_jit = Mat4::from_translation(jitter.extend(0.0)) * vp;
        let frustum = world::Frustum::new(DMat4::from_cols_array(&vp.to_cols_array().map(|v| v as f64)));
        self.world.update(&self.r, cam, &frustum);
        let focus_abs = if in_menu { cam } else if self.crashed { self.crash.as_ref().unwrap().site } else { self.plane.pos };
        {
            // Photorealistic 3D tiles in flight, day and night (after dusk the
            // mesh takes our street lamps and lit windows, photo.wgsl).
            let day = if in_menu { 0.0 } else { 1.0 };
            let px = h as f64 / (2.0 * (fov as f64 * 0.5).tan());
            self.world.update_photo(&self.r, cam, focus_abs, &frustum, px, day, dt_real);
        }
        if dt > 0.0 {
            self.world.update_traffic(dt as f32, focus_abs);
            let local = (self.utc + geo::pos_to_lla(focus_abs).1.to_degrees() / 15.0).rem_euclid(24.0);
            let player = (!self.crashed && !in_menu).then_some(self.plane.pos);
            self.world.update_airports(dt as f32, self.wx.wind_from as f32, (self.wx.wind * 1.94384) as f32, (cars::activity(local as f32) * 1.2).clamp(0.25, 1.0), player, focus_abs);
        }

        // ---- frame constants
        let (east, north, up) = geo::enu(cam);
        let delta = cam - self.prev_cam;
        self.prev_cam = cam;
        if delta.length() < 5000.0 {
            self.wave_off += DVec2::new(delta.dot(east), delta.dot(north));
            self.wave_off = DVec2::new(self.wave_off.x.rem_euclid(65536.0), self.wave_off.y.rem_euclid(65536.0));
            self.ocean_off += DVec2::new(delta.dot(east), delta.dot(north));
            self.ocean_off = DVec2::new(self.ocean_off.x.rem_euclid(ocean::WRAP), self.ocean_off.y.rem_euclid(ocean::WRAP));
        }
        // The sea follows the weather's wind (and the clock).
        let sinlat = cam.normalize().z;
        self.world.ocean.update(dt, self.time, self.wx.wind, self.wx.wind_from, sinlat, cam, east, north, self.ocean_off);
        let sun = self.sun_dir();
        let (moon, moon_lit) = self.moon_dir();
        // Detail building tiles in range stand in for "how much city is around".
        let city = if in_menu { 0.0 } else { (self.world.stats.2 as f32 / 45.0).min(1.0) };
        self.city_glow += (city - self.city_glow) * (dt_real as f32 * 0.5).min(1.0);
        // City light in the air: the lights come on as the sun sets.
        let glow_px = if in_menu { None } else { self.skyglow.update(cam) };
        // Today's real snow cover, when the snow follows the real world.
        let snow_px = self.snowmap.update(&self.src, cam, !in_menu && self.snow == snow::Mode::Auto);
        let snow_map = if in_menu || self.snow != snow::Mode::Auto { [[0.0; 4]; 3] } else { self.snowmap.globals(cam, self.doy) };
        let snow_tr_uploads = if in_menu || self.snow == snow::Mode::Off { Vec::new() } else { self.snowtrack.take_uploads() };
        let snow_tr = if in_menu || self.snow == snow::Mode::Off { [[0.0; 4]; 3] } else { self.snowtrack.globals(cam) };
        let glow = {
            let (la, lo, _) = geo::pos_to_lla(cam);
            let gh = self.world.ground(la, lo).map(|g| g.h).unwrap_or(0.0);
            let on = 1.0 - ((up.dot(sun) as f32 + 0.2) / 0.25).clamp(0.0, 1.0);
            if in_menu { [[0.0; 4]; 3] } else { self.skyglow.globals(cam, gh, on * on * (3.0 - 2.0 * on)) }
        };
        let mu = up.dot(sun) as f32;
        let day = ((mu + 0.2) / 0.4).clamp(0.0, 1.0);
        let day = day * day * (3.0 - 2.0 * day);
        // From the cockpit the eye adapts a little to the cabin.
        // Exposure moves in stops through twilight (a straight blend opens up
        // far too early, washing the city out while the sky still glows).
        // Night stays a night photograph: dark walls, bright windows.
        // Under a storm (in daylight) the eye opens up a stop or so to the gloom.
        let shade = if in_menu { 0.0 } else { storm::shade(&self.wx, cam).max(self.wx.deck_shade(cam)) as f32 };
        self.storm_adapt += (shade * day - self.storm_adapt) * (dt_real as f32 * 0.4).min(1.0);
        // The start screen's globe from orbit is exposed for its day side
        // (common.wgsl `globe_k`), wherever the night falls on it.
        let globe = if in_menu { let k = ((calt as f32 - 150e3) / 650e3).clamp(0.0, 1.0); k * k * (3.0 - 2.0 * k) } else { 0.0 };
        let exposure = (6.0f32.ln() + (GLOBE_EXPOSURE.ln() - 6.0f32.ln()) * globe + (0.5f32.ln() - 6.0f32.ln()) * day * (1.0 - globe)).exp() * if self.cam_mode == 1 && !self.crashed && !in_menu { 2f32.powf(self.head.adapt) } else { 1.0 } * 2f32.powf(1.3 * self.storm_adapt);
        let v4 = |v: DVec3, w: f64| [v.x as f32, v.y as f32, v.z as f32, w as f32];
        let cam_right = dir.cross(upv).normalize();
        let cam_up = cam_right.cross(dir);
        // Thunder from the lightning just set off, panned to where it struck.
        for th in self.lightning.thunder.drain(..) {
            if let Some(a) = &self.audio {
                a.event(audio::Event::Thunder { delay: th.delay as f32, near: th.near as f32, far: th.far as f32, power: th.power as f32, pan: th.from.dot(cam_right) as f32 });
            }
        }
        // Shadow cascades: three nested boxes around the aircraft, each snapped
        // to its own texel grid so shadows do not shimmer as it moves.
        let shadow_on = mu > 0.04 && !in_menu && self.gfx.shadows != graphics::Level::Off;
        let sres = self.r.shadow_res() as f32;
        // (the aircraft's layer may use only part of it: Renderer::plane_shadow_res)
        let pres = self.r.plane_shadow_res() as f32;
        const CASC: [f32; 3] = [260.0, 1500.0, 8500.0];
        let mut casc_vp = [[[0f32; 4]; 4]; render::CASCADES + 4];
        {
            let right = { let r = DVec3::Z.cross(sun); if r.length_squared() < 1e-6 { DVec3::X } else { r.normalize() } };
            let upl = sun.cross(right);
            for (i, r) in CASC.iter().enumerate() {
                let texel = (2.0 * r / sres) as f64;
                let snap = |v: f64| (v / texel).floor() * texel - v;
                let centre = focus_abs - cam + right * snap(focus_abs.dot(right)) + upl * snap(focus_abs.dot(upl));
                let d = r + 3500.0;
                let lview = look_to(centre.as_vec3() + sun.as_vec3() * d, -sun.as_vec3(), upl.as_vec3());
                casc_vp[i] = (ortho(-r, *r, -r, *r, 0.0, 2.0 * d) * lview).to_cols_array_2d();
            }
            // The aircraft's own shadow map: a few metres across, deep enough
            // to reach the ground below it.
            let centre = (self.plane.pos - cam).as_vec3();
            let (pr, pb) = (self.plane.shadow_r(), PSHADOW_BACK * self.plane.shadow_r() / PSHADOW_R);
            let lview = look_to(centre + sun.as_vec3() * pb, -sun.as_vec3(), upl.as_vec3());
            casc_vp[3] = (ortho(-pr, pr, -pr, pr, 0.0, PSHADOW_DEPTH) * lview).to_cols_array_2d();
        }
        let mut shadow_vp = [[[0f32; 4]; 4]; 3];
        shadow_vp.copy_from_slice(&casc_vp[..3]);
        let mut g = Globals {
            view_proj: vp_jit.to_cols_array_2d(),
            inv_view_proj: vp_jit.inverse().to_cols_array_2d(),
            shadow_vp,
            planet: v4(-cam, geo::R),
            sun_dir: v4(sun, SUN_E as f64),
            east: v4(east, self.wave_off.x),
            north: v4(north, self.wave_off.y),
            up: v4(up, calt),
            screen: [rw, rh, 1.0 / rw, 1.0 / rh],
            time: [self.time as f32, dt as f32, exposure, self.frame as f32],
            cloud: [if in_menu { 0.0 } else { self.wx.clouds }, self.wx.base(), self.wx.depth(), self.wx.cloud_ceiling()],
            light_pos: [0.0; 4],
            light_col: [0.0; 4],
            vol_o: [0.0; 4],
            vol_u: [0.0; 4],
            // screen space reflections on water and glass, off on low.
            misc: [near, self.city_glow, if self.gfx.matches() == Some(graphics::Level::Low) { 0.0 } else { 1.0 }, 2.0 * (fov * 0.5).tan() / rh],
            csm: [CASC[0] * 2.0 / sres, CASC[1] * 2.0 / sres, CASC[2] * 2.0 / sres, if shadow_on { 1.0 } else { 0.0 }],
            scorch: [[0.0; 4]; 8],
            crater: [[0.0; 4]; 8],
            prev_vp: self.prev_vp,
            // A jump (teleport, camera switch) invalidates the cloud history.
            cam_delta: v4(delta, if delta.length() < 500.0 { 1.0 } else { 0.0 }),
            moon: v4(moon, (MOON_E as f64) * moon_lit.powf(1.5)),
            vol_e: [0.0; 4],
            vol_n: [0.0; 4],
            pshadow_vp: casc_vp[3],
            pshadow: [self.plane.shadow_r() * 2.0 / pres, 0.0012 / PSHADOW_DEPTH, pres / sres, if shadow_on && !self.crashed { 1.0 } else { 0.0 }],
            taa: [jitter.x * 0.5, -jitter.y * 0.5, if self.gfx.taa { 1.0 } else { 0.0 }, 0.0],
            plane_inv: [[0.0; 4]; 4],
            plane_prev: [[0.0; 4]; 4],
            plane_box: self.plane_box,
            weather: { let o = self.wx.cloud_off; [o.x as f32, o.y as f32, o.z as f32, if in_menu { 1.0 } else { self.wx.haze }] },
            storm: { let w = self.wx.steady(cam, 2000.0); let d = if w.length() > 0.1 { w.normalize() } else { east }; v4(d, 0.0) },
            storm_cells: [[0.0; 4]; 2 * weather::MAX_STORMS],
            det: [[0.0; 4]; 16],
            photo_vp: [[0.0; 4]; 4],
            photo: [0.0; 4],
            photo_c: [0.0; 4],
            photo_sun: [0.0, 0.0, 0.0, -1.0],
            cut: [[0.0; 4]; 3 * tiles3d::MAX_CUTS],
            cut_v: [[0.0; 4]; tiles3d::MAX_CUT_V / 2],
            cs: [[0.0; 4]; 2 * render::CS_MAP],
            alto: [if in_menu { 0.0 } else { self.wx.alto }, 4800f32.max(self.wx.base() + 0.4 * self.wx.depth() + 800.0), 0.0, 0.0],
            ocean: self.world.ocean.globals(),
            shore_vp: [[0.0; 4]; 4],
            shore: [0.0; 4],
            foam: [[0.0; 4]; 16],
            // (yzw: the liquid rain, how wet the windows are, and the airspeed over them in kt; plane.wgsl rain_water)
            start: [if in_menu { 1.0 } else { 0.0 }, glass_rain, self.glass.wet, (self.plane.ias() * 1.94384) as f32],
            stormfx: [0.0; 4],
            flash: [[0.0; 4]; 2 * storm::MAX_FLASHES],
            rain: [[0.0; 4]; 2],
            quality: [0.0; 4],
            detail: [self.gfx.city_k(), self.gfx.tree_imp_k(), self.gfx.tree_far(), 0.0],
            nimbo: if in_menu { [[0.0; 4]; 4] } else { self.wx.deck_globals() },
            glow_c: glow[0],
            glow_e: glow[1],
            glow_n: glow[2],
            spot: [[0.0; 4]; 3],
            nav: self.nav_globals(),
            cockpit: self.cockpit_globals(agl),
            snow: if in_menu { [0.0; 4] } else { snow::globals(self.snow, clat.to_degrees(), self.doy, self.wx.freezing) },
            snow_c: snow_map[0],
            snow_e: snow_map[1],
            snow_n: snow_map[2],
            flakes: [0.0; 4],
            flakes_v: [0.0; 4],
            fresh: [0.0; 4],
            snow_tr_c: snow_tr[0],
            snow_tr_e: snow_tr[1],
            snow_tr_n: snow_tr[2],
        };
        if !in_menu {
            let cells = self.wx.storm_cells(cam);
            g.storm[3] = cells.len() as f32;
            for (i, c) in cells.iter().enumerate() {
                g.storm_cells[2 * i] = c[0];
                g.storm_cells[2 * i + 1] = c[1];
            }
            // The flight model flies through the same cells.
            self.wx.cells = cells;
            // The storms close to the camera (they darken the ground under them),
            // the lightning lighting the scene, and the rain falling around the camera.
            g.stormfx = [storm::near_mask(&self.wx, cam) as f32, self.cam_rain, self.lightning.globals(cam, dt, &mut g.flash) as f32, shade];
            let o = self.rain_field.off;
            let v = self.rain_field.rel;
            // Drops closer than this are inside the cockpit (or the camera's lens).
            let near_cut = if self.cam_mode == 1 && !self.crashed { 2.5 } else { 0.3 };
            // (Hail rides along in the intensity's whole part, in sixteenths: precip.wgsl.)
            let rain_w = self.cam_rain.min(0.999) + (self.cam_hail.clamp(0.0, 1.0) * 15.0).round();
            g.rain = [[o.x as f32, o.y as f32, o.z as f32, rain_w], [v.x as f32, v.y as f32, v.z as f32, near_cut as f32]];
            let (o, v) = (self.snow_field.off, self.snow_field.rel);
            g.flakes = [o.x as f32, o.y as f32, o.z as f32, self.cam_snow];
            g.flakes_v = [v.x as f32, v.y as f32, v.z as f32, self.cam_blow];
            g.fresh = [self.fresh_line, self.fresh.k as f32, 0.0, 0.0];
        }
        if !self.crashed && !in_menu {
            // The aircraft keeps pace with the camera, so its pixels are found
            // in last frame's image through where it was then, not through
            // where the ground under them was.
            let base = Mat4::from_rotation_translation(self.plane.rot.as_quat(), (self.plane.model_pos() - cam).as_vec3());
            g.taa[3] = 1.0;
            g.plane_inv = base.inverse().to_cols_array_2d();
            g.plane_prev = (Mat4::from_cols_array_2d(&self.prev_vp) * self.prev_plane).to_cols_array_2d();
            self.prev_plane = base;
        }
        self.prev_vp = vp.to_cols_array_2d();
        for (i, (p, r)) in self.world.craters.iter().take(8).enumerate() {
            g.crater[i] = v4(*p - cam, *r as f64);
        }
        let mut psim = PSim { anchor: v4(self.fx_anchor - cam, 0.0), upv: v4(self.fx_anchor.normalize(), dt), cam_r: v4(cam_right, self.time), cam_u: v4(cam_up, 0.0), wind: v4(self.wx.steady(self.fx_anchor, 10.0), 0.0), shift: v4(self.fx_shift, 0.0), ..Default::default() };
        if dt > 0.0 {
            // The simulation runs this frame and applies the shift.
            self.fx_shift = DVec3::ZERO;
        }
        let mut objs = Vec::new();
        if let Some(cr) = &self.crash {
            // The ground under an impact up the side of a building is further down.
            psim.anchor = v4(cr.site - cam, -cr.lift);
            psim.upv = v4(cr.up, dt);
            g.light_pos = v4(cr.site + cr.up * 8.0 * cr.size as f64 - cam, (12.0 + 8.0 * cr.power as f64) * cr.size as f64);
            let l = cr.light();
            g.light_col = [1.0 * l, 0.5 * l, 0.2 * l, 0.0];
            let grow = (cr.time as f32 / 1.2).min(1.0);
            for (i, (p, rad)) in cr.scorch.iter().take(8).enumerate() {
                g.scorch[i] = v4(cr.site + p.as_dvec3() - cam, (*rad * grow) as f64);
            }
            for (i, (p, rad, k, age, seed)) in cr.foam().into_iter().take(8).enumerate() {
                g.foam[2 * i] = v4(cr.site + p.as_dvec3() - cam, rad as f64);
                g.foam[2 * i + 1] = [k, age, seed, 0.0];
            }
            if self.crashed || cr.time < 240.0 {
                // (the airframe's dent stays on the pieces it breaks into: their meshes are in the body frame)
                let dent = (self.plane.dmg.crumple, self.plane.dmg.crumple_dir);
                for (i, d) in cr.debris.iter().enumerate() {
                    if d.sunk {
                        continue;
                    }
                    let m = Mat4::from_scale_rotation_translation(Vec3::splat(d.scale), d.rot.as_quat(), (d.pos - cam).as_vec3()) * Mat4::from_translation(-d.origin);
                    objs.push(DrawObj { mesh: d.mesh, model: m, tint: [1.0, 1.0, 1.0, d.char], glow: [if d.burn > 0.0 { if d.rigid { 0.35 } else { 1.0 } } else if cr.water { 0.0 } else { 0.25 }, self.spec.id as f32, 0.0, i as f32 * 0.37], cast: 0b11, cut: true, tear: [d.tear[0].gpu(), d.tear[1].gpu()], dmg: [d.tear[0].mode, d.tear[1].mode, 0.6, d.jag], crumple: if d.group.is_some() { dent.0 } else { [0.0; 4] }, crumple_dir: if d.group.is_some() { dent.1 } else { [0.0; 4] }, ..DrawObj::DEFAULT });
                }
            }
        }
        // The fire simulation: this frame's sources, and the light it gives off.
        let mut fluid = None;
        if let Some(cr) = self.crash.as_mut() {
            let wind = self.wx.steady(cr.site, 10.0);
            // Once the fire is out and its smoke has gone, the grid rests.
            let spent = cr.fluid.is_empty() && cr.time > 8.0 && self.r.fluid.latest.is_some_and(|st| st.soot < 2.0 && st.emit < 1.0);
            if let Some(dm) = cr.domain.as_mut().filter(|_| !spent) {
                fluid = Some(dm.frame(dt, cam, wind, sun, &cr.fluid, &self.world));
                if let Some(st) = self.r.fluid.latest {
                    // A ball about as big as the burning gas, as bright as flame.
                    let (t, p, _) = cr.last_blast;
                    let flash = p * 6.0 * (-(t as f32) / 0.25).exp();
                    let rad = (0.75 * st.emit.max(0.0).cbrt()).clamp(3.0, 25.0 * cr.size);
                    let l = flash + 2.5 * (st.emit / 300.0).min(1.0);
                    g.light_pos = v4(dm.at_frac(st.emit_c) - cam, rad as f64);
                    g.light_col = [1.0 * l, 0.48 * l, 0.17 * l, 0.0];
                }
            }
        }
        let params = self.instruments(agl);
        let mut own_lights = Vec::new();
        if !self.crashed && !in_menu {
            let base = Mat4::from_rotation_translation(self.plane.rot.as_quat(), (self.plane.model_pos() - cam).as_vec3());
            let pose = self.plane.pose(&self.model.pieces);
            // Blades while they can be followed by eye, a blur once they cannot.
            let blur = ((self.plane.prop_rate - 30.0) / 25.0).clamp(0.0, 1.0) as f32;
            let dm = self.plane.dmg;
            let kind = self.spec.id as f32;
            let rain_ph = self.glass.phases();
            let bend = self.plane.wing_bend();
            // What the exhaust shows (plane.wgsl, the plume and the nozzles): how far the
            // afterburners are lit, and how hard the engines are running dry (0 idle .. 1).
            let (burner, dry, nozzle_z) = match &self.spec.engine {
                aircraft::Engine::Fan(fe) => (self.plane.burner as f32, (((self.plane.n1 - fe.idle) / (1.0 - fe.idle)).clamp(0.0, 1.0) * if dm.engine > 0.0 { 1.0 } else { 0.0 }) as f32, fe.nozzle_z),
                // (A rocket: how hard the lit engines burn, and which stage's they are with how many of them, see plane.wgsl `rocket_plume`.)
                aircraft::Engine::Rocket(rk) => (self.plane.burner as f32, self.plane.plume_code(), rk.stages[self.plane.stage].z[1]),
                _ => (0.0, 0.0, 0.0),
            };
            // (A rocket's plume also wants to know how thin the air is: 0 at the sea, a half by 20 km, 1 in space; and its tiles how hot.)
            let rocket = self.spec.rocket().is_some();
            let thin = if rocket { 1.0 - (plane::density(geo::pos_to_lla(self.plane.pos).2) / plane::RHO0).min(1.0).powf(0.25) as f32 } else { 0.0 };
            let heat = if rocket { self.plane.heat() } else { 0.0 };
            if burner > 0.01 && self.crash.is_none() {
                // The flames light the tail from behind (turned down at night, as they are, for the eye that has opened up).
                let k = burner * (0.12 + 0.88 * day) * 2.2;
                g.light_pos = v4(self.plane.model_pos() + self.plane.rot * DVec3::new(0.0, 0.0, nozzle_z + 2.0) - cam, 1.3);
                g.light_col = [k, 0.5 * k, 0.2 * k, 0.0];
            }
            if let Some((at, power, t0)) = self.weapon_flash {
                let age = self.real - t0;
                if age < 0.45 {
                    let k = (1.0 - age / 0.45) as f32 * power * (0.25 + 0.75 * day);
                    g.light_pos = v4(at - cam, (6.0 * power as f64).max(2.0));
                    g.light_col = [2.8 * k, 1.1 * k, 0.35 * k, 0.0];
                } else {
                    self.weapon_flash = None;
                }
            }
            // from far outside the cabin interior cannot be seen, skip its vertices.
            let far_out = self.cam_mode != 1 && (cam - self.plane.model_pos()).length() > 40.0;
            for (i, p) in self.model.pieces.iter().enumerate() {
                if far_out && self.piece_inner[i] {
                    continue;
                }
                let hide = !self.model.shown(p, blur, self.plane.spoilers >= 0.02, &|x| self.plane.input(x));
                if hide || dm.hidden(p) || self.weapons.hides_piece(p.name) {
                    continue;
                }
                let tears = dm.tears_for(p);
                let torn = tears[0].is_some() || tears[1].is_some();
                objs.push(DrawObj {
                    mesh: self.piece_mesh[i],
                    model: base * pose[i],
                    // (Windows: the phases of the water on them, rain.rs Glass::phases; a
                    // rocket's parts: how thin the air is and how hot its tiles. A rocket
                    // has no windows: its "glass" is the exhaust plumes, which spread by
                    // that thinness; given the rain's ever-growing flow phase instead,
                    // they swelled into a white sheet across the whole view.)
                    tint: [blur, burner, dry, if p.glass && !rocket { rain_ph[2] } else { thin }],
                    glow: if p.glass && !rocket { [0.0, kind, rain_ph[0], rain_ph[1]] } else { [0.0, kind, heat, 0.0] },
                    params,
                    cast: 1 << render::PLANE_LAYER,
                    glass: p.glass,
                    cut: self.piece_cut[i] || torn,
                    tear: [tears[0].gpu(), tears[1].gpu()],
                    dmg: [tears[0].mode, tears[1].mode, dm.scuff[p.group], dm.jag_for(p)],
                    dmg2: [dm.oil, dm.crack[0], dm.crack[1], dm.crack[2]],
                    crumple: dm.crumple,
                    crumple_dir: dm.crumple_dir,
                    pose: pose_rows(&pose[i]),
                    flex: match p.group {
                        airframe::G_LWING => bend[0],
                        airframe::G_RWING => bend[1],
                        _ => 0.0,
                    },
                    ..DrawObj::DEFAULT
                });
            }
            // A buckled wing: the same pieces again past the break, swung about
            // the torn skin they hang from.
            for (side, f) in dm.fold.iter().enumerate() {
                if !f.on {
                    continue;
                }
                let g = if side == 0 { airframe::G_LWING } else { airframe::G_RWING };
                let swing = base * f.mat();
                for (i, p) in self.model.pieces.iter().enumerate() {
                    let structure = matches!(p.role, airframe::Role::WingStructure(_) | airframe::Role::TailStructure);
                    if p.group != g || p.glass || (dm.hidden(p) && !structure) {
                        continue;
                    }
                    let stick = if structure { 0.12 * self.spec.span / 5.5 } else { 0.0 };
                    let tears = [f.tear.flip().grow(stick), f.outer.grow(stick)];
                    objs.push(DrawObj {
                        mesh: self.piece_mesh[i],
                        model: swing * pose[i],
                        glow: [0.0, kind, 0.0, 0.0],
                        params,
                        cast: 1 << render::PLANE_LAYER,
                        cut: true,
                        tear: [tears[0].gpu(), tears[1].gpu()],
                        dmg: [tears[0].mode, tears[1].mode, dm.scuff[p.group].max(0.5), f.tear.jag.max(0.03)],
                        ..DrawObj::DEFAULT
                    });
                }
            }
            // Missiles in flight: the same wingtip piece, now on its own.
            for m in &self.weapons.missiles {
                let Some(st) = self.weapons.station(m.station) else { continue };
                let Some(pi) = st.piece else { continue };
                let local = self.model.pieces[pi].pivot.as_dvec3();
                let model = Mat4::from_rotation_translation(m.rot.as_quat(), (m.pos - cam).as_vec3()) * Mat4::from_translation((-local).as_vec3());
                objs.push(DrawObj { mesh: self.piece_mesh[pi], model, tint: [0.0; 4], glow: [0.0, kind, 0.0, 0.0], params, cast: 0, ..DrawObj::DEFAULT });
            }
            self.own_lights(&base, &pose, params[5], &mut g, &mut own_lights);
        }
        if !in_menu {
            self.sheds.draw(cam, params, &mut objs);
            // A booster just let go: its centre engines' flame dying away behind it.
            let ix = |n: &str| self.model.pieces.iter().position(|p| p.name == n);
            if let (true, Some(body), Some(plume)) = (self.plane.stage > 0, ix("booster body"), ix("booster plume")) {
                if let Some((model, age)) = self.sheds.find(self.piece_mesh[body], cam).filter(|s| s.1 < 2.5) {
                    let thin = 1.0 - (plane::density(geo::pos_to_lla(self.plane.pos).2) / plane::RHO0).min(1.0).powf(0.25) as f32;
                    objs.push(DrawObj { mesh: self.piece_mesh[plume], model, tint: [0.0, 0.7 * (1.0 - age / 2.5), 0.9 * 3.0 / 33.0, thin], glow: [0.0, self.spec.id as f32, 0.0, 0.0], params, glass: true, ..DrawObj::DEFAULT });
                }
            }
        }
        let mut insts = Vec::new();
        let mut fall = Vec::new();
        let mut air_lights = own_lights;
        if !in_menu {
            let local = (self.utc + clon.to_degrees() / 15.0).rem_euclid(24.0);
            let sun_up = geo::enu(cam).2.dot(self.sun_dir());
            self.world.car_instances(cam, cars::activity(local as f32), sun_up, &mut insts, &mut air_lights);
            self.n_cars = insts.len();
            self.destr.instances(cam, &mut insts, &mut fall);
            self.destr.skin_frame(cam);
            self.destr.upload_skins(&self.r);
            self.world.airport_instances(cam, self.wx.wind_from as f32, (self.wx.wind * 1.94384) as f32, self.time, &mut insts, &mut air_lights);
            self.world.photo_beacons(cam, &mut air_lights);
        }
        self.live.step(&self.src, (!in_menu).then_some(self.plane.pos));
        if !in_menu {
            // A downed aircraft that has reached the ground is gone (see
            // `step_kills`); one that outlived its kill entry goes here.
            let mut gone = Vec::new();
            for p in self.live.planes.iter().filter(|p| p.downed > 0.0) {
                let (la, lo, h) = geo::pos_to_lla(p.pos);
                let tracked = self.kills.iter().any(|k| matches!(&k.key, weapons::TargetKey::Live(x) if *x == p.hex));
                if !tracked && (h < 0.0 || self.world.ground(la, lo).is_some_and(|g| h < g.h + 2.0)) {
                    gone.push(p.hex.clone());
                }
            }
            for hex in gone {
                self.live.remove(&hex);
            }
        }
        let wx_at = if in_menu {
            self.menu.pick
        } else {
            let (la, lo, _) = geo::pos_to_lla(self.plane.pos);
            Some((la.to_degrees(), lo.to_degrees()))
        };
        self.livewx.step(&self.src, &mut self.wx, wx_at, if in_menu { 0.0 } else { dt_real });
        if let Some(px) = self.gclouds.step(&self.src, in_menu) {
            self.r.upload_globe_clouds(&px);
        }
        if !in_menu {
            self.live.instances(cam, &mut insts, &mut air_lights);
        }
        self.update_mp(in_menu);
        if !in_menu {
            let full = self.draw_mp_planes(cam, dt_real, &mut objs);
            mp::instances(&self.mp_players, &full, cam, &mut insts, &mut air_lights);
        }
        self.live_view = Some((vp, cam));

        // The globe's labels are placed in the window's pixels, not the (maybe smaller) 3D scene's.
        let (ww, wh) = (self.r.config.width as f32, self.r.config.height as f32);
        let ui = if in_menu { self.menu_ui(vp, cam, ww, wh) } else { self.ui(agl) };
        // The smoke volume: a grid around the effects anchor, on its local east/north/up axes.
        {
            let site = cam + DVec3::new(psim.anchor[0] as f64, psim.anchor[1] as f64, psim.anchor[2] as f64);
            let (ve, vn, vu) = geo::enu(site);
            // (coarser round a big fire, as its grid is: see fluid.rs `coarse`)
            let vs = self.crash.as_ref().and_then(|c| c.domain.as_ref()).map(|d| d.sc as f64).unwrap_or(1.0);
            let (cs, zmin) = (render::VOL_CELL as f64 * vs, render::VOL_ZMIN as f64 * vs);
            let lift = self.crash.as_ref().map(|c| c.lift).unwrap_or(0.0);
            let corner = site - ve * (cs * render::VOL_N[0] as f64 * 0.5) - vn * (cs * render::VOL_N[1] as f64 * 0.5) + vu * (zmin - lift);
            g.vol_o = v4(corner - cam, cs);
            g.vol_u = v4(vu, 0.0);
            g.vol_e = v4(ve, 0.0);
            g.vol_n = v4(vn, 0.0);
            psim.ve = v4(ve, cs);
            psim.vn = v4(vn, zmin - lift);
        }
        let pf = self.world.photo_frame(cam);
        let photo = self.world.photo_draws(cam);
        g.photo_vp = pf.mask_vp.to_cols_array_2d();
        g.photo = pf.zone;
        g.photo_c = pf.centre;
        g.photo_sun = pf.sun;
        g.cut = pf.cut;
        g.cut_v = pf.cut_v;
        casc_vp[render::MASK_SLOT] = g.photo_vp;
        let (tiles, tess, details, near) = self.world.draw_lists(cam, focus_abs, CASC);
        let cover = self.world.light_cover(cam);
        // The shore distance field: a step of its rebuild this frame.
        let sf = self.shore.frame(cam);
        casc_vp[render::SHORE_SLOT] = sf.build_vp;
        casc_vp[render::SHORE_WIDE_SLOT] = sf.build_vp_wide;
        g.shore_vp = sf.vp;
        g.shore = [if sf.on { 1.0 } else { 0.0 }, shore::TEXEL as f32, shore::MAX_D, 2.0 * shore::HALF as f32];
        let shore_tiles = if matches!(sf.step, Some(shore::Step::Raster)) { self.world.shore_tiles(cam, sf.build_c) } else { Vec::new() };
        let shore_c = sf.build_c.as_vec3().to_array();
        let fields = if in_menu { Vec::new() } else { self.world.field_draws(cam) };
        let mut shot = match self.shots.first() {
            Some((t, _, _)) if self.real >= *t => {
                // Logged with each scripted shot, for comparing builds.
                log::info!("fps {:.1}  rain {:.2} (glass {:.2}, wet {:.2}, hail {:.2})  sigma {:.2} draft {:.1}", self.fps, self.cam_rain, self.wx.rain, self.glass.wet, self.cam_hail, self.plane.air.sigma_w, self.plane.air.draft);
                Some(self.shots.remove(0).1)
            }
            _ => None,
        };
        let mut rec_done = false;
        if let Some((dir, start, secs)) = &self.record {
            // The synth runs every frame so it is settled when recording starts.
            let n = (audio::Audio::RATE as f64 / REC_FPS).round() as usize;
            let samples = self.audio.as_ref().map(|a| a.take(n)).unwrap_or_default();
            if self.real >= *start {
                if shot.is_none() {
                    let _ = std::fs::create_dir_all(dir);
                    shot = Some(dir.join(format!("f{:05}.png", self.rec_n)));
                    self.rec_n += 1;
                }
                self.rec_audio.extend(samples);
                if self.real >= start + secs {
                    write_wav(&dir.join("audio.wav"), &self.rec_audio, audio::Audio::RATE);
                    rec_done = true;
                }
            }
        }
        // Scripted shots can pre-position the chase camera for the next capture.
        if let Some((_, _, Some((y, p, d)))) = self.shots.first() {
            (self.cam_yaw, self.cam_pitch, self.cam_dist) = (y.to_radians(), *p, *d);
        }
        let had_shots = shot.is_some() && self.shots.is_empty();
        let (skin_slots, skins) = if in_menu { (Vec::new(), Vec::new()) } else { self.destr.skin_draws() };
        self.r.stats[0] += now.elapsed().as_secs_f64();
        let mut bolts = Vec::new();
        if !in_menu {
            self.lightning.segments(cam, dt, &mut bolts);
        }
        let rain_drops = if in_menu || (self.cam_rain < 0.002 && self.cam_blow < 0.01) { 0 } else { render::RAIN_DROPS };
        self.r.render(Frame { rain_drops, bolts, g, tiles, tess, details, photo, skins, skin_slots, fields, lights: air_lights, objs, insts, fall, near, casc_vp, shore: sf.step, shore_c, shore_tiles, psim, sim_steps: dt > 0.0, fluid, ocean: self.world.water_in_view().then(|| self.world.ocean.uniform()), egui: Some(ui), shot, cover, glow: glow_px, snow: snow_px, snow_tracks: snow_tr_uploads });
        self.finish_mcp_shots();
        if (had_shots && std::env::args().any(|a| a == "--shot") && self.record.is_none()) || rec_done {
            self.quit = true;
        }
    }

    /// The start screen drawn over the globe: labels, the chosen spot, and the panel.
    fn menu_ui(&mut self, vp: Mat4, cam: DVec3, w: f32, h: f32) -> (Vec<egui::ClippedPrimitive>, egui::TexturesDelta, f32) {
        let raw = self.egui_input();
        let mut go: Option<String> = None;
        let mut start = false;
        let mut jump: Option<(f64, f64, &'static str)> = None;
        let mut airport: Option<u32> = None;
        let alt = self.menu.alt;
        self.src.google.update_attribution(self.menu.lat, self.menu.lon, alt);
        let out = self.egui_ctx.clone().run_ui(raw, |root| {
            let ctx = &root.ctx().clone();
            self.pad_pass(ctx);
            // The main menu, the aircraft and the settings screens each take
            // the whole window; the rest of this draws the globe's.
            match self.menu.screen {
                menu::Screen::Main => {
                    self.main_menu(ctx);
                    padui::focus_ring(ctx, menu::ACCENT);
                    return;
                }
                menu::Screen::Settings => {
                    if settings_ui::show(ctx, self, false) {
                        self.menu.screen = self.menu.set_back;
                        self.pad_home = true;
                    }
                    padui::focus_ring(ctx, menu::ACCENT);
                    return;
                }
                menu::Screen::Aircraft => {
                    self.aircraft_screen(ctx);
                    padui::focus_ring(ctx, menu::ACCENT);
                    return;
                }
                menu::Screen::Fly => {}
            }
            use egui::{Align2, Color32, FontId, RichText};
            let ppp = ctx.pixels_per_point();
            // With a gamepad, X starts at the crosshair in the middle of the screen.
            let aim = padui::active().then(|| egui::pos2(w * 0.5 / ppp, h * 0.5 / ppp));
            // Screen position of a place, if it is on the near side of the planet.
            let project = |lat: f64, lon: f64| -> Option<egui::Pos2> {
                let p = geo::lla_to_pos(lat.to_radians(), lon.to_radians(), 0.0);
                if p.normalize().dot((cam - p).normalize()) < 0.05 {
                    return None;
                }
                let c = vp * (p - cam).as_vec3().extend(1.0);
                if c.w <= 0.0 {
                    return None;
                }
                let (x, y) = ((c.x / c.w * 0.5 + 0.5) * w, (0.5 - c.y / c.w * 0.5) * h);
                (x > -50.0 && y > -50.0 && x < w + 50.0 && y < h + 50.0).then(|| egui::pos2(x / ppp, y / ppp))
            };
            let painter = ctx.layer_painter(egui::LayerId::background());
            let label = |p: egui::Pos2, text: &str, size: f32, col: Color32| {
                painter.text(p + egui::vec2(1.0, 1.0), Align2::LEFT_CENTER, text, menu::medium(size), Color32::from_black_alpha(200));
                painter.text(p, Align2::LEFT_CENTER, text, menu::medium(size), col);
            };
            // (where the other players are on screen: city names give way to theirs)
            let mp_at: Vec<egui::Pos2> = self.mp_world.iter().filter_map(|sp| project(sp.lat_deg(), sp.lon_deg())).collect();
            if alt > 25_000.0 {
                // cities next to the start marker give way to its label.
                let pick_at = self.menu.pick.and_then(|(la, lo)| project(la, lo));
                for (name, lat, lon, tier) in menu::CITIES {
                    if *tier > 1 && alt > 7.0e6 {
                        continue;
                    }
                    let clear = |p: &egui::Pos2| pick_at.is_none_or(|k| (*p - k).length() > 70.0) && mp_at.iter().all(|m| (m.y - p.y).abs() > 16.0 || (m.x - p.x).abs() > 90.0);
                    if let Some(p) = project(*lat, *lon).filter(clear) {
                        painter.circle_filled(p, 2.5, Color32::from_rgba_unmultiplied(255, 255, 255, 220));
                        label(p + egui::vec2(7.0, 0.0), tr(name), if *tier == 1 { 14.0 } else { 12.5 }, Color32::from_rgba_unmultiplied(255, 255, 255, 225));
                    }
                }
            }
            // Airports, once zoomed in far enough to tell them apart. Each one can be clicked.
            self.menu.markers.clear();
            if alt < 450_000.0 {
                let db = airports::Db::get();
                let mut n = 0;
                for (i, _) in db.near(self.menu.lat, self.menu.lon, (alt * 1.6).min(600_000.0)) {
                    let a = &db.apts[i as usize];
                    if a.kind < 1 && alt > 60_000.0 || a.kind < 2 && alt > 200_000.0 || !a.any_paved() || n >= 80 {
                        continue;
                    }
                    if let Some(p) = project(a.lat, a.lon) {
                        n += 1;
                        self.menu.markers.push((i, p.x * ppp, p.y * ppp));
                        let chosen = self.menu.airport == Some(i);
                        let hover = ctx.pointer_hover_pos().is_some_and(|h| (h - p).length() < 14.0 / ppp) || aim.is_some_and(|c| (c - p).length() < 28.0);
                        let col = if chosen || hover { menu::ACCENT } else if a.kind == 2 { Color32::from_rgb(120, 200, 255) } else { Color32::from_rgb(170, 200, 220) };
                        if hover {
                            painter.circle_stroke(p, 8.0, egui::Stroke::new(1.5, col));
                            label(p + egui::vec2(if chosen { 18.0 } else { 6.0 }, 16.0), &a.name.chars().take(40).collect::<String>(), 13.0, col);
                        }
                        painter.rect_filled(egui::Rect::from_center_size(p, egui::vec2(5.0, 5.0)), 1.0, col);
                        // the chosen airport is named by the start marker.
                        if !chosen {
                            label(p + egui::vec2(6.0, 0.0), &a.ident, if a.kind == 2 { 13.0 } else { 12.0 }, col);
                        }
                    }
                }
            }
            // The other players (multiplayer, connected): an arrow each, the
            // way they fly, with their name; more about them under the mouse.
            if !self.mp_world.is_empty() {
                let col = MP_COLOUR;
                let shade = Color32::from_black_alpha(170);
                let d = (alt / 111_000.0 * 0.03).clamp(0.002, 0.6);
                let crowd = self.mp_world.len() > 24 && alt > 3.0e6;
                // (a name that would overlap one already written is left to the mouse)
                let mut named: Vec<egui::Rect> = Vec::new();
                for sp in &self.mp_world {
                    let (la, lo) = (sp.lat_deg(), sp.lon_deg());
                    let Some(p) = project(la, lo) else { continue };
                    let h = sp.heading_deg().to_radians();
                    let ahead = project(la + d * h.cos(), lo + d * h.sin() / la.to_radians().cos().max(0.05));
                    let dir = ahead.map(|a| a - p).filter(|v| v.length() > 0.05).map(|v| v.normalized()).unwrap_or(egui::vec2(0.0, -1.0));
                    let side = egui::vec2(-dir.y, dir.x);
                    let arrow = |k: f32| vec![p + dir * 8.0 * k, p - dir * 5.0 * k + side * 5.5 * k, p - dir * 2.0 * k, p - dir * 5.0 * k - side * 5.5 * k];
                    let hover = ctx.pointer_hover_pos().is_some_and(|m| (m - p).length() < 12.0) || aim.is_some_and(|c| (c - p).length() < 20.0);
                    let k = if hover { 1.35 } else { 1.0 };
                    painter.add(egui::Shape::closed_line(arrow(k), egui::Stroke::new(3.0, shade)));
                    // (two triangles: the arrowhead's notch makes it concave)
                    let a = arrow(k);
                    painter.add(egui::Shape::convex_polygon(vec![a[0], a[1], a[2]], col, egui::Stroke::NONE));
                    painter.add(egui::Shape::convex_polygon(vec![a[0], a[2], a[3]], col, egui::Stroke::NONE));
                    let at = p + egui::vec2(11.0, 0.0);
                    let room = egui::Rect::from_min_size(at - egui::vec2(0.0, 9.0), egui::vec2(9.0 * sp.name.chars().count() as f32 + 8.0, 18.0));
                    if hover || (!crowd && named.iter().all(|r| !r.intersects(room))) {
                        label(at, &sp.name, 13.0, col);
                        named.push(room);
                    }
                    if hover {
                        let ac = mp_proto::AIRCRAFT.get(sp.aircraft as usize).copied().unwrap_or("?");
                        let how = if sp.flags & mp_proto::F_GROUND != 0 { t!("on the ground", "au sol").to_string() } else { format!("{:.0} ft  ·  {} kt  ·  {:03.0}°", sp.alt_m() * 3.28084, sp.speed, sp.heading_deg()) };
                        label(p + egui::vec2(11.0, 15.0), &format!("{ac}  ·  {how}"), 11.5, Color32::from_rgba_unmultiplied(255, 255, 255, 220));
                    }
                }
            }
            if let Some((lat, lon)) = self.menu.pick {
                if let Some(p) = project(lat, lon) {
                    let accent = menu::ACCENT;
                    let pulse = 11.0 + 3.0 * (self.real * 3.0).sin() as f32;
                    painter.circle_stroke(p, pulse, egui::Stroke::new(2.0, accent));
                    painter.circle_filled(p, 3.5, accent);
                    let name = self.menu.airport.map(|i| airports::Db::get().apts[i as usize].ident.clone());
                    label(p + egui::vec2(18.0, 0.0), &name.unwrap_or_else(|| t!("Start here", "Départ ici").to_string()), 16.0, accent);
                }
            }
            if let Some(c) = aim {
                let st = egui::Stroke::new(1.5, Color32::from_white_alpha(200));
                let sh = egui::Stroke::new(3.0, Color32::from_black_alpha(110));
                for s in [sh, st] {
                    painter.circle_stroke(c, 9.0, s);
                    for d in [egui::vec2(1.0, 0.0), egui::vec2(-1.0, 0.0), egui::vec2(0.0, 1.0), egui::vec2(0.0, -1.0)] {
                        painter.line_segment([c + d * 13.0, c + d * 20.0], s);
                    }
                }
            }
            use menu::{ACCENT, DIM, TEXT};
            const W: f32 = 340.0;
            const M: f32 = 22.0;
            let screen = ctx.content_rect();
            // Top left: back to the main menu, then the aircraft card (hangar_ui.rs).
            let mut back = false;
            egui::Area::new(egui::Id::unique("menu_back")).fixed_pos(egui::pos2(M, M)).show(ctx, |ui| {
                let (r, resp) = ui.allocate_exact_size(egui::vec2(132.0, 34.0), egui::Sense::click());
                resp.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Button, true, t!("Menu", "Menu")));
                let lit = menu::ease(ctx, resp.id, resp.hovered() || resp.has_focus());
                let p = ui.painter();
                p.rect_filled(r, menu::R_WIDGET, menu::mix(menu::PANEL, Color32::from_rgba_premultiplied(40, 47, 58, 238), lit));
                p.rect_stroke(r, menu::R_WIDGET, egui::Stroke::new(1.0, menu::mix(menu::LINE_HI, Color32::from_white_alpha(70), lit)), egui::StrokeKind::Inside);
                menu::chevron(p, r.left_center() + egui::vec2(16.0 - 3.0 * lit, 0.0), 5.5, 2.0, menu::mix(TEXT, ACCENT, lit));
                p.text(r.left_center() + egui::vec2(30.0, 0.0), Align2::LEFT_CENTER, t!("Menu", "Menu"), menu::label(), menu::mix(TEXT, Color32::WHITE, lit));
                match Hint::of("Esc", "B") {
                    Hint::Text(h) => {
                        p.text(r.right_center() - egui::vec2(12.0, 0.0), Align2::RIGHT_CENTER, h, FontId::monospace(11.0), DIM);
                    }
                    Hint::Pad(spec) => {
                        let x = r.right() - 10.0 - padui::spec_width(p, spec, 18.0);
                        padui::paint_spec(p, x, r.center().y, spec, 18.0, DIM);
                    }
                    Hint::None => {}
                }
                if resp.clicked() {
                    back = true;
                }
                if self.menu.pick.is_none() {
                    self.pad_home_focus(&resp);
                }
            });
            if back {
                self.menu_back();
            }
            // Top middle, while multiplayer is on: how many fly (they are on
            // the globe), or how the connection stands; it opens its settings.
            if self.mp_on {
                let (col, line) = self.mp_line();
                let (title, sub) = match self.mp_flying() {
                    Some(n) => (n, line),
                    None => (line, String::new()),
                };
                let mut open = false;
                egui::Area::new(egui::Id::unique("menu_mp")).anchor(Align2::CENTER_TOP, [0.0, M]).show(ctx, |ui| {
                    let tg = ui.painter().layout_no_wrap(title.clone(), menu::label(), TEXT);
                    let sg = ui.painter().layout_no_wrap(sub.clone(), menu::caption(), DIM);
                    let w = (tg.size().x.max(sg.size().x) + 52.0).clamp(160.0, (screen.width() - 2.0 * (W + 2.0 * M)).max(160.0));
                    let h = if sub.is_empty() { 34.0 } else { 46.0 };
                    let (r, resp) = ui.allocate_exact_size(egui::vec2(w, h), egui::Sense::click());
                    resp.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Button, true, t!("Multiplayer", "Multijoueur")));
                    let lit = menu::ease(ctx, resp.id, resp.hovered() || resp.has_focus());
                    let p = ui.painter().with_clip_rect(r);
                    p.rect_filled(r, menu::R_WIDGET, menu::mix(menu::PANEL, Color32::from_rgba_premultiplied(40, 47, 58, 238), lit));
                    p.rect_stroke(r, menu::R_WIDGET, egui::Stroke::new(1.0, menu::mix(menu::LINE_HI, Color32::from_white_alpha(70), lit)), egui::StrokeKind::Inside);
                    p.circle_filled(egui::pos2(r.left() + 18.0, r.center().y), 4.0, col);
                    if sub.is_empty() {
                        p.galley(egui::pos2(r.left() + 32.0, r.center().y - tg.size().y * 0.5), tg, menu::mix(TEXT, Color32::WHITE, lit));
                    } else {
                        p.galley(egui::pos2(r.left() + 32.0, r.top() + 6.0), tg, menu::mix(TEXT, Color32::WHITE, lit));
                        p.galley(egui::pos2(r.left() + 32.0, r.bottom() - 6.0 - sg.size().y), sg, DIM);
                    }
                    if resp.clicked() {
                        open = true;
                    }
                });
                if open {
                    self.open_settings(settings_ui::Section::Multiplayer, menu::Screen::Fly);
                }
            }
            let r = self.aircraft_window(ctx, W, egui::pos2(M, M + 34.0 + 10.0));

            // Below it: where to start. Search, places and airports fold away;
            // the chosen spot and take-off stay in view.
            let top = r.bottom() + 12.0;
            // room for the pinned start point and take off, as last frame.
            let foot_h = ctx.data(|d| d.get_temp::<f32>(egui::Id::unique("globe_pinned_h"))).unwrap_or(150.0);
            // the list gives up height first so the panel never overlaps the aircraft card.
            let body_h = (screen.bottom() - M - top - 40.0 - 28.0 - foot_h).max(48.0);
            egui::Area::new(egui::Id::unique("menu")).anchor(Align2::LEFT_TOP, [M, top]).show(ctx, |ui| {
                ui.set_max_height(screen.bottom() - M - top);
                menu::panel().inner_margin(egui::Margin::symmetric(16, 14)).show(ui, |ui| {
                    ui.set_width(W - 32.0);
                    menu::window_title(ui, t!("Start location", "Lieu de départ"));
                    egui::ScrollArea::vertical().max_height(body_h).auto_shrink([false, true]).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        let summary = if self.menu.search.trim().is_empty() { String::new() } else { self.menu.search.chars().take(18).collect() };
                        menu::section(ui, "search", t!("Search", "Recherche"), &summary, true, |ui| {
                            ui.horizontal(|ui| {
                                let resp = ui.add(egui::TextEdit::singleline(&mut self.menu.search).desired_width(ui.available_width() - 56.0).hint_text(t!("City, address, or airport (SFO, EGLL)", "Ville, adresse ou aéroport (SFO, EGLL)")).margin(egui::vec2(8.0, 6.0)));
                                let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                                if (ui.add(egui::Button::new(RichText::new(t!("Go", "Aller")).font(menu::body())).min_size(egui::vec2(48.0, 28.0))).clicked() || enter) && !self.menu.search.trim().is_empty() {
                                    go = Some(self.menu.search.trim().to_string());
                                }
                            });
                            // Airports matching what is typed.
                            let hits = airports::Db::get().search(&self.menu.search, 5);
                            if !hits.is_empty() {
                                ui.add_space(2.0);
                                ui.horizontal_wrapped(|ui| {
                                    ui.spacing_mut().item_spacing = egui::vec2(5.0, 5.0);
                                    for i in hits {
                                        let a = &airports::Db::get().apts[i as usize];
                                        let label = format!("{}  {}", a.ident, a.name.chars().take(28).collect::<String>());
                                        if ui.add(egui::Button::new(RichText::new(label).font(menu::caption())).selected(self.menu.airport == Some(i))).clicked() {
                                            airport = Some(i);
                                        }
                                    }
                                });
                            }
                            ui.add_space(2.0);
                            let tip = if padui::active() { t!("Or turn the globe (left stick, triggers to zoom) and press X at the crosshair.", "Ou tournez le globe (stick gauche, gâchettes pour zoomer) et appuyez sur X au viseur.") } else { t!("Or drag the globe, scroll to zoom, and click where you want to start.", "Ou faites glisser le globe, zoomez à la molette et cliquez là où vous voulez partir.") };
                            ui.label(RichText::new(tip).font(menu::caption()).color(DIM));
                        });
                        let place = menu::QUICK.iter().find(|q| self.menu.pick == Some((q.1, q.2))).map(|q| tr(q.0)).unwrap_or("");
                        menu::section(ui, "places", t!("Places", "Lieux"), place, false, |ui| {
                            ui.horizontal_wrapped(|ui| {
                                ui.spacing_mut().item_spacing = egui::vec2(5.0, 5.0);
                                for (name, lat, lon) in menu::QUICK {
                                    let sel = self.menu.pick == Some((*lat, *lon));
                                    if ui.add(egui::Button::new(RichText::new(tr(name)).font(menu::caption())).selected(sel)).clicked() {
                                        jump = Some((*lat, *lon, name));
                                    }
                                }
                            });
                        });
                        let db = airports::Db::get();
                        let apt = self.menu.airport.map(|i| db.apts[i as usize].ident.clone()).unwrap_or_default();
                        menu::section(ui, "airports", t!("Airports", "Aéroports"), &apt, false, |ui| {
                            ui.horizontal_wrapped(|ui| {
                                ui.spacing_mut().item_spacing = egui::vec2(5.0, 5.0);
                                for (code, name) in menu::AIRPORTS {
                                    if let Some(i) = db.find(code) {
                                        if ui.add(egui::Button::new(RichText::new(*code).font(menu::data())).selected(self.menu.airport == Some(i))).on_hover_text(tr(name)).clicked() {
                                            airport = Some(i);
                                        }
                                    }
                                }
                            });
                            ui.label(RichText::new(if padui::active() { t!("Or put an airport square on the globe in the crosshair and press X.", "Ou placez un carré d'aéroport du globe dans le viseur et appuyez sur X.") } else { t!("Or click an airport square on the globe; double-click to go right away.", "Ou cliquez sur un carré d'aéroport du globe ; double-clic pour y aller directement.") }).font(menu::caption()).color(DIM));
                        });
                        padui::follow_focus(ui);
                    });
                    // pinned under the list: the start point, a summary and take off.
                    let pin_top = ui.cursor().top();
                    let db = airports::Db::get();
                    ui.add_space(12.0);
                    menu::heading(ui, t!("Start point", "Point de départ"));
                    ui.add_space(2.0);
                    match self.menu.pick {
                        Some((lat, lon)) => {
                            let name = if self.menu.pick_name.is_empty() { t!("Unnamed spot", "Lieu sans nom").to_string() } else { self.menu.pick_name.chars().take(90).collect() };
                            ui.add(egui::Label::new(RichText::new(name).font(menu::item_title()).color(Color32::WHITE)).wrap());
                            ui.label(RichText::new(t!("{}  ·  {} local", "{}  ·  {} heure locale", menu::latlon(lat, lon), menu::hhmm(self.menu.hour))).font(menu::caption()).color(DIM));
                            ui.add_space(6.0);
                            match self.menu.airport {
                                Some(i) => {
                                    let a = &db.apts[i as usize];
                                    menu::segments(ui, |ui| {
                                        menu::seg(ui, &mut self.menu.start, menu::Start::Runway, t!("Runway", "Piste"));
                                        menu::seg(ui, &mut self.menu.start, menu::Start::Approach, t!("Final", "Finale"));
                                        menu::seg(ui, &mut self.menu.start, menu::Start::Overhead, t!("Overhead", "Au-dessus"));
                                    });
                                    if self.menu.start != menu::Start::Overhead && !a.rwys.is_empty() {
                                        ui.add_space(4.0);
                                        let into_wind = t!("Into the wind", "Face au vent");
                                        ui.horizontal(|ui| {
                                            ui.label(RichText::new(t!("Runway", "Piste")).font(menu::label()).color(TEXT));
                                            egui::ComboBox::from_id_salt("start_runway").width(ui.available_width()).selected_text(self.menu.runway.clone().unwrap_or_else(|| into_wind.to_string())).show_ui(ui, |ui| {
                                                ui.selectable_value(&mut self.menu.runway, None, into_wind);
                                                for r in &a.rwys {
                                                    for id in r.ident.iter().filter(|id| !id.is_empty()) {
                                                        let what = format!("{id}   {:.0} m{}", r.len, if r.paved { "" } else { t!(", unpaved", ", non revêtue") });
                                                        ui.selectable_value(&mut self.menu.runway, Some(id.clone()), what);
                                                    }
                                                }
                                            });
                                        });
                                    }
                                }
                                None => {
                                    // offer the nearest airfield.
                                    let near = db.near(lat, lon, 60_000.0).into_iter().find(|(i, _)| db.apts[*i as usize].any_paved());
                                    if let Some((i, d)) = near {
                                        let a = &db.apts[i as usize];
                                        ui.horizontal(|ui| {
                                            ui.label(RichText::new(t!("Nearest airport {} · {:.0} km", "Aéroport le plus proche : {} · {:.0} km", a.ident, d / 1000.0)).font(menu::caption()).color(DIM));
                                            if ui.button(t!("Start there", "Partir de là")).clicked() {
                                                airport = Some(i);
                                            }
                                        });
                                    }
                                }
                            }
                        }
                        None => {
                            ui.label(RichText::new(t!("Pick a place, or click the globe.", "Choisissez un lieu, ou cliquez sur le globe.")).font(menu::body()).color(DIM));
                        }
                    }
                    ui.add_space(14.0);
                    ui.painter().hline(ui.max_rect().x_range(), ui.cursor().top() - 7.0, egui::Stroke::new(1.0, menu::LINE));
                    // the whole flight in one line.
                    if self.menu.pick.is_some() {
                        let ground = self.menu.airport.is_some() && self.menu.start != menu::Start::Overhead;
                        // the aircraft is named on its card.
                        let how = if ground {
                            let first = if self.menu.start == menu::Start::Runway { t!("Runway", "Piste") } else { t!("Final", "Finale") };
                            match &self.menu.runway {
                                Some(rw) => format!("{first} {rw}"),
                                None => t!("{}, into the wind", "{}, face au vent", first),
                            }
                        } else {
                            format!("{:.0} ft · {:03.0}°", (self.menu.start_agl * 3.28084 / 50.0).round() * 50.0, self.heading.to_degrees().rem_euclid(360.0))
                        };
                        let wx = if self.wx.live { t!("real weather", "météo réelle").to_string() } else { self.wx.summary().to_lowercase() };
                        let line = format!("{how}  ·  {}  ·  {wx}", menu::hhmm(self.menu.hour));
                        ui.add(egui::Label::new(RichText::new(line).font(menu::caption()).color(TEXT)).wrap());
                        ui.add_space(8.0);
                    }
                    let text = if self.menu.starting { t!("Preparing…", "Préparation…") } else { t!("TAKE OFF", "DÉCOLLER") };
                    let ready = self.menu.pick.is_some() && !self.menu.starting;
                    let take_off = menu::wide_button(ui, text, if ready { Hint::of("Enter", "Menu") } else { Hint::None }, true, ready);
                    if take_off.clicked() {
                        start = true;
                    }
                    if ready {
                        self.pad_home_focus(&take_off);
                    }
                    ctx.data_mut(|d| d.insert_temp(egui::Id::unique("globe_pinned_h"), ui.cursor().top() - pin_top));
                });
            });

            // On the right: how the flight starts, the weather, the time.
            let right_h = (screen.height() - M - 64.0 - 28.0).max(200.0);
            egui::Area::new(egui::Id::unique("menu_setup")).anchor(Align2::RIGHT_TOP, [-M, M]).show(ctx, |ui| {
                // (an area lays out in last frame's size: let it grow as sections open)
                ui.set_max_height(right_h);
                menu::panel().inner_margin(egui::Margin::symmetric(16, 14)).show(ui, |ui| {
                    ui.set_width(320.0);
                    menu::window_title(ui, t!("Conditions", "Conditions"));
                    egui::ScrollArea::vertical().max_height(right_h - 40.0).auto_shrink([false, true]).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        ui.spacing_mut().slider_width = 116.0;
                        let ground = self.menu.airport.is_some() && self.menu.start != menu::Start::Overhead;
                        let flight = if ground {
                            (if self.menu.start == menu::Start::Runway { t!("On the runway", "Sur la piste") } else { t!("Final approach", "Approche finale") }).to_string()
                        } else {
                            format!("{:.0} ft · {:03.0}°", (self.menu.start_agl * 3.28084 / 50.0).round() * 50.0, self.heading.to_degrees().rem_euclid(360.0))
                        };
                        menu::section(ui, "flight", t!("Flight", "Vol"), &flight, false, |ui| {
                            if ground {
                                ui.label(RichText::new(if self.menu.start == menu::Start::Runway { t!("Starting on the runway, ready for take-off.", "Départ sur la piste, prêt au décollage.") } else { t!("Starting on a 3° final approach, gear and flaps down.", "Départ en approche finale à 3°, train et volets sortis.") }).font(menu::caption()).color(DIM));
                            } else {
                                ui.label(RichText::new(t!("Starting in the air:", "Départ en vol :")).font(menu::caption()).color(DIM));
                            }
                            ui.horizontal(|ui| {
                                ui.label(t!("Fuel", "Carburant"));
                                let kg = self.spec.fuel_kg;
                                ui.add(egui::Slider::new(&mut self.menu.fuel, 0.05..=1.0).custom_formatter(move |v, _| format!("{:.0}%  ·  {:.0} kg", v * 100.0, v * kg)));
                            });
                            let [usual, most, _] = self.spec.payload;
                            if most > usual {
                                ui.horizontal(|ui| {
                                    ui.label(t!("Payload", "Charge"));
                                    let mut f = self.menu.payload.unwrap_or(usual / most);
                                    if ui.add(egui::Slider::new(&mut f, 0.15..=1.0).custom_formatter(move |v, _| format!("{:.0} kg", v * most))).changed() {
                                        self.menu.payload = Some(f);
                                    }
                                });
                            }
                            {
                                // Mass and balance as it will be loaded.
                                let mut probe = Plane::spawn(self.spec, 0.0, 0.0, 1000.0, 0.0, 50.0);
                                probe.fuel = self.spec.fuel_kg * self.menu.fuel;
                                probe.payload = self.start_payload();
                                let s = self.spec;
                                let out = probe.cg_excess() != 0.0;
                                let text = t!("{:.0} kg  ·  CG {:.0}% MAC (limits {:.0}-{:.0}%){}", "{:.0} kg  ·  centrage {:.0} % MAC (limites {:.0}-{:.0} %){}", probe.mass(), probe.cg() * 100.0, s.cg[1] * 100.0, s.cg[2] * 100.0, if out { t!("  OUT OF LIMITS", "  HORS LIMITES") } else { "" });
                                ui.label(RichText::new(text).font(menu::caption()).color(if out { Color32::from_rgb(255, 120, 70) } else { DIM }));
                            }
                            if ground {
                                ui.checkbox(&mut self.menu.cold, t!("Engine off (cold start: F4 starts it)", "Moteur arrêté (démarrage à froid : F4 pour démarrer)"));
                            }
                            ui.add_enabled_ui(!ground, |ui| {
                                egui::Grid::new("start_air").num_columns(2).min_col_width(92.0).spacing([10.0, 5.0]).show(ui, |ui| {
                                    ui.label(t!("Height", "Hauteur"));
                                    let mut ft = self.menu.start_agl * 3.28084;
                                    if ui.add(egui::Slider::new(&mut ft, 100.0..=15000.0).logarithmic(true).step_by(50.0).suffix(" ft").custom_formatter(|v, _| format!("{:.0}", (v / 50.0).round() * 50.0))).changed() {
                                        self.menu.start_agl = ft / 3.28084;
                                    }
                                    ui.end_row();
                                    ui.label("");
                                    ui.label(RichText::new(t!("above the ground", "au-dessus du sol")).font(menu::caption()).weak());
                                    ui.end_row();
                                    ui.label(t!("Heading", "Cap"));
                                    let mut hd = self.heading.to_degrees().rem_euclid(360.0);
                                    if ui.add(egui::Slider::new(&mut hd, 0.0..=359.0).step_by(1.0).custom_formatter(|v, _| format!("{v:03.0}°"))).changed() {
                                        self.heading = hd.to_radians();
                                    }
                                    ui.end_row();
                                    if !ground {
                                        ui.label("");
                                        menu::heading_compass(ui, &mut self.heading);
                                        ui.end_row();
                                    }
                                });
                            });
                        });
                        let wx = self.wx.summary();
                        menu::section(ui, "weather", t!("Weather", "Météo"), &wx, false, |ui| {
                            self.wx.start_weather(ui);
                        });
                        let when = format!("{} · {}", menu::hhmm(self.menu.hour), weather::date_label(self.doy));
                        menu::section(ui, "time", t!("Time", "Heure"), &when, false, |ui| {
                            weather::Weather::start_time(ui, &mut self.menu.hour, &mut self.doy, &mut self.wx.real_time);
                        });
                        padui::follow_focus(ui);
                    });
                });
            });
            self.credits(ctx);
            padui::focus_ring(ctx, ACCENT);
        });
        self.egui_state.handle_platform_output(&self.window, out.platform_output);
        if let Some((lat, lon, name)) = jump {
            self.menu.look_at(lat, lon, 30_000.0);
            self.menu.pick = Some((lat, lon));
            self.menu.pick_name = tr(name).to_string();
            self.menu.airport = None;
            self.menu.runway = None;
        }
        if let Some(i) = airport {
            self.choose_airport(i);
        }
        let want = aircraft::nth(self.menu.aircraft);
        if !std::ptr::eq(want, self.spec) {
            self.want_aircraft(want);
        }
        if let Some(i) = go.as_deref().and_then(|q| airports::Db::get().find(q)) {
            self.choose_airport(i);
            go = None;
        }
        if let Some(q) = go {
            self.menu.pick_name = t!("Searching for {}…", "Recherche de {}…", q);
            let (tx, rx) = std::sync::mpsc::channel();
            let src = self.src.clone();
            std::thread::spawn(move || {
                let _ = tx.send(geocode(&src.fetch, &q).map(|(la, lo, name)| (la, lo, 0.0, name)));
            });
            self.goto_rx = Some(rx);
        }
        if start {
            self.begin();
        }
        let jobs = self.egui_ctx.tessellate(out.shapes, out.pixels_per_point);
        (jobs, out.textures_delta, out.pixels_per_point)
    }

    /// The data credits for the corner of the screen (the Google Maps logo
    /// goes before them while Google's data is shown, see `credits`).
    fn attribution(&self) -> String {
        let g = &self.src.google;
        let p = self.world.photo();
        let photo = if p.zone_r > 0.0 && p.stats.0 > 0 { format!("3D: {}  ·  ", if p.credits.is_empty() { "Google" } else { &p.credits }) } else { String::new() };
        let mut live = if self.live.enabled() && !self.live.planes.is_empty() { format!("  ·  {}", t!("Traffic: adsb.lol (ODbL)", "Trafic : adsb.lol (ODbL)")) } else { String::new() };
        if self.wx.live && (!self.wx.live_cells.is_empty() || self.wx.deck_goal.is_some()) {
            live += t!("  ·  Weather radar: RainViewer", "  ·  Radar météo : RainViewer");
        }
        if self.mode != Mode::Menu && self.snow == snow::Mode::Auto && self.snowmap.shown().is_some() {
            live += t!("  ·  Snow cover: IMS, U.S. National Ice Center (NSIDC)", "  ·  Enneigement : IMS, U.S. National Ice Center (NSIDC)");
        }
        if self.mode == Mode::Menu && self.gclouds.shown {
            live += t!("  ·  Clouds: Live Cloud Maps (Matt Eason), contains modified EUMETSAT data", "  ·  Nuages : Live Cloud Maps (Matt Eason), contient des données EUMETSAT modifiées");
        }
        photo + &if g.enabled() {
            let c = g.attribution.lock();
            let c = if c.is_empty() { t!("Imagery © Google", "Imagerie © Google").to_string() } else { c.clone() };
            format!("{c}  ·  {}", attribution_rest())
        } else {
            format!("{}  ·  {}", attribution_default(), attribution_rest())
        } + &live
    }

    /// The bottom-right credits: Google's logo (its official outlined
    /// artwork, unaltered, 22 px with its outline, clear space round it) while
    /// their imagery or 3D tiles are on screen, then the data credits.
    fn credits(&mut self, ctx: &egui::Context) {
        use egui::{Align2, Color32, RichText};
        let logo = self.google_shown().then(|| {
            self.gm_logo
                .get_or_insert_with(|| {
                    let img = image::load_from_memory(include_bytes!("../assets/google_maps_logo.png")).expect("logo").to_rgba8();
                    let size = [img.width() as usize, img.height() as usize];
                    ctx.load_texture("google maps logo", egui::ColorImage::from_rgba_unmultiplied(size, img.as_raw()), egui::TextureOptions::LINEAR)
                })
                .clone()
        });
        let text = self.attribution();
        // The logo on a line of its own at the right, above the credits (which
        // can run long), so nothing else on screen comes near it.
        egui::Area::new(egui::Id::unique("attr")).anchor(Align2::RIGHT_BOTTOM, [-10.0, -5.0]).show(ctx, |ui| {
            ui.with_layout(egui::Layout::bottom_up(egui::Align::Max), |ui| {
                ui.label(RichText::new(text).size(10.0).color(Color32::from_rgba_unmultiplied(255, 255, 255, 110)));
                if let Some(t) = logo {
                    let s = t.size_vec2();
                    ui.add_space(4.0);
                    ui.add(egui::Image::new(&t).fit_to_exact_size(egui::vec2(s.x * 22.0 / s.y, 22.0)).alt_text("Google Maps"));
                }
            });
        });
    }

    /// Whether Google's data is on screen (its credit is shown, HUD or not).
    fn google_shown(&self) -> bool {
        self.src.google.enabled() || self.world.photo().stats.0 > 0
    }

    /// Hands changed graphics settings to the renderer and the world.
    fn apply_graphics(&mut self) {
        let g = self.gfx;
        if self.gfx_applied == Some(g) {
            return;
        }
        self.r.set_scale(g.scale);
        self.r.set_shadow_res(g.shadow_res());
        self.r.set_vsync(g.vsync);
        self.r.ao = g.ao;
        self.r.cloud_step = g.cloud_step();
        self.world.lod_k = g.lod_k();
        self.world.reach_k = g.reach();
        self.world.city_k = g.city_k();
        self.world.tree_far = g.tree_far();
        self.world.tree_imp_k = g.tree_imp_k();
        self.world.cars = g.car_density();
        if self.gfx_applied.is_some() {
            log::info!("graphics: {} at {:.0}%, shadows {}, clouds {}, terrain {}, scenery {}, city {}, trees {}, traffic {}, AO {}", g.matches().map(graphics::Level::name).unwrap_or("custom"), g.scale * 100.0, g.shadows.name(), g.clouds.name(), g.terrain.name(), g.scenery.name(), g.city.name(), g.trees.name(), g.cars.name(), g.ao);
        }
        self.gfx_applied = Some(g);
    }

    /// The pause menu (Esc): full screen over the dimmed, stopped game. A
    /// column of big buttons on the left; the key list and the failures open
    /// in a panel beside it; Settings opens the settings screen (settings_ui.rs).
    fn pause_menu(&mut self, ctx: &egui::Context, act: &mut Option<PauseAct>, lat: f64, lon: f64, local: f64) {
        use egui::{Align2, Color32, RichText};
        use menu::{Page, ACCENT, DIM, TEXT};
        if self.pause_page.is_info() {
            self.pause_page = Page::Game;
        }
        if self.pause_page == Page::Settings {
            if settings_ui::show(ctx, self, true) {
                self.pause_page = Page::Game;
                self.pad_home = true;
            }
            return;
        }
        // The backdrop dims the scene (darkest behind the buttons) and keeps the mouse from the camera behind it.
        let screen = ctx.content_rect();
        egui::Area::new(egui::Id::unique("pause_dim")).order(egui::Order::Middle).fixed_pos(screen.min).show(ctx, |ui| {
            ui.allocate_response(screen.size(), egui::Sense::click());
            let p = ui.painter();
            p.rect_filled(screen, 0.0, Color32::from_black_alpha(150));
            let mut mesh = egui::Mesh::default();
            let (l, r) = (Color32::from_black_alpha(130), Color32::TRANSPARENT);
            let mid = screen.left() + screen.width() * 0.6;
            mesh.colored_vertex(screen.left_top(), l);
            mesh.colored_vertex(egui::pos2(mid, screen.top()), r);
            mesh.colored_vertex(egui::pos2(mid, screen.bottom()), r);
            mesh.colored_vertex(screen.left_bottom(), l);
            mesh.add_triangle(0, 1, 2);
            mesh.add_triangle(0, 2, 3);
            p.add(mesh);
        });
        let pad = padui::active();
        let margin = if screen.width() < 1100.0 { 40.0 } else { 72.0 };
        let col_w = 380.0f32.min(screen.width() * 0.4);
        egui::Area::new(egui::Id::unique("pause")).order(egui::Order::Foreground).anchor(Align2::LEFT_CENTER, [margin, 0.0]).show(ctx, |ui| {
            ui.set_width(col_w);
            menu::screen_heading(ui, t!("Paused", "Pause"));
            ui.add_space(22.0);
            ui.spacing_mut().item_spacing.y = 8.0;
            let page = self.pause_page;
            let resume = menu::big_button(ui, t!("Resume", "Reprendre"), Hint::of("Esc", "B"), true, false);
            if resume.clicked() {
                *act = Some(PauseAct::Resume);
            }
            if page == Page::Game {
                self.pad_home_focus(&resume);
            }
            ui.add_space(8.0);
            for (p, text) in [(Page::Settings, t!("Settings", "Réglages")), (Page::Controls, t!("Controls reference", "Aide-mémoire des commandes")), (Page::Failures, t!("Failures", "Pannes"))] {
                let b = menu::big_button(ui, text, Hint::None, false, page == p);
                if b.clicked() {
                    self.pause_page = if page == p && p != Page::Settings { Page::Game } else { p };
                    self.pad_home = p == Page::Settings;
                }
                if page == p {
                    self.pad_home_focus(&b);
                }
            }
            ui.add_space(14.0);
            if menu::big_button(ui, t!("Respawn in the air", "Réapparaître en vol"), Hint::of("R", ""), false, false).clicked() {
                *act = Some(PauseAct::Respawn);
            }
            // back to the globe for a new start.
            if menu::big_button(ui, t!("End flight", "Terminer le vol"), Hint::Text(t!("pick a new start", "choisir un nouveau départ")), false, false).clicked() {
                *act = Some(PauseAct::Globe);
            }
            ui.add_space(14.0);
            // quit as a plain line.
            let q = ui.add(egui::Button::new(RichText::new(t!("Quit to desktop", "Quitter vers le bureau")).font(menu::label()).color(DIM)).frame(false));
            if q.clicked() {
                *act = Some(PauseAct::Quit);
            }
            ui.add_space(18.0);
            if pad {
                padui::prompts(ui, &[("Dpad", tr("move")), ("A", tr("press")), ("B", tr("back"))], 20.0, 12.0, DIM);
            } else {
                let t = t!("While you fly, Tab opens the flight menu: places, radio, autopilot, weather, the map and the camera.", "En vol, Tab ouvre le menu de vol : lieux, radio, pilote auto, météo, carte et caméra.");
                ui.add(egui::Label::new(RichText::new(t).font(menu::caption()).color(DIM)).wrap());
            }
        });

        // Where and when, small in the top right corner.
        egui::Area::new(egui::Id::unique("pause_where")).order(egui::Order::Foreground).anchor(Align2::RIGHT_TOP, [-margin * 0.5, margin * 0.5]).show(ctx, |ui| {
            ui.with_layout(egui::Layout::top_down(egui::Align::Max), |ui| {
                ui.spacing_mut().item_spacing.y = 3.0;
                ui.label(RichText::new(self.spec.name).font(menu::item_title()).color(TEXT));
                ui.label(RichText::new(menu::latlon(lat, lon)).font(menu::caption()).color(DIM));
                ui.label(RichText::new(t!("{} local", "{} heure locale", menu::hhmm(local))).font(menu::caption()).color(DIM));
            });
        });

        // The key list or the failures, beside the buttons.
        if matches!(self.pause_page, Page::Controls | Page::Failures) {
            let x = margin + col_w + 48.0;
            let w = (screen.width() - x - margin).clamp(300.0, 820.0);
            let h = screen.height() - 2.0 * margin.max(64.0);
            egui::Area::new(egui::Id::unique("pause_page")).order(egui::Order::Foreground).fixed_pos(egui::pos2(x, margin.max(64.0))).show(ctx, |ui| {
                menu::panel().inner_margin(24.0).show(ui, |ui| {
                    ui.set_width(w - 48.0);
                    ui.set_max_height(h - 48.0);
                    let title = if self.pause_page == Page::Controls { t!("Controls reference", "Aide-mémoire des commandes") } else { t!("Failures", "Pannes") };
                    ui.label(RichText::new(title).font(menu::page_title()).color(Color32::WHITE));
                    ui.add_space(10.0);
                    egui::ScrollArea::vertical().auto_shrink([false, true]).show(ui, |ui| {
                        if self.pause_page == Page::Controls {
                            if !self.binds.keys.is_empty() {
                                ui.label(RichText::new(t!("Some keys have been moved: see Settings › Controls.", "Certaines touches ont été déplacées : voir Réglages › Commandes.")).font(menu::caption()).color(ACCENT));
                            }
                            menu::controls_list(ui);
                        } else {
                            self.failures_page(ui);
                        }
                        padui::follow_focus(ui);
                    });
                });
            });
        }
    }

    /// The flight panel (Tab): a see-through column on the right, over the
    /// game that keeps running: instruments, places to go and the weather.
    #[allow(clippy::too_many_arguments)]
    fn info_panel(&mut self, ctx: &egui::Context, act: &mut Option<PauseAct>, lat: f64, lon: f64, local: f64, speed: f64, alt: f64, agl: f64, hdg: f64) {
        use egui::{Align2, Color32, RichText};
        use flightui::{chip, kv, list_row, pane_section, tiles};
        use menu::{Page, TEXT};
        if !self.info_page.is_info() {
            self.info_page = Page::Flight;
        }
        let warn = Color32::from_rgb(255, 196, 80);
        let bad = menu::BAD;
        // left of the altitude tape, under the heading strip.
        let (x, y) = if self.hud_on() { (-flightui::RIGHT_EDGE - 10.0, flightui::HEADING_H + 34.0) } else { (-14.0, 12.0) };
        // what is left below it, less the header, the margins and a gap.
        let max_h = (ctx.content_rect().height() - y - 16.0 - 32.0 - 48.0).max(120.0);
        egui::Area::new(egui::Id::unique("info_panel")).anchor(Align2::RIGHT_TOP, [x, y]).show(ctx, |ui| {
            flightui::frosted(ui, flightui::glass().inner_margin(16.0), |ui| {
                ui.set_width(flightui::PANEL_W);
                // the page's icon and name, picked from the flight menu, and a close button.
                ui.horizontal(|ui| {
                    let ic = match self.info_page {
                        Page::Places => menu::Icon::GoTo,
                        Page::Radio => menu::Icon::Radio,
                        Page::Autopilot => menu::Icon::Autopilot,
                        Page::Weather => menu::Icon::Weather,
                        _ => menu::Icon::Flight,
                    };
                    let (r, _) = ui.allocate_exact_size(egui::vec2(28.0, 28.0), egui::Sense::hover());
                    menu::icon(ui.painter(), ic, r.center(), 22.0, Color32::WHITE);
                    ui.add_space(4.0);
                    ui.label(RichText::new(self.info_page.label()).font(menu::page_title()).color(Color32::WHITE));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let x = chip(ui, "×", true, false).on_hover_text(t!("Close (Tab)", "Fermer (Tab)"));
                        self.pad_home_focus(&x);
                        if x.clicked() {
                            self.info_open = false;
                        }
                    });
                });
                ui.add_space(4.0);
                egui::ScrollArea::vertical().max_height(max_h).min_scrolled_height(max_h).auto_shrink([false, true]).show(ui, |ui| {
                    ui.set_width(flightui::PANEL_W);
                    ui.spacing_mut().item_spacing.y = 2.0;
                    match self.info_page {
                        Page::Places => {
                            ui.add_space(8.0);
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = 8.0;
                                let w = (ui.available_width() - 8.0) * 0.5;
                                if flightui::pane_button(ui, t!("Respawn here", "Réapparaître ici"), w).on_hover_text("R").clicked() {
                                    *act = Some(PauseAct::Respawn);
                                }
                                if flightui::pane_button(ui, t!("Search a place", "Chercher un lieu"), w).on_hover_text("G").clicked() {
                                    *act = Some(PauseAct::GoTo);
                                }
                            });
                            let db = airports::Db::get();
                            let (la, lo, _) = geo::pos_to_lla(self.plane.pos);
                            pane_section(ui, t!("Nearest airports", "Aéroports les plus proches"));
                            let near: Vec<(u32, f64)> = db.near(la.to_degrees(), lo.to_degrees(), 120_000.0).into_iter().filter(|(i, _)| { let a = &db.apts[*i as usize]; a.any_paved() || a.kind > 0 }).take(6).collect();
                            if near.is_empty() {
                                ui.label(RichText::new(t!("No airfield within 120 km.", "Aucun aérodrome à moins de 120 km.")).font(menu::caption()).color(flightui::SOFT));
                            }
                            let here = geo::enu(self.plane.pos);
                            for (i, d) in near {
                                let a = &db.apts[i as usize];
                                let to = geo::lla_to_pos(a.lat.to_radians(), a.lon.to_radians(), 0.0) - self.plane.pos;
                                let brg = to.dot(here.0).atan2(to.dot(here.1)).to_degrees().rem_euclid(360.0);
                                let detail = format!("{:.0} km  {:03.0}°  ·  {:.0} m", d / 1000.0, brg, a.longest());
                                list_row(ui, &a.ident, &a.name, &detail, |ui| {
                                    if chip(ui, t!("Final", "Finale"), false, false).on_hover_text(t!("Start on a final approach", "Partir en approche finale")).clicked() {
                                        *act = Some(PauseAct::Airport(i, true));
                                    }
                                    if chip(ui, t!("Runway", "Piste"), false, false).on_hover_text(t!("Line up on the runway", "S'aligner sur la piste")).clicked() {
                                        *act = Some(PauseAct::Airport(i, false));
                                    }
                                });
                            }
                            pane_section(ui, t!("Airports around the world", "Aéroports du monde"));
                            let tip = if padui::active() { t!("Lines up on the runway.", "Aligne sur la piste.") } else { t!("Lines up on the runway; shift-click for a final approach.", "Aligne sur la piste ; Maj+clic pour une approche finale.") };
                            ui.label(RichText::new(tip).font(menu::caption()).color(flightui::SOFT));
                            ui.add_space(4.0);
                            ui.horizontal_wrapped(|ui| {
                                ui.spacing_mut().item_spacing = egui::vec2(4.0, 4.0);
                                for (code, name) in menu::AIRPORTS {
                                    if let Some(i) = db.find(code) {
                                        if chip(ui, code, true, false).on_hover_text(tr(name)).clicked() {
                                            *act = Some(PauseAct::Airport(i, ui.input(|inp| inp.modifiers.shift)));
                                        }
                                    }
                                }
                            });
                            pane_section(ui, t!("Fly to", "Aller à"));
                            ui.horizontal_wrapped(|ui| {
                                ui.spacing_mut().item_spacing = egui::vec2(4.0, 4.0);
                                for (name, la, lo) in menu::QUICK {
                                    if chip(ui, tr(name), false, false).clicked() {
                                        *act = Some(PauseAct::FlyTo(*la, *lo, name));
                                    }
                                }
                            });
                        }
                        Page::Radio => self.radio_page(ui),
                        Page::Autopilot => self.autopilot_page(ui),
                        Page::Weather => {
                            ui.spacing_mut().slider_width = 150.0;
                            self.wx.controls(ui, &mut self.utc, &mut self.doy, lon, false);
                        }
                        _ => {
                            let mut engine_btn: Option<bool> = None;
                            let p = &self.plane;
                            let (_, _, u) = geo::enu(p.pos);
                            let vs = p.vel.dot(u);
                            let wind_kt = self.wx.wind * 1.94384;
                            let rpm = p.prop_rate * 60.0 / std::f64::consts::TAU;
                            let jet = p.spec.is_jet();
                            let trk = { let (e, n, _) = geo::enu(p.pos); p.vel.dot(e).atan2(p.vel.dot(n)).to_degrees().rem_euclid(360.0) };
                            // the airport here.
                            if let Some(lines) = self.airport_hud() {
                                pane_section(ui, t!("Airport", "Aéroport"));
                                ui.add(egui::Label::new(RichText::new(&lines[0].0).font(menu::item_title()).color(Color32::WHITE)).wrap());
                                for (l, _) in lines.iter().skip(1) {
                                    ui.add(egui::Label::new(RichText::new(l).font(menu::caption()).color(TEXT)).wrap());
                                }
                            }
                            let p = &self.plane;
                            pane_section(ui, t!("Flight", "Vol"));
                            let stall = if p.stalled { bad } else { Color32::WHITE };
                            tiles(ui, &[
                                (t!("Airspeed", "Vitesse"), format!("{:.0}", speed * 1.94384), "kt", stall),
                                (t!("Altitude", "Altitude"), format!("{:.0}", alt * 3.28084), "ft", Color32::WHITE),
                                (t!("Heading", "Cap"), format!("{hdg:03.0}"), "°", Color32::WHITE),
                                (t!("Vertical speed", "Vario"), format!("{:+.0}", vs * 196.85), "fpm", Color32::WHITE),
                            ]);
                            kv(ui, t!("True airspeed", "Vitesse vraie"), &format!("{:.0} kt", p.airspeed().length() * 1.94384), TEXT);
                            kv(ui, t!("Ground speed, track", "Vitesse sol, route"), &format!("{:.0} kt  {trk:03.0}°", p.vel.length() * 1.94384), TEXT);
                            let set = self.radios.setting(self.wx.qnh);
                            kv(ui, t!("Altimeter", "Altimètre"), &format!("{:.0} ft  {:.0} hPa", alt * 3.28084 + self.radios.alt_error_ft(self.wx.qnh), set), TEXT);
                            kv(ui, t!("Above ground", "Hauteur sol"), &format!("{:.0} ft", agl.max(0.0) * 3.28084), TEXT);
                            kv(ui, t!("Angle of attack", "Incidence"), &format!("{:.1}°", p.alpha.to_degrees()), if p.stalled { bad } else { TEXT });
                            kv(ui, t!("Load", "Facteur de charge"), &format!("{:.1} g", p.g_load), if p.g_load.abs() > 3.8 { warn } else { TEXT });

                            pane_section(ui, p.spec.name);
                            if let Some((name, lit, n)) = p.engines() {
                                kv(ui, t!("Stage", "Étage"), p.stage_name().unwrap_or(""), TEXT);
                                kv(ui, t!("Engines", "Moteurs"), &t!("{}  {} of {} lit", "{}  {} sur {} allumés", name, lit, n), TEXT);
                                kv(ui, t!("Thrust", "Poussée"), &format!("{:.0}%  {:.1} MN", p.throttle * 100.0, p.thrust_now() / 1.0e6), TEXT);
                            } else if jet {
                                kv(ui, t!("Thrust", "Poussée"), &format!("{:.0}%  N1 {:.0}%{}", p.throttle * 100.0, p.n1 * 100.0, if p.reverser > 0.0 { "  REV" } else { "" }), TEXT);
                            } else {
                                kv(ui, t!("Throttle", "Gaz"), &format!("{:.0}%  {:.0} rpm", p.throttle * 100.0, rpm), if rpm > 2710.0 { warn } else { TEXT });
                            }
                            let state = if p.running {
                                t!("running", "en marche").to_string()
                            } else if p.starter_on || (p.prop_rate > 5.0 && !p.spec.is_jet() && p.ignition) {
                                t!("starting", "démarrage").to_string()
                            } else {
                                t!("off", "arrêté").to_string()
                            };
                            kv(ui, t!("Engine", "Moteur"), &state, if p.running { TEXT } else { warn });
                            if !p.spec.is_jet() {
                                kv(ui, t!("Magnetos, mixture", "Magnétos, mélange"), &format!("{}  {:.0}%", if p.ignition { "ON" } else { "OFF" }, p.mixture * 100.0), TEXT);
                            } else {
                                kv(ui, t!("Fuel control", "Commande carburant"), if p.ignition { "RUN" } else { "CUTOFF" }, TEXT);
                            }
                            match &p.spec.lever {
                                Some(l) => kv(ui, l.name[i18n::is_fr() as usize], l.notch(p.flap_set), TEXT),
                                None => kv(ui, t!("Flaps", "Volets"), &format!("{:.0}°", p.flap_notch_deg()), TEXT),
                            }
                            if p.spec.spoiler_cd > 0.0 {
                                kv(ui, t!("Speedbrake", "Aérofreins"), if p.spoilers > 0.6 { t!("ground spoilers", "spoilers sol") } else if p.speedbrake > 0.0 { t!("flight detent", "cran vol") } else { t!("down", "rentrés") }, TEXT);
                            }
                            kv(ui, t!("Gear", "Train"), if p.gear >= 1.0 { t!("down", "sorti") } else if p.gear <= 0.0 { t!("up", "rentré") } else { t!("moving", "en mouvement") }, if p.gear > 0.0 && p.gear < 1.0 { warn } else { TEXT });
                            kv(ui, t!("Fuel", "Carburant"), &format!("{:.0} kg  {:.0}%", p.fuel, p.fuel_frac() * 100.0), if p.fuel_frac() < 0.1 { warn } else { TEXT });
                            let flow = match p.endurance() {
                                Some(e) => format!("{:.0} kg/h  {}h{:02}", p.fuel_flow * 3600.0, (e / 3600.0) as u32, (e / 60.0) as u32 % 60),
                                None => "—".into(),
                            };
                            kv(ui, t!("Fuel flow, endurance", "Débit, autonomie"), &flow, TEXT);
                            kv(ui, t!("Mass, balance", "Masse, centrage"), &format!("{:.0} kg  {:.0}% MAC", p.mass(), p.cg() * 100.0), if p.cg_excess() != 0.0 { bad } else { TEXT });
                            let el = &p.sys.elec;
                            let elec = if !el.master { t!("master off (Ins)", "master coupé (Inser)").to_string() } else { format!("{:.1} V  {:.0}%  {:+.0} A", el.volts, el.charge * 100.0, el.amps) };
                            kv(ui, t!("Electrics", "Électricité"), &elec, if p.sys.powered() { TEXT } else { warn });
                            if p.spec.chute.is_some() {
                                kv(ui, t!("Drag chute", "Parachute-frein"), if p.spoilers > 0.0 { t!("out", "sorti") } else if p.chute_spent { t!("gone", "largué") } else { t!("packed", "plié") }, TEXT);
                            }
                            if p.sys.fail.any() {
                                let list: Vec<&str> = (0..systems::Failures::N).filter(|&i| p.sys.fail.get(i)).map(systems::Failures::label).collect();
                                kv(ui, t!("Failures", "Pannes"), &list.join(", "), bad);
                            }
                            let report = p.dmg.report();
                            if self.crashed {
                                kv(ui, t!("Damage", "Dégâts"), t!("crashed: R to respawn", "crash : R pour réapparaître"), bad);
                            } else if report.is_empty() {
                                kv(ui, t!("Damage", "Dégâts"), t!("none", "aucun"), menu::OK);
                            } else {
                                for l in &report {
                                    kv(ui, t!("Damage", "Dégâts"), l, warn);
                                }
                            }
                            ui.add_space(8.0);
                            let on = p.running || p.starter_on;
                            if flightui::pane_button(ui, if on { t!("Shut the engine down", "Couper le moteur") } else { t!("Start the engine", "Démarrer le moteur") }, ui.available_width()).on_hover_text("F4").clicked() {
                                engine_btn = Some(!on);
                            }

                            pane_section(ui, t!("Conditions", "Conditions"));
                            kv(ui, "Position", &menu::latlon(lat, lon), TEXT);
                            kv(ui, t!("Local time", "Heure locale"), &menu::hhmm(local), TEXT);
                            kv(ui, t!("Wind", "Vent"), &if wind_kt < 1.0 { t!("calm", "calme").to_string() } else { format!("{:03.0}°  {:.0} kt", self.wx.wind_from, wind_kt) }, TEXT);
                            kv(ui, t!("Clouds", "Nuages"), &if self.wx.clouds < 0.05 { t!("clear", "dégagé").to_string() } else { format!("{:.0}%  base {:.0} ft", self.wx.clouds * 100.0, self.wx.cloud_base * 3.28084) }, TEXT);
                            kv(ui, "QNH", &format!("{:.0} hPa", self.wx.qnh), TEXT);
                            kv(ui, t!("Visibility", "Visibilité"), &format!("{:.0} km", self.wx.visibility_km()), TEXT);

                            pane_section(ui, t!("Live traffic", "Trafic réel"));
                            if !self.live.enabled() {
                                ui.label(RichText::new(t!("Off (Settings, General).", "Désactivé (Réglages, Général).")).font(menu::caption()).color(flightui::SOFT));
                            } else if self.live.planes.is_empty() {
                                let e = self.live.error();
                                let msg = if e.is_empty() { t!("Waiting for aircraft…", "En attente d'avions…").to_string() } else { t!("No data: {}", "Pas de données : {}", e.chars().take(60).collect::<String>()) };
                                ui.label(RichText::new(msg).font(menu::caption()).color(flightui::SOFT));
                            } else {
                                let me = self.plane.pos;
                                let mut near: Vec<&live::Plane> = self.live.planes.iter().collect();
                                near.sort_by(|a, b| (a.pos - me).length().total_cmp(&(b.pos - me).length()));
                                for lp in near.iter().take(6) {
                                    let ty = if lp.ty.is_empty() { "?" } else { lp.ty.as_str() };
                                    list_row(ui, &lp.call, ty, "", |ui| {
                                        ui.label(RichText::new(format!("{:.1} km  {:.0} ft", (lp.pos - me).length() / 1000.0, lp.alt_ft)).font(egui::FontId::monospace(12.5)).color(flightui::SOFT));
                                    });
                                }
                            }
                            pane_section(ui, t!("Multiplayer", "Multijoueur"));
                            let note = |ui: &mut egui::Ui, s: String| {
                                ui.label(RichText::new(s).font(menu::caption()).color(flightui::SOFT));
                            };
                            match self.mp.as_ref().map(|m| m.status()) {
                                None if self.mp_on => note(ui, t!("Looking for the fastest server…", "Recherche du serveur le plus rapide…").into()),
                                None => note(ui, t!("Off (Esc, Settings, Multiplayer).", "Désactivé (Échap, Réglages, Multijoueur).").into()),
                                Some(mp::Status::Error(e)) => note(ui, t!("No connection: {}", "Pas de connexion : {}", e.chars().take(60).collect::<String>())),
                                Some(mp::Status::Connecting) => {
                                    let to = self.mp.as_ref().map(|m| self.mp_server_name(&m.server)).unwrap_or_default();
                                    note(ui, t!("Connecting to {}…", "Connexion à {}…", to));
                                }
                                Some(mp::Status::Connected) => {
                                    let me = self.plane.pos;
                                    let on = self.mp.as_ref().map(|m| self.mp_server_name(&m.server)).unwrap_or_default();
                                    kv(ui, t!("Server", "Serveur"), &on, TEXT);
                                    kv(ui, t!("Pilots nearby", "Pilotes à proximité"), &self.mp_players.len().to_string(), TEXT);
                                    for p in self.mp_players.iter().take(8) {
                                        let ty = mp_proto::AIRCRAFT.get(p.aircraft as usize).copied().unwrap_or("?");
                                        let alt = geo::pos_to_lla(p.pos).2 * 3.28084;
                                        list_row(ui, &p.name, ty, "", |ui| {
                                            ui.label(RichText::new(format!("{:.1} km  {:.0} ft", (p.pos - me).length() / 1000.0, alt)).font(menu::caption()).color(flightui::SOFT));
                                        });
                                    }
                                }
                            }
                            match engine_btn {
                                Some(true) => self.plane.start_engine(),
                                Some(false) => self.plane.stop_engine(),
                                None => {}
                            }
                        }
                    }
                    ui.add_space(6.0);
                    padui::follow_focus(ui);
                });
            });
        });
    }

    /// What the airport panel says: the airfield, the runway in use and the
    /// wind, and on the way in a guide down the approach (distance, which side
    /// of the centreline, above or below the 3° glide path); on the ground,
    /// where on the airfield the aircraft is. (text, colour: 0 plain, 1 good, 2 caution, 3 warning)
    fn airport_hud(&self) -> Option<Vec<(String, u8)>> {
        if self.crashed {
            return None;
        }
        let f = self.world.airfield_near(self.plane.pos, 18_000.0)?;
        let g = &f.geo;
        let a = g.apt();
        let mut out = vec![(format!("{}  {}", a.code(), a.name), 0u8)];
        let wind_kt = self.wx.wind * 1.94384;
        let wind = if wind_kt < 1.0 { t!("calm", "calme").to_string() } else { format!("{:03.0}°/{:.0} kt", self.wx.wind_from, wind_kt) };
        let moving = f.traffic.movers.len();
        let active = f.traffic.active.map(|(ri, e)| t!("runway {} in use", "piste {} en service", g.runways[ri].ident[e])).unwrap_or_else(|| t!("no runway on the map", "aucune piste sur la carte").into());
        out.push((t!("{active}  ·  wind {wind}  ·  elevation {:.0} ft  ·  {} aircraft moving", "{active}  ·  vent {wind}  ·  altitude {:.0} ft  ·  {} avions en mouvement", g.elev * 3.28084, moving), 0));
        let p = g.local(self.plane.pos);
        let alt = geo::pos_to_lla(self.plane.pos).2 as f32;
        let fw = self.plane.forward();
        let hd = glam::Vec2::new(fw.dot(g.e) as f32, fw.dot(g.n) as f32).normalize_or_zero();
        if self.plane.on_ground || alt - g.h(p) < 3.0 {
            if let Some(ri) = g.on_runway(p, 0.0) {
                let r = &g.runways[ri];
                let e = if hd.dot(r.dir) >= 0.0 { 0 } else { 1 };
                let (s, _) = r.local(p);
                let left = if e == 0 { r.len - s } else { s };
                out.push((t!("On runway {}  ·  {:.0} m of runway ahead", "Sur la piste {}  ·  {:.0} m de piste devant", r.ident[e], left), 1));
            } else if let Some(h) = g.holds.iter().find(|h| (h.p - p).length() < 25.0) {
                let r = &g.runways[h.rwy as usize];
                out.push((t!("Holding point, runway {}", "Point d'attente, piste {}", r.name()), 2));
            }
            return Some(out);
        }
        // The runway end ahead that we are lined up with best.
        let mut best: Option<(f32, usize, usize)> = None;
        for (ri, r) in g.runways.iter().enumerate() {
            for e in 0..2 {
                let (thr, dir) = r.end(e);
                let along = (thr - p).dot(dir);
                let x = (p - thr).dot(airports::right_of(dir));
                if along > -200.0 && along < 20_000.0 && hd.dot(dir) > 0.75 && x.abs() < along.max(0.0) * 0.25 + 300.0 && best.map(|b| along < b.0).unwrap_or(true) {
                    best = Some((along, ri, e));
                }
            }
        }
        if let Some((along, ri, e)) = best {
            let r = &g.runways[ri];
            let (thr, dir) = r.end(e);
            let x = (p - thr).dot(airports::right_of(dir));
            let h = alt - g.h(thr);
            let aim = (along + traffic::aim_dist(r.len)).max(1.0);
            let gs = h.atan2(aim).to_degrees();
            let dev = gs - 3.0;
            let side = if x.abs() < 0.01 * along.max(300.0) + 5.0 { t!("on the centreline", "sur l'axe").to_string() } else { t!("{:.0} m {} of the centreline", "{:.0} m à {} de l'axe", x.abs(), if x > 0.0 { t!("right", "droite") } else { t!("left", "gauche") }) };
            let glide = if dev.abs() < 0.3 { t!("on glide path", "sur le plan de descente").to_string() } else if dev > 0.0 { t!("{:.1}° high", "{:.1}° trop haut", dev) } else { t!("{:.1}° low", "{:.1}° trop bas", -dev) };
            let col = if dev < -0.7 || x.abs() > 0.06 * along + 30.0 { 3 } else if dev.abs() > 0.3 { 2 } else { 1 };
            out.push((t!("Runway {}  {:.1} nm  ·  {}  ·  {}  ·  {:.0} ft above it", "Piste {}  {:.1} NM  ·  {}  ·  {}  ·  {:.0} ft au-dessus", r.ident[e], along.max(0.0) / 1852.0, side, glide, h * 3.28084), col));
        }
        Some(out)
    }

    fn ui(&mut self, agl: f64) -> (Vec<egui::ClippedPrimitive>, egui::TexturesDelta, f32) {
        let raw = self.egui_input();
        let (lat, lon, alt) = geo::pos_to_lla(self.plane.pos);
        let (e, n, u) = geo::enu(self.plane.pos);
        let f = self.plane.forward();
        let hdg = f.dot(e).atan2(f.dot(n)).to_degrees().rem_euclid(360.0);
        // Indicated airspeed: what the wing feels, and what the overspeed warnings are about.
        let speed = self.plane.ias();
        let vs = self.plane.vel.dot(u);
        // (A rocket's height is its engines' above the ground.)
        let base_up = self.world.ground(lat, lon).map_or(alt, |g| alt - g.h) - self.plane.wheel_height();
        let local = (self.utc + lon.to_degrees() / 15.0).rem_euclid(24.0);
        let mut go: Option<String> = None;
        let mut close_goto = false;
        let mut act = None;
        let mut bar: Option<flightui::Bar> = None;
        let airport_hud = self.airport_hud();
        let landing_guide = if self.minimap.landing_guidance && !self.crashed {
            self.world.airfield_near(self.plane.pos, 18_000.0).and_then(|field| {
                let g = &field.geo;
                field.traffic.active.or_else(|| g.active(self.wx.wind_from as f32, (self.wx.wind * 1.94384) as f32))
                    .map(|(runway, end)| approach::Guide::new(g, runway, end))
            })
        } else { None };
        self.src.google.update_attribution(lat.to_degrees(), lon.to_degrees(), agl);
        self.flash.retain(|(_, t)| self.real - t < FLASH_HOLD + FLASH_FADE);
        let out = self.egui_ctx.clone().run_ui(raw, |root| {
            let ctx = &root.ctx().clone();
            self.pad_pass(ctx);
            // the blurred frame behind the frosted glass.
            flightui::set_frost(ctx, self.r.frost_id);
            use egui::{Align2, Color32, FontId, RichText};
            let pad = padui::active();
            let white = Color32::from_rgba_unmultiplied(255, 255, 255, 235);
            // keys to press next, as key caps.
            let keys_line = |ui: &mut egui::Ui, keys: &[(&str, &str)]| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 6.0;
                    for (k, (key, what)) in keys.iter().enumerate() {
                        if k > 0 {
                            ui.add_space(12.0);
                        }
                        menu::key_cap(ui, key);
                        ui.label(RichText::new(*what).font(menu::label()).color(menu::TEXT));
                    }
                });
            };
            if self.pause_open {
                self.pause_menu(ctx, &mut act, lat.to_degrees(), lon.to_degrees(), local);
                padui::focus_ring(ctx, menu::ACCENT);
                return;
            }
            if self.info_open {
                self.info_panel(ctx, &mut act, lat.to_degrees(), lon.to_degrees(), local, speed, alt, agl, hdg);
            }
            // the flight menu, over everything but the pause menu.
            if self.mode == Mode::Fly {
                self.radial_menu(ctx, &mut bar);
            }
            // the hud.
            let mut nav_lift = 0.0;
            if self.hud_on() {
                self.hud(ctx, &flightui::HudIn { hdg, ias: speed, alt, agl, vs, local });
                // rocket figures or weapons, for the aircraft that have them.
                self.systems_card(ctx, vs, base_up);
                self.notices(ctx, airport_hud.as_deref());
                if self.radios.show && self.nav_overlay(ctx, hdg) {
                    nav_lift = flightui::NAV_H;
                }
            }
            self.hotspot_overlay(ctx);
            // a ring on each aircraft in range, tags on the nearest that do not overlap.
            if let (true, Some((vp, cam))) = (self.hud_on() && self.live.enabled(), self.live_view) {
                let screen = ctx.content_rect();
                let painter = ctx.layer_painter(egui::LayerId::background());
                let mut seen: Vec<(f64, egui::Pos2, &live::Plane)> = Vec::new();
                for p in &self.live.planes {
                    if p.downed > 0.0 {
                        continue;
                    }
                    let rel = p.pos - cam;
                    let d = rel.length();
                    if d > 40_000.0 || d < 30.0 {
                        continue;
                    }
                    let c = vp * rel.as_vec3().extend(1.0);
                    if c.w <= 0.0 {
                        continue;
                    }
                    let at = egui::pos2(screen.left() + (c.x / c.w * 0.5 + 0.5) * screen.width(), screen.top() + (0.5 - c.y / c.w * 0.5) * screen.height());
                    if screen.expand(40.0).contains(at) {
                        seen.push((d, at, p));
                    }
                }
                seen.sort_by(|a, b| a.0.total_cmp(&b.0));
                let mut taken: Vec<egui::Rect> = Vec::new();
                for (d, at, p) in seen {
                    // fainter with distance.
                    let a = (1.0 - d / 40_000.0).clamp(0.45, 1.0) as f32;
                    painter.circle_stroke(at, 5.0, egui::Stroke::new(1.5, flightui::LIT.gamma_multiply(a)));
                    let line2 = format!("{}  ·  {:.0} ft  ·  {:.1} km", if p.ty.is_empty() { "?" } else { p.ty.as_str() }, p.alt_ft, d / 1000.0);
                    let g1 = painter.layout_no_wrap(p.call.clone(), menu::semibold(13.0), Color32::WHITE.gamma_multiply(a));
                    let g2 = painter.layout_no_wrap(line2, menu::medium(11.5), flightui::SOFT.gamma_multiply(a));
                    let w = g1.size().x.max(g2.size().x) + 14.0;
                    let r = egui::Rect::from_min_size(at + egui::vec2(10.0, -16.0), egui::vec2(w, 34.0));
                    if taken.len() >= 8 || taken.iter().any(|t| t.expand(4.0).intersects(r)) {
                        continue;
                    }
                    taken.push(r);
                    painter.rect_filled(r, 6, Color32::from_black_alpha((120.0 * a) as u8));
                    painter.galley(r.min + egui::vec2(7.0, 3.0), g1, Color32::WHITE);
                    painter.galley(r.min + egui::vec2(7.0, 18.0), g2, Color32::WHITE);
                }
            }
            // The other players: a tag beside each.
            if let (true, Some((vp, cam))) = (self.show_hud && !self.mp_players.is_empty(), self.live_view) {
                let screen = ctx.content_rect();
                let painter = ctx.layer_painter(egui::LayerId::background());
                let col = Color32::from_rgb(255, 205, 90);
                for p in &self.mp_players {
                    let rel = p.pos - cam;
                    let d = rel.length();
                    if d > 60_000.0 || d < 15.0 {
                        continue;
                    }
                    let c = vp * rel.as_vec3().extend(1.0);
                    if c.w <= 0.0 {
                        continue;
                    }
                    let at = egui::pos2(screen.left() + (c.x / c.w * 0.5 + 0.5) * screen.width(), screen.top() + (0.5 - c.y / c.w * 0.5) * screen.height());
                    if !screen.expand(40.0).contains(at) {
                        continue;
                    }
                    painter.circle_stroke(at, 6.0, egui::Stroke::new(1.5, col));
                    let line2 = format!("{}  {:.0} ft  {:.0} kt  {:.1} km", mp_proto::AIRCRAFT.get(p.aircraft as usize).copied().unwrap_or("?"), geo::pos_to_lla(p.pos).2 * 3.28084, p.vel.length() * 1.94384, d / 1000.0);
                    for (k, (s, size)) in [(p.name.as_str(), 13.0), (line2.as_str(), 11.0)].into_iter().enumerate() {
                        let pos = at + egui::vec2(10.0, -10.0 + 14.0 * k as f32);
                        painter.text(pos + egui::vec2(1.0, 1.0), Align2::LEFT_TOP, s, FontId::proportional(size), Color32::from_black_alpha(160));
                        painter.text(pos, Align2::LEFT_TOP, s, FontId::proportional(size), col);
                    }
                }
            }
            // The seeker's target: a diamond on the aircraft, and its range.
            if let (true, Some((vp, cam))) = (self.hud_on(), self.live_view) {
                if let Some(t) = &self.weapons.target_view {
                    let screen = ctx.content_rect();
                    let c = vp * (t.pos - cam).as_vec3().extend(1.0);
                    if c.w > 0.0 {
                        let at = egui::pos2(screen.left() + (c.x / c.w * 0.5 + 0.5) * screen.width(), screen.top() + (0.5 - c.y / c.w * 0.5) * screen.height());
                        if screen.expand(100.0).contains(at) {
                            let painter = ctx.layer_painter(egui::LayerId::background());
                            let locked = self.weapons.lock >= 1.0;
                            let col = if locked { Color32::from_rgb(120, 230, 140) } else { Color32::from_rgb(255, 180, 50) };
                            let st = egui::Stroke::new(1.6, col);
                            let r = if locked { 16.0 } else { 12.0 };
                            painter.circle_stroke(at, r + 5.0, egui::Stroke::new(1.0, col));
                            for (a, b) in [(-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)] {
                                painter.line_segment([at + egui::vec2(a * r, b * r), at + egui::vec2(a * (r + 8.0), b * (r + 8.0))], st);
                            }
                            let line = format!("{}  {}  {:.1} km  {:+.0} m/s", if locked { "LOCK" } else { "TRACK" }, t.label, t.range / 1000.0, t.closure);
                            let pos = at + egui::vec2(0.0, r + 22.0);
                            painter.text(pos + egui::vec2(1.0, 1.0), Align2::CENTER_TOP, &line, FontId::monospace(12.0), Color32::from_black_alpha(180));
                            painter.text(pos, Align2::CENTER_TOP, &line, FontId::monospace(12.0), col);
                        }
                    }
                }
            }
            if self.hud_on() && !self.pause_open && !self.plane.on_ground {
                if let (Some(guide), Some((vp, camera))) = (&landing_guide, self.live_view) {
                    guide.draw_world(ctx, vp, camera, self.plane.pos);
                }
            }
            // The loading screen, over everything else.
            self.loading_overlay(ctx);
            // Google's imagery has to carry its credit, HUD or not.
            if self.hud_on() || self.google_shown() {
                self.credits(ctx);
            }
            // Damage: warnings while the airframe is being overstressed, what has
            // already broken, and a flash for each new loss.
            let p = &self.plane;
            let mut warn: Vec<(&str, Color32)> = Vec::new();
            let red = menu::BAD;
            let amber = Color32::from_rgb(255, 196, 80);
            if !self.crashed && self.show_hud {
                if p.stalled {
                    warn.push(("STALL", red));
                }
                if p.gear < 1.0 && agl < 150.0 && p.throttle < 0.35 && !p.on_ground {
                    warn.push((if pad { "GEAR UP  (L3)" } else { "GEAR UP  (L)" }, red));
                }
                let w = &self.warn;
                if w.pull_up {
                    warn.push(("PULL UP", red));
                } else if w.dive > 0.0 {
                    warn.push(("NOSE DIVE", red));
                } else if w.too_low {
                    warn.push(("TOO LOW  TERRAIN", amber));
                } else if w.sink {
                    warn.push(("SINK RATE", amber));
                }
                let over = p.overspeed();
                if over > 13.0 {
                    warn.push(("OVERSPEED  AIRFRAME BREAKING UP", red));
                } else if over > 0.0 {
                    warn.push(("OVERSPEED", amber));
                }
                if p.flaps > 0.25 && speed > p.vfe() {
                    warn.push((if p.spec.lever.is_some() { "NOSE OVERSPEED" } else { "FLAPS OVERSPEED" }, if p.dmg.flap_stress > 0.5 { red } else { amber }));
                }
                if p.g_load.abs() > 5.0 {
                    warn.push(("OVER G", if p.dmg.fatigue > 1.2 { red } else { amber }));
                }
                if p.fuel <= 0.0 {
                    warn.push(("NO FUEL", red));
                } else if p.fuel_frac() < 0.1 {
                    warn.push(("LOW FUEL", amber));
                }
                if !p.running && !p.on_ground {
                    warn.push(("ENGINE OFF", red));
                }
                let el = &p.sys.elec;
                if el.master && (el.volts < 24.0 || (p.sys.fail.alternator && p.running)) {
                    warn.push(("LOW VOLTS", amber));
                }
                if p.sys.fail.vacuum && !p.spec.is_jet() {
                    warn.push(("VACUUM", amber));
                }
                if p.cg_excess() != 0.0 {
                    warn.push(("CG LIMIT", amber));
                }
            }
            let report = if self.crashed { Vec::new() } else { p.dmg.report() };
            // warnings, damage and the approach line.
            if self.show_hud {
                let guide = if self.hud_on() { airport_hud.as_ref().and_then(|l| l.get(2)) } else { None };
                self.alerts(ctx, &warn, &report, guide, nav_lift);
            }
            if !self.flash.is_empty() && !self.crashed {
                // Each new loss: a small red plate down the left edge (under the systems card,
                // if any), newest on top, fading quickly. A crash sheds many pieces at once, so
                // the stack stops at half the screen height rather than covering the view.
                let top = if self.hud_on() { self.minimap.top } else { 0.0 } + 14.0;
                let floor = ctx.content_rect().height() * 0.5;
                egui::Area::new(egui::Id::unique("flash")).anchor(Align2::LEFT_TOP, [14.0, top]).interactable(false).show(ctx, |ui| {
                    ui.with_layout(egui::Layout::top_down(egui::Align::Min), |ui| {
                        ui.spacing_mut().item_spacing.y = 3.0;
                        for (s, t0) in self.flash.iter().rev() {
                            // (a plate is about 24 px tall: start one only if it ends above the floor)
                            if ui.cursor().top() + 24.0 > floor {
                                break;
                            }
                            let a = (1.0 - ((self.real - t0 - FLASH_HOLD) / FLASH_FADE).max(0.0)).clamp(0.0, 1.0) as f32;
                            ui.scope(|ui| {
                                ui.multiply_opacity(a);
                                egui::Frame::new().fill(menu::BAD.gamma_multiply(0.2)).stroke(egui::Stroke::new(1.0, menu::BAD)).corner_radius(menu::R_WIDGET).inner_margin(egui::Margin::symmetric(9, 3)).show(ui, |ui| {
                                    ui.label(RichText::new(i18n::caps(s)).font(menu::bold(14.0)).color(menu::BAD).extra_letter_spacing(0.6));
                                });
                            });
                        }
                    });
                });
            }
            // Come to rest in pieces without burning: say so, and how to go on.
            if !self.crashed && self.plane.wrecked() && self.plane.vel.length() < 2.0 && (self.plane.on_hull || self.plane.on_ground) {
                self.wreck_t += self.real - self.wreck_seen;
            } else {
                self.wreck_t = 0.0;
            }
            self.wreck_seen = self.real;
            if !self.crashed && self.wreck_t > 2.0 && self.mode == Mode::Fly {
                egui::Area::new(egui::Id::unique("wreck")).anchor(Align2::CENTER_CENTER, [0.0, 80.0]).show(ctx, |ui| {
                    flightui::frosted(ui, flightui::glass().inner_margin(egui::Margin::symmetric(24, 16)), |ui| {
                        ui.vertical_centered(|ui| {
                            ui.label(RichText::new(t!("AIRCRAFT WRECKED", "AVION DÉTRUIT")).font(menu::bold(34.0)).color(Color32::from_rgb(255, 160, 80)).extra_letter_spacing(1.5));
                            ui.add_space(6.0);
                            if pad {
                                let panel = padmap::spec_of("flight_panel");
                                let mut items = vec![(panel.as_str(), tr("go somewhere else")), ("Menu", tr("menu: respawn, globe"))];
                                items.retain(|i| !i.0.is_empty());
                                padui::prompts(ui, &items, 24.0, 16.0, white);
                            } else {
                                keys_line(ui, &[("R", t!("respawn", "réapparaître")), ("G", t!("go somewhere else", "aller ailleurs")), ("Esc", t!("menu", "menu"))]);
                            }
                        });
                    });
                });
            }
            if self.crashed && self.real - self.crash_real > 3.5 {
                egui::Area::new(egui::Id::unique("crash")).anchor(Align2::CENTER_BOTTOM, [0.0, -70.0]).show(ctx, |ui| {
                    flightui::frosted(ui, flightui::glass().inner_margin(egui::Margin::symmetric(20, 12)), |ui| {
                        if pad {
                            let panel = padmap::spec_of("flight_panel");
                            let mut items = vec![("A", tr("respawn")), (panel.as_str(), tr("go somewhere else")), ("Menu", tr("menu"))];
                            items.retain(|i| !i.0.is_empty());
                            padui::prompts(ui, &items, 24.0, 16.0, white);
                        } else {
                            keys_line(ui, &[("R", t!("respawn", "réapparaître")), ("G", t!("go somewhere else", "aller ailleurs")), ("Esc", t!("menu", "menu"))]);
                        }
                    });
                });
            }
            if self.hud_on() && !self.pause_open && !self.show_help && !self.goto_open {
                if self.minimap.visible {
                    let horizontal = f.dot(e).hypot(f.dot(n));
                    self.minimap.show(ctx, &self.src, lat, lon, (horizontal > 1e-5).then_some(hdg.to_radians()), self.info_open, landing_guide.as_ref(), &self.world);
                }
            }
            if self.show_help {
                egui::Area::new(egui::Id::unique("help")).anchor(Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
                    menu::panel().show(ui, |ui| {
                        ui.set_width(600.0);
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(t!("Controls", "Commandes")).font(menu::page_title()).color(Color32::WHITE));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                ui.label(RichText::new(t!("F1 / Esc  close", "F1 / Échap  fermer")).font(menu::caption()).color(menu::DIM));
                            });
                        });
                        ui.add_space(4.0);
                        menu::controls_list(ui);
                    });
                });
            }
            if self.wx.open {
                self.wx.panel(ctx, &mut self.utc, &mut self.doy, lon.to_degrees());
            }
            if self.goto_open {
                egui::Window::new(t!("Go to", "Aller à")).id(egui::Id::unique("goto_window")).collapsible(false).resizable(false).anchor(Align2::CENTER_CENTER, [0.0, -80.0]).show(ctx, |ui| {
                    ui.label(t!("City, landmark, street address, or an airport code (KSFO, LHR):", "Ville, site, adresse ou code d'aéroport (KSFO, LHR) :"));
                    let resp = ui.add(egui::TextEdit::singleline(&mut self.goto_text).desired_width(360.0));
                    resp.request_focus();
                    if ui.input(|i| i.key_pressed(egui::Key::Enter)) && !self.goto_text.trim().is_empty() {
                        go = Some(self.goto_text.trim().to_string());
                    }
                    if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                        close_goto = true;
                    }
                });
            }
            padui::focus_ring(ctx, menu::ACCENT);
        });
        self.egui_state.handle_platform_output(&self.window, out.platform_output);
        // A button or slider left focused on the flight panel would keep the
        // keys from the aircraft, which goes on flying while the panel is open.
        // (Except where the gamepad works the panel.)
        if !self.pause_open && !(self.info_open && padui::active()) && !self.egui_ctx.text_edit_focused() {
            self.egui_ctx.memory_mut(|m| {
                if let Some(id) = m.focused() {
                    m.surrender_focus(id);
                }
            });
        }
        if let Some(i) = go.as_deref().and_then(|q| airports::Db::get().find(q)) {
            self.goto_open = false;
            go = None;
            self.start_airport(i, None, false, true);
        }
        if let Some(q) = go {
            self.goto_open = false;
            self.status = t!("Searching for {}…", "Recherche de {}…", q);
            self.status_until = self.real + 30.0;
            let (tx, rx) = std::sync::mpsc::channel();
            let src = self.src.clone();
            std::thread::spawn(move || {
                let _ = tx.send(geocode(&src.fetch, &q).map(|(la, lo, name)| (la, lo, ground_elev(&src, la, lo), name)));
            });
            self.goto_rx = Some(rx);
        }
        if close_goto {
            self.goto_open = false;
        }
        if let Some(b) = bar {
            self.toolbar_act(b);
        }
        match act {
            Some(PauseAct::Resume) => self.set_pause_menu(false),
            Some(PauseAct::Respawn) => {
                self.set_pause_menu(false);
                self.info_open = false;
                self.respawn_here();
            }
            Some(PauseAct::GoTo) => {
                self.set_pause_menu(false);
                self.goto_open = true;
                self.goto_text.clear();
            }
            Some(PauseAct::FlyTo(lat, lon, name)) => {
                self.set_pause_menu(false);
                self.info_open = false;
                self.fly_to(lat, lon, name);
            }
            Some(PauseAct::Airport(i, appr)) => {
                self.set_pause_menu(false);
                self.info_open = false;
                self.start_airport(i, None, appr, true);
            }
            Some(PauseAct::Globe) => self.to_globe(),
            Some(PauseAct::Quit) => self.quit = true,
            None => {}
        }
        let jobs = self.egui_ctx.tessellate(out.shapes, out.pixels_per_point);
        (jobs, out.textures_delta, out.pixels_per_point)
    }
}

struct App {
    args: Args,
    game: Option<Game>,
    frame_pacer: frame_pacer::FramePacer,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, el: &ActiveEventLoop) {
        if self.game.is_some() {
            return;
        }
        let attrs = Window::default_attributes().with_title("Fable Flight").with_inner_size(winit::dpi::PhysicalSize::new(self.args.size.0, self.args.size.1));
        let window = Arc::new(el.create_window(attrs).expect("window"));
        match Game::new(window, &self.args) {
            Ok(g) => self.game = Some(g),
            Err(e) => {
                eprintln!("failed to start: {e:#}");
                el.exit();
            }
        }
    }

    fn window_event(&mut self, _el: &ActiveEventLoop, _: WindowId, event: WindowEvent) {
        let Some(g) = self.game.as_mut() else { return };
        let consumed = g.egui_state.on_window_event(&g.window, &event).consumed;
        match event {
            WindowEvent::CloseRequested => {
                g.keep_settings(true);
                if !std::env::args().any(|a| a == "--headless" || a == "--shot" || a == "--record") { g.src.google.save(); }
                // Dropped here, while the event loop (and with it the Wayland
                // connection) is still alive: egui's clipboard thread tears
                // down its Wayland objects on drop, and if the game outlived
                // `run_app` that would be on an already freed display.
                self.game = None;
                std::process::exit(0);
            }
            WindowEvent::Resized(s) => g.r.resize(s.width, s.height),
            WindowEvent::RedrawRequested => {
                // The OS can also request redraws before our frame deadline.
                let now = Instant::now();
                if self.frame_pacer.control_flow(now, g.gfx.fps_cap) != ControlFlow::Poll {
                    return;
                }
                self.frame_pacer.frame_started(now);
                g.frame();
                if g.quit {
                    g.keep_settings(true);
                    if !std::env::args().any(|a| a == "--headless" || a == "--shot" || a == "--record") { g.src.google.save(); }
                    // Worker threads may be mid-download, so the process just
                    // exits; but the GPU device is torn down first: exiting
                    // under the driver's shader compiler threads can crash it.
                    self.game = None;
                    std::process::exit(0);
                }
            }
            WindowEvent::Focused(false) => {
                g.cancel_cockpit_input();
                g.keys.clear();
                (g.pad_y, g.pad_y_combo) = (false, false);
            }
            WindowEvent::KeyboardInput { event: k, .. } => {
                if let PhysicalKey::Code(raw) = k.physical_key {
                    // Waiting for a key (or Esc) to bind, or Esc calling off a joystick capture.
                    let pressed = k.state == ElementState::Pressed;
                    let capturing = match g.binds.capture.clone() {
                        Some(bind::Capture::Key(id)) if pressed => {
                            if raw == KeyCode::Escape {
                                g.binds.capture = None;
                            } else if bind::bindable(raw) {
                                g.binds.capture = None;
                                g.binds.set_key(id, raw);
                            }
                            g.keys.clear();
                            true
                        }
                        Some(_) if pressed && raw == KeyCode::Escape => {
                            g.binds.capture = None;
                            true
                        }
                        _ => false,
                    };
                    let mapped = if capturing { None } else { g.binds.map_key(raw) };
                    if let Some(code) = mapped {
                    if k.state == ElementState::Pressed {
                        // egui always claims Tab (and, while one of its buttons has
                        // focus, every key), so the menu keys are let through here.
                        let typing = g.goto_open || g.egui_ctx.text_edit_focused();
                        let menu_key = code == KeyCode::Tab || code == KeyCode::Escape;
                        // A key while the gamepad has a button of the flight panel picked
                        // out: the keyboard flies again (egui would keep the keys).
                        let mut consumed = consumed;
                        if consumed && g.mode == Mode::Fly && !g.pause_open && !typing {
                            g.egui_ctx.memory_mut(|m| {
                                if let Some(id) = m.focused() {
                                    m.surrender_focus(id);
                                }
                            });
                            consumed = false;
                        }
                        if menu_key && !typing {
                            if !k.repeat {
                                g.key(code);
                            }
                        } else if !(consumed || g.goto_open) {
                            if !k.repeat {
                                g.key(code);
                            }
                            g.keys.insert(code);
                        }
                    } else {
                        g.keys.remove(&code);
                        // tab held, pointed and let go picks from the flight menu.
                        if code == KeyCode::Tab {
                            if let Some(b) = g.flight_menu_release() {
                                g.toolbar_act(b);
                            }
                        }
                    }
                    }
                }
            }
            WindowEvent::MouseInput { state, button: MouseButton::Left, .. } if g.mode == Mode::Menu => {
                if g.menu.screen != menu::Screen::Fly {
                    g.menu.press = None;
                    return;
                }
                let size = g.window.inner_size();
                let (w, h) = (size.width.max(1) as f64, size.height.max(1) as f64);
                if state == ElementState::Pressed {
                    if !consumed {
                        g.menu.press = g.last_cursor;
                        g.menu.dragged = 0.0;
                    }
                } else if let Some((x, y)) = g.menu.press.take() {
                    // A click (not a drag) chooses the spot; a second click on it starts.
                    if g.menu.dragged < 6.0 {
                        if let Some(i) = g.menu.marker_at(x, y) {
                            let double = g.real - g.last_click < 0.45;
                            g.last_click = g.real;
                            if double && g.menu.airport == Some(i) {
                                g.begin();
                            } else {
                                g.choose_airport(i);
                            }
                        } else if let Some((lat, lon)) = g.menu.pick_at(x, y, w, h, 60f64.to_radians()) {
                            let double = g.real - g.last_click < 0.45;
                            g.last_click = g.real;
                            if double && g.menu.pick.is_some() {
                                g.begin();
                            } else {
                                g.pick(lat, lon);
                            }
                        }
                    }
                }
            }
            WindowEvent::MouseInput { state, button: MouseButton::Middle, .. }
                if g.lean_drag.is_some() || (!consumed && g.mode == Mode::Fly && g.cam_mode == 1 && !g.pause_open && !g.crashed) => {
                if state == ElementState::Pressed {
                    g.lean_drag = g.last_cursor;
                } else if let (Some((sx, sy)), Some((x, y))) = (g.lean_drag.take(), g.last_cursor) {
                    if (sx - x).hypot(sy - y) < 3.0 { g.head.center_seat(); }
                }
            }
            // In the cockpit, a click on a control works it (see `hot_click`).
            WindowEvent::MouseInput { state, button: MouseButton::Left, .. } if if state == ElementState::Pressed { !consumed && g.hotspot_under_mouse().is_some() } else { g.hot_press.is_some() } => {
                g.hot_click(state == ElementState::Pressed);
            }
            WindowEvent::MouseInput { state, button: MouseButton::Left | MouseButton::Right, .. } if !consumed => {
                g.drag = state == ElementState::Pressed;
                g.last_cursor = None;
            }
            WindowEvent::CursorMoved { position, .. } => {
                if g.mode == Mode::Menu {
                    if let (Some(_), Some((x, y))) = (g.menu.press, g.last_cursor) {
                        let h = g.window.inner_size().height.max(1) as f64;
                        g.menu.drag(position.x - x, position.y - y, h, 60f64.to_radians());
                    }
                } else if g.lean_drag.is_some() && !g.pause_open {
                    if let Some((x, y)) = g.last_cursor {
                        let scale = 0.85 / g.window.inner_size().height.max(1) as f64;
                        g.head.lean_by((position.x - x) * scale, (y - position.y) * scale);
                    }
                } else if g.drag && !g.pause_open {
                    if let Some((x, y)) = g.last_cursor {
                        // Zoomed in, the view turns slower under the mouse.
                        let k = g.look_sens * if g.cam_mode == 1 { g.head.zoom } else { 1.0 };
                        g.cam_yaw -= (position.x - x) * 0.005 * k;
                        g.cam_pitch = (g.cam_pitch + (position.y - y) * 0.004 * k).clamp(-1.2, 1.4);
                    }
                }
                // The mouse moved (not just a twitch): it points at the controls again, not the pad.
                if g.cursor.is_none_or(|(x, y)| (position.x - x).abs() + (position.y - y).abs() > 3.0) {
                    g.pad_aim = false;
                }
                g.last_cursor = Some((position.x, position.y));
                // (--cursor pins it, for scripted screenshots.)
                if g.script_cursor.is_none() {
                    g.cursor = Some((position.x, position.y));
                }
            }
            WindowEvent::MouseWheel { delta, .. } if !consumed => {
                let d = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(p) => p.y / 40.0,
                };
                if g.mode == Mode::Menu {
                    if g.menu.screen == menu::Screen::Fly {
                        g.menu.zoom(d);
                    }
                } else if g.cam_mode == 1 && !g.crashed && g.hot_wheel(d) {
                    // (A cockpit control under the mouse took it.)
                } else if g.cam_mode == 1 && !g.crashed {
                    g.head.zoom_by(d);
                } else {
                    g.cam_dist = (g.cam_dist * (1.0 - d * 0.1)).clamp(6.0, 400.0);
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        if let Some(g) = &mut self.game {
            // Answer tool calls even when redraws are delayed; screenshots wait for a frame.
            g.pump_mcp();
            let flow = self.frame_pacer.control_flow(Instant::now(), g.gfx.fps_cap);
            if flow == ControlFlow::Poll {
                g.window.request_redraw();
            }
            event_loop.set_control_flow(flow);
        }
    }
}

/// The UI library ships no fonts of its own here, so borrow the system's.
fn load_fonts(ctx: &egui::Context) {
    // Each candidate is "path" or "path#face", the face picking one font out of a
    // .ttc collection. fontconfig's index carries the named instance of a variable
    // font in its upper 16 bits; only the low 16 are the face.
    let find = |kind: &str, fallbacks: &[&str]| -> Option<(Vec<u8>, u32)> {
        let fc = std::process::Command::new("fc-match").args(["-f", "%{file}#%{index}", kind]).output().ok().and_then(|o| String::from_utf8(o.stdout).ok());
        fc.iter().map(|s| s.trim().to_string()).chain(fallbacks.iter().map(|s| s.to_string())).find_map(|c| {
            let (path, face) = match c.rsplit_once('#') {
                Some((p, i)) => (p.to_string(), i.parse::<u32>().ok()? & 0xFFFF),
                None => (c.clone(), 0),
            };
            let ext = path.rsplit('.').next().unwrap_or("").to_ascii_lowercase();
            if !matches!(ext.as_str(), "ttf" | "otf" | "ttc" | "otc") {
                return None;
            }
            std::fs::read(&path).ok().map(|d| (d, face))
        })
    };
    let mut fonts = egui::FontDefinitions::empty();
    // fontconfig's answer where there is one (Linux), then each system's own fonts.
    let mono: &[&str] = &[
        "/usr/share/fonts/google-noto/NotoSansMono-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",
        "/System/Library/Fonts/SFNSMono.ttf",
        "/System/Library/Fonts/Monaco.ttf",
        "/System/Library/Fonts/Supplemental/Andale Mono.ttf",
        "/System/Library/Fonts/Supplemental/Courier New.ttf",
        "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/cour.ttf",
    ];
    let sans: &[&str] = &[
        "/usr/share/fonts/google-noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf",
        "/System/Library/Fonts/SFNS.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Geneva.ttf",
        "/Library/Fonts/Arial Unicode.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
    ];
    // A Korean (Hangul) face, for the Korean interface. The bundled Barlow and
    // the Latin system fonts carry no Hangul, so without this Korean text would
    // render as empty boxes. Noto Sans CJK collections hold JP, KR, SC, TC, HK
    // in that order, so "#1" picks the Korean face.
    let korean: &[&str] = &[
        "/usr/share/fonts/google-noto-sans-cjk-vf-fonts/NotoSansCJK-VF.ttc#1",
        "/usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc#1",
        "/usr/share/fonts/opentype/noto-cjk/NotoSansCJK-Regular.ttc#1",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc#1",
        "/usr/share/fonts/google-noto/NotoSansKR-Regular.otf",
        "/usr/share/fonts/opentype/noto/NotoSansKR-Regular.otf",
        "/usr/share/fonts/truetype/noto/NotoSansKR-Regular.ttf",
        "/System/Library/Fonts/AppleSDGothicNeo.ttc",
        "/Library/Fonts/AppleGothic.ttf",
        "C:/Windows/Fonts/malgun.ttf",
    ];
    // our bundled faces first, then the system's for missing glyphs.
    let own: [(&str, &'static [u8]); 5] = [("barlow", menu::FONT_REGULAR), ("barlow-medium", menu::FONT_MEDIUM), ("barlow-semibold", menu::FONT_SEMIBOLD), ("barlow-condensed", menu::FONT_DISPLAY), ("plex-mono", menu::FONT_MONO)];
    for (name, data) in own {
        fonts.font_data.insert(name.into(), Arc::new(egui::FontData::from_static(data)));
    }
    let families = [
        (egui::FontFamily::Proportional, "barlow"),
        (egui::FontFamily::Monospace, "plex-mono"),
        (egui::FontFamily::Name(menu::MEDIUM.into()), "barlow-medium"),
        (egui::FontFamily::Name(menu::SEMIBOLD.into()), "barlow-semibold"),
        (egui::FontFamily::Name(menu::BOLD.into()), "barlow-condensed"),
    ];
    for (family, name) in &families {
        fonts.families.entry(family.clone()).or_default().push((*name).into());
    }
    for (name, kind, fb) in [("mono", "monospace", mono), ("sans", "sans", sans), ("korean", "sans:lang=ko", korean)] {
        if let Some((data, face)) = find(kind, fb) {
            let mut fd = egui::FontData::from_blob(Arc::new(data), face);
            // A variable font starts at its default instance, which for Noto Sans
            // CJK VF is Thin (wght 100); ask for Regular instead.
            if fd.variation_axes().iter().any(|a| a.tag.to_be_bytes() == *b"wght") {
                fd.tweak.coords.push(b"wght", 400.0);
            }
            fonts.font_data.insert(name.into(), Arc::new(fd));
        }
    }
    for (family, _) in families {
        // Korean is the last fallback everywhere so Latin keeps the bundled face
        // and only the Hangul glyphs come from the Korean font.
        let fallback = if family == egui::FontFamily::Monospace { ["mono", "sans", "korean"] } else { ["sans", "mono", "korean"] };
        for f in fallback {
            if fonts.font_data.contains_key(f) {
                fonts.families.entry(family.clone()).or_default().push(f.into());
            }
        }
    }
    ctx.set_fonts(fonts);
}

/// Right-handed view matrix looking along `dir`.
fn look_to(eye: Vec3, dir: Vec3, up: Vec3) -> Mat4 {
    let f = dir.normalize();
    let s = f.cross(up).normalize();
    let u = s.cross(f);
    Mat4::from_cols(s.x_axis4(u, f, 0), s.x_axis4(u, f, 1), s.x_axis4(u, f, 2), glam::Vec4::new(-s.dot(eye), -u.dot(eye), f.dot(eye), 1.0))
}
trait Axis4 {
    fn x_axis4(self, u: Vec3, f: Vec3, i: usize) -> glam::Vec4;
}
impl Axis4 for Vec3 {
    fn x_axis4(self, u: Vec3, f: Vec3, i: usize) -> glam::Vec4 {
        glam::Vec4::new(self[i], u[i], -f[i], 0.0)
    }
}
/// Infinite far plane, reversed depth (1 at the near plane, 0 at infinity).
fn perspective_rev(fov_y: f32, aspect: f32, near: f32) -> Mat4 {
    let f = 1.0 / (fov_y * 0.5).tan();
    Mat4::from_cols_array(&[f / aspect, 0.0, 0.0, 0.0, 0.0, f, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, near, 0.0])
}
fn ortho(l: f32, r: f32, b: f32, t: f32, n: f32, f: f32) -> Mat4 {
    Mat4::from_cols_array(&[2.0 / (r - l), 0.0, 0.0, 0.0, 0.0, 2.0 / (t - b), 0.0, 0.0, 0.0, 0.0, -1.0 / (f - n), 0.0, -(r + l) / (r - l), -(t + b) / (t - b), -n / (f - n), 1.0])
}

/// `--jsbsim-compare FILE [--aircraft NAME]`: prints how a JSBSim aircraft's
/// aerodynamics compare with ours (the Cessna unless NAME picks another).
fn jsbsim_compare(file: Option<&String>, which: Option<&String>) -> i32 {
    let Some(file) = file else {
        eprintln!("--jsbsim-compare takes a JSBSim aircraft file (…/c172p/c172p.xml)");
        return 2;
    };
    let j = match jsbsim::Aircraft::load(std::path::Path::new(file)) {
        Ok(j) => j,
        Err(e) => {
            eprintln!("{file}: {e}");
            return 1;
        }
    };
    let want = which.map(|w| w.to_lowercase()).unwrap_or_else(|| "172".into());
    let Some(spec) = aircraft::BUILTIN.iter().copied().find(|s| s.name.to_lowercase().contains(&want)) else {
        eprintln!("no aircraft of ours matches {want:?}: {}", aircraft::BUILTIN.iter().map(|s| s.name).collect::<Vec<_>>().join(", "));
        return 2;
    };
    println!("{}", jsbsim::compare(&j, spec, our_roll_rate(spec)));
    0
}

/// Ours flown: the highest roll rate full aileron reaches from 90 kt, level,
/// in the first four seconds (rad/s). No world is loaded (offline, empty).
fn our_roll_rate(spec: &'static aircraft::Spec) -> Option<f64> {
    let dir = std::env::temp_dir().join("fablegame-jsbsim-compare");
    let src = Arc::new(sources::Sources::new(fetch::Fetcher::new(dir, true), String::new()));
    let w = world::World::new(sources::Loader::new(src, 0), 14);
    let mut p = Plane::spawn(spec, 45f64.to_radians(), 5f64.to_radians(), 1500.0, 0.0, 90.0 / 1.943_84);
    p.throttle = 0.7;
    let c = Controls { roll: 1.0, ..Default::default() };
    let dt = 1.0 / 180.0;
    let mut best: f64 = 0.0;
    for _ in 0..(4.0 / dt) as usize {
        if p.step(dt, &c, &w).is_some() {
            break;
        }
        best = best.max(p.omega.z.abs());
    }
    Some(best)
}

/// `--fly-check KEY`: flies an aircraft in an empty world and prints how it
/// does: a take-off run at full power, level flight at full power (its top
/// speed) and a power-off slow-down to the stall. For checking a fit (hangar.rs).
fn fly_check(key: Option<&String>) -> i32 {
    hangar::load_installed();
    let Some(spec) = key.and_then(|k| aircraft::by_key(k)) else {
        eprintln!("--fly-check KEY: one of {}", aircraft::list().iter().map(|s| s.keys[0]).collect::<Vec<_>>().join(", "));
        return 1;
    };
    let dir = std::env::temp_dir().join("fablegame-jsbsim-compare");
    let src = Arc::new(sources::Sources::new(fetch::Fetcher::new(dir, true), String::new()));
    let mut w = world::World::new(sources::Loader::new(src, 0), 14);
    w.flat = Some(0.0);
    let dt = 1.0 / 120.0;
    let kt = |v: f64| v * 1.943_84;
    println!("{}: mass {:.0} kg, cruise {:.0} kt, approach {:.0} kt, vne {:.0} kt", spec.name, spec.mass, kt(spec.cruise), kt(spec.approach.0), kt(spec.vne));
    let (lat, lon) = (37.6f64, -122.37f64);
    // wings held level; pitch to an attitude (degrees), gently
    let level = |p: &Plane| (p.right().dot(p.pos.normalize()) * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0);
    let attitude = |p: &Plane, want: f64| ((want - p.forward().dot(p.pos.normalize()).asin().to_degrees()) * 0.1 - p.omega.x * 0.3).clamp(-1.0, 1.0);
    // Take-off: first flap notch, full power, rotate to 10° at 1.1 times the clean stall.
    let mut p = Plane::spawn_ground(spec, lat.to_radians(), lon.to_radians(), 0.0, 0.5, 0.0);
    p.hold = false;
    p.t = 2.0;
    if spec.max_notch() > 0 {
        p.flap_set = 1;
        p.flaps = spec.flap_frac(1);
    }
    let vr = spec.approach.0 / 1.3 * 1.15;
    let start = p.pos;
    let (mut off, mut t, mut crashed) = (None, 0.0, false);
    while t < 120.0 {
        let c = Controls { throttle: 1.0, pitch: if p.vel.length() > vr { attitude(&p, 10.0) } else { 0.0 }, roll: level(&p), ..Default::default() };
        if p.step(dt, &c, &w).is_some() {
            crashed = true;
            break;
        }
        if off.is_none() && !p.on_ground && t > 5.0 {
            off = Some((t, (p.pos - start).length(), kt(p.vel.length())));
        }
        if off.is_some_and(|o| t > o.0 + 20.0) {
            break;
        }
        t += dt;
    }
    let h = geo::pos_to_lla(p.pos).2;
    match off {
        Some((t, d, v)) => println!("  take-off: lift-off after {t:.0} s, {d:.0} m, {v:.0} kt; 20 s later {h:.0} m{}", if crashed { ", CRASHED" } else { "" }),
        None => println!("  take-off: never left the ground ({:.0} kt){}", kt(p.vel.length()), if crashed { ", CRASHED" } else { "" }),
    }
    println!("    notes {:?}, gear {:?}", p.notes, p.dmg.gear);
    // Level at full power: hold the height, see where the speed settles.
    let alt0 = 1500.0;
    let hold = |p: &Plane| {
        let h = geo::pos_to_lla(p.pos).2;
        ((alt0 - h) * 0.002 - p.vel.dot(p.pos.normalize()) * 0.02 - p.omega.x * 0.3).clamp(-1.0, 1.0)
    };
    let mut p = Plane::spawn(spec, lat.to_radians(), lon.to_radians(), alt0, 0.0, spec.cruise);
    p.throttle = 1.0;
    let mut line = String::new();
    for k in 0..(180.0 / dt) as usize {
        let c = Controls { throttle: 1.0, pitch: hold(&p), roll: level(&p), ..Default::default() };
        if p.step(dt, &c, &w).is_some() {
            line += " CRASHED";
            break;
        }
        if k % (30.0 / dt) as usize == 0 {
            line += &format!("  {:.0}kt/{:.0}m", kt(p.ias()), geo::pos_to_lla(p.pos).2);
        }
    }
    println!("  level, full power:{line}  -> {:.0} kt, alpha {:.1}°, wings {:?}, notes {:?}", kt(p.ias()), p.alpha.to_degrees(), p.dmg.wing, p.notes);
    // Power off, hold the height until it stalls.
    let mut p = Plane::spawn(spec, lat.to_radians(), lon.to_radians(), alt0, 0.0, spec.cruise * 0.7);
    p.throttle = 0.0;
    let mut stall = None;
    for k in 0..(240.0 / dt) as usize {
        let c = Controls { throttle: 0.0, pitch: hold(&p), roll: level(&p), ..Default::default() };
        if p.step(dt, &c, &w).is_some() {
            break;
        }
        if stall.is_none() && k as f64 * dt > 5.0 && (p.alpha > spec.stall_alpha * 0.98 || alt0 - geo::pos_to_lla(p.pos).2 > 60.0) {
            stall = Some((kt(p.ias()), p.alpha.to_degrees()));
        }
    }
    match stall {
        Some((v, a)) => println!("  slow-down, power off: stalls at {v:.0} kt, {a:.1}°"),
        None => println!("  slow-down: no stall in 240 s"),
    }
    0
}

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn,fablegame=info")).init();
    // A JSBSim aircraft compared with one of ours (see jsbsim.rs), and out.
    let argv: Vec<String> = std::env::args().collect();
    if let Some(i) = argv.iter().position(|a| a == "--jsbsim-compare") {
        std::process::exit(jsbsim_compare(argv.get(i + 1), argv.iter().position(|a| a == "--aircraft").and_then(|k| argv.get(k + 1))));
    }
    if let Some(i) = argv.iter().position(|a| a == "--fg-survey") {
        std::process::exit(hangar::survey(argv.get(i + 1).map(|s| s.as_str())));
    }
    if let Some(i) = argv.iter().position(|a| a == "--jsbsim-fit") {
        std::process::exit(hangar::fit_report(&argv[i + 1..]));
    }
    if let Some(i) = argv.iter().position(|a| a == "--fly-check") {
        std::process::exit(fly_check(argv.get(i + 1)));
    }
    if let Some(i) = argv.iter().position(|a| a == "--fg-install") {
        std::process::exit(hangar::install_report(&argv[i + 1..]));
    }
    // Downloaded JSBSim aircraft join the list before `--aircraft` is read.
    hangar::load_installed();
    let args = parse_args();
    if args.find_storms {
        let cache = dirs::cache_dir().unwrap_or_else(|| PathBuf::from(".")).join("fablegame");
        match livewx::find_storms(&fetch::Fetcher::new(cache, false)) {
            Ok(s) if s.is_empty() => println!("no strong storms on radar right now"),
            Ok(s) => {
                println!("strongest radar echoes now (fly there with --lat --lon --live-weather):");
                for (lat, lon, z) in s {
                    println!("  --lat {lat:.2} --lon {lon:.2}   {z:.0} dBZ");
                }
            }
            Err(e) => println!("weather radar: {e}"),
        }
        return Ok(());
    }
    i18n::set_lang(args.lang.or(settings::Settings::load().lang).unwrap_or_else(i18n::Lang::detect));
    // gamemode while we fly, if installed.
    let _gamemode = gamemode::GameMode::request();
    let el = EventLoop::new()?;
    el.set_control_flow(ControlFlow::Poll);
    el.run_app(&mut App { args, game: None, frame_pacer: frame_pacer::FramePacer::default() })?;
    Ok(())
}


const REC_FPS: f64 = 30.0;
/// Airborne starts: 107 kt, about a 172's cruise down low.

/// 16-bit stereo WAV.
fn write_wav(path: &std::path::Path, x: &[[f32; 2]], rate: u32) {
    let data = (x.len() * 4) as u32;
    let mut w = Vec::with_capacity(44 + data as usize);
    w.extend_from_slice(b"RIFF");
    w.extend_from_slice(&(36 + data).to_le_bytes());
    w.extend_from_slice(b"WAVEfmt ");
    for v in [16u32.to_le_bytes(), 0x0002_0001u32.to_le_bytes(), rate.to_le_bytes(), (rate * 4).to_le_bytes(), 0x0010_0004u32.to_le_bytes()] {
        w.extend_from_slice(&v);
    }
    w.extend_from_slice(b"data");
    w.extend_from_slice(&data.to_le_bytes());
    for s in x {
        for c in s {
            w.extend_from_slice(&((c.clamp(-1.0, 1.0) * 32767.0) as i16).to_le_bytes());
        }
    }
    if let Err(e) = std::fs::write(path, w) {
        log::error!("writing {}: {e}", path.display());
    }
}

fn smoothstep(a: f64, b: f64, x: f64) -> f64 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The xpad driver's Xbox layout, in SDL's game controller mapping format.
const XBOX_LAYOUT: &str = "a:b0,b:b1,back:b6,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,dpup:h0.1,guide:b8,leftshoulder:b4,leftstick:b9,lefttrigger:a2,leftx:a0,lefty:a1,rightshoulder:b5,rightstick:b10,righttrigger:a5,rightx:a3,righty:a4,start:b7,x:b2,y:b3,platform:Linux,";

/// An Xbox pad gilrs found no mapping for: a firmware newer than its copy of
/// the SDL controller database, say. Left so, its triggers show up as bare
/// axes and X and Y trade places (xpad reports X as BTN_NORTH).
fn needs_xbox_layout(p: &gilrs::Gamepad) -> bool {
    let name = p.os_name().to_lowercase();
    cfg!(target_os = "linux")
        && p.mapping_source() != gilrs::MappingSource::SdlMappings
        && (p.vendor_id() == Some(0x045e) || name.contains("xbox") || name.contains("x-box"))
}

/// The gamepads, with any Xbox pad that has no mapping given the standard Xbox one.
fn open_gamepads() -> Option<gilrs::Gilrs> {
    // (scripted test runs: a pad someone is holding would steer the aircraft)
    if std::env::var_os("FABLE_NO_PAD").is_some() {
        return None;
    }
    let g = gilrs::Gilrs::new().map_err(|e| log::warn!("gamepads: {e}")).ok()?;
    let extra: String = g
        .gamepads()
        .filter(|(_, p)| needs_xbox_layout(p))
        .map(|(_, p)| {
            let guid: String = p.uuid().iter().map(|b| format!("{b:02x}")).collect();
            log::info!("gamepad: {} ({guid}) has no mapping; using the Xbox layout", p.os_name());
            format!("{guid},{},{XBOX_LAYOUT}\n", p.os_name().replace(',', " "))
        })
        .collect();
    if extra.is_empty() {
        return Some(g);
    }
    drop(g);
    let g = gilrs::GilrsBuilder::new().add_mappings(&extra).build().map_err(|e| log::warn!("gamepads: {e}")).ok()?;
    for (_, p) in g.gamepads() {
        log::info!("gamepad: {} mapping {:?}", p.os_name(), p.mapping_source());
    }
    Some(g)
}

/// A button that steps once when pressed, then repeats while held
/// (after 0.4 s, ten steps a second, speeding to twenty-five after 2 s).
fn hold_repeat(r: &egui::Response) -> bool {
    let id = r.id.with("hold_repeat");
    // Keep holding while the primary button stays down over the button —
    // egui's own press tracking drops out after a long press.
    let held_on = r.ctx.input(|i| {
        i.pointer.primary_down() && i.pointer.interact_pos().is_some_and(|p| r.rect.contains(p))
    });
    let started = r.ctx.data(|d| d.get_temp::<(f64, u32)>(id).is_some());
    if !(r.is_pointer_button_down_on() || (started && held_on)) {
        r.ctx.data_mut(|d| d.remove::<(f64, u32)>(id));
        return false;
    }
    r.ctx.request_repaint();
    let t = r.ctx.input(|i| i.time);
    let (start, fired) = r.ctx.data_mut(|d| *d.get_temp_mut_or_insert_with(id, || (t, 0u32)));
    let held = t - start;
    let due = if held < 0.4 {
        1
    } else if held < 2.0 {
        1 + ((held - 0.4) / 0.1) as u32
    } else {
        17 + ((held - 2.0) / 0.04) as u32
    };
    if due > fired {
        r.ctx.data_mut(|d| d.insert_temp(id, (start, fired + 1)));
        true
    } else {
        false
    }
}

/// An aircraft shot down, on its way to the ground.
struct Kill {
    key: weapons::TargetKey,
    pos: DVec3,
    vel: DVec3,
    /// its wingspan (m), which sizes the fire and the explosions
    span: f32,
    /// seconds since the hit, and the trail particles owed
    t: f64,
    acc: f64,
}

/// The top three rows of a piece's pose, for the shader (render.rs `DrawObj::pose`).
fn pose_rows(m: &Mat4) -> [[f32; 4]; 3] {
    [m.row(0).to_array(), m.row(1).to_array(), m.row(2).to_array()]
}
