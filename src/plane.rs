//! The aircraft's systems and a six-degree-of-freedom flight model, driven
//! by the type's `Spec` (aircraft/): lift, drag and stall from its wing,
//! flaps, slats and spoilers; a piston engine turning a fixed-pitch propeller
//! (revs and thrust follow the airspeed, power falls with altitude, torque
//! and slipstream yaw it) or turbofans (the fans spool up and down, thrust
//! lapses with height and Mach, reversers on the ground); the airspeed
//! indicator reading indicated (not true) airspeed; adverse yaw, stall
//! buffet and wing drop, ground effect, wave drag near the speed of sound,
//! and the turbulence the weather stirs up.
//! The models are built in `aircraft/`; `pose` animates their pieces;
//! `damage.rs` describes what has been knocked off. Trees, rooftops, the
//! ground, over-speed and over-G all take parts off the airframe here, and
//! the aircraft flies on (as best it can) with what is left.
use crate::aircraft::{Engine, Part, Spec, MAX_GEAR};
use crate::airframe::{Anim, Drive, Input, Piece, Role, G_ENGINE, G_FUSE, G_LWING, G_RWING, G_TAIL};
use crate::crash::Rng;
use crate::damage::{self, Damage, Sel, ShedReq, Tear};
use crate::weather::Air;
use crate::geo;
use crate::world::{TreeRef, World};
use glam::{DMat3, DQuat, DVec3, Mat4, Vec3};

mod rocket;

#[derive(Default, Clone, Copy)]
pub struct Controls {
    pub pitch: f64, // +1 = nose up
    pub roll: f64,  // +1 = roll right
    pub yaw: f64,   // +1 = nose right
    pub throttle: f64,
    pub brake: bool,
    /// Independent toe-brake pressure, 0..1.
    pub brake_left: f64,
    pub brake_right: f64,
    pub boost: bool,
    /// thrust reversers (on the ground, jets)
    pub reverse: bool,
    /// the starter held (a piston engine cranks while it is)
    pub starter: bool,
    /// a throttle lever's position (0..1), which then sets the power
    /// outright instead of `throttle` moving it
    pub throttle_set: Option<f64>,
}

/// An aircraft ploughing into a building after hitting its wall. It does not
/// go up at the first touch of the nose: the wall gives, and the fuel goes up
/// once the cabin and the wing tanks are in. How far in that happens depends
/// on the building: a heavy one (big and tall, concrete) stops it sooner, a
/// light one lets it further in, and each building differs a little (floor
/// slabs, columns, where it hit). Estimates for the look, not crash data.
#[derive(Clone, Copy, Debug)]
struct Bury {
    building: (geo::TileId, usize),
    /// distance travelled since the wall (m) and where it goes up
    gone: f64,
    depth: f64,
    /// fraction of its speed it keeps per metre in
    keep_per_m: f64,
    /// what the first touch was (for the cause)
    cause: &'static str,
}

pub struct Impact {
    pub pos: DVec3,
    pub vel: DVec3,
    pub water: bool,
    pub ground_h: f64,
    /// the building that was hit, if any (tile, footprint index)
    pub building: Option<(geo::TileId, usize)>,
}

/// Positions around the piston aircraft's selector. The centre feed is BOTH
/// on the 172RG and the main fuselage tank on the Régent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FuelSelector {
    Center,
    Right,
    Off,
    Left,
}

impl FuelSelector {
    pub fn turn(self, clockwise: bool) -> Self {
        use FuelSelector::*;
        match (self, clockwise) {
            (Center, true) | (Off, false) => Right,
            (Right, true) | (Left, false) => Off,
            (Off, true) | (Center, false) => Left,
            (Left, true) | (Right, false) => Center,
        }
    }
}

pub struct Plane {
    /// what it is
    pub spec: &'static Spec,
    pub pos: DVec3,
    pub vel: DVec3,
    /// the air's own motion where the aircraft is (planet frame); the wing only feels `vel - wind`
    pub wind: DVec3,
    /// how rough the air is here (set by the caller from the weather each frame)
    pub air: Air,
    /// the turbulent part of the air's motion (planet frame), on top of `wind`
    pub gust: DVec3,
    /// rotational gusts (body frame, rad/s): the air rolling, pitching and yawing across the airframe
    pub gust_rot: DVec3,
    /// turbulence filter states: translational in the flight-path frame (along, across, up), and rotational
    gust_a: DVec3,
    gust_b: DVec3,
    rot_a: DVec3,
    rot_b: DVec3,
    /// small eddies (sharp jolts), and the slow level that gathers the
    /// turbulence into bursts
    gust_s: DVec3,
    burst: f64,
    /// the pilot's head swaying on its neck as the airframe is shaken (body frame, m)
    pub head: DVec3,
    /// specific force (what an accelerometer reads: everything but gravity), body frame, m/s²
    pub sf: DVec3,
    head_v: DVec3,
    /// shaft power being made, as a fraction of rated (a jet: thrust as a fraction of rated)
    pub power: f64,
    /// how far open the throttle effectively is (the engine takes a moment to answer it)
    charge: f64,
    /// fuel on board (kg), and what the engines are burning (kg/s)
    pub fuel: f64,
    pub fuel_flow: f64,
    /// the magnetos on (a jet: the fuel control at RUN), and the mixture
    /// lever (0 idle cut-off .. 1 full rich; jets have none)
    pub ignition: bool,
    pub mixture: f64,
    /// the engine is burning fuel and turning itself
    pub running: bool,
    /// a jet's starter engaged (it stays in until the engine is up to idle),
    /// and an automatic start under way (seconds it still has)
    pub starter_on: bool,
    /// the starter is turning the engine over this instant (the key at START)
    pub cranking: bool,
    /// the parking brake is set (it holds the wheel brakes on)
    pub parking_brake: bool,
    pub toe_brake: [f64; 2],
    pub landing_lights: bool,
    /// A two-engine jet's L and R fuel control switches ([0]) and battery
    /// switches ([1]) work the one shared `ignition` / `sys.elec.master`:
    /// the shared state is on while either switch of its pair is. True: that
    /// switch is off while its partner holds the state on. (All false, both
    /// show the shared state; keys and other controls set both together.)
    pub pair_off: [[bool; 2]; 2],
    pub fuel_selector: FuelSelector,
    pub engine_hours: f64,
    pub propeller: f64,
    prop_coarse: f64,
    pub cowl_flaps: f64,
    auto_start: f64,
    /// the electrical system and the failures (systems.rs)
    pub sys: crate::systems::Systems,
    /// payload aboard (kg): crew, passengers, baggage
    pub payload: f64,
    /// a jet's fan speed, fraction of 100% N1
    pub n1: f64,
    /// how far the afterburners are lit, 0..1 (what is seen and heard of them; the thrust is immediate)
    pub burner: f64,
    /// which wing a stall would rather drop (slowly wandering, -1..1)
    drop: f64,
    pub rot: DQuat,
    pub omega: DVec3,
    pub throttle: f64,
    pub prop: f64,
    pub on_ground: bool,
    pub stalled: bool,
    pub alpha: f64,
    pub g_load: f64,
    /// smoothed control inputs: pitch, yaw, roll
    pub stick: DVec3,
    /// flap extension, 0 up .. 1 full travel, and the selected lever notch
    pub flaps: f64,
    pub flap_set: u8,
    /// leading-edge devices, 0 stowed .. 1 out
    pub slats: f64,
    /// the speedbrake lever (0 down .. 1) and where the spoiler panels are (0..1)
    pub speedbrake: f64,
    pub spoilers: f64,
    /// its braking parachute has been used (let go, or torn away)
    pub chute_spent: bool,
    /// thrust reversers, 0 stowed .. 1 deployed
    pub reverser: f64,
    /// gear position, 0 stowed .. 1 down and locked, and where the handle is
    pub gear: f64,
    pub gear_down: bool,
    /// a sliding canopy, 0 shut .. 1 open, and which way it is going
    pub canopy: f64,
    pub canopy_open: bool,
    /// per landing gear leg (see `Spec::gear`): how far its oleo or spring is
    /// compressed (m), how far its wheels have turned (rad) and how fast (rad/s)
    pub compress: [f64; MAX_GEAR],
    pub wheel_rot: [f64; MAX_GEAR],
    pub wheel_spin: [f64; MAX_GEAR],
    /// propeller (or fan) speed, rad/s
    pub prop_rate: f64,
    /// the angle of attack the auto-trim is holding (drives the trim wheel)
    pub trim: f64,
    /// trees struck during the last step, for the caller to knock down
    pub tree_hits: Vec<TreeRef>,
    /// recent brushes with foliage
    pub brushes: f64,
    /// what is broken, and what was broken at the start of the step (so the
    /// caller can still find the pieces that have just gone)
    pub dmg: Damage,
    pub dmg_prev: Damage,
    /// parts that came off during the last step, for the caller to let go of
    pub shed: Vec<ShedReq>,
    /// one line per thing that just broke, for the HUD and the sound
    pub notes: Vec<String>,
    /// where metal is grinding along the ground: position, velocity of the sparks
    pub sparks: Vec<(DVec3, DVec3)>,
    /// seconds flown
    pub t: f64,
    near_trees: Vec<TreeRef>,
    /// trees something went into that are still standing, with how deep the
    /// worst contact went (1 = foliage, 2 = trunk), so they are not struck again
    struck: Vec<(geo::TileId, usize, u8)>,
    scan: u32,
    rng: Rng,
    /// held where it is (waiting on the ground for the terrain under it to load)
    pub hold: bool,
    /// where it is splashing through water: position, velocity of the spray
    pub splashes: Vec<(DVec3, DVec3)>,
    /// where something is ploughing or rolling through the ground: position,
    /// velocity the dirt is thrown at, how much (wheel ~1, airframe more)
    pub dirt: Vec<(DVec3, DVec3, f32)>,
    /// tracks and gouges carved into snow on the ground this step (snowtrack.rs)
    pub snow_marks: Vec<crate::snowtrack::Mark>,
    /// previous ground-contact position of each gear leg, for continuous tyre tracks
    wheel_prev: [Option<DVec3>; MAX_GEAR],
    /// resting or sliding on the airframe itself rather than the wheels
    pub on_hull: bool,
    /// contact points (reused each step)
    pts: Vec<(DVec3, Part)>,
    /// moments since the left wing, right wing, tail and nose last broke
    /// something (so one blow is one break, not one per step)
    cool: [f64; 4],
    /// torque the ground puts on each hanging wing this step
    fold_tq: [f64; 2],
    /// how far a ditched aircraft has settled into the water
    sink: f64,
    /// wing tip deflection from the modelled (1 g) shape (m), left then right;
    /// drawn only (see `flex_wings`)
    pub wing_flex: [f64; 2],
    /// wing flex rate of change (m/s)
    wing_flex_v: [f64; 2],
    /// what made it go up in flames, once it has
    pub cause: &'static str,
    /// going into a building: it ploughs on through the floors before it goes up
    bury: Option<Bury>,
    /// a rocket (plane/rocket.rs): the stage it is flying on (0 the whole
    /// stack), which of that stage's engine groups is lit, a stage separation
    /// asked for, and its attitude hold's memory
    pub stage: usize,
    pub lit: usize,
    stage_cmd: bool,
    sas: DVec3,
    /// its hot staging under way (s since it was asked for; 0: not), the
    /// smoothed stick, where along the model `pos` is (body z: the stack's
    /// origin, then the middle of the ship), and where it left the ground
    pub hot: f64,
    cmd: DVec3,
    pub origin: f64,
    pub launch: DVec3,
    /// the ship flies its own landing: the flip and the burn down to its skirt
    pub auto: bool,
}

/// The breakable group a contact point belongs to.
fn part_group(p: Part) -> usize {
    match p {
        Part::Nose | Part::Prop => G_ENGINE,
        Part::Wing(0, _) | Part::Fold(0) | Part::Pod(0) => G_LWING,
        Part::Wing(..) | Part::Fold(_) | Part::Pod(_) => G_RWING,
        Part::Tail | Part::Fin | Part::Stab(_) => G_TAIL,
        _ => G_FUSE,
    }
}

/// Sea-level air density (kg/m³).
pub const RHO0: f64 = 1.225;

impl Plane {
    pub fn spawn(spec: &'static Spec, lat: f64, lon: f64, alt: f64, heading: f64, speed: f64) -> Self {
        let pos = geo::lla_to_pos(lat, lon, alt);
        let (e, n, u) = geo::enu(pos);
        let f = n * heading.cos() + e * heading.sin();
        let mut p = Self {
            spec,
            pos,
            vel: f * speed,
            wind: DVec3::ZERO,
            air: Air::default(),
            gust: DVec3::ZERO,
            gust_rot: DVec3::ZERO,
            gust_a: DVec3::ZERO,
            gust_b: DVec3::ZERO,
            rot_a: DVec3::ZERO,
            rot_b: DVec3::ZERO,
            gust_s: DVec3::ZERO,
            burst: 0.0,
            head: DVec3::ZERO,
            sf: DVec3::new(0.0, 9.81, 0.0),
            head_v: DVec3::ZERO,
            power: 0.7,
            charge: 0.0,
            fuel: spec.fuel_kg * 0.5,
            fuel_flow: 0.0,
            ignition: true,
            mixture: 1.0,
            running: true,
            starter_on: false,
            cranking: false,
            parking_brake: false,
            toe_brake: [0.0; 2],
            landing_lights: true,
            pair_off: [[false; 2]; 2],
            fuel_selector: FuelSelector::Center,
            engine_hours: 1834.6,
            propeller: 1.0,
            prop_coarse: 0.0,
            cowl_flaps: 1.0,
            auto_start: 0.0,
            sys: Default::default(),
            payload: spec.payload[0],
            n1: 0.0,
            burner: 0.0,
            drop: 0.0,
            rot: DQuat::IDENTITY,
            omega: DVec3::ZERO,
            throttle: 0.0,
            prop: 0.0,
            on_ground: false,
            stalled: false,
            alpha: 0.0,
            g_load: 1.0,
            stick: DVec3::ZERO,
            flaps: 0.0,
            flap_set: 0,
            slats: 0.0,
            speedbrake: 0.0,
            spoilers: 0.0,
            chute_spent: false,
            reverser: 0.0,
            // Airborne starts are cleaned up: gear stowed (unless it is fixed).
            gear: if spec.fixed_gear() { 1.0 } else { 0.0 },
            gear_down: spec.fixed_gear(),
            canopy: 0.0,
            canopy_open: false,
            compress: [0.0; MAX_GEAR],
            wheel_rot: [0.0; MAX_GEAR],
            wheel_spin: [0.0; MAX_GEAR],
            prop_rate: 0.0,
            trim: 0.0,
            tree_hits: Vec::new(),
            brushes: 0.0,
            dmg: Damage::new(spec),
            dmg_prev: Damage::new(spec),
            shed: Vec::new(),
            notes: Vec::new(),
            sparks: Vec::new(),
            dirt: Vec::new(),
            snow_marks: Vec::new(),
            wheel_prev: [None; MAX_GEAR],
            t: 0.0,
            near_trees: Vec::new(),
            struck: Vec::new(),
            scan: 0,
            rng: Rng(pos.x.to_bits() | 1),
            hold: false,
            splashes: Vec::new(),
            on_hull: false,
            pts: Vec::new(),
            cool: [0.0; 4],
            fold_tq: [0.0; 2],
            sink: 0.0,
            wing_flex: [0.0; 2],
            wing_flex_v: [0.0; 2],
            cause: "",
            bury: None,
            stage: 0,
            lit: 0,
            stage_cmd: false,
            sas: DVec3::ZERO,
            hot: 0.0,
            cmd: DVec3::ZERO,
            origin: 0.0,
            launch: pos,
            auto: false,
        };
        if spec.rocket().is_some() {
            p.rocket_air_start(f, u, speed);
            return p;
        }
        // Nose up by the angle of attack that carries the weight at this speed
        // and height, and trimmed there, so it starts out in level flight.
        let rho = density(alt);
        p.trim = p.alpha_level(speed, rho, 0.0);
        // Power for level flight too, and the propeller (or fans) already
        // turning as that power has them, or it starts out climbing or sinking.
        p.throttle = if speed > 20.0 { p.throttle_for(speed, alt, 0.0, 0.0, 0.0) } else { 0.0 };
        p.charge = p.throttle;
        p.settle_engine(speed, alt);
        p.rot = DQuat::from_mat3(&DMat3::from_cols(f.cross(u), u, -f)) * DQuat::from_rotation_x(p.trim);
        p
    }

    /// Standing on its wheels, engine idling, brakes off: ready to taxi.
    /// `h` is the ground height under it. It is held still until released.
    pub fn spawn_ground(spec: &'static Spec, lat: f64, lon: f64, h: f64, heading: f64, pitch: f64) -> Self {
        if spec.rocket().is_some() {
            return Self::spawn_pad(spec, lat, lon, h, heading);
        }
        // The wheels just touching (it settles onto its oleos).
        let sag = -spec.gear.iter().map(|l| l.at.y).fold(0.0f64, f64::min);
        let mut p = Self::spawn(spec, lat, lon, h + sag, heading, 0.0);
        let (e, n, u) = geo::enu(p.pos);
        let f = n * heading.cos() + e * heading.sin();
        p.rot = DQuat::from_mat3(&DMat3::from_cols(f.cross(u), u, -f)) * DQuat::from_rotation_x(pitch);
        p.vel = DVec3::ZERO;
        p.gear = 1.0;
        p.gear_down = true;
        p.throttle = 0.0;
        p.charge = 0.0;
        p.power = 0.0;
        match &spec.engine {
            Engine::Piston(_) => p.prop_rate = 65.0,
            Engine::Fan(fe) => {
                p.n1 = fe.idle;
                p.prop_rate = fe.idle * fe.n1_rpm * std::f64::consts::TAU / 60.0;
            }
            Engine::Rocket(_) => {}
        }
        p.trim = 0.08;
        p.on_ground = true;
        p.hold = true;
        p
    }
    /// How high above the ground the origin is with the wheels just touching it.
    pub fn wheel_height(&self) -> f64 {
        if let Some(rk) = self.spec.rocket() {
            return rk.stages[self.stage].z[1] - self.origin;
        }
        -self.spec.gear.iter().map(|l| l.at.y).fold(0.0f64, f64::min)
    }
    /// How high above the ground its origin is as it stands on it now.
    pub fn stand_height(&self) -> f64 {
        self.wheel_height() * if self.spec.rocket().is_some() { 1.0 } else { self.up().dot(self.pos.normalize()) }
    }
    pub fn forward(&self) -> DVec3 {
        self.rot * DVec3::NEG_Z
    }
    pub fn up(&self) -> DVec3 {
        self.rot * DVec3::Y
    }
    pub fn right(&self) -> DVec3 {
        self.rot * DVec3::X
    }

    /// Nose wheel steering angle (rad, + = right), only with the gear down;
    /// on an airliner it fades to what the rudder pedals give as it speeds up.
    pub fn steer_angle(&self) -> f64 {
        let s = self.spec;
        let fade = if s.steer_fade > 0.0 { 0.2 + 0.8 * (1.0 - smooth(3.0, s.steer_fade, self.vel.length())) } else { 1.0 };
        self.stick.y * s.steer_max * fade * self.gear
    }
    /// Angle of attack the wing stalls at (flaps make it stall a little earlier, slats later).
    /// What it weighs now (kg): `spec.mass` with half tanks, more or less the fuel.
    pub fn mass(&self) -> f64 {
        if let Some(rk) = self.spec.rocket() {
            return self.rocket_mass(rk).0;
        }
        self.spec.mass + self.fuel - 0.5 * self.spec.fuel_kg + self.payload - self.spec.payload[0]
    }

    /// Centre of gravity, as a fraction of the mean aerodynamic chord: where
    /// the fuel and the payload put it, against the reference at `spec.mass`.
    pub fn cg(&self) -> f64 {
        let s = self.spec;
        let moment = (self.fuel - 0.5 * s.fuel_kg) * s.fuel_arm + (self.payload - s.payload[0]) * s.payload[2];
        s.cg[0] + moment / self.mass() / s.mac
    }

    /// How far the CG is outside its limits (fraction of MAC; 0 inside, - forward, + aft).
    pub fn cg_excess(&self) -> f64 {
        let (c, s) = (self.cg(), self.spec);
        if c < s.cg[1] { c - s.cg[1] } else if c > s.cg[2] { c - s.cg[2] } else { 0.0 }
    }

    /// Fuel left, as a fraction of full tanks.
    pub fn fuel_frac(&self) -> f64 {
        if let Some(rk) = self.spec.rocket() {
            return self.fuel / rk.stages[self.stage].prop;
        }
        if self.spec.fuel_kg > 0.0 { self.fuel / self.spec.fuel_kg } else { 0.0 }
    }

    /// How long the fuel lasts at the present burn (s), while it burns any.
    pub fn endurance(&self) -> Option<f64> {
        (self.fuel_flow > 1e-6).then(|| self.fuel / self.fuel_flow)
    }

    /// Cold and dark: magnetos off and mixture at cut-off (a jet: fuel
    /// controls to CUTOFF), the engine stopped, the throttle closed.
    pub fn cold_and_dark(&mut self) {
        (self.ignition, self.mixture, self.running, self.starter_on, self.auto_start) = (false, 0.0, false, false, 0.0);
        (self.throttle, self.charge, self.power, self.prop_rate, self.n1, self.fuel_flow) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        self.sys.elec.master = false;
    }

    /// The start a pilot would make: mixture rich, magnetos (fuel controls)
    /// on, throttle at idle, and the starter until the engine runs.
    pub fn start_engine(&mut self) {
        (self.mixture, self.ignition) = (1.0, true);
        self.sys.elec.master = true;
        self.throttle = self.throttle.min(0.1);
        if self.spec.is_jet() {
            self.starter_on = true;
            self.auto_start = 120.0;
        } else {
            self.auto_start = 8.0;
        }
    }

    /// Shut down: mixture to cut-off (a jet: fuel controls to CUTOFF).
    pub fn stop_engine(&mut self) {
        if self.spec.is_jet() {
            self.ignition = false;
        } else {
            self.mixture = 0.0;
        }
        (self.auto_start, self.starter_on) = (0.0, false);
    }

    pub fn stall_alpha(&self) -> f64 {
        let s = self.spec;
        s.stall_alpha - s.stall_flap * self.flaps + s.stall_slat * self.slats
    }
    pub fn flap_deg(&self) -> f64 {
        self.flaps * self.spec.flaps.last().copied().unwrap_or(0.0)
    }
    /// The flap lever's notch, as the pilot calls it (degrees).
    pub fn flap_notch_deg(&self) -> f64 {
        self.spec.flaps.get(self.flap_set as usize).copied().unwrap_or(0.0)
    }
    fn brake_pressure(&self, c: &Controls, side: f64) -> f64 {
        if c.brake || self.parking_brake { 1.0 } else if side < 0.0 { c.brake_left.clamp(0.0, 1.0) } else { c.brake_right.clamp(0.0, 1.0) }
    }

    pub fn toggle_gear(&mut self) {
        // The squat switch blocks retraction on the ground. (A rocket has none to raise.)
        if !(self.gear_down && self.on_ground) && self.spec.rocket().is_none() && !self.spec.fixed_gear() {
            self.gear_down = !self.gear_down;
        }
    }
    /// Slides the canopy open or shut (the model decides whether it has one).
    pub fn toggle_canopy(&mut self) {
        self.canopy_open = !self.canopy_open;
    }
    pub fn set_flaps(&mut self, delta: i32) {
        self.flap_set = (self.flap_set as i32 + delta).clamp(0, self.spec.max_notch() as i32) as u8;
    }
    /// Speedbrake lever: down, or out to the flight detent (airliners only).
    pub fn toggle_speedbrake(&mut self) {
        if self.spec.chute.is_some() {
            // A braking parachute instead: the key streams it, and again lets it go.
            if self.speedbrake > 0.0 {
                self.drop_chute();
            } else if !self.chute_spent {
                self.speedbrake = 1.0;
            }
        } else if self.spec.spoiler_cd > 0.0 {
            self.speedbrake = if self.speedbrake > 0.0 { 0.0 } else { 0.5 };
        }
    }
    /// The braking parachute goes: it is not there to stream again.
    fn drop_chute(&mut self) {
        (self.speedbrake, self.spoilers, self.chute_spent) = (0.0, 0.0, true);
    }
    /// The highest speed (indicated) the flaps may be out at as they are now.
    pub fn vfe(&self) -> f64 {
        let s = self.spec;
        let k = (0..s.flaps.len()).find(|&k| s.flap_frac(k as u8) >= self.flaps - 0.01).unwrap_or(s.flaps.len() - 1);
        s.vfe.get(k).copied().unwrap_or(1e9)
    }
    /// Indicated airspeed (m/s): what the pitot tube feels, true airspeed scaled
    /// by the square root of the density. It is what the wing flies by: the
    /// aircraft stalls, and its limits are set, at the same indicated speed at any height.
    pub fn ias(&self) -> f64 {
        let (_, _, h) = geo::pos_to_lla(self.pos);
        self.airspeed().length() * (density(h) / RHO0).sqrt()
    }
    /// Mach number.
    pub fn mach(&self) -> f64 {
        let (_, _, h) = geo::pos_to_lla(self.pos);
        self.airspeed().length() / sound_speed(h)
    }
    /// How far past its speed limits it is (m/s indicated; the Mach limit counted
    /// as the same margin in speed), negative within them.
    pub fn overspeed(&self) -> f64 {
        self.excess(self.ias(), self.mach())
    }
    fn excess(&self, ias: f64, mach: f64) -> f64 {
        let s = self.spec;
        let e = ias - s.vne;
        if s.mmo > 0.0 { e.max((mach - s.mmo) * 300.0) } else { e }
    }
    /// Start out with the flaps at `notch` and the gear as given, re-trimmed
    /// for the airspeed it has (to fly level, or on a glide of `descent` m/s),
    /// the power set to match.
    pub fn configure(&mut self, notch: u8, gear_down: bool, descent: f64) {
        self.flap_set = notch.min(self.spec.max_notch());
        self.flaps = self.spec.flap_frac(self.flap_set);
        self.slats = if self.spec.slats_at > 0 && self.flap_set >= self.spec.slats_at { 1.0 } else { 0.0 };
        self.gear_down = gear_down || self.spec.fixed_gear();
        self.gear = if self.gear_down { 1.0 } else { 0.0 };
        let (_, _, h) = geo::pos_to_lla(self.pos);
        let rho = density(h);
        let v = self.airspeed().length();
        let flap_cl = self.spec.flap_cl * self.flaps;
        let a_old = self.trim;
        self.trim = self.alpha_level(v, rho, flap_cl);
        // Pitch the nose to the new angle of attack and the flight path down the glide.
        let up = self.pos.normalize();
        let fp = (self.airspeed() - up * descent).normalize();
        self.vel = self.wind + fp * v;
        let right = self.right();
        self.rot = DQuat::from_axis_angle(right, self.trim - a_old - (descent / v).asin()) * self.rot;
        self.throttle = self.throttle_for(v, h, self.flaps, self.gear, descent);
        self.charge = self.throttle;
        self.settle_engine(v, h);
    }

    /// Whether switch `k` (0 L, 1 R) of a pair on a shared state is on: `w` 0
    /// the fuel controls (`ignition`), 1 the batteries (`sys.elec.master`).
    /// See `pair_off`; both marked off cannot hold the state on, so both show it.
    pub fn pair_on(&self, w: usize, k: usize) -> bool {
        let on = if w == 0 { self.ignition } else { self.sys.elec.master };
        let off = self.pair_off[w];
        on && !(off[k] && !off[1 - k])
    }

    /// The value an animation input has this instant (see airframe::Input).
    fn input(&self, i: Input) -> f32 {
        let s = self.stick.as_vec3();
        match i {
            Input::Aileron(side) => -side * s.z,
            Input::Elevator | Input::Pitch => s.x,
            Input::Rudder | Input::Yaw => s.y,
            Input::Roll => s.z,
            Input::Flaps => self.flaps as f32,
            Input::FlapLever => self.flap_set as f32 / self.spec.max_notch().max(1) as f32,
            Input::Slats => self.slats as f32,
            Input::Spoiler(side) => {
                // Roll spoilers rise on the wing going down, on top of the speedbrake.
                let roll = (side * s.z).max(0.0) * if self.spec.spoiler_cd > 0.0 { 0.55 } else { 0.0 };
                (self.spoilers as f32 + roll).min(1.0)
            }
            Input::Gear => self.gear as f32,
            Input::GearHandle => if self.gear_down { 1.0 } else { -1.0 },
            Input::Compress(k) => self.compress[k as usize] as f32,
            Input::Wheel(k) => self.wheel_rot[k as usize] as f32,
            Input::Steer => self.steer_angle() as f32,
            Input::Prop => (self.prop % std::f64::consts::TAU) as f32,
            Input::Throttle => self.throttle as f32,
            Input::Trim => self.trim as f32,
            Input::Master => self.sys.elec.master as u8 as f32,
            Input::Ignition => if self.starter_on || self.cranking { 1.5 } else if self.ignition { 1.0 } else { 0.0 },
            Input::FuelControl(k) => self.pair_on(0, k as usize) as u8 as f32,
            Input::Battery(k) => self.pair_on(1, k as usize) as u8 as f32,
            Input::Mixture => self.mixture as f32,
            Input::ParkingBrake => self.parking_brake as u8 as f32,
            Input::Brake(side) => self.toe_brake[if side < 0.0 { 0 } else { 1 }] as f32,
            Input::LandingLights => self.landing_lights as u8 as f32,
            Input::FuelValve => self.fuel_selector as u8 as f32,
            Input::PropControl => self.propeller as f32,
            Input::CowlFlaps => self.cowl_flaps as f32,
            Input::Reverser => self.reverser as f32,
            Input::Mach => self.mach() as f32,
            Input::Canopy => self.canopy as f32,
            Input::Speedbrake => self.speedbrake as f32,
            Input::N1 => (self.n1 * 100.0) as f32,
        }
    }

    /// Where every piece of the model is, relative to the body, this instant.
    pub fn pose(&self, pieces: &[Piece]) -> Vec<Mat4> {
        let gear = self.gear as f32;
        // The doors open first, the legs swing, then the doors close again behind them.
        let legs = smooth01(0.15, 0.85, gear);
        let comp: Vec<f32> = self.compress.iter().map(|c| *c as f32).collect();
        let t = self.t as f32;
        // Past the never-exceed speed the control surfaces buzz on their hinges.
        let fl = smooth01(5.0, 30.0, self.overspeed() as f32) * 0.12;
        let d = &self.dmg;
        let drive = |dr: &Drive| -> f32 {
            let mut a = dr.eval(self.input(dr.input));
            if dr.gain != 0.0 {
                match dr.input {
                    Input::Aileron(side) => {
                        let k = if side < 0.0 { 0 } else { 1 };
                        if d.aileron[k] > 0.0 && d.aileron[k] < 1.0 {
                            // Hanging from one hinge: drooped, flapping in the slipstream.
                            return 0.55 + 0.14 * (t * 27.0 + side).sin();
                        }
                        a += fl * (t * 41.0 + side).sin();
                    }
                    Input::Elevator => a += fl * 0.7 * (t * 37.0).sin(),
                    Input::Rudder => a += fl * (t * 43.0).sin(),
                    _ => {}
                }
            }
            a
        };
        let mut out: Vec<Mat4> = Vec::with_capacity(pieces.len());
        for p in pieces {
            let local = match p.anim {
                Anim::Fixed => Mat4::IDENTITY,
                Anim::Move(m) => {
                    let a = drive(&m.turn[0]) + drive(&m.turn[1]);
                    let mut mat = if a != 0.0 { Mat4::from_translation(p.pivot) * Mat4::from_axis_angle(p.axis, a) * Mat4::from_translation(-p.pivot) } else { Mat4::IDENTITY };
                    if m.slide.gain != 0.0 || m.slide.offset != 0.0 {
                        mat = Mat4::from_translation(m.dir * drive(&m.slide)) * mat;
                    }
                    mat
                }
                a => crate::aircraft::c172::kinematics(a, p, legs, self.steer_angle() as f32 * smooth01(0.9, 1.0, gear), &comp),
            };
            out.push(match p.parent {
                Some(i) => out[i] * local,
                None => local,
            });
        }
        out
    }

    // ---------------------------------------------------------------- breaking things

    fn note(&mut self, s: impl Into<String>) {
        self.notes.push(s.into());
    }
    fn scuff(&mut self, group: usize, amount: f32) {
        self.dmg.scuff[group] = (self.dmg.scuff[group] + amount).min(1.0);
    }
    /// A blow that sets it turning (rad/s for a light single; a heavy
    /// aircraft takes the same blow far more calmly).
    fn kick(&mut self, w: DVec3) {
        self.omega += w * (1100.0 / self.spec.mass).cbrt();
    }
    fn wing_group(side: usize) -> usize {
        if side == 0 { G_LWING } else { G_RWING }
    }
    fn lr(side: usize) -> &'static str {
        if side == 0 { "LEFT" } else { "RIGHT" }
    }
    /// A span station given as a fraction of the half span.
    fn sta(&self, f: f32) -> f32 {
        self.spec.span * f
    }
    /// How much bigger this aircraft is than the light single the break
    /// distances were first worked out for.
    fn scale(&self) -> f32 {
        self.spec.span / 5.5
    }

    /// The wing on `side` (0 left) breaks off outboard of span `station`.
    fn break_wing(&mut self, side: usize, station: f32, why: &str) {
        self.snap_wing(side, station, 0.5, 0.0, true, why);
    }
    /// The wing on `side` breaks near span `station`, and how is left to the
    /// break: `rough` 0 is a clean shear (an edge struck hard), 1 skin torn
    /// apart, slanting across the chord and through the thickness; `fold` is
    /// the chance it buckles and hangs on by its skin rather than letting go
    /// (bending `up` or down). Breaks further in leave the bits between as
    /// pieces of their own.
    fn snap_wing(&mut self, side: usize, station: f32, rough: f32, fold: f32, up: bool, why: &str) {
        // A wing already hanging lets go before it breaks again.
        if self.dmg.fold[side].on {
            self.release_fold(side, "");
        }
        let k = self.scale();
        let station = station.max(0.3 * k);
        if station >= self.dmg.wing[side] - 0.05 * k {
            return;
        }
        let spec = self.spec;
        let sgn = if side == 0 { -1.0 } else { 1.0 };
        let r = &mut self.rng;
        let tz = r.range(-1.0, 1.0) * (0.12 + 0.6 * rough) * k;
        let ty = r.range(-1.0, 1.0) * (0.04 + 0.22 * rough);
        let jag = (0.04 + 0.26 * rough) * r.range(0.6, 1.25) * k.sqrt();
        let folds = r.f() < fold;
        let hold = r.range(500.0, 2600.0) * self.fold_k();
        let prev = self.dmg.stump(side);
        let tear = damage::wing_break(spec, side, station, tz, ty, jag, &prev);
        let tip = self.dmg.wing[side];
        let lost = tip - station;
        let tank = station < spec.tank[1] && tip >= spec.tank[1];
        self.dmg.wing[side] = station;
        self.dmg.wing_tear[side] = tear;
        self.scuff(Self::wing_group(side), 0.5);
        // On the ground the blow itself does the shoving.
        let (sg, lo) = (sgn as f64 * if self.on_hull || self.on_ground { 0.3 } else { 1.0 }, (lost / k) as f64);
        if folds && lost > 0.7 * k {
            self.dmg.fold[side] = damage::fold_at(spec, side, station, tear, prev, tip, up, hold);
            // Still attached: less of a shove than losing it.
            self.kick(DVec3::new(0.15, -sg * 0.3 * lo, -sg * (0.4 + 0.4 * lo)));
            self.note(format!("{} WING BUCKLED ({why})", Self::lr(side)));
        } else {
            // The part that leaves: the far side of the break, with whatever flap
            // or aileron (or engine) was riding on it, as one piece.
            let mut req = ShedReq::new(Sel::GroupAll(Self::wing_group(side)), tear.flip(), Vec3::new(sgn * 2.5, 1.5, 2.0), 5.0 / k);
            req.tear2 = prev;
            req.own2 = true;
            req.rigid = true;
            self.shed.push(req);
            if station < spec.aileron[0] + 0.05 * spec.span {
                self.dmg.aileron[side] = 0.0;
            }
            // Shoved: it rolls toward the stump and yaws into the drag of the break.
            self.kick(DVec3::new(0.3, -sg * 0.6 * lo, -sg * (0.8 + 0.9 * lo)));
            self.note(format!("{} WING TORN OFF ({why})", Self::lr(side)));
        }
        if tank {
            self.note(format!("{} FUEL TANK RUPTURED", Self::lr(side)));
        }
    }
    /// How much heavier a hanging wing is to swing than the light single's
    /// (mass per metre times the square of the length).
    fn fold_k(&self) -> f32 {
        let k = self.scale();
        self.wing_kg_m() / 24.0 * k * k
    }
    /// Mass of the wing per metre of span (kg).
    fn wing_kg_m(&self) -> f32 {
        24.0 * (self.spec.groups[G_LWING].1 / 170.0) * (5.5 / self.spec.span)
    }
    /// A buckled wing lets go of the stump.
    fn release_fold(&mut self, side: usize, why: &str) {
        let f = self.dmg.fold[side];
        if !f.on {
            return;
        }
        self.dmg.fold[side].on = false;
        let cm = f.mat().transform_point3(damage::wing_pt(self.spec, side, (f.root + f.tip) * 0.5, 0.4, 0.0));
        let swing = f.axis.cross(cm - f.pivot) * f.rate;
        let mut req = ShedReq::new(Sel::GroupAll(Self::wing_group(side)), f.tear.flip(), swing + Vec3::new(0.0, 0.5, 1.5), (2.0 + f.rate.abs()) / self.scale());
        req.tear2 = f.outer;
        req.own2 = true;
        req.rigid = true;
        req.pre = f.mat();
        self.shed.push(req);
        if f.root < self.spec.aileron[0] + 0.05 * self.spec.span {
            self.dmg.aileron[side] = 0.0;
        }
        if !why.is_empty() {
            self.note(format!("{} WING TORN AWAY ({why})", Self::lr(side)));
        }
    }
    /// Let go of anything still hanging (the aircraft is about to go up in flames).
    fn release_folds(&mut self) {
        for s in 0..2 {
            self.release_fold(s, "");
        }
    }
    /// The tail breaks off through the tail cone, somewhere about `z`.
    fn snap_tail(&mut self, z: f32, why: &str) {
        if self.dmg.tail.is_some() {
            return;
        }
        let k = self.scale();
        let r = &mut self.rng;
        let tear = damage::tail_break(self.spec, z, r.range(-0.35, 0.35), r.range(-0.45, 0.45), r.range(0.06, 0.24) * k.sqrt());
        let mut req = ShedReq::new(Sel::Group(G_TAIL), tear.flip(), Vec3::new(r.range(-1.0, 1.0), 1.5, 3.0), 4.0 / k);
        req.rigid = true;
        self.shed.push(req);
        self.dmg.tail = tear;
        self.dmg.stab = [0.0; 2];
        self.dmg.elevator = [0.0; 2];
        self.dmg.fin = 0.0;
        self.dmg.rudder = 0.0;
        self.dmg.hull = (self.dmg.hull + 0.2).min(1.0);
        self.kick(DVec3::new(0.8, 0.0, 0.0));
        self.note(format!("TAIL BROKEN OFF ({why})"));
    }
    fn tail_z(&mut self) -> f32 {
        let [a, b] = self.spec.tail.break_z;
        self.rng.range(a, b)
    }
    /// Into the wall of a building, fast: start ploughing into it (see `Bury`).
    fn start_bury(&mut self, building: (geo::TileId, usize), world: &World, cause: &'static str) {
        if self.bury.is_some() {
            return;
        }
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        building.hash(&mut h);
        let jitter = (h.finish() % 10_000) as f64 / 10_000.0;
        // The building's mass (as in main.rs: ~120 kg/m³ of built volume,
        // a third of it structure) against the aircraft's.
        let mb = world.footprint(building.0, building.1).map(|(base, top, area)| area.max(25.0) as f64 * (top - base).max(3.0) * 120.0 * 0.3).unwrap_or(2.0e6);
        let heavy = mb / (mb + self.spec.mass); // 0 light .. 1 massive
        // From a third of its length in (massive) to three quarters (light), ± a bit per building.
        let frac = (0.75 - 0.4 * heavy + (jitter - 0.5) * 0.25).clamp(0.25, 0.85);
        // Fuselage length from its contact points (nose to tail cone).
        let zs = self.spec.contacts.iter().map(|c| c.0[2]).chain(self.spec.tail_pts.iter().map(|p| p[2]));
        let (lo, hi) = zs.fold((0.0f64, 0.0f64), |(a, b), z| (a.min(z), b.max(z)));
        let depth = ((hi - lo).max(4.0) * frac).max(1.5);
        // Speed kept by the time it goes up: 85% (light) .. 55% (massive).
        let keep = 0.85 - 0.3 * heavy;
        self.bury = Some(Bury { building, gone: 0.0, depth, keep_per_m: keep.powf(1.0 / depth), cause });
    }
    /// Something hits hard enough to explode: anything still hanging off goes too.
    fn impact(&mut self, imp: Impact, cause: &'static str) -> Option<Impact> {
        self.cause = cause;
        self.release_folds();
        Some(imp)
    }
    /// A hard hit at body point `r` (contact velocity `pv`, ground normal
    /// `upw`, closing at `closing` m/s) leaves a lasting dent there.
    ///
    /// Sizes follow the square-cube law: geometrically similar airframes crush
    /// at a force that grows with area (L²) while their mass grows with L³, so
    /// the deceleration the structure can hold falls as 1/L and the crush depth
    /// for a given closing speed grows with L: depth = v² L/L₀ / (2 a₀), with
    /// a₀ = 30 g for a light aircraft (L₀ = an 11 m span). That gives about
    /// 8 cm at 7 m/s and 25 cm at 12 m/s on a Cessna 172; a 747 about 0.4 m at
    /// 6 m/s. The bowl spans a fuselage-sized patch, 0.4 to 0.9 of a light
    /// aircraft's width scaled the same way, and is never deeper than a
    /// quarter of its radius. (Estimates for the look: no crash test data.)
    /// It is pushed in along the contact force, the ground's normal leaning
    /// back against the slide as friction does.
    fn dent(&mut self, r: DVec3, pv: DVec3, upw: DVec3, closing: f64) {
        let ls = (self.scale() as f64).max(0.3);
        let depth = closing * closing * ls / (2.0 * 30.0 * 9.81);
        let radius = ((0.35 + 0.04 * closing) * ls).min(0.9 * ls);
        let slide = pv - upw * pv.dot(upw);
        let force = upw - slide.normalize_or_zero() * 0.4;
        self.record_crumple(r, self.rot.inverse() * force, radius as f32, depth.min(radius * 0.25) as f32);
    }
    /// Add plastic damage to the strongest current dent. Repeated hits near
    /// it deepen the same crush zone; a separate weak tap cannot erase it.
    fn record_crumple(&mut self, center: DVec3, direction: DVec3, radius: f32, depth: f32) {
        let old_radius = self.dmg.crumple[3];
        let old_depth = self.dmg.crumple_dir[3];
        let old_center = DVec3::new(self.dmg.crumple[0] as f64, self.dmg.crumple[1] as f64, self.dmg.crumple[2] as f64);
        let dir = direction.normalize_or_zero();
        if old_radius > 0.0 && (center - old_center).length() < (old_radius + radius) as f64 * 0.5 {
            let combined = (old_depth + depth).min(radius.max(old_radius) * 0.25);
            let center = (old_center * old_depth.max(0.01) as f64 + center * depth as f64) / (old_depth.max(0.01) + depth) as f64;
            let old_dir = DVec3::new(self.dmg.crumple_dir[0] as f64, self.dmg.crumple_dir[1] as f64, self.dmg.crumple_dir[2] as f64);
            let dir = (old_dir * old_depth as f64 + dir * depth as f64).normalize_or_zero();
            self.dmg.crumple = [center.x as f32, center.y as f32, center.z as f32, old_radius.max(radius)];
            self.dmg.crumple_dir = [dir.x as f32, dir.y as f32, dir.z as f32, combined];
        } else if depth >= old_depth {
            self.dmg.crumple = [center.x as f32, center.y as f32, center.z as f32, radius];
            self.dmg.crumple_dir = [dir.x as f32, dir.y as f32, dir.z as f32, depth.min(radius * 0.25)];
        }
    }
    /// Displace a collision point with the same permanent dent shown on the
    /// model. This lets a crushed panel recede instead of colliding as intact.
    fn crumple_offset(&self, point: DVec3) -> DVec3 {
        let [x, y, z, radius] = self.dmg.crumple;
        let depth = self.dmg.crumple_dir[3];
        if radius <= 0.001 || depth <= 0.001 { return DVec3::ZERO; }
        let delta = point - DVec3::new(x as f64, y as f64, z as f64);
        let t = (1.0 - delta.length() / radius as f64).clamp(0.0, 1.0);
        let falloff = t * t * (3.0 - 2.0 * t);
        DVec3::new(self.dmg.crumple_dir[0] as f64, self.dmg.crumple_dir[1] as f64, self.dmg.crumple_dir[2] as f64) * (depth as f64 * falloff)
    }
    /// Wing bending, for the look only (the flight model's wing stays rigid):
    /// how far each tip deflects from the modelled shape, which is taken as
    /// the wing at 1 g. In the air the tips follow the load factor, a mass on
    /// a spring at the wing's first bending frequency.
    ///
    /// Deflection per g is estimated from the half-span s as 6e-5 s³ m: about
    /// 2 m per g for the 747-400 (s = 32 m; Boeing's 777 test wing rose some
    /// 7.4 m at 1.5x its 2.5 g limit, about 2 m per g, and the 747's is
    /// similar), a centimetre for the strut-braced 172, 1.4 cm for the stiff
    /// Hornet (9 cm at 7.5 g), half that for the DR400. The bending frequency
    /// goes as 1/s: about 1 Hz for the 747, 6 Hz for the 172. A rolling
    /// wing is loaded unevenly: the rising tip meets the air p s / V lower in
    /// incidence and the falling one higher, adding a lift curve slope of
    /// about 4.5 per radian times q over the wing loading, in g, at the tip
    /// (half of that, for the bending, from its linear spread along the span).
    /// On the ground, skidding or past a wing's loss the tips settle back to
    /// the modelled shape; the deflection never exceeds a fifth of the
    /// half-span (beyond ultimate load, where the wing has broken anyway).
    /// The wing bending for drawing, left then right: tip deflection over the
    /// half-span squared, so a point x out along the span rises by bend x²
    /// (plane.wgsl `deform`, main.rs `own_lights`).
    pub fn wing_bend(&self) -> [f32; 2] {
        // (`Spec::span` is the tip's station: the half-span)
        let s = (self.spec.span as f64).max(0.5);
        self.wing_flex.map(|f| (f / (s * s)) as f32)
    }
    fn flex_wings(&mut self, dt: f64, skidding: bool) {
        let s = (self.spec.span as f64).max(0.5);
        let per_g = 6e-5 * s * s * s;
        let w = std::f64::consts::TAU * (32.0 / s).clamp(0.8, 8.0);
        let zeta = 0.25;
        let flying = !self.on_ground && !skidding;
        let n = if flying { self.g_load.clamp(-6.0, 12.0) } else { 1.0 };
        // Rolling about +Z (aft) lifts the right wing (+X): the right tip's
        // incidence drops by p s / V, the left one's rises.
        let (_, _, h) = geo::pos_to_lla(self.pos);
        let v = self.airspeed().length().max(20.0);
        let q = 0.5 * density(h) * v * v;
        let loading = self.mass() * 9.81 / self.spec.wing_area.max(1.0);
        let tip_alpha = (self.omega.z * s / v).clamp(-0.2, 0.2);
        let roll_n = (0.5 * 4.5 * tip_alpha * q / loading).clamp(-3.0, 3.0);
        let cap = 0.2 * s;
        for side in 0..2 {
            let broken = (self.dmg.wing[side] as f64) < self.spec.span as f64 - 0.01;
            let sign = if side == 0 { 1.0 } else { -1.0 };
            let target = if broken || !flying { 0.0 } else { ((n - 1.0 + sign * roll_n) * per_g).clamp(-cap, cap) };
            // implicit (backward Euler) spring-damper: stable at any step
            let x = self.wing_flex[side];
            let vel = (self.wing_flex_v[side] + dt * w * w * (target - x)) / (1.0 + 2.0 * zeta * w * dt + w * w * dt * dt);
            self.wing_flex_v[side] = vel;
            self.wing_flex[side] = (x + vel * dt).clamp(-cap, cap);
        }
    }
    fn lose_aileron(&mut self, side: usize, hanging: bool, why: &str) {
        if self.dmg.aileron[side] <= 0.0 || (hanging && self.dmg.aileron[side] < 1.0) {
            return;
        }
        if hanging {
            self.dmg.aileron[side] = 0.5;
            self.note(format!("{} AILERON HINGE BROKEN ({why})", Self::lr(side)));
            return;
        }
        self.dmg.aileron[side] = 0.0;
        self.shed.push(ShedReq::new(Sel::Role(Role::Aileron(side as u8)), Tear::NONE, Vec3::new(0.0, 1.0, 3.0), 9.0 / self.scale()));
        self.note(format!("{} AILERON TORN OFF ({why})", Self::lr(side)));
    }
    fn lose_flap(&mut self, side: usize, why: &str) {
        if self.dmg.flap[side] <= 0.0 {
            return;
        }
        self.dmg.flap[side] = 0.0;
        self.shed.push(ShedReq::new(Sel::Role(Role::Flap(side as u8)), Tear::NONE, Vec3::new(0.0, -1.0, 4.0), 8.0 / self.scale()));
        self.kick(DVec3::new(0.0, 0.0, if side == 0 { -0.4 } else { 0.4 }));
        match &self.spec.lever {
            Some(l) => self.note(format!("{} TORN OFF ({why})", l.parts[side])),
            None => self.note(format!("{} FLAP TORN OFF ({why})", Self::lr(side))),
        }
    }
    fn lose_elevator(&mut self, side: usize, with_stab: bool, why: &str) {
        let tl = &self.spec.tail;
        let sgn = if side == 0 { -1.0 } else { 1.0 };
        if with_stab && self.dmg.stab[side] > 0.5 {
            self.dmg.stab[side] = 0.0;
            self.dmg.elevator[side] = 0.0;
            self.shed.push(ShedReq::new(Sel::Roles(&[Role::Stab, Role::Elevator]), Tear::new(Vec3::new(sgn, 0.0, 0.0), tl.stab_root, -1.0), Vec3::new(sgn * 2.0, 0.5, 2.0), 6.0 / self.scale()));
            self.kick(DVec3::new(-0.5, 0.0, sgn as f64 * 0.3));
            self.note(format!("{} STABILISER TORN OFF ({why})", Self::lr(side)));
        } else if self.dmg.elevator[side] > 0.5 {
            self.dmg.elevator[side] = 0.0;
            self.shed.push(ShedReq::new(Sel::Role(Role::Elevator), Tear::new(Vec3::new(sgn, 0.0, 0.0), tl.elev_root, -1.0), Vec3::new(sgn, 1.0, 3.0), 9.0 / self.scale()));
            self.note(format!("{} ELEVATOR TORN OFF ({why})", Self::lr(side)));
        }
    }
    /// The top of the rudder (and perhaps the fin with it) goes.
    fn lose_tail_top(&mut self, rudder: f32, fin: f32, why: &str) {
        let tl = &self.spec.tail;
        let rudder = rudder.min(self.dmg.rudder);
        let fin = fin.min(self.dmg.fin);
        let spin = 1.0 / self.scale();
        if rudder < self.dmg.rudder - 0.01 {
            if rudder <= 0.0 {
                self.shed.push(ShedReq::new(Sel::Role(Role::Rudder), Tear::NONE, Vec3::new(1.0, 1.0, 3.0), 7.0 * spin));
                self.note(format!("RUDDER TORN OFF ({why})"));
            } else {
                self.shed.push(ShedReq::new(Sel::Role(Role::Rudder), Tear::new(Vec3::Y, tl.rudder_y0 + tl.rudder_h * rudder, -1.0), Vec3::new(1.0, 1.5, 3.0), 8.0 * spin));
                self.note(format!("RUDDER DAMAGED ({why})"));
            }
            self.dmg.rudder = rudder;
        }
        if fin < self.dmg.fin - 0.01 {
            self.shed.push(ShedReq::new(Sel::Role(Role::Stab), Tear::new(Vec3::Y, tl.fin_y0 + tl.fin_h * fin, -1.0), Vec3::new(1.0, 2.0, 2.0), 6.0 * spin));
            self.dmg.fin = fin;
            self.note(format!("FIN BROKEN ({why})"));
        }
    }
    /// The propeller hits something solid (a jet: an engine pod scrapes the ground).
    fn prop_strike(&mut self, why: &str) {
        if let Engine::Piston(pe) = &self.spec.engine {
            if self.dmg.prop <= 0.0 {
                return;
            }
            let spinning = self.prop_rate > 40.0;
            if self.dmg.prop > 0.75 {
                self.dmg.prop = 0.5;
                self.shed.push(ShedReq::new(Sel::Role(Role::Prop), Tear::new(Vec3::Y, pe.blade_break_y, -1.0), Vec3::new(2.0, 4.0, -3.0), 30.0));
                self.note(format!("PROPELLER BLADE BROKEN ({why})"));
            }
            if spinning && self.dmg.engine > 0.0 {
                // A sudden stop wrecks the crankshaft.
                self.dmg.engine = 0.0;
                self.dmg.smoke_black = true;
                self.note("ENGINE SEIZED");
            }
            self.scuff(G_ENGINE, 0.6);
        }
    }
    /// An engine pod hanging under a wing scrapes or strikes the ground.
    fn pod_strike(&mut self, side: usize, hard: f32, why: &str) {
        if self.cool[side] > 0.0 {
            return;
        }
        self.cool[side] = 0.4;
        self.scuff(Self::wing_group(side), 0.5);
        self.engine_hit(0.12 + 0.2 * hard, why);
        self.dmg.smoke_black = true;
        self.note(format!("{} ENGINE POD STRUCK ({why})", Self::lr(side)));
    }
    fn engine_hit(&mut self, amount: f32, why: &str) {
        if self.dmg.engine <= 0.0 {
            return;
        }
        let was = self.dmg.engine;
        self.dmg.engine = (self.dmg.engine - amount).max(0.0);
        if was >= 1.0 {
            self.note(if self.spec.is_jet() { format!("ENGINE DAMAGED ({why})") } else { format!("ENGINE DAMAGED, LOSING OIL ({why})") });
        }
    }
    fn crack_windshield(&mut self, why: &str) {
        if self.dmg.crack[2] > 0.0 {
            return;
        }
        self.dmg.crack = [self.rng.range(-1.55, -1.25), self.rng.range(0.3, 0.6), self.rng.range(0.14, 0.3)];
        self.note(format!("WINDSHIELD CRACKED ({why})"));
    }
    fn lose_gear(&mut self, gi: usize, why: &str) {
        if !self.dmg.gear[gi] {
            return;
        }
        self.dmg.gear[gi] = false;
        self.shed.push(ShedReq::new(Sel::Role(Role::Gear(gi as u8)), Tear::NONE, Vec3::new(0.0, -1.0, 2.0), 6.0 / self.scale()));
        self.note(format!("{} TORN OFF ({why})", self.spec.gear[gi].name));
    }
    /// Above the never-exceed speed something lets go: whatever is next.
    fn flutter_loss(&mut self) {
        let r = self.rng.f();
        let s = (r * 2.0) as usize;
        if self.dmg.aileron[s] > 0.0 {
            self.lose_aileron(s, r > 0.6, "FLUTTER");
        } else if self.dmg.aileron[1 - s] > 0.0 {
            self.lose_aileron(1 - s, false, "FLUTTER");
        } else if self.dmg.rudder > 0.5 {
            self.lose_tail_top(0.45, 1.0, "FLUTTER");
        } else if self.dmg.elevator[s] > 0.5 {
            self.lose_elevator(s, false, "FLUTTER");
        } else if self.dmg.elevator[1 - s] > 0.5 {
            self.lose_elevator(1 - s, false, "FLUTTER");
        } else {
            self.break_wing(s, self.sta(0.78), "FLUTTER");
        }
    }

    /// Velocity through the air (planet frame).
    pub fn airspeed(&self) -> DVec3 {
        self.vel - self.wind - self.gust
    }

    // ---------------------------------------------------------------- engines

    /// The torque the engine's cylinders make (N m, before friction) with the
    /// throttle `open` (0 at the idle stop .. 1). A normally aspirated engine
    /// loses power with the air (Gagg-Ferrar); `boost` is the game's
    /// over-boost, nothing a real 172 has.
    fn engine_torque(pe: &crate::aircraft::Piston, open: f64, rho: f64, boost: bool, health: f64) -> f64 {
        let sigma = rho / RHO0;
        let alt = ((sigma - 0.12) / 0.88).max(0.0);
        (pe.power / pe.w_rated + Self::friction(pe, pe.w_rated)) * alt * (0.11 + 0.89 * open.clamp(0.0, 1.0)) * health * if boost { 1.35 } else { 1.0 }
    }
    /// Friction and pumping losses (N m) at shaft speed `w` (rad/s).
    fn friction(pe: &crate::aircraft::Piston, w: f64) -> f64 {
        pe.fric0 + pe.fric1 * (w / pe.w_rated).powi(2)
    }
    /// The fixed-pitch propeller turning at `w` rad/s with the air coming at
    /// it at `v`: its thrust (N) and the torque it takes (N m; negative when
    /// the air drives it round). `k` is what is left of the blades.
    /// Ct = Ct0 (1 - (J/Jt)²), Cp = Cp0 (1 - (J/Jp)²), written in n² so a
    /// stopped propeller needs no special case.
    pub fn prop_load(pe: &crate::aircraft::Piston, w: f64, v: f64, rho: f64, k: f64) -> (f64, f64) {
        let n = w / std::f64::consts::TAU;
        let vd = v.max(0.0) / pe.prop_d;
        let t = pe.ct0 * rho * pe.prop_d.powi(4) * (n * n - (vd / pe.j_t).powi(2));
        let q = pe.cp0 * rho * pe.prop_d.powi(5) / std::f64::consts::TAU * (n * n - (vd / pe.j_p).powi(2));
        // Windmilling, or stopped, it drags no worse than a flat plate the size of its blades.
        let t = t.max(-0.28 * 0.5 * rho * v * v);
        (t * k, q * k)
    }
    /// The revs (rad/s) an undamaged engine and propeller settle at.
    pub fn prop_equilibrium(pe: &crate::aircraft::Piston, throttle: f64, v: f64, rho: f64) -> f64 {
        let (mut lo, mut hi) = (0.0, 2.0 * pe.w_rated);
        for _ in 0..50 {
            let w = 0.5 * (lo + hi);
            if Self::engine_torque(pe, throttle, rho, false, 1.0) - Self::friction(pe, w) > Self::prop_load(pe, w, v, rho, 1.0).1 { lo = w } else { hi = w }
        }
        0.5 * (lo + hi)
    }
    /// The fan speed (fraction of N1) the throttle asks for.
    fn n1_target(fe: &crate::aircraft::Fan, throttle: f64) -> f64 {
        fe.idle + (1.0 - fe.idle) * throttle.clamp(0.0, 1.0)
    }
    /// Thrust of all the fans together (N) at fan speed `n1`, true airspeed
    /// `v`, height `h`: a fraction of the static thrust that grows steeply
    /// with the fan speed, lapsing with the density and with the Mach number
    /// (the faster it flies, the less the fan adds to the air it swallows).
    pub fn fan_thrust(fe: &crate::aircraft::Fan, n1: f64, v: f64, h: f64) -> f64 {
        let x = ((n1 - fe.idle) / (1.0 - fe.idle)).max(0.0);
        let frac = if n1 >= fe.idle { fe.idle_thrust + (1.0 - fe.idle_thrust) * x.powf(1.7) } else { fe.idle_thrust * (n1 / fe.idle).powi(2) };
        let m = v / sound_speed(h);
        let sigma = density(h) / RHO0;
        fe.n as f64 * fe.thrust * frac * sigma.powf(0.8) * (1.0 - fe.mach_lapse[0] * m + fe.mach_lapse[1] * m * m).max(0.05)
    }
    /// What a turbo-ramjet's inlets pass of its thrust at this Mach number:
    /// all of it up to the speed they are built for, under half of it a tenth
    /// of a Mach past that (see `Spec::inlet_mach`; 1 for any other engine).
    fn inlet(&self, mach: f64) -> f64 {
        let im = self.spec.inlet_mach;
        if im > 0.0 { 1.0 - 0.65 * smooth(im - 0.05, im + 0.15, mach) } else { 1.0 }
    }
    /// What is left of the thrust without the afterburners at this Mach
    /// number, for an engine built to cruise in them (see `Spec::dry_mach`; 1
    /// for any other engine): the ram air that swells its afterburning thrust
    /// does nothing for its core, which makes a little less the faster it
    /// goes and next to nothing by `dry_mach`.
    fn dry_fade(&self, fe: &crate::aircraft::Fan, mach: f64) -> f64 {
        let dm = self.spec.dry_mach;
        if dm > 0.0 {
            (1.0 - 0.18 * mach.min(1.0)) * (1.0 - 0.97 * smooth(0.4 * dm, dm, mach)) / (1.0 - fe.mach_lapse[0] * mach + fe.mach_lapse[1] * mach * mach).max(0.05)
        } else {
            1.0
        }
    }
    /// Thrust (N) once the engine has settled at this throttle.
    pub fn thrust(&self, throttle: f64, v: f64, h: f64) -> f64 {
        match &self.spec.engine {
            Engine::Piston(pe) => {
                let rho = density(h);
                Self::prop_load(pe, Self::prop_equilibrium(pe, throttle, v, rho), v, rho, 1.0).0
            }
            Engine::Fan(fe) => Self::fan_thrust(fe, Self::n1_target(fe, throttle), v, h) * self.inlet(v / sound_speed(h)) * self.dry_fade(fe, v / sound_speed(h)),
            Engine::Rocket(_) => 0.0,
        }
    }
    /// The engine already running as the throttle has it.
    fn settle_engine(&mut self, v: f64, h: f64) {
        match &self.spec.engine {
            Engine::Piston(pe) => self.prop_rate = Self::prop_equilibrium(pe, self.throttle, v, density(h)),
            Engine::Fan(fe) => {
                self.n1 = Self::n1_target(fe, self.throttle);
                self.prop_rate = self.n1 * fe.n1_rpm * std::f64::consts::TAU / 60.0;
            }
            Engine::Rocket(_) => {}
        }
    }

    // ---------------------------------------------------------------- trim

    /// Drag coefficient of the shock waves near the speed of sound (Lock's fourth-power rise).
    fn wave_cd(&self, mach: f64) -> f64 {
        if self.spec.cd_wave > 0.0 {
            self.spec.cd_wave * smooth(self.spec.mcrit, self.spec.mcrit + 0.25, mach)
        } else if self.spec.mcrit > 0.0 && mach > self.spec.mcrit {
            20.0 * (mach - self.spec.mcrit).powi(4)
        } else {
            0.0
        }
    }
    /// the 1 g stall speed (m/s indicated) at this mass with flaps at `flaps`.
    /// for the hud's speed bands, the model itself stalls by angle of attack.
    pub fn stall_ias_at(&self, flaps: f64) -> f64 {
        let s = self.spec;
        let notch = (0..s.flaps.len() as u8).find(|&k| s.flap_frac(k) >= flaps - 0.01).unwrap_or(0);
        let slats = if s.slats_at > 0 && notch >= s.slats_at { 1.0 } else { 0.0 };
        let alpha = s.stall_alpha - s.stall_flap * flaps + s.stall_slat * slats;
        let cl_max = (s.cl0 + s.flap_cl * flaps + s.cl_a * alpha).max(0.1);
        (2.0 * self.mass() * 9.81 / (RHO0 * s.wing_area * cl_max)).sqrt()
    }
    /// Angle of attack (rad) that carries the weight at true airspeed `v`.
    fn alpha_level(&self, v: f64, rho: f64, flap_cl: f64) -> f64 {
        let s = self.spec;
        let q = 0.5 * rho * v * v;
        ((self.mass() * 9.81 / (q * s.wing_area).max(1.0) - s.cl0 - flap_cl) / s.cl_a).clamp(-0.05, s.alpha_trim)
    }
    /// Drag (N) in steady flight at true airspeed `v`, height `h`.
    fn drag_level(&self, v: f64, h: f64, flaps: f64, gear: f64) -> f64 {
        let s = self.spec;
        let qs = 0.5 * density(h) * v * v * s.wing_area;
        let cl = self.mass() * 9.81 / qs.max(1.0);
        qs * (s.cd0 + s.k_ind * cl * cl + s.cd_flap[0] * flaps + s.cd_flap[1] * flaps * flaps + s.cd_gear * gear + self.wave_cd(v / sound_speed(h)))
    }
    /// Throttle to hold `v` while coming down at `descent` m/s.
    fn throttle_for(&self, v: f64, h: f64, flaps: f64, gear: f64, descent: f64) -> f64 {
        let need = self.drag_level(v, h, flaps, gear) - self.mass() * 9.81 * (descent / v.max(1.0));
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..30 {
            let t = 0.5 * (lo + hi);
            if self.thrust(t, v, h) < need { lo = t } else { hi = t }
        }
        0.5 * (lo + hi)
    }

    /// Gaussian noise, unit variance (sum of three uniforms: close enough, and cheap).
    fn gauss(&mut self) -> f64 {
        (self.rng.range(-1.0, 1.0) + self.rng.range(-1.0, 1.0) + self.rng.range(-1.0, 1.0)) as f64
    }

    /// Turbulence: white noise through two first-order lags in series (a
    /// smoothed Dryden spectrum), the length scales turned into time by the
    /// airspeed. `a` is the coarse state, `b` the smoothed one; the second lag
    /// takes a fifth of the variance, which the output scale gives back.
    fn turbulence(&mut self, dt: f64, v: f64, upw: DVec3) {
        let air = self.air;
        let vt = v.max(12.0);
        // Bursts: a slow random level (patches a kilometre or two long) scales
        // the eddies up and down, lognormal with the mean variance unchanged:
        // quiet stretches, then a run of hard jolts. Rougher air is patchier.
        let eb = (-vt * dt / 1200.0).exp();
        let nb = self.gauss();
        self.burst = self.burst * eb + (1.0 - eb * eb).sqrt() * nb;
        let a = 0.3 + 0.2 * smooth(1.0, 5.0, air.sigma_w);
        let m = (a * self.burst - a * a).exp().min(3.0);
        // Two scales of eddy: the big ones (hundreds of metres: slow heaves
        // the aircraft rides up and down) and small ones a few tens of metres
        // across that it cannot follow (sharp jolts). The small ones carry a
        // third of the variance and come and go with the bursts the most.
        let len = DVec3::new(air.len_h, air.len_h, air.len_w);
        let sig = DVec3::new(air.sigma_h, air.sigma_h, air.sigma_w);
        let one = DVec3::ONE;
        let n = DVec3::new(self.gauss(), self.gauss(), self.gauss());
        let e1 = (-vt * dt / len).exp();
        let e2 = (-vt * dt * 4.0 / len).exp();
        let big = sig * (0.8 * (0.4 + 0.6 * m));
        self.gust_a = self.gust_a * e1 + big * (one - e1 * e1).max(DVec3::ZERO).sqrt() * n;
        self.gust_b = self.gust_b * e2 + self.gust_a * (one - e2);
        let ls = (len / 10.0).clamp(DVec3::splat(6.0), DVec3::splat(45.0));
        let es = (-vt * dt / ls).exp();
        let n = DVec3::new(self.gauss(), self.gauss(), self.gauss());
        self.gust_s = self.gust_s * es + sig * (0.6 * m) * (one - es * es).max(DVec3::ZERO).sqrt() * n;
        // (A big airframe spans many of the small ones and averages them out.)
        let span = (ls / (ls + DVec3::splat(0.5 * self.spec.b))).powf(0.5);
        let g = self.gust_b * 1.118 + self.gust_s * span;
        // Along and across the flight path, and up.
        let air_v = self.vel - self.wind;
        let mut along = air_v - upw * air_v.dot(upw);
        if along.length_squared() < 1.0 {
            along = self.forward() - upw * self.forward().dot(upw);
        }
        let along = along.normalize_or_zero();
        let across = along.cross(upw);
        self.gust = along * g.x + across * g.y + upw * g.z;
        // Rotational gusts: the vertical gust differs from one wing tip to the
        // other (roll), from the nose to the tail (pitch), and the side gust
        // from the nose to the fin (yaw). Eddies about the size of the airframe;
        // a big aircraft averages more of them out.
        let lr = self.spec.b * 2.5;
        let big = (11.0 / self.spec.b).sqrt();
        // (In bursts, roll upsets too.)
        let sr = DVec3::new(0.018 * air.sigma_w, 0.020 * air.sigma_h, 0.03 * air.sigma_w) * big * m;
        let n = DVec3::new(self.gauss(), self.gauss(), self.gauss());
        let e1 = (-vt * dt / lr).exp();
        let e2 = (-vt * dt * 4.0 / lr).exp();
        self.rot_a = self.rot_a * e1 + sr * (1.0 - e1 * e1).max(0.0).sqrt() * n;
        self.rot_b = self.rot_b * e2 + self.rot_a * (1.0 - e2);
        self.gust_rot = self.rot_b * 1.118 * smooth(5.0, 25.0, v);
    }

    /// Points on the airframe that meet the ground (body frame), as it is now:
    /// what has broken off has no points left, a hanging wing has its own.
    fn contacts(&self, out: &mut Vec<(DVec3, Part)>) {
        let s = self.spec;
        let d = &self.dmg;
        let v = DVec3::new;
        if d.prop > 0.0 && !s.is_jet() {
            out.push((s.hard[0], Part::Prop));
            out.push((s.hard[6], Part::Prop));
        }
        for &(p, part) in s.contacts {
            // An engine pod goes with the wing it hangs from.
            if let Part::Pod(side) = part {
                if p[0].abs() as f32 > d.wing[side] - 0.3 || d.fold[side].on {
                    continue;
                }
            }
            out.push((v(p[0], p[1], p[2]), part));
        }
        let tl = &s.tail;
        if d.tail.is_some() {
            // The stump of the tail cone, a little ahead of the break.
            let z = (d.tail.d / d.tail.n.z.max(0.5)) as f64 - 0.15 * self.scale() as f64;
            let (top, _, bot) = (s.fuse)(z as f32);
            out.push((v(0.0, bot as f64, z), Part::Belly));
            out.push((v(0.0, top as f64, z), Part::Roof));
        } else {
            out.extend(s.tail_pts.iter().map(|p| (v(p[0], p[1], p[2]), Part::Tail)));
            if d.fin > 0.05 {
                out.push((tl.fin_base.lerp(tl.fin_tip, d.fin as f64), Part::Fin));
                out.push((tl.fin_base.lerp(tl.fin_tip, 0.52 * d.fin as f64), Part::Fin));
            }
            for side in 0..2 {
                if d.stab[side] > 0.5 {
                    out.push((tl.stab_tip * DVec3::new(if side == 0 { -1.0 } else { 1.0 }, 1.0, 1.0), Part::Stab(side)));
                }
            }
        }
        for side in 0..2 {
            let b = d.wing[side];
            let k = self.scale();
            let mut at = |st: f32, tip: bool| {
                for (xc, skin) in [(0.35, 1.0), (0.35, -1.0), (0.0, 0.0)] {
                    out.push((damage::wing_pt(s, side, st, xc, skin).as_dvec3(), Part::Wing(side, st)));
                }
                if tip {
                    out.push((damage::wing_pt(s, side, st, 0.92, 0.0).as_dvec3(), Part::Wing(side, st)));
                }
            };
            for f in s.wing_st {
                let st = f * s.span;
                if st < b - 0.3 * k {
                    at(st, false);
                }
            }
            if b > 0.4 * k {
                at(b - 0.08 * k, true);
            }
            let f = &d.fold[side];
            if f.on {
                let m = f.mat();
                for (st, skin) in [(f.tip - 0.08 * k, 1.0), (f.tip - 0.08 * k, -1.0), ((f.root + f.tip) * 0.5, -1.0), ((f.root + f.tip) * 0.5, 1.0)] {
                    out.push((m.transform_point3(damage::wing_pt(s, side, st, 0.4, skin)).as_dvec3(), Part::Fold(side)));
                }
            }
        }
    }

    /// Wrecked: it will not fly again (a wing or the tail gone, the cabin
    /// caved in, or lying on its back).
    pub fn wrecked(&self) -> bool {
        let d = &self.dmg;
        if self.spec.rocket().is_some() {
            // (A dropped booster is no damage; it only fails by blowing up.)
            return d.hull > 0.6;
        }
        let short = self.sta(0.58);
        d.wing[0] < short || d.wing[1] < short || d.fold.iter().any(|f| f.on) || d.tail.is_some() || d.hull > 0.6 || self.sink > 0.3 || (self.on_hull && self.up().dot(self.pos.normalize()) < 0.2)
    }

    /// Advance one physics step. Returns an impact if the aircraft was destroyed.
    pub fn step(&mut self, dt: f64, c: &Controls, world: &World) -> Option<Impact> {
        let spec = self.spec;
        let hd = &spec.handling;
        let mass = self.mass();
        self.t += dt;
        if self.hold {
            self.vel = self.wind * 0.0;
            self.omega = DVec3::ZERO;
            if !spec.is_jet() {
                // (idling while the ground loads, unless it was left cold)
                let idle = if self.running { 65.0 } else { 0.0 };
                self.prop_rate += (idle - self.prop_rate) * (dt * 2.0).min(1.0);
            }
            self.prop += dt * self.prop_rate;
            return None;
        }
        if let Engine::Rocket(rk) = &spec.engine {
            return self.step_rocket(dt, c, world, rk);
        }
        self.dmg_prev = self.dmg;
        self.dmg.vibe = 0.0;
        for c in self.cool.iter_mut() {
            *c -= dt;
        }
        self.fold_tq = [0.0; 2];
        self.throttle = match c.throttle_set {
            Some(t) => t.clamp(0.0, 1.0),
            None => (self.throttle + c.throttle * dt * 0.6).clamp(0.0, 1.0),
        };
        // Control surfaces do not snap.
        let target = DVec3::new(c.pitch, c.yaw, c.roll);
        self.stick += (target - self.stick) * (dt * 7.0).min(1.0);
        // Flaps run on an electric motor (or hydraulics), the gear on a pump;
        // the squat switch keeps the gear down while there is weight on the wheels.
        let actuator_power = spec.id != 0 || self.sys.powered();
        let flap_target = spec.flap_frac(self.flap_set);
        if actuator_power {
            self.flaps += (flap_target - self.flaps).clamp(-spec.flap_rate * dt, spec.flap_rate * dt);
        }
        self.toe_brake = [self.brake_pressure(c, -1.0), self.brake_pressure(c, 1.0)];
        if self.running { self.engine_hours += dt / 3600.0; }
        let slat_target = if spec.slats_at > 0 && self.flap_set >= spec.slats_at { 1.0 } else { 0.0 };
        self.slats += (slat_target - self.slats).clamp(-dt * 0.12, dt * 0.12);
        if !self.gear_down && self.on_ground {
            self.gear_down = true;
        }
        if spec.gear_rate <= 0.0 {
            // fixed gear (a downloaded aircraft's): always down
            (self.gear, self.gear_down) = (1.0, true);
        }
        if actuator_power {
            self.gear = (self.gear + if self.gear_down { spec.gear_rate * dt } else { -spec.gear_rate * dt }).clamp(0.0, 1.0);
        }
        // A canopy slides open or shut in about two seconds.
        self.canopy = (self.canopy + if self.canopy_open { 0.5 * dt } else { -0.5 * dt }).clamp(0.0, 1.0);

        let (lat, lon, h) = geo::pos_to_lla(self.pos);
        let upw = self.pos.normalize();
        let rho = density(h);
        // Limits are set in indicated airspeed (the dynamic pressure the airframe feels).
        let eas_k = (rho / RHO0).sqrt();
        let ground = world.ground(lat, lon);
        // The turbulent air the aircraft is flying through this instant.
        let v_air = (self.vel - self.wind).length();
        self.turbulence(dt, v_air, upw);
        let vb = self.rot.inverse() * self.airspeed();
        let v = vb.length();
        let mach = v / sound_speed(h);
        let mut force_b = DVec3::ZERO;
        let mut wdot = DVec3::ZERO;
        let q = 0.5 * rho * v * v;
        let k = self.scale();

        // Spoilers: the speedbrake lever in flight; on the ground, with the
        // power off and still rolling fast, they all come up to dump the lift.
        let ground_spoil = self.on_ground && self.throttle < 0.08 && self.vel.length() > 15.0 && spec.spoiler_cd > 0.0;
        let sp_target = if ground_spoil { 1.0 } else { self.speedbrake };
        self.spoilers += (sp_target - self.spoilers).clamp(-dt * 0.8, dt * 0.8);
        // A braking parachute (`spoilers` is how far it has blossomed): too
        // fast and it tears away; nearly stopped, it is let go before it drags.
        if let (Some(ch), true) = (&spec.chute, self.speedbrake > 0.0) {
            let ias = v_air * (rho / RHO0).sqrt();
            if ias > ch.v_max * 1.05 {
                self.drop_chute();
                self.note("DRAG CHUTE TORN AWAY (OVERSPEED)");
            } else if self.on_ground && ias < ch.v_min {
                self.drop_chute();
            }
        }

        // What is left to fly with.
        let dm = self.dmg;
        let lost = [dm.wing_lost(dm.wing[0]) as f64, dm.wing_lost(dm.wing[1]) as f64];
        let area = (spec.wing_area - lost[0] - lost[1]).max(spec.wing_area * 0.25);
        let ail = [dm.aileron[0] * dm.aileron_frac(dm.wing[0]), dm.aileron[1] * dm.aileron_frac(dm.wing[1])];
        // A hanging aileron does nothing for you.
        let roll_auth = ail.iter().map(|&a| if a < 0.99 && a > 0.0 { 0.0 } else { a as f64 }).sum::<f64>() * 0.5;
        let flap_eff = (dm.flap[0] + dm.flap[1]) as f64 * 0.5;
        let el = (dm.elevator[0] * dm.stab[0] + dm.elevator[1] * dm.stab[1]) as f64 * 0.5;
        let st = (dm.stab[0] + dm.stab[1]) as f64 * 0.5;
        let fin = (dm.fin * 0.6 + dm.rudder * 0.4) as f64;
        let rud = dm.rudder as f64;

        if v > 0.5 {
            let vh = vb / v;
            let alpha = (-vb.y).atan2(-vb.z);
            let beta = (vb.x / v).clamp(-1.0, 1.0).asin();
            self.alpha = alpha;
            // Flaps add lift (and stall a little earlier); flaps and gear add drag.
            let flap_cl = spec.flap_cl * self.flaps * flap_eff;
            let stall_a = self.stall_alpha();
            let stall = stall_amount(stall_a, alpha);
            self.stalled = stall > 0.5 && v > 8.0;
            // Ground effect: within a span of the ground the wing tip vortices are
            // squashed, so there is less induced drag and a little more lift (the
            // float in the flare).
            let ge = match &ground {
                Some(g) if !self.on_ground || v > 5.0 => {
                    let r = 16.0 * ((h - g.h - 0.5).max(0.2) / spec.b);
                    r * r / (1.0 + r * r)
                }
                _ => 1.0,
            };
            let (cl, cd) = lift_drag(spec, alpha, beta, flap_cl, stall, self.flaps, self.gear, self.spoilers, ge);
            // Ragged stumps and missing fairings all drag.
            let rag = ((spec.span - dm.wing[0]) + (spec.span - dm.wing[1])) as f64 / k as f64;
            let cd = cd
                // An open canopy: the cockpit is a hole in the airframe.
                + 0.03 * self.canopy
                + spec.chute.as_ref().map_or(0.0, |ch| ch.cda * self.spoilers / area)
                + self.wave_cd(mach)
                + 0.02 * rag
                + 0.03 * (1.0 - st)
                + 0.01 * (1.0 - rud);
            // Buffet as the airflow starts to break away over the wing, before it
            // stalls (and, near its Mach limit, as the shock waves shake it).
            self.dmg.vibe += (smooth(stall_a - 0.07, stall_a + 0.03, alpha.abs()) * (1.0 - 0.5 * stall) * 0.9 * (q / (hd.q_ref * 0.7)).min(1.0)) as f32;
            if spec.mcrit > 0.0 {
                self.dmg.vibe += smooth(spec.mmo - 0.02, spec.mmo + 0.05, mach) as f32 * 0.8;
            }
            let lift_dir = DVec3::X.cross(vh).normalize_or_zero();
            force_b += lift_dir * (q * area * cl) - vh * (q * area * cd) + DVec3::X * (-0.9 * beta * q * spec.wing_area);

            // Direct angular accelerations, scaled by dynamic pressure; the
            // damping of a rotation grows only with airspeed (it comes from the
            // angle the rotation adds at the wing tip or tail, ω·arm / V), so a
            // slow aircraft is sluggish and a fast one crisp.
            let a = (q / hd.q_ref).clamp(0.0, hd.a_max);
            let ad = (a * hd.damp_v / v.max(20.0)).clamp(hd.ad_min, hd.ad_max);
            // Relative to the air, which is itself turning in the eddies.
            let om = self.omega - self.gust_rot;
            // Auto-trim: while the pilot is flying the stick it winds the trim to
            // the angle that carries the weight at this speed; let go and it stays
            // put, so the aircraft holds the airspeed it was trimmed for, as a real
            // one does: power then makes it climb or descend, and a bump sets it
            // porpoising gently back to that speed. Without an elevator the trim
            // stays where it was.
            let level = self.up().dot(upw).max(0.0);
            let alpha_1g = ((mass * 9.81 * level / (q * area).max(1.0) - spec.cl0 - flap_cl + spec.spoiler_cl * self.spoilers) / spec.cl_a).clamp(-0.05, spec.alpha_trim);
            if el > 0.05 && self.stick.x.abs() > 0.06 {
                self.trim += (alpha_1g - self.trim) * (dt * 0.5).min(1.0);
            }
            // Balance: aft, the static margin shrinks, so the nose is lighter and
            // quicker to answer (beyond the aft limit, barely stable); forward, it
            // is stiff and heavy, and the elevator runs out sooner (a hard flare).
            let shift = self.cg() - spec.cg[0];
            let margin = 0.15 - shift;
            let cg_stiff = (0.4 + 0.6 * margin / 0.15).clamp(0.15, 1.8);
            let cg_auth = (1.0 + 1.5 * shift).clamp(0.5, 1.4);
            // (The stick sets the elevator; how much angle of attack that buys is
            // its moment over the stability: less stable, more for the same pull.)
            let cg_sens = (1.0 / cg_stiff).min(2.5);
            let mut alpha_cmd = self.trim + self.stick.x * hd.alpha_ctl * el * cg_auth * cg_sens;
            if spec.fbw_g > 0.0 {
                // Fly-by-wire: the stick asks for load, never more than the limit.
                let alpha_for = |g: f64| (g * mass * 9.81 / (q * area).max(1.0) - spec.cl0 - flap_cl) / spec.cl_a;
                alpha_cmd = alpha_cmd.clamp(alpha_for(-0.4 * spec.fbw_g), alpha_for(spec.fbw_g));
            }
            // Pitch stiffness comes from the tailplane; without one the nose wanders off.
            let stiff = (hd.pitch_k * st - hd.pitch_k_lost * (1.0 - st)) * cg_stiff;
            wdot.x = a * (stiff * (alpha_cmd - alpha) + self.stick.x * hd.pitch_ctl * el * cg_auth) - hd.pitch_d * ad * (0.3 + 0.7 * st) * om.x;
            // Roll damping fades in a stall, and past it turns round: the wing going
            // down meets the air at a still higher angle and loses still more lift.
            // An airliner's outboard ailerons lock out once the flaps are up.
            let roll_damp = (1.0 - 1.5 * stall).max(-0.35);
            let clean = hd.roll_clean + (1.0 - hd.roll_clean) * smooth(0.0, 0.1, self.flaps);
            wdot.z = a * (-hd.roll_ctl * clean * self.stick.z * roll_auth - hd.roll_beta * beta) - hd.roll_d * ad * roll_damp * om.z;
            wdot.y = a * (-hd.yaw_ctl * self.stick.y * rud - hd.yaw_beta * beta * fin) - hd.yaw_d * ad * (0.25 + 0.75 * fin) * om.y;
            // Dihedral: a banked aircraft slips toward the low wing, which then
            // meets the air at a higher angle and lifts it back, slowly (but not upside down).
            wdot.z -= a * hd.dihedral * self.right().dot(upw) * self.up().dot(upw).max(0.0);
            // Adverse yaw: the aileron that goes down drags its wing back.
            wdot.y += a * hd.adverse * self.stick.z * roll_auth * cl.clamp(0.0, 1.6);
            // A stalled wing drops the nose, and one wing goes first: which one
            // wanders, and a yawing aircraft drops the wing that is going back (a spin).
            // (Only with real airflow over the wing: parked in a tailwind the
            // angle of attack is backwards, and that is no stall.)
            let flow = smooth(0.02, 0.1, a);
            wdot.x -= stall * flow * hd.stall_pitch * alpha.signum();
            self.drop = (self.drop + (self.gauss() * 1.2 - self.drop * 0.4) * dt).clamp(-1.0, 1.0);
            wdot.z += stall * flow * a.max(0.25) * (hd.stall_drop * self.drop + hd.stall_yaw * om.y);
            // A wing that ends short of the tip lifts less than its opposite: it
            // drops, and the drag of its ragged end pulls the nose round.
            let arm = |b: f32| ((b + spec.span) * 0.5) as f64;
            let roll = (lost[0] * arm(dm.wing[0]) - lost[1] * arm(dm.wing[1])) * q * (cl.max(0.05) + 0.1) * 0.6 / spec.inertia.z;
            wdot.z += roll;
            wdot.y -= (lost[1] * arm(dm.wing[1]) - lost[0] * arm(dm.wing[0])) * q * 0.06 / spec.inertia.y;
            // One flap gone with the other out: the lift is lopsided (retract them!).
            wdot.z += (dm.flap[1] - dm.flap[0]) as f64 * self.flaps * q * spec.wing_area * 0.275 * 1.8 * k as f64 * 0.35 / spec.inertia.z;
            // A hanging aileron droops (lifting that wing) and flaps about.
            for s in 0..2 {
                if dm.aileron[s] > 0.0 && dm.aileron[s] < 1.0 {
                    let sgn = if s == 0 { -1.0 } else { 1.0 };
                    wdot.z += sgn * a * (0.45 + 0.35 * (self.t * 23.0 + s as f64).sin()) * hd.roll_ctl / 4.0;
                    self.dmg.vibe += 0.12 * a as f32;
                }
            }
        }
        // The engines. A damaged engine (oil leaking away, a pod struck) loses
        // power and fails; a snapped propeller blade shakes it to pieces; with
        // no blades it just windmills.
        if self.dmg.engine < 1.0 && self.dmg.engine > 0.0 {
            self.dmg.engine = (self.dmg.engine - dt as f32 * (0.002 + 0.01 * (1.0 - self.dmg.engine))).max(0.0);
            if self.dmg.engine <= 0.0 {
                self.note(if spec.is_jet() { "ENGINES FAILED" } else { "ENGINE FAILED" });
            }
        }
        let eng = (self.dmg.engine as f64).powf(0.6);
        // The engine takes a moment to answer the throttle.
        self.charge += (self.throttle - self.charge) * (dt * 4.0).min(1.0);
        match &spec.engine {
            Engine::Piston(pe) => {
                let prop_k = if dm.prop > 0.75 { 1.0 } else if dm.prop > 0.25 { 0.45 } else { 0.0 };
                // The fixed-pitch propeller: the engine's torque against what the
                // blades take at these revs and this airspeed. Faster through the air,
                // the blades meet it at a smaller angle and take less, so the revs
                // rise (and over-rev in a dive); slow, the propeller holds the engine
                // well below its rated revs and power. Without power it windmills, and
                // only slowing nearly to the stall stops it.
                let w = self.prop_rate.max(0.0);
                let v_ax = (-vb.z).max(0.0);
                let mut prop = pe.clone();
                if spec.id == 0 && self.running && self.propeller < 0.999 {
                    // Coarsening the blades increases their load, governing the selected RPM.
                    let target = (1800.0 + 900.0 * self.propeller) * std::f64::consts::TAU / 60.0;
                    self.prop_coarse = (self.prop_coarse + (w - target) * dt * 0.08).clamp(0.0, 2.0);
                } else { self.prop_coarse = (self.prop_coarse - dt * 2.0).max(0.0); }
                prop.cp0 *= 1.0 + self.prop_coarse;
                prop.ct0 *= 1.0 + 0.3 * self.prop_coarse;
                prop.j_t *= 1.0 + 0.2 * self.prop_coarse;
                prop.j_p *= 1.0 + 0.2 * self.prop_coarse;
                let (mut thrust, q_prop) = Self::prop_load(&prop, w, v_ax, rho, prop_k);
                // It fires with the magnetos on, fuel, a mixture that will burn and the
                // crankshaft turning (by the starter, or windmilling in the air); the
                // starter motor's torque falls away as it spins up (it cranks at ~200 rpm).
                // (The starter needs power from the bus; the magnetos do not.)
                let cranking = (c.starter || self.auto_start > 0.0) && self.sys.powered();
                self.cranking = cranking;
                self.running = self.ignition && self.fuel_selector != FuelSelector::Off && self.fuel > 0.0 && self.mixture > 0.08 && w > 8.0 && eng > 0.0 && !self.sys.fail.engine;
                let mix = smooth(0.08, 0.45, self.mixture);
                let q_eng = if self.running { Self::engine_torque(pe, self.charge, rho, c.boost, eng) * mix } else { 0.0 } + if cranking { 120.0 * (1.0 - w / 25.0).max(0.0) } else { 0.0 };
                if self.auto_start > 0.0 {
                    self.auto_start = if self.running && w > 50.0 { 0.0 } else { self.auto_start - dt };
                }
                let drive = q_eng - q_prop;
                if prop_k <= 0.0 {
                    // No blades left: nothing to turn.
                    self.prop_rate += (0.0 - self.prop_rate) * (dt * 2.0).min(1.0);
                } else if w > 1.0 || drive > 40.0 {
                    // Not firing, each cylinder in turn compresses its charge for
                    // nothing: a drag that matters slowly turning (it stops a shut-down
                    // engine in a couple of seconds) but not windmilling in a glide.
                    let comp = if self.running { 0.0 } else { 40.0 * (1.0 - w / 90.0).clamp(0.0, 1.0) };
                    self.prop_rate = (w + (drive - Self::friction(pe, w) - comp) / pe.prop_i * dt).max(0.0);
                } else {
                    // Stopped: the compression holds it until the air pushes harder.
                    self.prop_rate = 0.0;
                }
                self.power = ((q_eng - Self::friction(pe, w)) * w / pe.power).max(0.0);
                let ff = spec.fuel_flow;
                self.fuel_flow = if self.running { (ff[0] + (ff[1] - ff[0]) * self.power.min(1.0) + if c.boost { ff[2] - ff[1] } else { 0.0 }) * (0.7 + 0.3 * self.mixture) } else { 0.0 };
                if dm.prop <= 0.25 {
                    thrust = 0.0;
                }
                force_b += DVec3::NEG_Z * thrust;
                // The propeller turns clockwise seen from the cockpit: its torque rolls the
                // aircraft left and its spiralling slipstream (and P-factor, climbing
                // slowly) yaws it left. The fin and ailerons are rigged for the cruise, so
                // it is full power at low speed that needs right rudder.
                if v > 2.0 {
                    let slow = (1.0 - v / 60.0).max(0.0);
                    let tk = thrust.max(0.0) / pe.static_thrust;
                    wdot.y += tk * slow * (0.22 + 0.25 * self.alpha.max(0.0));
                    wdot.z += tk * slow * 0.12 * if self.on_ground { 0.3 } else { 1.0 };
                    // Taxiing slowly, the propwash over the fin and rudder adds to their
                    // authority. Momentum theory: the slipstream's dynamic pressure is
                    // the free stream's plus T/A (A the disc area); only that excess is
                    // added here, the free stream's part already being in the rudder
                    // term above. A share of it reaches the rudder (the rest passes
                    // below and beside it), and it fades out by 30 m/s, where the
                    // airflow itself does the work.
                    let disc = std::f64::consts::FRAC_PI_4 * pe.prop_d * pe.prop_d;
                    if self.on_ground && disc > 0.01 {
                        let q_wash = thrust.max(0.0) / disc;
                        let fade = 1.0 - smooth(10.0, 30.0, v);
                        wdot.y += fade * (0.45 * q_wash / hd.q_ref).min(1.0) * (-hd.yaw_ctl * self.stick.y * rud);
                    }
                }
                if dm.prop > 0.25 && dm.prop < 0.75 {
                    let spin = (self.prop_rate / 250.0) as f32;
                    self.dmg.vibe += spin * 1.4;
                    if self.dmg.engine > 0.0 {
                        self.dmg.engine = (self.dmg.engine - dt as f32 * 0.06 * spin).max(0.0);
                    }
                }
                if self.dmg.engine < 1.0 && self.dmg.engine > 0.0 {
                    self.dmg.oil = (self.dmg.oil + dt as f32 * 0.04 * (1.0 - self.dmg.engine) * (self.prop_rate / 250.0) as f32).min(1.0);
                }
                self.dmg.smoke = if self.dmg.engine < 1.0 { (1.0 - self.dmg.engine) * (0.3 + 0.7 * (self.prop_rate / 250.0) as f32) * if self.dmg.engine <= 0.0 { 0.5 } else { 1.0 } } else { 0.0 };
            }
            Engine::Fan(fe) => {
                // Reversers: only with weight on the wheels and the thrust levers
                // closed; deployed, the fans spool up to reverse thrust.
                let rev_want = c.reverse && self.on_ground && self.throttle < 0.05;
                let rate = dt / fe.reverser_time;
                self.reverser = (self.reverser + if rev_want { rate } else { -rate }).clamp(0.0, 1.0);
                let reversing = rev_want && self.reverser > 0.95;
                // The fans spool slowly from idle and quicker near the top; a
                // dead engine windmills in the airflow.
                // Starting: the starter turns the core up; past light-off, with the fuel
                // controls at RUN, it lights and runs itself up to idle.
                let can_run = self.ignition && self.fuel > 0.0 && eng > 0.0 && !self.sys.fail.engine;
                if !can_run {
                    self.running = false;
                } else if !self.running && self.n1 > 0.4 * fe.idle {
                    self.running = true;
                }
                if self.auto_start > 0.0 {
                    self.auto_start -= dt;
                    self.starter_on = true;
                }
                if self.running && self.n1 > 0.95 * fe.idle {
                    (self.starter_on, self.auto_start) = (false, 0.0);
                }
                let mut target = if reversing { 0.84 } else { Self::n1_target(fe, self.throttle) };
                target = target.min(fe.idle + (1.0 - fe.idle) * eng);
                if !self.running {
                    // Not burning: windmilling in the airflow, or turned by the starter.
                    target = if self.starter_on && self.sys.powered() { 0.5 * fe.idle } else { (0.02 + v / 900.0).min(fe.idle * 0.8) };
                }
                let up = target > self.n1;
                let kr = if up { fe.spool_up * (0.3 + 0.7 * self.n1) } else { fe.spool_down };
                self.n1 += (target - self.n1) * (dt * kr).min(1.0);
                self.prop_rate = self.n1 * fe.n1_rpm * std::f64::consts::TAU / 60.0;
                let mut thrust = Self::fan_thrust(fe, self.n1, v.max(0.0), h) * self.inlet(mach) * if c.boost { fe.boost } else { self.dry_fade(fe, mach) };
                // An afterburner (not an airliner's take-off bump) lights with a
                // thump a moment after it is asked for and dies away as quickly.
                let lit = c.boost && fe.boost > 1.2 && self.dmg.engine > 0.0 && self.n1 > fe.idle * 0.9;
                self.burner += ((if lit { 1.0 } else { 0.0 }) - self.burner) * (dt * if lit { 5.0 } else { 8.0 }).min(1.0);
                self.power = thrust / (fe.n as f64 * fe.thrust);
                // The burn follows the thrust made (it lapses with height as the
                // thrust does): idle to full rated thrust, then the afterburners.
                let ff = spec.fuel_flow;
                let dry = (self.power / if c.boost { fe.boost } else { 1.0 }).clamp(0.0, 1.2);
                // (A turbo-ramjet's afterburners are fed by the air it swallows: their flow lapses with the thrust.)
                let wet = if spec.inlet_mach > 0.0 { dry.min(1.0) } else { 1.0 };
                self.fuel_flow = if self.running { ff[0] + (ff[1] - ff[0]) * dry + (ff[2] - ff[1]) * self.burner * wet } else { 0.0 };
                thrust *= 1.0 - self.reverser * (1.0 + fe.reverse);
                force_b += DVec3::NEG_Z * thrust;
                self.dmg.smoke = if self.dmg.engine < 1.0 { (1.0 - self.dmg.engine) * (0.3 + 0.7 * self.n1 as f32) } else { 0.0 };
            }
            Engine::Rocket(_) => {}
        }
        self.prop += dt * self.prop_rate;
        self.fuel = (self.fuel - self.fuel_flow * dt).max(0.0);
        if self.sys.fail.fuel_leak {
            // A leak bleeds about 40% of a full-power burn, engine or not.
            self.fuel = (self.fuel - spec.fuel_flow[1] * 0.4 * dt).max(0.0);
        }
        // The electrics: the alternator (the jets' generators) charging once the
        // engine turns fast enough (a piston's above ~900 rpm), the starter drawing.
        let charging = match &spec.engine {
            Engine::Piston(_) => self.running && self.prop_rate > 95.0,
            Engine::Fan(fe) => self.running && self.n1 > 0.8 * fe.idle,
            Engine::Rocket(_) => true,
        };
        let cranking = !self.running && (c.starter || self.auto_start > 0.0 || self.starter_on);
        self.sys.update(dt, charging, cranking, true);

        // Ground contact.
        let mut force_w = self.rot * force_b - upw * (mass * 9.81);
        let mut torque_b = DVec3::ZERO;
        self.on_ground = false;
        // Nothing is solid for the first moments after a spawn: the terrain
        // under the aircraft is still a coarse guess until its tile arrives.
        if let Some(mut b) = self.bury {
            let ds = self.vel.length() * dt;
            b.gone += ds;
            self.vel *= b.keep_per_m.powf(ds);
            self.omega *= 0.9f64.powf(dt * 60.0);
            self.bury = Some(b);
            // In deep enough, or stopped in it: it goes up there, where the
            // aircraft now is (the middle of the fuselage, not the nose tip).
            if b.gone >= b.depth || self.vel.length() < 12.0 {
                self.bury = None;
                let p = self.pos;
                let imp = Impact { pos: p, vel: self.vel, water: false, ground_h: geo::pos_to_lla(p).2, building: Some(b.building) };
                return self.impact(imp, b.cause);
            }
        }
        let reach = 60.0 + 2.0 * spec.b;
        let close = self.t > 1.5 && (ground.as_ref().map(|g| h - g.h < reach).unwrap_or(false) || world.building(lat, lon).is_some());
        self.brushes = (self.brushes - dt * 0.5).max(0.0);
        let mut skidding = false;
        if close {
            if let Some(imp) = self.trees(world) {
                return self.impact(imp, "tree");
            }
            let hit = |p: DVec3, world: &World| -> Option<(f64, bool, Option<(geo::TileId, usize)>)> {
                let (la, lo, ph) = geo::pos_to_lla(p);
                let g = world.ground(la, lo)?;
                if let Some(b) = world.building(la, lo) {
                    if ph < b.top && ph > b.base - if b.bridge { 0.0 } else { 3.0 } {
                        return Some((b.top - ph, false, Some((b.tile, b.idx))));
                    }
                } else if ph < g.h {
                    // Under the 3D tiles the ground is the mesh, buildings and all:
                    // into one of them, it is that building that was hit.
                    if let Some(b) = world.photo_building(la, lo, g.h) {
                        if ph > b.base - 3.0 {
                            return Some((g.h - ph, false, Some((b.tile, b.idx))));
                        }
                    }
                }
                Some((g.h - ph, g.water, None))
            };
            // Every point of the airframe that can touch something, and what it
            // takes to break it: gentle enough, things only scrape; harder, the
            // part that hit comes off (a wing, the tail, the gear); only a hit
            // hard enough to burst the cabin or the tanks ends in fire.
            let mut pts = std::mem::take(&mut self.pts);
            pts.clear();
            self.contacts(&mut pts);
            let mut walled = false;
            for &(r0, part) in &pts {
                let r = if matches!(part, Part::Nose | Part::Belly | Part::Side | Part::Roof) { r0 + self.crumple_offset(r0) } else { r0 };
                let p = self.pos + self.rot * r;
                let Some((mut pen, water, building)) = hit(p, world) else { continue };
                if water {
                    // A ditched aircraft settles lower in the water as it floods.
                    pen -= self.sink;
                }
                if pen <= 0.0 {
                    continue;
                }
                let mut pv = self.vel + self.rot * self.omega.cross(r);
                if let Part::Fold(s) = part {
                    let f = &self.dmg.fold[s];
                    pv += self.rot * (f.axis.cross(r.as_vec3() - f.pivot) * f.rate).as_dvec3();
                }
                let vn = pv.dot(upw);
                let closing = (-vn).max(0.0);
                let sp = pv.length();
                let crash = Impact { pos: p, vel: self.vel, water, ground_h: geo::pos_to_lla(p).2 + if building.is_some() { 0.0 } else { pen }, building };
                let up_right = self.up().dot(upw) > 0.0;
                if building.is_some() && pen > 1.0 {
                    // Into the side of a building: all of the speed goes into it.
                    match part {
                        Part::Wing(s, st) => {
                            // A wing's root goes in with the cabin right behind it.
                            if st < self.sta(0.33) && sp > 28.0 {
                                self.start_bury(building.unwrap(), world, "wing root into building");
                                continue;
                            }
                            if self.cool[s] <= 0.0 {
                                let into = (self.rng.range(0.1, 0.6) + (sp as f32 / 30.0).min(2.0) * self.rng.f()) * k;
                                let (rough, up) = (self.rng.range(0.0, 0.5), self.rng.f() < 0.5);
                                self.snap_wing(s, st - into, rough, if sp < 25.0 { 0.35 } else { 0.1 }, up, "BUILDING");
                                self.cool[s] = 0.25;
                                self.vel *= 0.85;
                            }
                        }
                        Part::Pod(s) => self.pod_strike(s, 1.0, "BUILDING"),
                        Part::Fold(s) => self.release_fold(s, "BUILDING"),
                        Part::Fin => self.lose_tail_top(0.0, 0.3, "BUILDING"),
                        Part::Stab(s) => self.lose_elevator(s, true, "BUILDING"),
                        Part::Tail if sp > 12.0 => {
                            let z = self.tail_z();
                            self.snap_tail(z, "BUILDING");
                        }
                        _ if sp > 18.0 || self.bury.is_some() => {
                            self.start_bury(building.unwrap(), world, "building");
                        }
                        _ if !walled => {
                            // Taxiing into a wall: crumpled, stopped and pushed back off it.
                            walled = true;
                            if matches!(part, Part::Prop | Part::Nose) {
                                self.prop_strike("BUILDING");
                                if sp > 8.0 {
                                    self.engine_hit(0.35, "BUILDING");
                                    self.crack_windshield("BUILDING");
                                }
                            }
                            self.dmg.hull = (self.dmg.hull + sp as f32 * 0.03).min(1.0);
                            self.scuff(part_group(part), 0.6);
                            let back = (self.vel - upw * self.vel.dot(upw)).normalize_or_zero();
                            self.pos -= back * 0.25;
                            self.vel = -self.vel * 0.15;
                            self.omega *= 0.4;
                            self.dmg.vibe += 2.0;
                        }
                        _ => {}
                    }
                    continue;
                }
                // Ground, a roof or the water. First what this hit breaks.
                let nose_on = matches!(part, Part::Nose | Part::Prop);
                let body = matches!(part, Part::Belly | Part::Side | Part::Roof);
                if water && (nose_on || body) && (closing > 9.0 || (sp > 42.0 && closing > 4.0)) {
                    return self.impact(crash, "water");
                }
                match part {
                    Part::Prop | Part::Nose => {
                        if self.dmg.prop > 0.25 && (part == Part::Prop || self.prop_rate > 30.0) && (self.prop_rate > 5.0 || sp > 3.0) {
                            self.prop_strike(if water { "WATER" } else { "GROUND" });
                        }
                        // Nose first into the ground: the engine is driven back into the cabin.
                        if closing > 15.0 || (sp > 50.0 && closing > 7.5) {
                            return self.impact(crash, "water");
                        }
                        if closing > 6.0 && self.cool[3] <= 0.0 {
                            self.cool[3] = 0.3;
                            self.engine_hit(0.15 + (closing as f32 - 6.0) * 0.05, "NOSE IMPACT");
                            self.crack_windshield("NOSE IMPACT");
                            self.dmg.hull = (self.dmg.hull + (closing as f32 - 6.0) * 0.04).min(1.0);
                            self.dent(r, pv, upw, closing);
                            if closing > 8.5 {
                                self.lose_gear(0, "NOSE IMPACT");
                            }
                        }
                    }
                    Part::Pod(s) => {
                        // An engine pod dug in hard enough tears open its tanks' wing.
                        if closing > 14.0 && !water {
                            return self.impact(crash, "engine");
                        }
                        if closing > 0.5 || sp > 5.0 {
                            self.pod_strike(s, (closing as f32 / 6.0).min(1.0), if water { "WATER" } else { "GROUND" });
                        }
                    }
                    Part::Belly | Part::Side | Part::Roof => {
                        // The cabin can take a hard landing on its belly; on its side or
                        // its roof it caves in sooner. Fast and steep, nothing survives.
                        let lim = if part == Part::Belly { 15.0 } else { 12.0 };
                        if closing > lim || (sp > 55.0 && closing > lim * 0.55) {
                            return self.impact(crash, "cabin");
                        }
                        if closing > 4.5 && self.cool[3] <= 0.0 {
                            self.cool[3] = 0.3;
                            self.dmg.hull = (self.dmg.hull + (closing as f32 - 4.5) * 0.05).min(1.0);
                            self.dent(r, pv, upw, closing);
                            self.dmg.vibe += 3.0;
                            if closing > 6.5 || part != Part::Belly {
                                self.crack_windshield(if part == Part::Roof { "ROLLED OVER" } else { "HARD IMPACT" });
                            }
                            if part == Part::Belly && closing > 7.0 {
                                for gi in 0..spec.gear.len() {
                                    if self.gear > 0.5 {
                                        self.lose_gear(gi, "HARD IMPACT");
                                    }
                                }
                            }
                        }
                    }
                    Part::Tail => {
                        // A tail slammed down (or dug in sliding fast) snaps off the cone.
                        if closing > 7.5 || (closing > 4.0 && sp > 25.0) {
                            let z = self.tail_z();
                            self.snap_tail(z, "GROUND");
                            continue;
                        }
                    }
                    Part::Fin => {
                        // Scraped along the ground upside down, the fin wears away.
                        if self.cool[2] <= 0.0 && (closing > 2.5 || (sp > 7.0 && pen > 0.04)) {
                            self.cool[2] = 0.35;
                            let fin = (self.dmg.fin - self.rng.range(0.2, 0.5)).max(0.0);
                            self.lose_tail_top((self.dmg.rudder - 0.4).max(0.0).min(fin + 0.1), fin, "GROUND");
                            continue;
                        }
                    }
                    Part::Stab(s) => {
                        if closing > 3.0 || (sp > 12.0 && pen > 0.06) {
                            self.lose_elevator(s, true, "GROUND");
                            continue;
                        }
                    }
                    Part::Wing(s, st) => {
                        // A wing struck into the ground breaks in from where it hit,
                        // the further the harder; scraped along fast it tears too.
                        let hard = closing as f32 / 4.0 + sp as f32 / 45.0;
                        if self.cool[s] <= 0.0 && (closing > 3.5 || (sp > 14.0 && pen > 0.1) || (closing > 2.0 && sp > 25.0)) {
                            // (One blow takes a couple of metres at most: breaking the
                            // outer part soaks up what would have hit further in.)
                            let into = (self.rng.range(0.05, 0.5) + (hard - 1.0).clamp(0.0, 3.0) * self.rng.range(0.2, 0.6)) * k;
                            let station = (st - into).max(0.3 * k);
                            // Through the tank fast: the fuel goes up.
                            if station < spec.tank[1] && sp > 32.0 && closing > 8.0 && !water {
                                return self.impact(crash, "wing tank");
                            }
                            let rough = self.rng.range(0.35, 1.0);
                            let fold = if water { 0.2 } else if sp < 30.0 { 0.5 } else { 0.25 };
                            self.snap_wing(s, station, rough, fold, up_right, if water { "WATER" } else { "GROUND" });
                            self.cool[s] = 0.3;
                            self.vel *= 0.94;
                            continue;
                        }
                    }
                    Part::Fold(_) => {}
                }
                // Then it skids: a spring out of the surface, friction along it.
                // Water gives less and drags far more.
                let hl = &spec.hull;
                self.pos += upw * if water { pen * 0.3 } else { pen };
                let (kk, cc, mu) = if water { (hl.k * 0.42, hl.c * 0.67, 0.9) } else { (hl.k, hl.c, 0.45) };
                let nf = (pen * kk - vn * cc).clamp(0.0, hl.nf_max);
                let tang = pv - upw * vn;
                let tl = tang.length();
                let f = upw * nf - tang.normalize_or_zero() * (mu * nf).min(tl * hl.drag);
                force_w += f;
                let fb_ = self.rot.inverse() * f;
                torque_b += r.cross(fb_);
                if let Part::Fold(s) = part {
                    let fd = &self.dmg.fold[s];
                    self.fold_tq[s] += (r.as_vec3() - fd.pivot).as_dvec3().cross(fb_).dot(fd.axis.as_dvec3());
                }
                skidding = true;
                if water {
                    self.sink = (self.sink + dt * 0.03).min(1.6 * k as f64);
                    if tl > 4.0 {
                        self.splashes.push((p, tang * 0.4 + upw * (tl * 0.15)));
                    }
                    continue;
                }
                self.scuff(part_group(part), (dt * tl * 0.04) as f32);
                self.dmg.vibe += (tl * 0.03) as f32;
                if tl > 3.0 {
                    self.sparks.push((p, tang * 0.5));
                }
                if tl > 1.5 && building.is_none() {
                    self.dirt.push((p, tang * 0.45 + upw * (2.0 + tl * 0.12), 2.5));
                }
                if building.is_none() && (tl > 0.05 || !self.on_hull) {
                    let rad = match part {
                        Part::Belly | Part::Nose | Part::Side | Part::Roof => (0.75 * k).clamp(0.65, 3.8),
                        Part::Pod(_) => (0.55 * k).clamp(0.45, 2.2),
                        Part::Wing(..) | Part::Fold(_) => (0.42 * k).clamp(0.35, 1.8),
                        Part::Tail | Part::Fin | Part::Stab(_) | Part::Prop => (0.38 * k).clamp(0.32, 1.5),
                    };
                    let clear = (0.82 + (tl as f32 / 14.0).min(0.18)).min(1.0);
                    self.snow_marks.push(crate::snowtrack::Mark {
                        a: p - tang * dt,
                        b: p,
                        radius: rad,
                        clear,
                        berm: 0.92,
                        rough: 0.65,
                    });
                }
            }
            self.pts = pts;
            let mut touching = [false; MAX_GEAR];
            for (gi, leg) in spec.gear.iter().enumerate() {
                // Only gear that is down and locked carries the aircraft.
                if self.gear < 0.999 {
                    break;
                }
                if !self.dmg.gear[gi] {
                    continue;
                }
                let r = leg.at;
                let p = self.pos + self.rot * r;
                if let Some((pen, water, building)) = hit(p, world) {
                    if pen <= 0.0 {
                        continue;
                    }
                    let pv = self.vel + self.rot * self.omega.cross(r);
                    let vn = pv.dot(upw);
                    // Wheels do nothing in water or against a wall, and a leg driven
                    // in far too hard just goes: the airframe takes the rest.
                    if water || (building.is_some() && pen > 1.0) || pen > leg.travel * 4.0 + 0.8 {
                        if !water {
                            self.lose_gear(gi, if building.is_some() { "BUILDING" } else { "HARD LANDING" });
                        }
                        continue;
                    }
                    if vn < -(leg.fold_vn + 4.5) {
                        self.lose_gear(gi, "HARD LANDING");
                        continue;
                    }
                    // Too hard a landing and the leg folds; the airframe comes down on its belly.
                    if vn < -leg.fold_vn {
                        self.lose_gear(gi, "HARD LANDING");
                        self.kick(leg.fold_kick);
                        self.crack_windshield("HARD LANDING");
                        continue;
                    }
                    self.on_ground = true;
                    touching[gi] = true;
                    self.compress[gi] = pen.min(leg.travel);
                    let nf = (pen * leg.k - vn * leg.c).clamp(0.0, leg.nf_max);
                    let mut fwd = (self.forward() - upw * self.forward().dot(upw)).normalize_or_zero();
                    if leg.steer != 0.0 {
                        // The nose wheel steers with the rudder pedals.
                        fwd = DQuat::from_axis_angle(upw, -self.steer_angle() * leg.steer) * fwd;
                    }
                    let side = fwd.cross(upw);
                    let (vl, vs) = (pv.dot(fwd), pv.dot(side));
                    let f_side = (-vs * leg.side_k).clamp(-leg.side_mu * nf, leg.side_mu * nf);
                    let brake = if leg.brakes { self.brake_pressure(c, leg.at.x) } else { 0.0 };
                    let mu = leg.roll_mu + (leg.brake_mu - leg.roll_mu) * brake;
                    let drag = leg.roll_k + (leg.brake_k - leg.roll_k) * brake;
                    let f_long = (-vl * drag).clamp(-mu * nf, mu * nf);
                    let f = upw * nf + side * f_side + fwd * f_long;
                    force_w += f;
                    torque_b += r.cross(self.rot.inverse() * f);
                    self.wheel_spin[gi] = vl / leg.wheel_r;
                    if pv.length() > 3.0 && building.is_none() {
                        self.dirt.push((p, (pv - upw * vn) * 0.3 + upw * 1.5, 1.0));
                    }
                    if building.is_none() {
                        // Continuous landing-gear track in snow: wider for heavy multi-wheel
                        // bogies, deeper when braking, skidding sideways or touching down hard.
                        let bogie = if spec.mass > 30_000.0 { 1.4 } else { 1.0 };
                        let skid = ((vs.abs() / 6.0).min(1.0) * 0.5 + brake * 0.5) as f32;
                        let touch = ((-vn / 3.0).clamp(0.0, 1.0)) as f32;
                        let rad = (leg.wheel_r as f32 * bogie).max(0.28) * (1.0 + 0.20 * skid);
                        let clear = (0.80 + 0.14 * skid.max(touch)).min(0.95);
                        let berm = (0.68 + 0.24 * skid.max(touch)).min(0.95);
                        match self.wheel_prev[gi] {
                            Some(p0) => {
                                if (p - p0).length_squared() >= 0.04 * 0.04 {
                                    self.snow_marks.push(crate::snowtrack::Mark { a: p0, b: p, radius: rad, clear, berm, rough: 0.12 + 0.35 * skid });
                                    self.wheel_prev[gi] = Some(p);
                                }
                            }
                            None => {
                                self.snow_marks.push(crate::snowtrack::Mark { a: p, b: p, radius: rad, clear, berm, rough: 0.15 });
                                self.wheel_prev[gi] = Some(p);
                            }
                        }
                    } else {
                        self.wheel_prev[gi] = None;
                    }
                }
            }
            for gi in 0..spec.gear.len() {
                if !touching[gi] {
                    self.compress[gi] *= (1.0 - dt * 12.0).max(0.0);
                    self.wheel_prev[gi] = None;
                }
            }
        } else {
            self.compress = [0.0; MAX_GEAR];
            self.wheel_prev = [None; MAX_GEAR];
        }
        // Wheels spin down in the air; brakes stop the mains at once.
        for (gi, leg) in spec.gear.iter().enumerate() {
            if !self.on_ground {
                self.wheel_spin[gi] *= (1.0 - dt * if leg.brakes && leg.steer == 0.0 { 0.25 + 7.75 * self.brake_pressure(c, leg.at.x) } else { 0.25 }).max(0.0);
            }
            self.wheel_rot[gi] = (self.wheel_rot[gi] + self.wheel_spin[gi] * dt) % std::f64::consts::TAU;
        }

        self.on_hull = skidding;
        let acc = force_w / mass;
        self.swing_folds(dt, acc, upw, vb, rho);
        self.g_load = (acc + upw * 9.81).dot(self.up()) / 9.81;
        // The pilot's head on its neck: a stiff, well-damped spring shoved about
        // by every change in the load, about 3 cm per g.
        self.sf = self.rot.inverse() * (acc + upw * 9.81);
        let fb = self.sf - DVec3::Y * 9.81;
        let (w0, zeta) = (11.0, 0.55);
        self.head_v += (-self.head * w0 * w0 - self.head_v * 2.0 * zeta * w0 - fb * 0.35) * dt;
        if self.on_ground {
            // Rolling over the pavement's joints and patches: a few millimetres
            // of shake by taxi speed, more on the takeoff roll.
            let s = smooth(0.5, 35.0, self.vel.length()) * 0.375 * dt.sqrt();
            let mut k = || self.rng.range(-1.0, 1.0) as f64;
            self.head_v += DVec3::new(k() * 0.35, k(), k() * 0.2) * s;
        }
        self.head = (self.head + self.head_v * dt).clamp_length_max(0.09);
        // Rough air rattles the cabin.
        self.dmg.vibe += ((self.air.sigma_w - 0.6).max(0.0) * 0.07 * smooth(10.0, 40.0, v)) as f32;
        self.vel += acc * dt;
        self.omega += (wdot + torque_b / spec.inertia) * dt;
        self.omega = self.omega.clamp_length_max(6.0);
        self.pos += self.vel * dt;
        self.rot = (self.rot * DQuat::from_scaled_axis(self.omega * dt)).normalize();

        self.flex_wings(dt, skidding);

        // Airframe limits: pull too hard and a wing goes; too fast and the
        // control surfaces flutter themselves off; flaps out too fast tear away.
        if !self.on_ground && !skidding {
            let g = self.g_load.abs() as f32;
            // Past the limit load the spar fatigues; at the ultimate load it snaps outright.
            self.dmg.fatigue = (self.dmg.fatigue + ((g - spec.g_limit).max(0.0) * 1.5 - 0.3) * dt as f32).max(-1.5);
            if self.dmg.fatigue < 0.0 {
                self.dmg.fatigue = (self.dmg.fatigue + dt as f32).min(0.0);
            }
            if (g > spec.g_break && self.dmg.fatigue >= 0.0) || self.dmg.fatigue > 2.2 {
                let s = (self.rng.f() * 2.0) as usize;
                self.break_wing(s, self.sta(0.64), "OVER G");
                // The load comes off the rest of the structure for a moment.
                self.dmg.fatigue = -1.5;
            }
        }
        let ve = v * eas_k;
        let excess = self.excess(ve, mach);
        let over = (excess - (spec.flutter_v - spec.vne)).max(0.0) as f32;
        self.dmg.flutter = (self.dmg.flutter + (over * 0.035 - 0.25) * dt as f32).max(0.0);
        self.dmg.vibe += smooth(5.0, 30.0, excess) as f32 * 0.6;
        if self.dmg.flutter > 1.0 {
            self.flutter_loss();
            self.dmg.flutter = 0.35;
        }
        let vfe = self.vfe();
        if self.flaps > 0.25 * spec.flap_frac(1) && ve > vfe {
            self.dmg.flap_stress += (dt * (ve - vfe) * 0.06 * self.flaps.max(0.3)) as f32;
            if self.dmg.flap_stress > 1.0 {
                let s = (self.rng.f() * 2.0) as usize;
                let s = if self.dmg.flap[s] > 0.0 { s } else { 1 - s };
                self.lose_flap(s, "OVERSPEED");
                self.dmg.flap_stress = 0.3;
            }
        } else {
            self.dmg.flap_stress = (self.dmg.flap_stress - dt as f32 * 0.3).max(0.0);
        }
        None
    }

    /// A buckled wing swings on its torn skin: its weight and the aircraft's
    /// own acceleration, the air on it as on a flat plate, the ground under
    /// it, and what stiffness the crumpled skin has left. Load it past what
    /// the skin holds (as the slipstream does at once in flight) and it goes.
    fn swing_folds(&mut self, dt: f64, acc: DVec3, upw: DVec3, vb: DVec3, rho: f64) {
        let spec = self.spec;
        let wk = self.fold_k() as f64;
        let kg_m = self.wing_kg_m() as f64;
        for s in 0..2 {
            let mut f = self.dmg.fold[s];
            if !f.on {
                continue;
            }
            let len = (f.tip - f.root).max(0.3) as f64;
            let m = kg_m * len;
            let inertia = m * len * len / 3.0 + 2.0 * wk;
            let mat = f.mat();
            let ax = f.axis.as_dvec3();
            let cm = mat.transform_point3(damage::wing_pt(spec, s, (f.root + f.tip) * 0.5, 0.4, 0.0)).as_dvec3();
            let arm = cm - f.pivot.as_dvec3();
            // Weight, and the shove of the aircraft's own acceleration, as the part feels them.
            let felt = self.rot.inverse() * (acc + upw * 9.81);
            let t_g = arm.cross(-felt * m).dot(ax);
            // The air on it: a plate across its swung upper skin, the air meeting
            // it at the aircraft's speed plus its own swinging.
            let np = mat.transform_vector3(Vec3::Y).as_dvec3();
            let area = (self.dmg.wing_lost(f.root) - self.dmg.wing_lost(f.tip)) as f64;
            let rel = -(vb + self.omega.cross(cm)) - ax.cross(arm) * f.rate as f64;
            let vr = rel.length();
            let mut t_a = 0.0;
            if vr > 0.5 {
                let a_n = (rel.dot(np) / vr).clamp(-1.0, 1.0).asin();
                let cn = (spec.cl_a * a_n + spec.cl0 * (1.0 - a_n.abs() * 2.0).max(0.0)).clamp(-1.2, 1.2);
                let fa = np * (0.5 * rho * vr * vr * area * cn);
                t_a = arm.cross(fa).dot(ax);
                // The aircraft feels it through the hinge.
                let m = self.mass();
                self.vel += self.rot * fa * (dt / m);
                self.omega += cm.cross(fa) / spec.inertia * dt;
            }
            // The crumpled skin still springs back a little, and damps the swing.
            let w = f.rate as f64;
            let t_s = -350.0 * wk * f.ang as f64 - 220.0 * wk * w - 0.5 * rho * 1.2 * 1.4 * w * w.abs() * len.powi(4) / 4.0 * (kg_m / 24.0).sqrt();
            let t_c = self.fold_tq[s];
            f.rate += ((t_g + t_a + t_s + t_c) / inertia * dt) as f32;
            f.ang += f.rate * dt as f32;
            // Folded right over (or down against the strut), it stops hard.
            let (lo, hi) = (-1.75, 2.7);
            if f.ang > hi || f.ang < lo {
                f.ang = f.ang.clamp(lo, hi);
                f.stress += (f.rate.abs() * 0.04).min(0.3);
                f.rate *= -0.2;
            }
            // How hard the skin is being worked: steady load past what it holds,
            // and every big swing.
            let load = (t_g + t_a + t_c).abs() as f32;
            // Crumpled skin in a fast airflow flutters itself apart in moments.
            let flog = 0.5 * ((0.5 * rho * vb.length_squared()) / 600.0).powf(1.5);
            // (Tearing takes a moment, however hard it is pulled: at least a fifth of a second.)
            let over = (((load - f.hold).max(0.0) / f.hold) * 4.0).min(5.0);
            f.stress += (over + (f.rate.abs() - 4.0).max(0.0) * 0.1 + 0.01 + flog as f32) * dt as f32;
            self.dmg.vibe += (f.rate.abs() * 0.05).min(1.0);
            self.dmg.fold[s] = f;
            if f.stress >= 1.0 {
                self.release_fold(s, if self.on_hull || self.on_ground { "GROUND" } else { "AIRFLOW" });
            }
        }
    }

    /// Trees: refresh the list of nearby ones a few times a second, test every
    /// step. Foliage scrapes and tears things off; a trunk through the nose
    /// or belly is the end. A tree only goes over when the blow carries more
    /// energy than its trunk can take: a light aircraft's wing shears off
    /// against a mature trunk and leaves it standing, an airliner mows them down.
    fn trees(&mut self, world: &World) -> Option<Impact> {
        let spec = self.spec;
        if self.scan == 0 {
            self.near_trees.clear();
            world.trees_near(self.pos, 30.0 + spec.b * 0.5 + self.vel.length() * 0.2, &mut self.near_trees);
            let (pos, reach) = (self.pos, 60.0 + spec.b + self.vel.length());
            let near = &self.near_trees;
            self.struck.retain(|s| near.iter().any(|t| t.tile == s.0 && t.idx == s.1 && (t.pos - pos).length() < reach));
        }
        self.scan = (self.scan + 1) % 12;
        let v = self.vel.length();
        let mut k = 0;
        while k < self.near_trees.len() {
            let t = &self.near_trees[k];
            let tu = t.pos.normalize();
            let (cc, ch) = ((t.crown_base + t.height) * 0.5, ((t.height - t.crown_base) * 0.5).max(0.5));
            // 0 = miss, 1 = through the foliage, 2 = into the trunk or the heart of the crown.
            let mut worst = 0;
            let mut at = 5;
            let mut core: Option<DVec3> = None;
            for (hi, r) in spec.hard.into_iter().enumerate() {
                let d = self.pos + self.rot * r - t.pos;
                let hgt = d.dot(tu);
                if hgt < -1.0 || hgt > t.height + 1.0 {
                    continue;
                }
                let flat = d - tu * hgt;
                let rad = flat.length();
                let trunk = rad < t.trunk_r + 0.35 && hgt < cc;
                if trunk && matches!(hi, 0 | 5 | 6) {
                    core = Some(flat / rad.max(1e-3));
                }
                let e = ((rad / t.crown_r.max(0.5)).powi(2) + ((hgt - cc) / ch).powi(2)).sqrt();
                let level = if trunk || e < 0.45 { 2 } else if e < 1.0 { 1 } else { 0 };
                if level > worst {
                    worst = level;
                    at = hi;
                }
            }
            let prev = self.struck.iter().position(|s| s.0 == t.tile && s.1 == t.idx);
            let before = prev.map(|i| self.struck[i].2).unwrap_or(0);
            if worst <= before {
                // Already went through this one: a trunk still standing stays solid
                // against the fuselage instead of letting it creep through.
                if before == 2 {
                    if let Some(n) = core {
                        let vn = self.vel.dot(n);
                        if vn < 0.0 {
                            self.vel -= n * vn;
                        }
                    }
                }
                k += 1;
                continue;
            }
            // Energy the trunk soaks up before it snaps, against the share of
            // the aircraft's that goes into it before the part that hit gives.
            let ke = 0.5 * self.mass() * v * v;
            let snap_e = 3.0e5 * (t.trunk_r / 0.25).powi(3) * (t.height / 15.0).clamp(0.6, 1.4);
            let share = match (worst, at) {
                (2, 1 | 2) => 0.6,
                (2, 3 | 4) => 0.4,
                (2, _) => 1.0,
                _ => 0.03,
            };
            let snaps = ke * share > snap_e;
            let base_h = geo::pos_to_lla(t.pos).2;
            if snaps {
                let t = self.near_trees.swap_remove(k);
                if let Some(i) = prev {
                    self.struck.swap_remove(i);
                }
                // Breaking it costs what it took to break.
                let v2 = (v * v - 2.0 * snap_e / self.mass()).max(0.36 * v * v);
                self.vel *= (v2 / (v * v).max(1e-6)).sqrt();
                self.tree_hits.push(t);
            } else {
                let id = (t.tile, t.idx);
                match prev {
                    Some(i) => self.struck[i].2 = worst,
                    None => self.struck.push((id.0, id.1, worst)),
                }
                k += 1;
            }
            self.brushes += 1.0;
            let side = if at == 1 { 0 } else { 1 };
            let sgn = if side == 0 { -1.0 } else { 1.0 };
            let r = self.rng.f();
            // A tree that holds: what hit it is stopped, and the aircraft pivots on it.
            let hold = if snaps { 1.0 } else { 1.7 };
            if worst == 2 {
                match at {
                    1 | 2 => {
                        // A trunk through the outer wing shears it off.
                        self.break_wing(side, self.sta(if v > 45.0 { 0.64 } else { 0.78 }), "TREE");
                        self.vel *= if snaps { 0.9 } else { 0.72 };
                        self.kick(DVec3::new(0.2, -sgn * 0.7, -sgn * 1.5) * hold);
                    }
                    3 | 4 => {
                        self.lose_tail_top(0.4, 0.45, "TREE");
                        self.lose_elevator((r * 2.0) as usize, true, "TREE");
                        self.vel *= if snaps { 0.92 } else { 0.8 };
                        self.kick(DVec3::new(-0.8, 0.6, 0.4) * hold);
                    }
                    // Rolling slowly into a trunk crumples the nose and stops it dead.
                    _ if v < 18.0 => {
                        self.prop_strike("TREE");
                        self.crack_windshield("TREE");
                        self.dmg.hull = (self.dmg.hull + 0.2).min(1.0);
                        self.vel *= if snaps { 0.4 } else { 0.1 };
                        self.omega *= 0.3;
                    }
                    _ => return Some(Impact { pos: self.pos, vel: self.vel, water: false, ground_h: base_h + 0.2, building: None }),
                }
                continue;
            }
            // Clipped the canopy: it costs speed, throws the aircraft about, and
            // whatever went through the branches comes out worse for it.
            if !snaps {
                self.vel *= 1.0 - 0.08 * (1100.0 / spec.mass).cbrt();
            }
            let kick = (self.scan as f64 * 2.4).sin();
            self.kick(DVec3::new(0.25 * kick, 0.5 * kick, 0.9 * (self.scan as f64 * 1.7).cos()) * hold.sqrt());
            let hard = (v / 45.0) as f32;
            match at {
                1 | 2 => {
                    self.scuff(Self::wing_group(side), 0.35);
                    if r < 0.12 * hard {
                        self.break_wing(side, self.sta(0.91), "TREE");
                    } else if r < 0.4 * hard {
                        self.lose_aileron(side, r > 0.2 * hard, "TREE");
                    }
                }
                0 | 6 => {
                    self.scuff(G_ENGINE, 0.4);
                    if r < 0.3 * hard {
                        self.prop_strike("TREE");
                    } else if r < 0.6 * hard {
                        self.engine_hit(0.15, "TREE");
                    }
                    if self.rng.f() < 0.45 * hard {
                        self.crack_windshield("TREE");
                    }
                }
                3 | 4 => {
                    self.scuff(G_TAIL, 0.35);
                    if r < 0.3 * hard {
                        self.lose_tail_top(0.5, 1.0, "TREE");
                    } else if r < 0.5 * hard {
                        self.lose_elevator((self.scan % 2) as usize, false, "TREE");
                    }
                }
                _ => {
                    self.scuff(damage::hard_group(at), 0.3);
                    if self.gear > 0.5 && r < 0.3 * hard {
                        self.lose_gear(1 + (self.scan % 2) as usize, "TREE");
                    }
                }
            }
        }
        None
    }
}

/// US Standard Atmosphere 1976, with layer bases in geopotential metres,
/// base temperature in kelvin, and lapse rate in K/m. Base pressures are
/// integrated from sea level so pressure, density and temperature stay
/// thermodynamically consistent at every layer boundary.
const ISA_LAYERS: [(f64, f64); 7] = [
    (11_000.0, -0.0065),
    (20_000.0, 0.0),
    (32_000.0, 0.0010),
    (47_000.0, 0.0028),
    (51_000.0, 0.0),
    (71_000.0, -0.0028),
    (84_852.0, -0.0020),
];

/// Temperature (K) and pressure (Pa) at geometric altitude `h` (m), using
/// the USSA 1976 geopotential layers. Above 84.852 km, continue isothermally
/// from the last tabulated layer; aircraft performance checks stay well below
/// this extension, while it avoids freezing density for high rocket flights.
fn atmosphere(h: f64) -> (f64, f64) {
    const G0: f64 = 9.80665;
    const R_AIR: f64 = 287.05287;
    const EARTH_RADIUS: f64 = 6_356_766.0;
    const P0: f64 = 101_325.0;
    const T0: f64 = 288.15;

    // The standard tables use geopotential altitude; convert the simulator's
    // geometric altitude before selecting a layer.
    let h = h.max(-500.0);
    let z = EARTH_RADIUS * h / (EARTH_RADIUS + h);
    let mut base_z = 0.0;
    let mut base_t = T0;
    let mut base_p = P0;

    for &(top_z, lapse) in &ISA_LAYERS {
        let layer_z = z.min(top_z);
        let dz = layer_z - base_z;
        let t = base_t + lapse * dz;
        let p = if lapse.abs() < 1e-12 {
            base_p * (-G0 * dz / (R_AIR * base_t)).exp()
        } else {
            base_p * (t / base_t).powf(-G0 / (R_AIR * lapse))
        };
        if z <= top_z {
            return (t, p);
        }
        let full_dz = top_z - base_z;
        base_t += lapse * full_dz;
        base_p = p;
        base_z = top_z;
    }

    let dz = z - base_z;
    let p = base_p * (-G0 * dz / (R_AIR * base_t)).exp();
    (base_t, p)
}

/// Air density (kg/m³) at geometric altitude `h` (m) in the USSA 1976.
pub fn density(h: f64) -> f64 {
    let (t, p) = atmosphere(h);
    p / (287.05287 * t)
}

/// Static pressure as a fraction of sea level's, in the USSA 1976.
pub fn pressure_ratio(h: f64) -> f64 {
    atmosphere(h).1 / 101_325.0
}

/// Speed of sound (m/s) at geometric altitude `h` in the USSA 1976.
pub fn sound_speed(h: f64) -> f64 {
    let (t, _) = atmosphere(h);
    (1.4 * 287.05287 * t).sqrt()
}

/// The intact airframe's lift and drag coefficients (wave drag and the
/// damage's own drag are added in `Plane::step`). `flap_cl`: the lift the
/// flaps add as they are now; `stall`: how far into the stall (0..1); `ge`:
/// ground effect (1 clear of the ground, less near it). Also used by the
/// JSBSim comparison (jsbsim.rs), so the two always agree.
#[allow(clippy::too_many_arguments)]
pub fn lift_drag(spec: &Spec, alpha: f64, beta: f64, flap_cl: f64, stall: f64, flaps: f64, gear: f64, spoilers: f64, ge: f64) -> (f64, f64) {
    let cl = ((spec.cl0 + flap_cl + spec.cl_a * alpha) * (1.0 - stall) + (2.0 * alpha).sin() * 0.95 * stall) * (1.0 + 0.12 * (1.0 - ge)) - spec.spoiler_cl * spoilers;
    let cd = spec.cd0
        + spec.k_ind * cl * cl * ge
        + stall * 1.2 * alpha.sin().powi(2)
        + 0.6 * beta * beta
        + spec.cd_flap[0] * flaps
        + spec.cd_flap[1] * flaps * flaps
        + spec.cd_gear * gear
        + spec.spoiler_cd * spoilers;
    (cl, cd)
}

/// How far into the stall at this angle of attack (0..1), past `stall_a`.
pub fn stall_amount(stall_a: f64, alpha: f64) -> f64 {
    smooth(stall_a, stall_a + 0.13, alpha.abs())
}

fn smooth(a: f64, b: f64, x: f64) -> f64 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn smooth01(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::{b747, c172};
    use std::sync::Arc;

    const C172: &Spec = &c172::SPEC;
    const B747: &Spec = &b747::SPEC;
    const DR400: &Spec = &crate::aircraft::dr400::SPEC;

    #[test]
    fn wing_bending_is_sized_to_the_airframe_and_bounded() {
        for (spec, per_g) in [(C172, 0.010), (B747, 2.0)] {
            let s = spec.span as f64;
            let mut p = Plane::spawn(spec, 0.6, -2.1, 3000.0, 0.0, 120.0);
            p.on_ground = false;
            // Steady 2 g: the tips settle about one g's deflection above the 1 g shape.
            p.g_load = 2.0;
            for _ in 0..2400 {
                p.flex_wings(1.0 / 120.0, false);
            }
            let f = p.wing_flex[0];
            assert!((f / per_g - 1.0).abs() < 0.25, "{}: {f} m at 2 g, want about {per_g}", spec.id);
            assert!((p.wing_flex[1] - f).abs() < 1e-9);
            // A crash's load spikes and a long frame never bend it past a fifth of the half-span.
            for g in [80.0, -60.0, 1e6, f64::MAX] {
                p.g_load = g;
                for _ in 0..30 {
                    p.flex_wings(0.25, false);
                    assert!(p.wing_flex.iter().all(|x| x.is_finite() && x.abs() <= 0.2 * s + 1e-9), "{}: {:?}", spec.id, p.wing_flex);
                    assert!(p.wing_bend().iter().all(|b| (*b as f64 * s * s).abs() <= 0.2 * s + 1e-6));
                }
            }
            // On the ground it goes back to the modelled shape.
            p.on_ground = true;
            for _ in 0..2400 {
                p.flex_wings(1.0 / 120.0, false);
            }
            assert!(p.wing_flex.iter().all(|x| x.abs() < 1e-3));
        }
    }

    #[test]
    fn dents_scale_with_the_airframe() {
        let up = DVec3::Y;
        let depth = |spec: &'static Spec, closing: f64| {
            let mut p = Plane::spawn(spec, 0.6, -2.1, 3000.0, 0.0, 60.0);
            p.rot = DQuat::IDENTITY;
            p.dent(DVec3::new(0.0, -1.0, -2.0), DVec3::new(0.0, -closing, -20.0), up, closing);
            (p.dmg.crumple_dir, p.dmg.crumple[3])
        };
        let (d7, r7) = depth(C172, 7.0);
        assert!((0.05..0.12).contains(&d7[3]), "172 at 7 m/s: {} m", d7[3]);
        assert!((0.3..1.0).contains(&r7), "172 dent radius {r7} m");
        // Pushed in mostly up (into the belly), leaning back against the slide.
        assert!(d7[1] > 0.8 && d7[2] > 0.0, "{d7:?}");
        let (d12, _) = depth(C172, 12.0);
        assert!(d12[3] > d7[3] && d12[3] < 0.3);
        let (b6, rb) = depth(B747, 6.0);
        assert!((0.25..0.6).contains(&b6[3]), "747 at 6 m/s: {} m", b6[3]);
        assert!(b6[3] <= 0.25 * rb + 1e-6);
    }

    #[test]
    fn atmosphere_matches_ussa_1976_layer_bases() {
        // USSA 1976 layer base pressures (NASA/TM—2005-213659, Appendix A,
        // https://ntrs.nasa.gov/api/citations/20050207438/downloads/20050207438.pdf).
        // The table uses geopotential altitude; convert each to geometric
        // altitude because that's the coordinate used by the flight model.
        const EARTH_RADIUS: f64 = 6_356_766.0;
        let bases: [(f64, f64, f64); 7] = [
            (0.0, 101_325.0, 288.15),
            (11_000.0, 22_632.06, 216.65),
            (20_000.0, 5_474.89, 216.65),
            (32_000.0, 868.02, 228.65),
            (47_000.0, 110.91, 270.65),
            (51_000.0, 66.94, 270.65),
            (71_000.0, 3.96, 214.65),
        ];
        for (z, p, t) in bases {
            let h = EARTH_RADIUS * z / (EARTH_RADIUS - z);
            assert!((pressure_ratio(h) * 101_325.0 / p - 1.0).abs() < 0.002, "pressure at {z} m geopotential");
            assert!((sound_speed(h) / (1.4 * 287.05287 * t).sqrt() - 1.0).abs() < 0.0001, "temperature at {z} m geopotential");
            assert!(density(h).is_finite() && density(h) > 0.0);
        }
    }

    fn empty_world() -> World {
        let dir = std::env::temp_dir().join("fablegame-test-cache");
        let src = Arc::new(crate::sources::Sources::new(crate::fetch::Fetcher::new(dir, true), String::new()));
        World::new(crate::sources::Loader::new(src, 0), 14)
    }

    /// Fly for `secs` with a pitch law; returns (airspeed, vertical speed) averaged over the last 20 s.
    fn fly(p: &mut Plane, w: &World, secs: f64, throttle: f64, mut pitch: impl FnMut(&Plane) -> f64) -> (f64, f64) {
        let dt = 1.0 / 180.0;
        p.throttle = throttle;
        let (mut sv, mut svs, mut n) = (0.0, 0.0, 0.0);
        let steps = (secs / dt) as usize;
        for i in 0..steps {
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: pitch(p).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), ..Default::default() };
            assert!(p.step(dt, &c, w).is_none());
            if (steps - i) as f64 * dt < 20.0 {
                sv += p.airspeed().length();
                svs += p.vel.dot(p.pos.normalize());
                n += 1.0;
            }
        }
        (sv / n, svs / n)
    }

    const KT: f64 = 1.94384;
    fn rpm(p: &Plane) -> f64 {
        p.prop_rate * 60.0 / std::f64::consts::TAU
    }
    fn piston() -> &'static crate::aircraft::Piston {
        match &C172.engine {
            Engine::Piston(pe) => pe,
            _ => unreachable!(),
        }
    }
    /// An altitude hold for the test pilot (gentle, or it porpoises).
    fn hold_alt(alt0: f64) -> impl FnMut(&Plane) -> f64 {
        move |p| {
            let (_, _, h) = geo::pos_to_lla(p.pos);
            let vs = p.vel.dot(p.pos.normalize());
            (alt0 - h) * 0.002 - vs * 0.02
        }
    }

    #[test]
    fn performance_is_a_hornet() {
        let hornet: &Spec = &crate::aircraft::fa18::SPEC;
        let w = empty_world();
        let dt = 1.0 / 180.0;
        let level = |boost: bool| {
            let mut p = Plane::spawn(hornet, 0.6, -2.1, 3000.0, 0.0, 200.0);
            p.throttle = 1.0;
            let mut hold = hold_alt(3000.0);
            let mut gmax: f64 = 0.0;
            for i in 0..(180.0 / dt) as usize {
                // the stick does a lot at Mach 1: the test pilot goes gently
                let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost, ..Default::default() };
                assert!(p.step(dt, &c, &w).is_none());
                if i as f64 * dt > 60.0 {
                    gmax = gmax.max((p.g_load - 1.0).abs());
                }
            }
            println!("  boost {boost}: Mach {:.3}, worst load swing {gmax:.2} g", p.mach());
            assert!(gmax < 0.5, "porpoising: {gmax}");
            p.mach()
        };
        let (dry, wet) = (level(false), level(true));
        println!("level at 3000 m: Mach {dry:.2} dry, {wet:.2} in afterburner");
        assert!(dry > 0.85 && dry < 1.02 && wet > 1.1 && wet < 1.45, "{dry} {wet}");

        // A full pull at 450 kt: the fly-by-wire holds it near 7.5 g.
        let mut p = Plane::spawn(hornet, 0.6, -2.1, 3000.0, 0.0, 232.0);
        let mut gmax: f64 = 0.0;
        for _ in 0..(3.0 / dt) as usize {
            let c = Controls { pitch: 1.0, throttle: 1.0, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none());
            gmax = gmax.max(p.g_load);
        }
        println!("full pull at 450 kt: {gmax:.1} g");
        assert!(gmax > 6.0 && gmax < 8.5, "{gmax}");

        // On approach: gear and full flaps at 135 kt, the angle of attack it flies at.
        let mut p = Plane::spawn(hornet, 0.6, -2.1, 600.0, 0.0, 70.0);
        p.gear_down = true;
        p.gear = 1.0;
        p.flap_set = 2;
        p.flaps = 1.0;
        let mut hold = hold_alt(600.0);
        for _ in 0..(60.0 / dt) as usize {
            // auto-throttle on 135 kt
            let v = p.airspeed().length();
            let thr = (0.7 + (70.0 - v) * 0.08).clamp(0.0, 1.0);
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: hold(&p).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: thr, ..Default::default() };
            p.throttle = thr;
            assert!(p.step(dt, &c, &w).is_none());
        }
        println!("approach: {:.0} kt, alpha {:.1} deg, throttle {:.0}%", p.airspeed().length() * KT, p.alpha.to_degrees(), p.throttle * 100.0);
        assert!(p.alpha.to_degrees() > 5.0 && p.alpha.to_degrees() < 14.0 && !p.stalled);
    }

    /// The fastest Mach number at which full power still matches the drag in
    /// level flight at `alt`, with this share of its fuel aboard: where it
    /// ends up if the throttle is left there for as long as it takes.
    fn top_mach(spec: &'static Spec, alt: f64, boost: bool, fuel: f64) -> f64 {
        let Engine::Fan(fe) = &spec.engine else { panic!("a jet") };
        let mut p = Plane::spawn(spec, 0.6, -2.1, alt, 0.0, 200.0);
        p.fuel = spec.fuel_kg * fuel;
        let mut top = 0.0;
        for k in 0..1750 {
            let m = 0.5 + 0.002 * k as f64;
            let v = m * sound_speed(alt);
            if p.thrust(1.0, v, alt) * if boost { fe.boost } else { 1.0 } >= p.drag_level(v, alt, 0.0, 0.0) {
                top = m;
            }
        }
        top
    }

    #[test]
    fn performance_is_a_phantom() {
        let phantom: &Spec = &crate::aircraft::f4::SPEC;
        let w = empty_world();
        let dt = 1.0 / 180.0;
        // Flat out and level for `secs`: the Mach number it settles at.
        let level = |alt: f64, v0: f64, boost: bool, secs: f64| {
            let mut p = Plane::spawn(phantom, 0.6, -2.1, alt, 0.0, v0);
            // (full tanks: minutes of afterburner empty half-full ones)
            p.fuel = phantom.fuel_kg;
            p.throttle = 1.0;
            let mut hold = hold_alt(alt);
            let mut gmax: f64 = 0.0;
            for i in 0..(secs / dt) as usize {
                // the stick does a lot at speed: the test pilot goes gently
                let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost, ..Default::default() };
                assert!(p.step(dt, &c, &w).is_none());
                if i as f64 * dt > 60.0 {
                    gmax = gmax.max((p.g_load - 1.0).abs());
                }
            }
            println!("  {alt:.0} m, boost {boost}: Mach {:.3}, worst load swing {gmax:.2} g", p.mach());
            assert!(gmax < 0.5, "porpoising: {gmax}");
            p.mach()
        };
        // Down low it runs up against Mach 1 at military power and is not much past it in afterburner.
        let (dry, wet) = (level(3000.0, 200.0, false, 180.0), level(3000.0, 200.0, true, 180.0));
        println!("level at 3000 m: Mach {dry:.2} dry, {wet:.2} in afterburner");
        assert!(dry > 0.9 && dry < 1.01 && wet > 1.05 && wet < 1.35, "{dry} {wet}");
        // At 40,000 ft in afterburner, from Mach 1.6: past Mach 2 inside seven minutes (2.23, 1,280 kt, by the book).
        let top = level(12_192.0, 470.0, true, 400.0);
        // Left in afterburner there until the tanks are nearly dry it ends up at
        // its limit, not past it, where the airframe starts to shake. (Higher
        // still the thrust model keeps gaining on the drag, as the Hornet's
        // does: there the Mach limit's buffet is what tells the pilot to come back.)
        let limit = top_mach(phantom, 12_192.0, true, 0.1);
        println!("level at 40,000 ft in afterburner: Mach {top:.2}, settling at {limit:.2}");
        assert!(top > 1.95 && top <= limit && limit > phantom.mmo - 0.08 && limit <= phantom.mmo + 0.01, "{top} {limit}");

        // Climbing away in afterburner at 350 kt near sea level (the book's best is 41,300 ft/min, light).
        let mut p = Plane::spawn(phantom, 0.6, -2.1, 300.0, 0.0, 180.0);
        p.throttle = 1.0;
        let mut vs: f64 = 0.0;
        for _ in 0..(25.0 / dt) as usize {
            // hold the speed with the nose
            let v = p.airspeed().length();
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: ((v - 180.0) * 0.02 - p.omega.x * 1.0).clamp(-0.5, 0.5), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost: true, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none());
            vs = p.vel.dot(p.pos.normalize());
        }
        println!("climb in afterburner: {:.0} ft/min at {:.0} kt", vs * 196.85, p.airspeed().length() * KT);
        assert!(vs * 196.85 > 20_000.0 && vs * 196.85 < 45_000.0, "{}", vs * 196.85);

        // A full pull: at 450 kt the wing gives out before the airframe does,
        // but nothing holds the stick back, and at 600 kt down low the same
        // pull goes past what the airframe is good for (mind the g).
        let pull = |alt: f64, v: f64| {
            let mut p = Plane::spawn(phantom, 0.6, -2.1, alt, 0.0, v);
            let mut gmax: f64 = 0.0;
            for _ in 0..(3.0 / dt) as usize {
                let c = Controls { pitch: 1.0, throttle: 1.0, ..Default::default() };
                if p.step(dt, &c, &w).is_some() || gmax > phantom.g_break as f64 {
                    break;
                }
                gmax = gmax.max(p.g_load);
            }
            gmax
        };
        let (g450, g600) = (pull(3000.0, 232.0), pull(1000.0, 309.0));
        println!("full pull: {g450:.1} g at 450 kt, {g600:.1} g at 600 kt");
        assert!(g450 > 5.5 && g450 < phantom.g_limit as f64 && g600 > phantom.g_limit as f64, "{g450} {g600}");

        // On approach: gear and full flaps at 150 kt, the angle of attack it flies at.
        let mut p = Plane::spawn(phantom, 0.6, -2.1, 600.0, 0.0, 77.0);
        p.gear_down = true;
        p.gear = 1.0;
        p.flap_set = 2;
        p.flaps = 1.0;
        let mut hold = hold_alt(600.0);
        for _ in 0..(60.0 / dt) as usize {
            // auto-throttle on 150 kt
            let v = p.airspeed().length();
            let thr = (0.7 + (77.0 - v) * 0.08).clamp(0.0, 1.0);
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: hold(&p).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: thr, ..Default::default() };
            p.throttle = thr;
            assert!(p.step(dt, &c, &w).is_none());
        }
        println!("approach: {:.0} kt, alpha {:.1} deg, throttle {:.0}%", p.airspeed().length() * KT, p.alpha.to_degrees(), p.throttle * 100.0);
        assert!((p.airspeed().length() * KT - 150.0).abs() < 8.0 && p.alpha.to_degrees() > 5.0 && p.alpha.to_degrees() < 14.0 && !p.stalled);
    }

    #[test]
    fn performance_is_a_raptor() {
        let raptor: &Spec = &crate::aircraft::f22::SPEC;
        let w = empty_world();
        let dt = 1.0 / 180.0;
        // Flat out and level for `secs`: the Mach number it settles at.
        let level = |alt: f64, v0: f64, boost: bool, secs: f64| {
            let mut p = Plane::spawn(raptor, 0.6, -2.1, alt, 0.0, v0);
            // (full tanks: minutes of afterburner empty half-full ones)
            p.fuel = raptor.fuel_kg;
            p.throttle = 1.0;
            let mut hold = hold_alt(alt);
            let mut gmax: f64 = 0.0;
            for i in 0..(secs / dt) as usize {
                // the stick does a lot at speed: the test pilot goes gently
                let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost, ..Default::default() };
                assert!(p.step(dt, &c, &w).is_none());
                if i as f64 * dt > 60.0 {
                    gmax = gmax.max((p.g_load - 1.0).abs());
                }
            }
            println!("  {alt:.0} m, boost {boost}: Mach {:.3}, worst load swing {gmax:.2} g", p.mach());
            assert!(gmax < 0.5, "porpoising: {gmax}");
            p.mach()
        };
        // Down low it sits just past Mach 1 at military power (Mach 1.21, 800 kt, is its limit at sea level).
        let (dry, wet) = (level(3000.0, 220.0, false, 180.0), level(3000.0, 220.0, true, 180.0));
        println!("level at 3000 m: Mach {dry:.2} dry, {wet:.2} in afterburner");
        assert!(dry > 0.92 && dry < 1.15 && wet > 1.15 && wet < 1.6, "{dry} {wet}");
        // At 40,000 ft it supercruises: about Mach 1.5 without afterburner. (By
        // the book 1.76: with this thrust model more of it dry would take the
        // afterburners well past the airframe's Mach limit.)
        let cruise = level(12_192.0, 400.0, false, 300.0);
        println!("level at 40,000 ft at military power: Mach {cruise:.2}");
        assert!(cruise > 1.4 && cruise < 1.7, "{cruise}");
        // And in afterburner, from Mach 1.6: past Mach 2 inside three minutes,
        // settling at its limit of Mach 2.25, not past it. (Higher still the
        // thrust keeps gaining on the drag, as the Hornet's does: there the
        // Mach limit's buffet is what tells the pilot to come back.)
        let top = level(12_192.0, 470.0, true, 180.0);
        let limit = top_mach(raptor, 12_192.0, true, 0.1);
        println!("level at 40,000 ft in afterburner: Mach {top:.2}, settling at {limit:.2}");
        assert!(top > 2.0 && top <= limit && limit > raptor.mmo - 0.08 && limit <= raptor.mmo + 0.01, "{top} {limit}");

        // A full pull at 450 kt: the fly-by-wire holds it near 9 g.
        let mut p = Plane::spawn(raptor, 0.6, -2.1, 3000.0, 0.0, 232.0);
        let mut gmax: f64 = 0.0;
        for _ in 0..(3.0 / dt) as usize {
            let c = Controls { pitch: 1.0, throttle: 1.0, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none());
            gmax = gmax.max(p.g_load);
        }
        println!("full pull at 450 kt: {gmax:.1} g");
        assert!(gmax > 7.5 && gmax < 10.0, "{gmax}");

        // On approach: gear and full flaps at 145 kt, the angle of attack it flies at.
        let mut p = Plane::spawn(raptor, 0.6, -2.1, 600.0, 0.0, 75.0);
        p.gear_down = true;
        p.gear = 1.0;
        p.flap_set = 2;
        p.flaps = 1.0;
        let mut hold = hold_alt(600.0);
        for _ in 0..(60.0 / dt) as usize {
            // auto-throttle on 145 kt
            let v = p.airspeed().length();
            let thr = (0.5 + (75.0 - v) * 0.08).clamp(0.0, 1.0);
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: hold(&p).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: thr, ..Default::default() };
            p.throttle = thr;
            assert!(p.step(dt, &c, &w).is_none());
        }
        println!("approach: {:.0} kt, alpha {:.1} deg, throttle {:.0}%", p.airspeed().length() * KT, p.alpha.to_degrees(), p.throttle * 100.0);
        assert!((p.airspeed().length() * KT - 146.0).abs() < 8.0 && p.alpha.to_degrees() > 5.0 && p.alpha.to_degrees() < 14.0 && !p.stalled);
    }

    #[test]
    fn performance_is_a_blackbird() {
        let bird: &Spec = &crate::aircraft::sr71::SPEC;
        let dt = 1.0 / 180.0;
        // Taking off with a part load (it tanked up once airborne), in afterburner:
        // by the book it unsticks at 210 kt after some 4,500 ft.
        let w = flat_world();
        let mut p = Plane::spawn_ground(bird, 37.6, -122.37, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.fuel = 20_000.0;
        let start = p.pos;
        let (mut lifted, mut t) = (None, 0.0);
        while t < 60.0 {
            let v = p.vel.length();
            // Rotate at 190 kt to 10 degrees nose up and hold it there.
            let pitch_deg = p.forward().dot(p.pos.normalize()).asin().to_degrees();
            let pitch = if v * KT > 190.0 { ((10.0 - pitch_deg) * 0.15 - p.omega.x * 1.0).clamp(-1.0, 1.0) } else { 0.0 };
            let c = Controls { throttle: 1.0, boost: true, pitch, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            if lifted.is_none() && !p.on_ground && t > 5.0 {
                lifted = Some(((p.pos - start).length(), v, t));
            }
            t += dt;
        }
        let (dist, vlo, tlo) = lifted.expect("it never left the ground");
        let (_, _, h) = geo::pos_to_lla(p.pos);
        println!("take-off: unstuck after {dist:.0} m and {tlo:.0} s at {:.0} kt; a minute after brake release {h:.0} m up at {:.0} kt", vlo * KT, p.vel.length() * KT);
        assert!(dist > 1000.0 && dist < 2200.0 && vlo * KT > 185.0 && vlo * KT < 235.0, "{dist} {}", vlo * KT);
        assert!(h > 300.0 && p.dmg.gear.iter().all(|g| *g), "{h} {:?}", p.notes);

        let w = empty_world();
        // Climbing away in afterburner at its 450 kt indicated with the tanks
        // full, the 68.9 t the book's 11,820 ft/min is for: timed from 1,000 m
        // to 2,000 m (with what speed it gains or loses on the way counted as height).
        let mut p = Plane::spawn(bird, 0.6, -2.1, 200.0, 0.0, 232.0);
        p.fuel = bird.fuel_kg;
        p.throttle = 1.0;
        let (mut t, mut marks) = (0.0, Vec::new());
        while marks.len() < 2 && t < 200.0 {
            // hold the speed with the nose
            let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
            let roll_err = -p.right().dot(p.pos.normalize());
            let pitch = ((p.ias() * KT - 450.0) * 0.004 - p.vel.dot(p.pos.normalize()) * 0.0005) * 8.0 * gentle - p.omega.x * 1.0;
            let c = Controls { pitch: pitch.clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost: true, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none());
            t += dt;
            let (_, _, h) = geo::pos_to_lla(p.pos);
            if h > 1_000.0 + 1_000.0 * marks.len() as f64 {
                marks.push((t, p.ias() * KT, h + p.airspeed().length_squared() / (2.0 * 9.81)));
            }
        }
        let rate = (marks[1].2 - marks[0].2) / (marks[1].0 - marks[0].0) * 196.85;
        println!("climb in afterburner at {:.1} t: {rate:.0} ft/min at {:.0} to {:.0} kt indicated", p.mass() / 1000.0, marks[0].1, marks[1].1);
        assert!((rate / 11_820.0 - 1.0).abs() < 0.1 && (marks[0].1 - 450.0).abs() < 40.0 && (marks[1].1 - 450.0).abs() < 40.0, "{rate} {marks:?}");

        // Level at 80,000 ft from Mach 3 for `secs`: (the Mach number it settles at, the most it reached, kg/s burnt, true airspeed).
        let level = |boost: bool, secs: f64| {
            let alt = 24_384.0;
            let mut p = Plane::spawn(bird, 0.6, -2.1, alt, 0.0, 3.0 * sound_speed(alt));
            p.fuel = 20_000.0;
            p.throttle = 1.0;
            let mut hold = hold_alt(alt);
            let (mut gmax, mut top): (f64, f64) = (0.0, 0.0);
            for i in 0..(secs / dt) as usize {
                let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost, ..Default::default() };
                assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
                top = top.max(p.mach());
                if i as f64 * dt > 60.0 {
                    gmax = gmax.max((p.g_load - 1.0).abs());
                }
            }
            let (_, _, h) = geo::pos_to_lla(p.pos);
            println!("  80,000 ft, boost {boost}: Mach {:.3} (most {top:.3}) at {h:.0} m, {:.2} kg/s, {:.0} kt indicated, worst load swing {gmax:.2} g, notes {:?}", p.mach(), p.fuel_flow, p.ias() * KT, p.notes);
            assert!(gmax < 0.5, "porpoising: {gmax}");
            (p.mach(), top, p.fuel_flow, p.airspeed().length())
        };
        // In afterburner it settles at its cruise, Mach 3.2, and never reaches its limit (3.3) however long it is held.
        let (cruise, top, burn, v) = level(true, 600.0);
        assert!(cruise > 3.12 && cruise < 3.28 && top < bird.mmo - 0.01, "{cruise} {top}");
        // What it burns there: 36,000 lb an hour and less as it gets lighter,
        // by the book. (How far a tankful goes with the climb out of it, 2,824
        // nm by the book, is flown in `a_blackbird_lives_in_afterburner`.)
        println!("cruise: Mach {cruise:.2}, {:.0} lb/h, {:.1} nm for each tonne", burn * 7936.6, 1000.0 / burn * v / 1852.0);
        assert!(burn * 7936.6 > 28_000.0 && burn * 7936.6 < 42_000.0, "{burn}");
        // Without the afterburners it cannot stay there: it slows, and quickly.
        let (dry, ..) = level(false, 300.0);
        assert!(dry < 2.2, "{dry}");

        // The whole way up, tanks full at 30,000 ft and Mach 0.9: level to 450 kt
        // indicated, hold that with the nose up to Mach 3.15, then level off.
        let mut p = Plane::spawn(bird, 0.6, -2.1, 9_144.0, 0.0, 0.9 * sound_speed(9_144.0));
        p.fuel = bird.fuel_kg;
        p.throttle = 1.0;
        let mut hold = hold_alt(9_144.0);
        let (mut t, mut phase, mut done) = (0.0, 0, None);
        while t < 1800.0 && done.is_none() {
            let (_, _, h) = geo::pos_to_lla(p.pos);
            let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
            if phase == 0 && p.ias() * KT > 450.0 {
                phase = 1;
            }
            if phase == 1 && p.mach() > 3.15 {
                (phase, hold) = (2, hold_alt(h));
            }
            if phase == 2 && p.mach() > 3.19 {
                done = Some((t, h));
            }
            let vs = p.vel.dot(p.pos.normalize());
            let pitch = if phase == 1 { ((p.ias() * KT - 450.0) * 0.004 - vs * 0.0005) * 8.0 * gentle - p.omega.x * 1.0 } else { hold(&p) * gentle };
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: pitch.clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost: true, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            t += dt;
        }
        let (t_up, h_up) = done.expect("it never got to Mach 3.2");
        println!("30,000 ft and Mach 0.9 to Mach 3.2: {:.1} min, levelling at {:.0} ft with {:.1} t of its {:.1} t of fuel burnt, notes {:?}", t_up / 60.0, h_up * 3.28084, (bird.fuel_kg - p.fuel) / 1000.0, bird.fuel_kg / 1000.0, p.notes);
        assert!(t_up < 1800.0 && h_up > 21_000.0 && p.fuel > 0.5 * bird.fuel_kg && p.notes.is_empty(), "{t_up} {h_up} {}", p.fuel);

        // A full pull at 400 kt: there is not much stick to pull with (2.5 g by the book).
        let mut p = Plane::spawn(bird, 0.6, -2.1, 3000.0, 0.0, 206.0);
        let mut gmax: f64 = 0.0;
        for _ in 0..(4.0 / dt) as usize {
            let c = Controls { pitch: 1.0, throttle: 1.0, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none());
            gmax = gmax.max(p.g_load);
        }
        println!("full pull at 400 kt: {gmax:.1} g");
        assert!(gmax > 2.0 && gmax < bird.g_limit as f64 + 0.3, "{gmax}");

        // On approach, light, gear down at 175 kt: the angle of attack it flies at.
        let mut p = Plane::spawn(bird, 0.6, -2.1, 600.0, 0.0, 90.0);
        p.fuel = 5_000.0;
        p.gear_down = true;
        p.gear = 1.0;
        let mut hold = hold_alt(600.0);
        for _ in 0..(60.0 / dt) as usize {
            // auto-throttle on 175 kt
            let v = p.airspeed().length();
            let thr = (0.5 + (90.0 - v) * 0.08).clamp(0.0, 1.0);
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: hold(&p).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: thr, ..Default::default() };
            p.throttle = thr;
            assert!(p.step(dt, &c, &w).is_none());
        }
        println!("approach: {:.0} kt, alpha {:.1} deg, throttle {:.0}%", p.airspeed().length() * KT, p.alpha.to_degrees(), p.throttle * 100.0);
        assert!((p.airspeed().length() * KT - 175.0).abs() < 8.0 && p.alpha.to_degrees() > 7.0 && p.alpha.to_degrees() < 13.0 && !p.stalled);
    }

    /// The Blackbird at 80,000 ft and Mach 3 with the afterburners out, its
    /// military power at low level, and its range flown from brake release.
    #[test]
    fn a_blackbird_lives_in_afterburner() {
        let bird: &Spec = &crate::aircraft::sr71::SPEC;
        let w = empty_world();
        let dt = 1.0 / 120.0;
        let wings_level = |p: &Plane| (3.0 * p.right().dot(p.pos.normalize()) + p.omega.z * 0.5).clamp(-1.0, 1.0);
        // Out of afterburner at Mach 3 the engines' cores give next to nothing: it is through Mach 2.5 in two minutes.
        let alt = 24_384.0;
        let mut p = Plane::spawn(bird, 0.6, -2.1, alt, 0.0, 3.0 * sound_speed(alt));
        p.fuel = 20_000.0;
        p.throttle = 1.0;
        let mut hold = hold_alt(alt);
        let (mut t, mut t25) = (0.0, None);
        while t < 300.0 {
            let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
            let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: wings_level(&p), throttle: 1.0, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            t += dt;
            if t25.is_none() && p.mach() < 2.5 {
                t25 = Some(t);
            }
        }
        let t25 = t25.expect("it never slowed to Mach 2.5");
        println!("burners out at Mach 3 and 80,000 ft: Mach 2.5 after {t25:.0} s, Mach {:.2} after five minutes", p.mach());
        assert!(t25 < 125.0 && p.mach() < 2.0, "{t25} {}", p.mach());

        // Low down at military power (18,000 lbf an engine) it is still a lot
        // of aeroplane: left there it runs up to the speed of sound, far past
        // the 450 kt it is good for, as the real one would. Just over half
        // throttle holds 450 kt.
        let mut p = Plane::spawn(bird, 0.6, -2.1, 1000.0, 0.0, 200.0);
        p.throttle = 1.0;
        let mut hold = hold_alt(1000.0);
        let mut worst: f64 = 0.0;
        for _ in 0..(120.0 / dt) as usize {
            let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
            let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: wings_level(&p), throttle: 1.0, ..Default::default() };
            if p.step(dt, &c, &w).is_some() {
                break;
            }
            worst = worst.max(p.ias() * KT);
        }
        let v450 = 450.0 / KT * (RHO0 / density(1000.0)).sqrt();
        let thr = p.throttle_for(v450, 1000.0, 0.0, 0.0, 0.0);
        let Engine::Fan(fe) = &bird.engine else { panic!() };
        let spare = 1.0 - p.drag_level(v450, 1000.0, 0.0, 0.0) / p.thrust(1.0, v450, 1000.0);
        let tw = |k: f64| fe.n as f64 * fe.thrust * k / (68_946.0 * 9.81);
        println!("military power at 1,000 m: {worst:.0} kt indicated within two minutes; {:.0}% throttle holds 450 kt ({:.0}% of its military thrust to spare there); installed static thrust over gross weight {:.2} dry, {:.2} in afterburner", thr * 100.0, spare * 100.0, tw(1.0), tw(fe.boost));
        assert!(thr > 0.6 && thr < 0.8 && worst > 500.0, "{thr} {worst}");
        // (0.44 by the book, which takes the bare engine's 34,000 lbf)
        assert!((tw(fe.boost) - 0.44 * 25_500.0 / 34_000.0).abs() < 0.02, "{}", tw(fe.boost));

        // The whole flight on one load of fuel: off the runway with full
        // tanks (heavier than it usually left: it tanked up once airborne), up
        // at 400 kt to 30,000 ft, on to 450 kt, up at that to Mach 3.15, then
        // on up to 80,000 ft and level there until 2.5 t are left to come down
        // with. 2,824 nm by the book.
        let w = flat_world();
        let mut p = Plane::spawn_ground(bird, 37.6, -122.37, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.fuel = bird.fuel_kg;
        let start = p.pos.normalize();
        let (mut t, mut phase, mut marks) = (0.0, 0, Vec::new());
        let mut hold = hold_alt(9_144.0);
        let up_at = |p: &Plane, kt: f64, gentle: f64| ((p.ias() * KT - kt) * 0.004 - p.vel.dot(p.pos.normalize()) * 0.0005) * 8.0 * gentle - p.omega.x * 1.0;
        while p.fuel > 2_500.0 && t < 9_000.0 {
            let (_, _, h) = geo::pos_to_lla(p.pos);
            let gentle = (120.0 / p.airspeed().length().max(1.0)).powi(2).min(1.0);
            let pitch_deg = p.forward().dot(p.pos.normalize()).asin().to_degrees();
            let next = match phase {
                0 => !p.on_ground && h > 60.0,
                1 => p.ias() * KT > 400.0,
                2 => h > 9_144.0,
                3 => p.ias() * KT > 450.0,
                4 => p.mach() > 3.15,
                _ => false,
            };
            if next {
                marks.push((phase, t, h, (bird.fuel_kg - p.fuel) / 1000.0, p.pos.normalize().dot(start).clamp(-1.0, 1.0).acos() * 6_371.0 / 1.852));
                phase += 1;
                match phase {
                    1 => p.toggle_gear(),
                    3 => hold = hold_alt(9_144.0),
                    5 => hold = hold_alt(24_384.0),
                    _ => {}
                }
            }
            let pitch = match phase {
                // rotate at 215 kt to 11 degrees and hold it there
                0 | 1 => if p.vel.length() * KT > 215.0 { (11.0 - pitch_deg) * 0.15 - p.omega.x * 1.0 } else { 0.0 },
                2 => up_at(&p, 400.0, gentle),
                4 => up_at(&p, 450.0, gentle),
                _ => hold(&p) * gentle,
            };
            let c = Controls { pitch: pitch.clamp(-1.0, 1.0), roll: wings_level(&p), throttle: 1.0, boost: true, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "phase {phase} at {t:.0} s: {:?}", p.notes);
            assert!(p.notes.is_empty(), "phase {phase} at {t:.0} s: {:?}", p.notes);
            t += dt;
        }
        let range = p.pos.normalize().dot(start).clamp(-1.0, 1.0).acos() * 6_371.0 / 1.852;
        let (_, _, h) = geo::pos_to_lla(p.pos);
        for (ph, t, h, burnt, nm) in &marks {
            println!("  end of leg {ph}: {:.1} min, {:.0} ft, {burnt:.1} t burnt, {nm:.0} nm", t / 60.0, h * 3.28084);
        }
        println!("brake release to 2.5 t of fuel: {range:.0} nm in {:.0} min, ending at Mach {:.2} and {:.0} ft burning {:.0} lb/h", t / 60.0, p.mach(), h * 3.28084, p.fuel_flow * 7936.6);
        assert_eq!(phase, 5);
        assert!(range > 2_824.0 * 0.9 && range < 2_824.0 * 1.1, "{range}");
    }

    /// The Blackbird's braking parachute: what it does to the landing roll,
    /// and what becomes of it streamed too fast.
    #[test]
    fn a_blackbird_stops_with_its_parachute() {
        let bird: &Spec = &crate::aircraft::sr71::SPEC;
        let w = flat_world();
        let dt = 1.0 / 120.0;
        // Down at 155 kt, light, throttles closed and on the brakes: how far it rolls.
        let roll = |chute: bool| {
            let mut p = Plane::spawn_ground(bird, 37.6, -122.37, 0.0, 0.5, 0.0);
            p.hold = false;
            p.t = 2.0;
            p.fuel = 5_000.0;
            p.vel = p.forward() * (155.0 / KT);
            let start = p.pos;
            if chute {
                p.toggle_speedbrake();
            }
            let (mut t, mut out_for, mut pull) = (0.0, 0.0, 0.0f64);
            while p.vel.length() > 1.0 && t < 240.0 {
                let c = Controls { throttle: -1.0, brake: true, ..Default::default() };
                let v0 = p.vel.length();
                assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
                if p.spoilers > 0.99 {
                    out_for += dt;
                    if out_for < 1.0 {
                        pull = pull.max((v0 - p.vel.length()) / dt * p.mass());
                    }
                }
                t += dt;
            }
            assert!(p.notes.is_empty() && p.dmg.gear.iter().all(|g| *g), "{:?}", p.notes);
            // (it lets the parachute go by itself at 55 kt)
            assert!(p.spoilers == 0.0 && p.chute_spent == chute);
            ((p.pos - start).length(), t, pull)
        };
        let ((bare, t0, _), (with, t1, pull)) = (roll(false), roll(true));
        println!("landing roll from 155 kt: {bare:.0} m in {t0:.0} s on the brakes alone, {with:.0} m in {t1:.0} s with the parachute (it and the brakes hold it back with {:.0} kN at first)", pull / 1000.0);
        assert!(with < 0.62 * bare && with > 0.35 * bare && with > 600.0 && with < 1_500.0, "{bare} {with}");

        // Streamed in the air at 190 kt it holds, and slows it; the key again lets it go for good.
        let w = empty_world();
        let mut p = Plane::spawn(bird, 0.6, -2.1, 600.0, 0.0, 190.0 / KT);
        p.toggle_speedbrake();
        for _ in 0..(3.0 / dt) as usize {
            assert!(p.step(dt, &Controls::default(), &w).is_none());
        }
        assert!(p.spoilers > 0.99 && p.notes.is_empty() && p.airspeed().length() * KT < 180.0, "{} {:?}", p.airspeed().length() * KT, p.notes);
        p.toggle_speedbrake();
        p.toggle_speedbrake();
        assert!(p.spoilers == 0.0 && p.speedbrake == 0.0 && p.chute_spent);
        // At 300 kt it is torn away as it opens.
        let mut p = Plane::spawn(bird, 0.6, -2.1, 600.0, 0.0, 300.0 / KT);
        p.toggle_speedbrake();
        assert!(p.step(dt, &Controls::default(), &w).is_none());
        assert!(p.chute_spent && p.spoilers == 0.0 && p.notes == ["DRAG CHUTE TORN AWAY (OVERSPEED)"], "{:?}", p.notes);
        // No other type has one, or an engine that fades without its afterburners.
        for s in crate::aircraft::BUILTIN.iter().filter(|s| s.id != bird.id) {
            assert!(s.chute.is_none() && s.dry_mach == 0.0, "{}", s.name);
        }
    }

    /// The Blackbird's fins stand on its nacelles: scraping an inner elevon off
    /// (to the damage model, a stabiliser) must not take a fin with it.
    #[test]
    fn a_blackbird_keeps_its_fins() {
        use crate::damage::Sel;
        let bird: &Spec = &crate::aircraft::sr71::SPEC;
        let m = (bird.build)();
        let fins: Vec<_> = m.pieces.iter().filter(|pc| pc.name.ends_with(" fin")).collect();
        assert_eq!(fins.len(), 2);
        let takes = |p: &Plane, pc: &crate::airframe::Piece| p.shed.iter().any(|r| match r.sel {
            Sel::Group(g) | Sel::GroupAll(g) => pc.group == g,
            Sel::Role(role) => pc.role == role,
            Sel::Roles(rs) => rs.contains(&pc.role),
        });
        let mut p = Plane::spawn(bird, 0.6, -2.1, 1000.0, 0.0, 120.0);
        p.lose_elevator(0, true, "RUNWAY");
        p.lose_elevator(1, false, "RUNWAY");
        assert_eq!(p.notes.len(), 2);
        assert!(!p.shed.is_empty() && fins.iter().all(|f| !takes(&p, f)), "{:?}", p.shed);
        assert!(m.pieces.iter().any(|pc| pc.name == "left inner elevon" && takes(&p, pc)));
        // They are its rudders: they go as rudders do.
        p.shed.clear();
        p.lose_tail_top(0.0, 1.0, "BUILDING");
        assert!(fins.iter().all(|f| takes(&p, f)) && fins.iter().all(|f| p.dmg.hidden(f)));
    }

    /// The Blackbird flown with the keys alone (every input is a key held or
    /// not, looked at ten times a second): off the runway, round a turn at
    /// Mach 3, and down an approach.
    #[test]
    fn a_blackbird_is_flown_by_hand() {
        let bird: &Spec = &crate::aircraft::sr71::SPEC;
        let dt = 1.0 / 120.0;
        let up = |p: &Plane| p.pos.normalize();
        let pitch_deg = |p: &Plane| p.forward().dot(up(p)).asin().to_degrees();
        let bank_deg = |p: &Plane| (-p.right().dot(up(p))).asin().to_degrees();
        // A key: held while what it corrects (with a little lead from the rate) is outside the dead band.
        // (`omega.z` is positive rolling left, `omega.x` positive pitching up.)
        let key = |err: f64, band: f64| if err > band { 1.0 } else if err < -band { -1.0 } else { 0.0 };

        // Off the runway with 20 t of fuel: Shift to full power with Space held, S at 200 kt.
        let w = flat_world();
        let mut p = Plane::spawn_ground(bird, 37.6, -122.37, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.fuel = 20_000.0;
        let (mut t, mut c) = (0.0, Controls::default());
        let (mut pulled, mut rotated, mut off, mut worst): (Option<f64>, Option<f64>, Option<(f64, f64)>, (f64, f64, f64)) = (None, None, None, (0.0, 0.0, 0.0));
        let mut ticks = 0;
        while t < 90.0 {
            if ticks % 12 == 0 {
                let (pd, kt) = (pitch_deg(&p), p.vel.length() * KT);
                c = Controls { throttle: 1.0, boost: true, ..Default::default() };
                if kt > 200.0 {
                    pulled.get_or_insert(t);
                    // nose to 10 degrees, then to whatever holds 400 kt in the climb
                    let want = if p.on_ground || kt < 380.0 { 10.0 } else { (10.0 + (kt - 400.0) * 0.25).clamp(8.0, 30.0) };
                    c.pitch = key(want - pd - p.omega.x.to_degrees() * 0.8, 1.2);
                    if pd > 8.0 {
                        rotated.get_or_insert(t);
                    }
                }
                c.roll = key(-bank_deg(&p) + p.omega.z.to_degrees() * 0.5, 2.0);
                if off.is_some_and(|(t0, _)| t > t0 + 3.0) && p.gear_down {
                    p.toggle_gear();
                }
            }
            ticks += 1;
            assert!(p.step(dt, &c, &w).is_none(), "{t:.0} s: {:?}", p.notes);
            t += dt;
            if !p.on_ground && off.is_none() && t > 5.0 {
                off = Some((t, p.vel.length() * KT));
            }
            if off.is_some() {
                worst = (worst.0.max(p.alpha.to_degrees()), worst.1.max(p.g_load), worst.2.max(bank_deg(&p).abs()));
            }
        }
        let (t_off, v_off) = off.expect("it never left the ground");
        let (_, _, h) = geo::pos_to_lla(p.pos);
        let t_rot = rotated.unwrap() - pulled.unwrap();
        println!("by hand, take-off: S at 200 kt, nose at 8 deg {t_rot:.1} s later, off at {v_off:.0} kt after {t_off:.0} s; at 90 s {h:.0} m up at {:.0} kt, {:.0} deg nose up; worst alpha {:.1} deg, {:.2} g, {:.1} deg of bank; notes {:?}", p.vel.length() * KT, pitch_deg(&p), worst.0, worst.1, worst.2, p.notes);
        assert!(p.notes.is_empty() && (1.5..6.0).contains(&t_rot) && (195.0..235.0).contains(&v_off) && h > 1_000.0, "{t_rot} {v_off} {h}");
        assert!(worst.0 < 14.0 && worst.1 < 2.0 && worst.2 < 6.0 && !p.stalled && (p.vel.length() * KT - 400.0).abs() < 60.0, "{worst:?}");

        // How it answers the stick: full aileron for two seconds, a full pull for two, at 300 kt and at Mach 3.
        let w = empty_world();
        let answers = |alt: f64, v: f64| {
            let mut rate: f64 = 0.0;
            let mut p = Plane::spawn(bird, 0.6, -2.1, alt, 0.0, v);
            p.fuel = 20_000.0;
            for _ in 0..(2.0 / dt) as usize {
                assert!(p.step(dt, &Controls { roll: 1.0, boost: alt > 20_000.0, ..Default::default() }, &w).is_none());
                rate = rate.max(p.omega.z.abs().to_degrees());
            }
            let mut p = Plane::spawn(bird, 0.6, -2.1, alt, 0.0, v);
            p.fuel = 20_000.0;
            let mut g: f64 = 0.0;
            for _ in 0..(2.0 / dt) as usize {
                assert!(p.step(dt, &Controls { pitch: 1.0, boost: alt > 20_000.0, ..Default::default() }, &w).is_none(), "{:?}", p.notes);
                g = g.max(p.g_load);
            }
            (rate, g, p.notes.clone())
        };
        let (slow, fast) = (answers(3_000.0, 300.0 / KT * (RHO0 / density(3_000.0)).sqrt()), answers(24_384.0, 3.0 * sound_speed(24_384.0)));
        println!("by hand, full stick for 2 s: {:.0} deg/s of roll and {:.1} g at 300 kt; {:.0} deg/s and {:.1} g at Mach 3 and 80,000 ft {:?}", slow.0, slow.1, fast.0, fast.1, fast.2);
        assert!((15.0..45.0).contains(&slow.0) && (1.6..3.0).contains(&slow.1) && (10.0..40.0).contains(&fast.0) && (1.5..3.0).contains(&fast.1) && slow.2.is_empty() && fast.2.is_empty(), "{slow:?} {fast:?}");

        // Round a turn at Mach 3 and 80,000 ft in afterburner: D to 30 degrees of bank, S and W to hold the height.
        let alt = 24_384.0;
        let mut p = Plane::spawn(bird, 0.6, -2.1, alt, 0.0, 3.0 * sound_speed(alt));
        p.fuel = 20_000.0;
        p.throttle = 1.0;
        let (mut t, mut c, mut ticks) = (0.0, Controls::default(), 0);
        let (mut turned, mut last, mut worst, mut dh): (f64, DVec3, (f64, f64, f64), f64) = (0.0, p.forward(), (0.0, 0.0, 0.0), 0.0);
        while t < 180.0 {
            if ticks % 12 == 0 {
                let (_, _, h) = geo::pos_to_lla(p.pos);
                let vs = p.vel.dot(up(&p));
                c = Controls { boost: true, ..Default::default() };
                c.roll = key(30.0 - bank_deg(&p) + p.omega.z.to_degrees() * 0.6, 1.5);
                c.pitch = key((alt - h) * 0.02 - vs * 0.25 - p.omega.x.to_degrees() * 2.0, 1.0);
            }
            ticks += 1;
            assert!(p.step(dt, &c, &w).is_none(), "{t:.0} s: {:?}", p.notes);
            t += dt;
            if t > 30.0 {
                let (_, _, h) = geo::pos_to_lla(p.pos);
                let f = p.forward();
                let flat = |v: DVec3| (v - up(&p) * v.dot(up(&p))).normalize();
                turned += flat(f).dot(flat(last)).clamp(-1.0, 1.0).acos();
                worst = (worst.0.max(p.alpha.to_degrees()), worst.1.max(p.g_load), worst.2.max((bank_deg(&p) - 30.0).abs()));
                dh = dh.max((h - alt).abs());
            }
            last = p.forward();
        }
        let radius = p.airspeed().length() / (turned / 150.0) / 1852.0;
        println!("by hand, a 30 deg banked turn at Mach {:.2}: {:.0} deg in 150 s, {radius:.0} nm radius; height held within {dh:.0} m, bank within {:.1} deg, {:.2} g, alpha {:.1} deg; notes {:?}", p.mach(), turned.to_degrees(), worst.2, worst.1, worst.0, p.notes);
        assert!(p.notes.is_empty() && (60.0..110.0).contains(&radius) && dh < 500.0 && worst.2 < 6.0 && worst.1 < 1.6 && worst.0 < 9.0, "{radius} {dh} {worst:?}");

        // Down an approach: gear down, light, Shift and Ctrl for 175 kt, S and W for three degrees down.
        let mut p = Plane::spawn(bird, 0.6, -2.1, 900.0, 0.0, 200.0 / KT);
        p.fuel = 5_000.0;
        p.configure(0, true, 0.0);
        let (mut t, mut c, mut ticks) = (0.0, Controls::default(), 0);
        let mut worst: (f64, f64, f64, f64) = (0.0, 0.0, 0.0, 0.0);
        while t < 120.0 {
            if ticks % 12 == 0 {
                let (v, vs) = (p.airspeed().length(), p.vel.dot(up(&p)));
                c = Controls::default();
                c.throttle = key((175.0 / KT - v) * 0.5 + (0.45 - p.throttle) * 4.0, 0.5);
                c.pitch = key((-v * 3f64.to_radians().sin() - vs) * 0.6 - p.omega.x.to_degrees() * 1.5, 0.8);
                c.roll = key(-bank_deg(&p) + p.omega.z.to_degrees() * 0.5, 2.0);
            }
            ticks += 1;
            assert!(p.step(dt, &c, &w).is_none(), "{t:.0} s: {:?}", p.notes);
            t += dt;
            if t > 40.0 {
                let glide = (-p.vel.dot(up(&p)) / p.airspeed().length()).asin().to_degrees();
                worst = (worst.0.max((p.airspeed().length() * KT - 175.0).abs()), worst.1.max(p.alpha.to_degrees()), worst.2.max((glide - 3.0).abs()), worst.3.max(bank_deg(&p).abs()));
            }
            assert!(!p.stalled);
        }
        println!("by hand, approach: within {:.0} kt of 175, alpha to {:.1} deg, within {:.1} deg of a 3 deg glide, {:.1} deg of bank; throttle {:.0}%, {:.0} deg nose up", worst.0, worst.1, worst.2, worst.3, p.throttle * 100.0, pitch_deg(&p));
        assert!(worst.0 < 12.0 && worst.1 < 13.0 && worst.2 < 1.5 && worst.3 < 4.0, "{worst:?}");
    }

    #[test]
    fn performance_is_a_concorde() {
        let cc: &Spec = &crate::aircraft::concorde::SPEC;
        let fe = match &cc.engine { Engine::Fan(fe) => fe, _ => unreachable!() };
        let dt = 1.0 / 120.0;

        // Take-off at full weight (184 t), nose at 5 degrees, reheat lit: rotate at
        // 199 kt to 10.5 degrees (its nozzles and tail wheel touch at 11.5), and
        // on to the book's 13 once it is off.
        let w = flat_world();
        let mut p = Plane::spawn_ground(cc, 37.6, -122.37, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.fuel = cc.fuel_kg;
        p.flap_set = 2;
        p.flaps = cc.flap_frac(2);
        let (mut lifted, mut t) = (None, 0.0);
        let start = p.pos;
        let (mut want, mut last, mut most): (f64, f64, f64) = (0.0, 0.0, 0.0);
        while t < 90.0 {
            let v = p.vel.length();
            let pitch_deg = (p.forward().dot(p.pos.normalize())).asin().to_degrees();
            let (_, _, hh) = geo::pos_to_lla(p.pos);
            // (the nose comes up at 2.5 degrees a second, to 10.5 until the nozzles are well clear of the runway)
            if v * KT > 199.0 {
                want = (want + 2.5 * dt).min(if hh > p.wheel_height() + 6.0 { 13.0 } else { 10.5 });
            }
            // (nose and visor up again by 250 kt: they are not built for more than 325)
            if v * KT > 250.0 {
                p.flap_set = 0;
            }
            let rate = (pitch_deg - last) / dt;
            last = pitch_deg;
            if p.on_ground {
                most = most.max(pitch_deg);
            }
            let pitch = if v * KT > 199.0 { ((want - pitch_deg) * 0.25 - rate * 0.08).clamp(-1.0, 1.0) } else { 0.0 };
            let c = Controls { throttle: 1.0, boost: true, pitch, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            if lifted.is_none() && !p.on_ground && t > 5.0 {
                lifted = Some(((p.pos - start).length(), v, t, pitch_deg));
                p.toggle_gear();
            }
            t += dt;
        }
        let (dist, vlo, tlo, pitch_lo) = lifted.expect("it never left the ground");
        let (_, _, h) = geo::pos_to_lla(p.pos);
        println!("take-off at {:.0} t: off after {dist:.0} m and {tlo:.0} s at {:.0} kt, {pitch_lo:.1} deg nose up (at most {most:.1} on the runway); at 90 s: {h:.0} m, {:.0} kt", p.mass() / 1000.0, vlo * KT, p.vel.length() * KT);
        // (The book: about 217 kt, on a 3,600 m runway.)
        assert!(dist > 1800.0 && dist < 3600.0, "{dist}");
        assert!(vlo * KT > 207.0 && vlo * KT < 228.0, "{}", vlo * KT);
        // (the nozzles' lips and the tail wheel meet the runway at 11.5 degrees)
        assert!(most > 9.0 && most < 11.2, "{most}");
        assert!(h > 300.0, "{h}");
        assert!(p.notes.is_empty() && p.dmg.gear.iter().all(|g| *g), "{:?}", p.notes);

        // Level and flat out for `secs`, from Mach `m0`: the Mach number and fuel flow it ends at.
        let w = empty_world();
        let dt = 1.0 / 180.0;
        let level = |alt: f64, m0: f64, boost: bool, secs: f64| {
            let mut p = Plane::spawn(cc, 0.6, -2.1, alt, 0.0, m0 * sound_speed(alt));
            p.throttle = 1.0;
            let mut hold = hold_alt(alt);
            let mut gmax: f64 = 0.0;
            for i in 0..(secs / dt) as usize {
                // the stick does a lot at speed: the test pilot goes gently
                let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost, ..Default::default() };
                assert!(p.step(dt, &c, &w).is_none());
                if i as f64 * dt > 60.0 {
                    gmax = gmax.max((p.g_load - 1.0).abs());
                }
            }
            println!("  {alt:.0} m, reheat {boost}: Mach {:.3}, {:.2} kg/s, lift over drag {:.1}, worst load swing {gmax:.2} g, notes {:?}", p.mach(), p.fuel_flow, p.mass() * 9.81 / p.drag_level(p.airspeed().length(), alt, 0.0, 0.0), p.notes);
            assert!(gmax < 0.5, "porpoising: {gmax}");
            assert!(p.notes.is_empty(), "{:?}", p.notes);
            (p.mach(), p.fuel_flow)
        };
        // The cruise: Mach 2 at 55,000 ft without reheat, where it settles (run to equilibrium), not past its Mach 2.04.
        let (cruise, burn) = level(16_764.0, 1.9, false, 600.0);
        let again = level(16_764.0, 2.1, false, 600.0).0;
        println!("level at 55,000 ft dry: Mach {cruise:.3} from below, {again:.3} from above, burning {:.1} t/h", burn * 3.6);
        assert!((cruise - again).abs() < 0.02, "not settled: {cruise} {again}");
        assert!(cruise > 1.95 && cruise < cc.mmo + 0.02, "{cruise}");
        // What that burn would take it on all its fuel (the book, with the climb and reserves out of it: 3,900 nmi at 1,165 kt).
        let range = cc.fuel_kg / burn * cruise * sound_speed(16_764.0) / 1852.0;
        println!("cruise at {:.0} kt; all its fuel at that rate: {range:.0} nmi", cruise * sound_speed(16_764.0) * KT);
        assert!(range > 3900.0 && range < 5800.0, "{range}");
        // With reheat held on up there its intakes let it gain a few hundredths and no more: well short of where it would shake itself apart.
        let hot = level(16_764.0, 2.0, true, 600.0).0;
        let hot2 = level(16_764.0, 2.12, true, 600.0).0;
        let shake = cc.mmo + (cc.flutter_v - cc.vne) / 300.0;
        println!("level at 55,000 ft with reheat: Mach {hot:.3} from below, {hot2:.3} from above (flutter from Mach {shake:.2})");
        assert!(hot > cruise && (hot - hot2).abs() < 0.02 && hot.max(hot2) < cc.mmo + 0.03 && hot.max(hot2) < shake, "{hot} {hot2}");
        // Through the sound barrier at 40,000 ft, Mach 0.95 to 1.7: minutes of it, and far fewer with reheat (what it is for).
        let through = |boost: bool| {
            let alt = 12_192.0;
            let mut p = Plane::spawn(cc, 0.6, -2.1, alt, 0.0, 0.95 * sound_speed(alt));
            p.throttle = 1.0;
            let mut hold = hold_alt(alt);
            let mut t = 0.0;
            while p.mach() < 1.7 && t < 900.0 {
                let gentle = (120.0 / p.airspeed().length()).powi(2).min(1.0);
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { pitch: (hold(&p) * gentle).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: 1.0, boost, ..Default::default() };
                assert!(p.step(dt, &c, &w).is_none());
                t += dt;
            }
            t
        };
        let (t_dry, t_hot) = (through(false), through(true));
        println!("Mach 0.95 to 1.7 at 40,000 ft: {t_dry:.0} s dry, {t_hot:.0} s with reheat");
        assert!(t_hot < 0.7 * t_dry && t_hot > 150.0 && t_hot < 420.0, "{t_dry} {t_hot}");
        // Static thrust for its weight at take-off, with reheat: 0.373 by the book.
        let tw = fe.n as f64 * fe.thrust * fe.boost / (185_070.0 * 9.81);
        assert!((tw - 0.373).abs() < 0.01, "{tw}");

        // On approach at landing weight (108 t): gear down, nose at 12.5 degrees, 160 kt, and how nose-high it flies.
        let mut p = Plane::spawn(cc, 0.6, -2.1, 600.0, 0.0, 82.3);
        p.fuel = 0.2 * cc.fuel_kg;
        p.gear_down = true;
        p.gear = 1.0;
        p.flap_set = 3;
        p.flaps = 1.0;
        let mut hold = hold_alt(600.0);
        for _ in 0..(90.0 / dt) as usize {
            // auto-throttle on 160 kt
            let v = p.airspeed().length();
            let thr = (0.5 + (82.3 - v) * 0.08).clamp(0.0, 1.0);
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { pitch: hold(&p).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), throttle: thr, ..Default::default() };
            p.throttle = thr;
            assert!(p.step(dt, &c, &w).is_none());
        }
        let ld = p.mass() * 9.81 / p.drag_level(p.airspeed().length(), 600.0, p.flaps, 1.0);
        println!("approach at {:.0} t: {:.0} kt, alpha {:.1} deg, throttle {:.0}%, lift over drag {ld:.1}", p.mass() / 1000.0, p.airspeed().length() * KT, p.alpha.to_degrees(), p.throttle * 100.0);
        assert!((p.airspeed().length() * KT - 160.0).abs() < 8.0 && p.alpha.to_degrees() > 8.0 && p.alpha.to_degrees() < 14.0 && !p.stalled && p.notes.is_empty());
        assert!(ld > 3.5 && ld < 6.5, "{ld}");
    }

    #[test]
    fn performance_is_a_cessna_172() {
        let w = empty_world();
        // Flat out and level near sea level: about 125 kt with the gear up, a
        // few knots less with it down (a fixed-gear 172 does 126 kt).
        let mut p = Plane::spawn(C172, 0.6, -2.1, 300.0, 0.0, 55.0);
        let (v, _) = fly(&mut p, &w, 240.0, 1.0, hold_alt(300.0));
        println!("max level speed {:.1} m/s = {:.0} kt at {:.0} rpm", v, v * KT, rpm(&p));
        assert!(v * KT > 118.0 && v * KT < 132.0, "{}", v * KT);
        assert!(rpm(&p) > 2550.0 && rpm(&p) < 2800.0, "{}", rpm(&p));
        let mut p = Plane::spawn(C172, 0.6, -2.1, 300.0, 0.0, 55.0);
        p.configure(0, true, 0.0);
        let (vg, _) = fly(&mut p, &w, 240.0, 1.0, hold_alt(300.0));
        println!("gear down: {:.0} kt", vg * KT);
        assert!(vg < v - 1.0 && vg * KT > 112.0, "{}", vg * KT);

        // 75% power at 8000 ft: about 120 kt true.
        let mut p = Plane::spawn(C172, 0.6, -2.1, 2438.0, 0.0, 58.0);
        let mut thr = 0.8;
        for _ in 0..40 {
            fly(&mut p, &w, 5.0, thr, hold_alt(2438.0));
            thr = (thr + (0.75 - p.power) * 0.5).clamp(0.0, 1.0);
        }
        let (v, _) = fly(&mut p, &w, 120.0, thr, hold_alt(2438.0));
        println!("75% at 8000 ft: {:.0} kt true, {:.0} kt indicated, {:.0} rpm, power {:.2}", v * KT, p.ias() * KT, rpm(&p), p.power);
        assert!(v * KT > 110.0 && v * KT < 128.0, "{}", v * KT);

        // Full power at the best-rate speed (74 kt): 700-900 fpm.
        let mut p = Plane::spawn(C172, 0.6, -2.1, 300.0, 0.0, 40.0);
        let (v, vs) = fly(&mut p, &w, 90.0, 1.0, |p| (p.airspeed().length() - 38.0) * 0.04);
        println!("climb at {:.0} kt: {:.0} fpm, {:.0} rpm", v * KT, vs * 196.85, rpm(&p));
        assert!(vs * 196.85 > 650.0 && vs * 196.85 < 950.0, "{}", vs * 196.85);

        // Engine at idle at the best glide speed (68 kt): about 9 to 1.
        let mut p = Plane::spawn(C172, 0.6, -2.1, 1500.0, 0.0, 35.0);
        let (v, vs) = fly(&mut p, &w, 90.0, 0.0, |p| (p.airspeed().length() - 35.0) * 0.04);
        let glide = (v * v - vs * vs).sqrt() / -vs;
        println!("idle glide at {:.0} kt: {:.0} fpm, {:.1}:1, {:.0} rpm", v * KT, vs * 196.85, glide, rpm(&p));
        assert!(glide > 7.5 && glide < 11.0, "{glide}");
        assert!(rpm(&p) > 700.0 && rpm(&p) < 1600.0, "{}", rpm(&p));

        // Stall: about 50 kt clean, 42 kt with full flaps.
        let s = C172;
        let vs_at = |flap_cl: f64, a: f64| (2.0 * s.mass * 9.81 / (RHO0 * s.wing_area * (s.cl0 + flap_cl + s.cl_a * a))).sqrt() * KT;
        let (clean, full) = (vs_at(0.0, 0.27), vs_at(0.55, 0.23));
        println!("stall {clean:.0} kt clean, {full:.0} kt full flaps");
        assert!(clean > 46.0 && clean < 54.0 && full > 39.0 && full < 46.0);

        // On the ground: about 2300 rpm flat out, 600-800 at idle, static thrust a
        // quarter of the weight.
        let pe = piston();
        let ws = Plane::prop_equilibrium(pe, 1.0, 0.0, RHO0) * 60.0 / std::f64::consts::TAU;
        let wi = Plane::prop_equilibrium(pe, 0.0, 0.0, RHO0) * 60.0 / std::f64::consts::TAU;
        let t0 = Plane::spawn(C172, 0.6, -2.1, 0.0, 0.0, 0.0).thrust(1.0, 0.0, 0.0);
        println!("static {ws:.0} rpm, idle {wi:.0} rpm, thrust {t0:.0} N = {:.2} W", t0 / (s.mass * 9.81));
        assert!(ws > 2200.0 && ws < 2450.0 && wi > 550.0 && wi < 850.0, "{ws} {wi}");
        assert!(t0 / (s.mass * 9.81) > 0.18 && t0 / (s.mass * 9.81) < 0.28, "{t0}");
        // Diving at Vne at full throttle the fixed-pitch propeller over-revs.
        let wd = Plane::prop_equilibrium(pe, 1.0, s.vne, RHO0) * 60.0 / std::f64::consts::TAU;
        println!("full throttle at Vne: {wd:.0} rpm");
        assert!(wd > 2850.0, "{wd}");
    }

    #[test]
    fn performance_is_a_robin_dr400() {
        let w = empty_world();
        let s = DR400;
        // Flat out and level near sea level: about 150 kt (278 km/h).
        let mut p = Plane::spawn(s, 0.6, -2.1, 300.0, 0.0, 70.0);
        assert!(p.gear_down && p.gear == 1.0, "fixed gear starts down");
        p.toggle_gear();
        assert!(p.gear_down, "and stays down");
        let (v, _) = fly(&mut p, &w, 240.0, 1.0, hold_alt(300.0));
        println!("DR 400 max level speed {:.0} kt at {:.0} rpm", v * KT, rpm(&p));
        assert!(v * KT > 140.0 && v * KT < 158.0, "{}", v * KT);
        assert!(rpm(&p) > 2550.0 && rpm(&p) < 2850.0, "{}", rpm(&p));

        // 75% power at 6000 ft: about 135 kt true (250 km/h).
        let mut p = Plane::spawn(s, 0.6, -2.1, 1830.0, 0.0, 66.0);
        let mut thr = 0.8;
        for _ in 0..40 {
            fly(&mut p, &w, 5.0, thr, hold_alt(1830.0));
            thr = (thr + (0.75 - p.power) * 0.5).clamp(0.0, 1.0);
        }
        let (v, _) = fly(&mut p, &w, 120.0, thr, hold_alt(1830.0));
        println!("DR 400 75% at 6000 ft: {:.0} kt true, {:.0} rpm", v * KT, rpm(&p));
        assert!(v * KT > 122.0 && v * KT < 142.0, "{}", v * KT);

        // Full power at the best-rate speed (75 kt): 850-1100 fpm, lighter than the book's 1100 kg.
        let mut p = Plane::spawn(s, 0.6, -2.1, 300.0, 0.0, 40.0);
        let (v, vs) = fly(&mut p, &w, 90.0, 1.0, |p| (p.airspeed().length() - 38.5) * 0.04);
        println!("DR 400 climb at {:.0} kt: {:.0} fpm", v * KT, vs * 196.85);
        assert!(vs * 196.85 > 800.0 && vs * 196.85 < 1200.0, "{}", vs * 196.85);

        // Idle glide at 70 kt: about 9 or 10 to 1.
        let mut p = Plane::spawn(s, 0.6, -2.1, 1500.0, 0.0, 36.0);
        let (v, vs) = fly(&mut p, &w, 90.0, 0.0, |p| (p.airspeed().length() - 36.0) * 0.04);
        let glide = (v * v - vs * vs).sqrt() / -vs;
        println!("DR 400 idle glide at {:.0} kt: {:.1}:1", v * KT, glide);
        assert!(glide > 7.5 && glide < 11.5, "{glide}");

        // Stall at 900 kg: about 50 kt clean, 44 kt with full flaps (the book: 55 and 50 at 1100 kg).
        let vs_at = |flap_cl: f64, a: f64| (2.0 * s.mass * 9.81 / (RHO0 * s.wing_area * (s.cl0 + flap_cl + s.cl_a * a))).sqrt() * KT;
        let (clean, full) = (vs_at(0.0, s.stall_alpha), vs_at(s.flap_cl, s.stall_alpha - s.stall_flap));
        println!("DR 400 stall {clean:.0} kt clean, {full:.0} kt full flaps");
        assert!(clean > 46.0 && clean < 54.0 && full > 40.0 && full < 47.0);

        // As the game sets up a final: 66 kt, full flaps, a 3 degree glide, hands off.
        let (va, notch) = s.approach;
        let mut p = Plane::spawn(s, 0.6, -2.1, 400.0, 0.0, va);
        let sink = va * 3f64.to_radians().sin();
        p.configure(notch, true, sink);
        let thr = p.throttle;
        let (v, vs) = fly(&mut p, &w, 30.0, thr, |_| 0.0);
        println!("DR 400 approach: throttle {thr:.2}, {:.0} kt, {:.0} fpm", v * KT, vs * 196.85);
        assert!((v - va).abs() < 3.0, "{v}");
        assert!((vs + sink).abs() < 1.0, "{vs}");
    }

    #[test]
    fn a_dr400_canopy_slides_open_and_drags() {
        let w = empty_world();
        let model = (DR400.build)();
        let k = model.pieces.iter().position(|p| p.name == "canopy").unwrap();
        assert!(model.pieces[k].anim.uses(Input::Canopy));
        let mut p = Plane::spawn(DR400, 0.6, -2.1, 300.0, 0.0, 50.0);
        let shut = p.pose(&model.pieces)[k];
        p.toggle_canopy();
        let (v_open, _) = fly(&mut p, &w, 60.0, 0.75, hold_alt(300.0));
        assert_eq!(p.canopy, 1.0);
        let moved = p.pose(&model.pieces)[k].transform_point3(Vec3::ZERO) - shut.transform_point3(Vec3::ZERO);
        println!("DR 400 canopy open: moved {moved:?}, {:.0} kt", v_open * KT);
        assert!(moved.z < -0.5 && moved.y > 0.05, "{moved:?}");
        p.toggle_canopy();
        let (v_shut, _) = fly(&mut p, &w, 60.0, 0.75, hold_alt(300.0));
        assert_eq!(p.canopy, 0.0);
        assert!(v_shut > v_open + 1.0, "{v_shut} {v_open}");
    }

    #[test]
    fn a_dr400_takes_off_and_climbs() {
        let w = flat_world();
        let s = DR400;
        let mut p = Plane::spawn_ground(s, 37.6, -122.37, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.flap_set = 1;
        p.flaps = s.flap_frac(1);
        let dt = 1.0 / 180.0;
        let (mut lifted, mut t) = (None, 0.0);
        let start = p.pos;
        while t < 60.0 {
            let v = p.vel.length();
            let pitch_deg = (p.forward().dot(p.pos.normalize())).asin().to_degrees();
            // Rotate at 50 kt, then hold 9 degrees.
            let pitch = if v * KT > 50.0 { ((9.0 - pitch_deg) * 0.15).clamp(-1.0, 1.0) } else { 0.0 };
            let c = Controls { throttle: 1.0, pitch, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            if lifted.is_none() && !p.on_ground && t > 3.0 {
                lifted = Some(((p.pos - start).length(), v));
            }
            t += dt;
        }
        let (dist, vlo) = lifted.expect("it never left the ground");
        let (_, _, h) = geo::pos_to_lla(p.pos);
        println!("DR 400 lift-off after {dist:.0} m at {:.0} kt; at 60 s: {h:.0} m, {:.0} kt", vlo * KT, p.vel.length() * KT);
        assert!(dist > 120.0 && dist < 300.0 && vlo * KT < 62.0, "{dist} {}", vlo * KT);
        assert!(h > 150.0, "{h}");
        assert!(p.notes.is_empty() && p.dmg.gear.iter().all(|g| *g), "{:?}", p.notes);
    }

    #[test]
    fn piston_fuel_selectors_cut_and_restore_flow() {
        use FuelSelector::*;
        let world = flat_world();
        for spec in [C172, DR400] {
            let mut plane = Plane::spawn(spec, 0.6, -2.1, 1000.0, 0.0, 55.0);
            for expected in [Right, Off, Left, Center] {
                let previous = plane.fuel_selector;
                plane.fuel_selector = previous.turn(true);
                assert_eq!(plane.fuel_selector, expected);
                assert_eq!(expected.turn(false), previous);
                plane.step(1.0 / 180.0, &Controls::default(), &world);
                assert_eq!(plane.running, expected != Off);
            }
        }
    }

    #[test]
    fn cessna_actuators_wait_for_electrical_power() {
        let world = flat_world();
        let mut p = Plane::spawn(C172, 0.6, -2.1, 1000.0, 0.0, 55.0);
        p.gear_down = true;
        p.flap_set = 2;
        p.sys.elec.master = false;
        for _ in 0..180 { p.step(1.0/180.0, &Controls::default(), &world); }
        assert_eq!(p.gear, 0.0);
        assert_eq!(p.flaps, 0.0);
        p.sys.elec.master = true;
        for _ in 0..180 { p.step(1.0/180.0, &Controls::default(), &world); }
        assert!(p.gear > 0.1 && p.flaps > 0.1);
    }

    #[test]
    fn toe_brakes_are_independent_and_parking_brake_overrides_both() {
        let world = flat_world();
        let mut p = Plane::spawn(C172, 0.6, -2.1, 1000.0, 0.0, 55.0);
        p.step(1.0/180.0, &Controls { brake_left: 0.7, ..Default::default() }, &world);
        assert!((p.input(Input::Brake(-1.0))-0.7).abs()<1e-5);
        assert_eq!(p.input(Input::Brake(1.0)),0.0);
        p.parking_brake = true;
        p.step(1.0/180.0, &Controls::default(), &world);
        assert_eq!(p.toe_brake,[1.0,1.0]);
        assert!(!C172.gear[0].brakes);
    }

    #[test]
    fn prop_control_reduces_rpm_without_cutting_ignition() {
        let world = flat_world();
        let mut p = Plane::spawn(C172, 0.6, -2.1, 1000.0, 0.0, 55.0);
        let mut fast = p.prop_rate;
        p.propeller = 0.2;
        for _ in 0..900 {
            p.step(1.0/180.0, &Controls { throttle_set: Some(0.75), ..Default::default() }, &world);
            fast = fast.max(p.prop_rate);
        }
        assert!(p.running && p.prop_rate < fast * 0.9, "{} / {}",p.prop_rate,fast);
    }

    #[test]
    fn engine_starts_runs_and_stops() {
        let w = flat_world();
        let dt = 1.0 / 180.0;
        let run = |p: &mut Plane, secs: f64, c: &Controls| {
            for _ in 0..(secs / dt) as usize {
                assert!(p.step(dt, c, &w).is_none());
            }
        };
        // On the ground, cold and dark: nothing turns, nothing burns.
        let mut p = Plane::spawn_ground(C172, 37.8, -122.46, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        let full = p.fuel;
        p.cold_and_dark();
        let idle = Controls { brake: true, ..Default::default() };
        run(&mut p, 2.0, &idle);
        assert!(!p.running && p.prop_rate < 1.0 && p.fuel == full);
        // The starter on a dead bus (master off): nothing. With the master on
        // but the magnetos off: it cranks (~200 rpm) but does not fire.
        let crank = Controls { starter: true, ..idle };
        run(&mut p, 1.0, &crank);
        assert!(rpm(&p) < 5.0);
        p.sys.elec.master = true;
        run(&mut p, 2.0, &crank);
        assert!(!p.running && rpm(&p) > 120.0 && rpm(&p) < 300.0, "{}", rpm(&p));
        // The proper start: it fires and settles at idle on its own.
        p.start_engine();
        run(&mut p, 10.0, &idle);
        println!("idle {:.0} rpm, {:.1} l/h", rpm(&p), p.fuel_flow * 3600.0 / 0.72);
        assert!(p.running && rpm(&p) > 500.0 && rpm(&p) < 1100.0, "{}", rpm(&p));
        assert!(p.fuel < full);
        // Mixture to cut-off: it dies at once and the propeller stops within seconds.
        p.stop_engine();
        run(&mut p, 1.0, &idle);
        assert!(!p.running && p.fuel_flow == 0.0);
        run(&mut p, 4.0, &idle);
        assert!(rpm(&p) < 5.0, "{}", rpm(&p));

        // In the air, the tanks run dry: the engine quits and the propeller windmills.
        let mut p = Plane::spawn(C172, 0.6, -2.1, 1500.0, 0.0, 50.0);
        p.throttle = 0.75;
        p.fuel = 0.002;
        run(&mut p, 3.0, &Controls::default());
        assert!(!p.running && p.fuel == 0.0 && p.fuel_flow == 0.0);
        // Fuel back (a tank switched over): it catches again from the windmilling.
        p.fuel = 50.0;
        run(&mut p, 2.0, &Controls::default());
        assert!(p.running, "{}", rpm(&p));
    }

    #[test]
    fn balance_follows_the_load_and_changes_the_handling() {
        // As loaded by default the CG sits at the reference, inside the limits.
        let mut p = Plane::spawn(C172, 0.6, -2.1, 1500.0, 0.0, 50.0);
        assert!((p.cg() - C172.cg[0]).abs() < 1e-9 && p.cg_excess() == 0.0);
        // Four up and full baggage: heavier, and the CG goes past the aft limit.
        p.payload = C172.payload[1];
        assert!(p.mass() > C172.mass + 200.0);
        assert!(p.cg() > C172.cg[2] && p.cg_excess() > 0.0, "{}", p.cg());
        // Burning fuel (it sits aft of the CG) moves it forward.
        let c0 = p.cg();
        p.fuel = 10.0;
        assert!(p.cg() < c0);

        // The same pull on the stick: the aft-loaded aircraft pitches up quicker.
        let w = empty_world();
        let pitch_rate = |payload: f64, fuel: f64| {
            let mut p = Plane::spawn(C172, 0.6, -2.1, 1500.0, 0.0, 50.0);
            (p.payload, p.fuel) = (payload, fuel);
            p.throttle = 0.7;
            let dt = 1.0 / 180.0;
            let mut best: f64 = 0.0;
            for _ in 0..(1.0 / dt) as usize {
                p.step(dt, &Controls { pitch: 0.4, ..Default::default() }, &w);
                best = best.max(p.omega.x.abs());
            }
            best
        };
        let (fwd, aft) = (pitch_rate(80.0, C172.fuel_kg * 0.1), pitch_rate(C172.payload[1], C172.fuel_kg));
        println!("pitch rate after a pull: forward CG {:.1}°/s, aft CG {:.1}°/s", fwd.to_degrees(), aft.to_degrees());
        assert!(aft > fwd * 1.15, "{fwd} {aft}");
    }

    #[test]
    fn electrics_and_failures() {
        let w = flat_world();
        let dt = 1.0 / 180.0;
        let run = |p: &mut Plane, secs: f64| {
            for _ in 0..(secs / dt) as usize {
                assert!(p.step(dt, &Controls { brake: true, ..Default::default() }, &w).is_none());
            }
        };
        let ground = || {
            let mut p = Plane::spawn_ground(C172, 37.8, -122.46, 0.0, 0.5, 0.0);
            (p.hold, p.t) = (false, 2.0);
            p.cold_and_dark();
            p
        };
        // Cold and dark with a flat battery: the starter is dead, nothing turns.
        let mut p = ground();
        p.sys.fail.battery = true;
        p.start_engine();
        run(&mut p, 6.0);
        assert!(!p.running && rpm(&p) < 5.0 && !p.sys.powered());
        // Battery back: it starts, and once running fast enough the alternator charges.
        p.sys.fail.battery = false;
        p.sys.elec.charge = 0.8;
        p.start_engine();
        run(&mut p, 6.0);
        assert!(p.running);
        p.throttle = 0.3;
        run(&mut p, 3.0);
        assert!(p.sys.elec.amps > 0.0 && p.sys.elec.volts > 27.0, "{} A {} V at {:.0} rpm", p.sys.elec.amps, p.sys.elec.volts, rpm(&p));
        // An engine failure: it quits whatever the switches say.
        p.sys.fail.engine = true;
        run(&mut p, 1.0);
        assert!(!p.running && p.fuel_flow == 0.0);
        // A leak drains the tanks with the engine off.
        let before = p.fuel;
        p.sys.fail.fuel_leak = true;
        run(&mut p, 10.0);
        assert!(p.fuel < before - 0.03, "{} -> {}", before, p.fuel);

        // In the air: a blocked pitot turns the airspeed indicator into an
        // altimeter; a failed vacuum pump lets the suction fall away.
        let w = empty_world();
        let mut p = Plane::spawn(C172, 0.6, -2.1, 1500.0, 0.0, 50.0);
        let mut g = crate::gauges::Gauges::new(&p, 1);
        g.update(0.1, &p, 0.0);
        let asi0 = g.readings(&p).0[0][0];
        p.sys.fail.pitot = true;
        p.sys.fail.vacuum = true;
        g.update(0.1, &p, 0.0);
        p.pos = p.pos.normalize() * (p.pos.length() + 300.0);
        for _ in 0..200 {
            g.update(0.05, &p, 0.0);
        }
        let r = g.readings(&p).0;
        let _ = &w;
        assert!((r[0][0] - asi0 - 9.84).abs() < 1.5, "asi {} from {}", r[0][0], asi0);
        assert!(r[4][0] < 1.0, "suction {}", r[4][0]);
    }

    #[test]
    fn jets_start_up_to_idle_and_burn_like_the_real_ones() {
        let w = flat_world();
        let dt = 1.0 / 60.0;
        // (Started in the air, slowly, where only the engines matter.)
        let mut p = Plane::spawn(B747, 0.6, -2.1, 3000.0, 0.0, 80.0);
        p.cold_and_dark();
        let idle = Controls::default();
        p.start_engine();
        let mut t = 0.0;
        while !(p.running && p.n1 > 0.24) && t < 90.0 {
            assert!(p.step(dt, &idle, &w).is_none());
            t += dt;
        }
        println!("747 start: {t:.0} s to idle, burning {:.1} t/h", p.fuel_flow * 3.6);
        assert!(t > 5.0 && t < 90.0 && !p.starter_on, "{t}");
        // At about cruise thrust the four burn some 10-12 t an hour.
        let mut p = Plane::spawn(B747, 0.6, -2.1, 10_500.0, 0.0, 250.0);
        p.throttle = 0.85;
        for _ in 0..(20.0 / dt) as usize {
            p.step(dt, &Controls { throttle_set: Some(0.85), ..Default::default() }, &w);
        }
        let th = p.fuel_flow * 3.6;
        println!("747 at 85% throttle, FL345: {th:.1} t/h, {:.1} h left", p.endurance().unwrap() / 3600.0);
        assert!(th > 7.0 && th < 16.0, "{th}");
        // The Cessna at 75% power: some 30-38 l/h (8-10 US gal/h).
        let mut c = Plane::spawn(C172, 0.6, -2.1, 1500.0, 0.0, 55.0);
        c.fuel = C172.fuel_kg;
        for _ in 0..(10.0 / dt) as usize {
            c.step(dt, &Controls { throttle_set: Some(0.75), ..Default::default() }, &w);
        }
        let lh = c.fuel_flow * 3600.0 / 0.72;
        println!("C172 at 75%: {lh:.0} l/h, {:.1} h", c.endurance().unwrap() / 3600.0);
        assert!(lh > 22.0 && lh < 45.0, "{lh}");
        // And the fuel weighs: full tanks are heavier than empty.
        c.fuel = 0.0;
        assert!((C172.mass + 0.5 * C172.fuel_kg - c.mass()).abs() < 1e-9 + C172.fuel_kg);
    }

    #[test]
    fn approach_is_trimmed_down_the_glide() {
        let w = empty_world();
        // As the game sets up a final approach: 70 kt, flaps 20, a 3 degree glide.
        let mut p = Plane::spawn(C172, 0.6, -2.1, 400.0, 0.0, 36.0);
        let sink = 36.0 * 3f64.to_radians().sin();
        p.configure(2, true, sink);
        let thr = p.throttle;
        let (v, vs) = fly(&mut p, &w, 30.0, thr, |_| 0.0);
        println!("approach: throttle {thr:.2}, {:.0} kt, {:.0} fpm, {:.0} rpm", v * KT, vs * 196.85, rpm(&p));
        assert!((v - 36.0).abs() < 3.0, "{v}");
        assert!((vs + sink).abs() < 1.0, "{vs}");
    }

    #[test]
    fn turbulence_bumps_the_aircraft() {
        let w = empty_world();
        let mut spread = Vec::new();
        for sigma in [0.0, 1.5, 4.0] {
            let mut p = Plane::spawn(C172, 0.6, -2.1, 1000.0, 0.0, 55.0);
            p.air = Air { sigma_w: sigma, sigma_h: sigma, len_w: 300.0, len_h: 300.0, draft: 0.0, ..Default::default() };
            let dt = 1.0 / 180.0;
            let (mut s, mut s2, mut n, mut bank) = (0.0, 0.0, 0.0, 0.0f64);
            for i in 0..(60.0 / dt) as usize {
                // A pilot holding the wings roughly level.
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), ..Default::default() };
                p.step(dt, &c, &w);
                if i > 1800 {
                    s += p.g_load;
                    s2 += p.g_load * p.g_load;
                    n += 1.0;
                    bank = bank.max(roll_err.abs().asin().to_degrees());
                }
            }
            let sd = (s2 / n - (s / n).powi(2)).max(0.0).sqrt();
            println!("sigma {sigma}: g rms {sd:.3}, worst bank {bank:.1} deg");
            spread.push(sd);
        }
        assert!(spread[0] < 0.02 && spread[1] > 0.05 && spread[2] > spread[1] * 1.8, "{spread:?}");
    }

    /// Straight and level passes through each kind of weather, a pilot holding
    /// the wings level and the pitch attitude (as you do in turbulence), the air
    /// fed in as main.rs does. Prints peak storm sigma, drafts, g, height and
    /// bank excursions. `cargo test --release storm_passes -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn storm_passes() {
        use crate::weather::{CellKind, Weather};
        let w = empty_world();
        let (lat, lon) = (41.2f64.to_radians(), -96.0f64.to_radians());
        let p0 = geo::lla_to_pos(lat, lon, 0.0);
        // (name, spec, alt, setup, distance flown m)
        type Setup = fn(&mut Weather, DVec3);
        let cases: Vec<(&str, &'static Spec, f64, Setup, f64)> = vec![
            ("clear", C172, 2500.0, |_, _| {}, 15000.0),
            ("deck 0.4", C172, 1000.0, |wx, p| { wx.rain_set = Some(0.4); for _ in 0..600 { wx.step_deck(1.0, p, Some(0.0)); } }, 15000.0),
            ("deck 1.0 in", C172, 1500.0, |wx, p| { wx.rain_set = Some(1.0); for _ in 0..600 { wx.step_deck(1.0, p, Some(0.0)); } }, 15000.0),
            ("shower", C172, 2500.0, |wx, p| wx.place_storm(p, 6.0, 270.0, 1.8, Some(CellKind::Shower)), 12000.0),
            ("shower low", C172, 800.0, |wx, p| wx.place_storm(p, 6.0, 270.0, 1.8, Some(CellKind::Shower)), 12000.0),
            ("thunder", C172, 2500.0, |wx, p| wx.place_storm(p, 6.0, 270.0, 5.0, Some(CellKind::Thunder)), 20000.0),
            ("thunder low", C172, 300.0, |wx, p| wx.place_storm(p, 6.0, 270.0, 5.0, Some(CellKind::Thunder)), 20000.0),
            ("severe", C172, 2500.0, |wx, p| wx.place_storm(p, 8.0, 270.0, 6.0, None), 24000.0),
            ("squall", C172, 2500.0, |wx, p| wx.place_squall(p, 6.0, 270.0, 100.0), 24000.0),
            ("squall low", C172, 400.0, |wx, p| wx.place_squall(p, 6.0, 270.0, 100.0), 24000.0),
            ("supercell 2.5", C172, 2500.0, |wx, p| wx.place_storm(p, 12.0, 270.0, crate::weather::SUPER_R, Some(CellKind::Supercell)), 30000.0),
            ("supercell", C172, 4000.0, |wx, p| wx.place_storm(p, 12.0, 270.0, crate::weather::SUPER_R, Some(CellKind::Supercell)), 30000.0),
            ("thunder 747", B747, 2500.0, |wx, p| wx.place_storm(p, 6.0, 270.0, 5.0, Some(CellKind::Thunder)), 20000.0),
            ("supercell 747", B747, 4000.0, |wx, p| wx.place_storm(p, 12.0, 270.0, crate::weather::SUPER_R, Some(CellKind::Supercell)), 30000.0),
        ];
        for (name, spec, alt, setup, dist) in cases {
          for seed in [7u64, 99, 12345] {
            let mut wx = Weather::new(0.3);
            wx.storms = 0.0;
            setup(&mut wx, p0);
            wx.cells = wx.storm_cells(p0);
            let speed = if spec.b > 30.0 { 130.0 } else { 55.0 };
            let mut p = Plane::spawn(spec, lat, lon, alt, -std::f64::consts::FRAC_PI_2, speed);
            p.rng = Rng(seed);
            let theta = |p: &Plane| p.forward().dot(p.pos.normalize()).asin();
            let th0 = theta(&p);
            let dt = 1.0 / 120.0;
            let (mut t, mut prev) = (0.0, th0);
            let (mut sig, mut sw, mut up, mut dn, mut gmax, mut gmin, mut dh, mut bank, mut dv) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 1.0f64, 1.0f64, 0.0f64, 0.0f64, 0.0f64);
            let mut gust = 0.0f64;
            let mut prof = String::new();
            let mut broke = false;
            let mut vmin = 999.0f64;
            let mut gl: Vec<f64> = Vec::new();
            let mut notes = Vec::new();
            while (p.pos - geo::lla_to_pos(lat, lon, alt)).length() < dist && t < 900.0 {
                let (_, _, h) = geo::pos_to_lla(p.pos);
                let air = wx.air(p.pos, h, t, false);
                let st = crate::storm::local(&wx, p.pos, h, t);
                p.air = air;
                p.wind = wx.at(p.pos, h, t) + p.pos.normalize() * air.draft + air.gust;
                let th = theta(&p);
                let q = (th - prev) / dt;
                prev = th;
                let roll_err = -p.right().dot(p.pos.normalize());
                let c = Controls { pitch: ((th0 - th) * 4.0 - q * 1.0).clamp(-1.0, 1.0), roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), ..Default::default() };
                if p.step(dt, &c, &w).is_some() {
                    broke = true;
                    break;
                }
                t += dt;
                if t > 2.0 {
                    sig = sig.max(st.sigma);
                    sw = sw.max(air.sigma_w);
                    up = up.max(air.draft);
                    dn = dn.min(air.draft);
                    gust = gust.max(air.gust.length());
                    gmax = gmax.max(p.g_load);
                    gmin = gmin.min(p.g_load);
                    dh = dh.max((h - alt).abs());
                    bank = bank.max(roll_err.abs().min(1.0).asin().to_degrees());
                    dv = dv.max((p.airspeed().length() - speed).abs());
                    vmin = vmin.min(p.airspeed().length());
                    gl.push(p.g_load);
                    notes.extend(p.notes.drain(..));
                }
                if (t / dt) as usize % (10.0 / dt) as usize == 0 {
                    let km = (p.pos - geo::lla_to_pos(lat, lon, alt)).length() / 1000.0;
                    prof += &format!(" {km:.1}:{:.1}/{:+.0}", air.sigma_w, air.draft);
                }
            }
            println!("{name:14} storm sigma {sig:4.1} sigma_w {sw:4.1}  draft {up:+5.1} {dn:+5.1}  gust {gust:4.1}  g {gmin:+5.2}..{gmax:+5.2}  dh {dh:5.0} m  bank {bank:4.0}  dv {dv:4.1} m/s vmin {vmin:4.1} grms {:.2} {notes:?}{}", gl.chunks((20.0 / dt) as usize).map(|c| { let n = c.len() as f64; let m = c.iter().sum::<f64>() / n; (c.iter().map(|g| (g - m) * (g - m)).sum::<f64>() / n).sqrt() }).fold(0.0, f64::max), if broke { "  CRASHED" } else { "" });
            if seed == 7 { println!("    {prof}"); }
          }
        }
    }

    fn flat_world() -> World {
        let mut w = empty_world();
        w.flat = Some(0.0);
        w
    }
    /// Run until it explodes or `secs` pass; every note it made along the way.
    fn run(p: &mut Plane, w: &World, secs: f64, c: Controls) -> (bool, Vec<String>) {
        run_pushed(p, w, secs, c, 0.0)
    }
    /// The same, with something rolling it over (rad/s²) for the first 0.8 s: a gust under one wing.
    fn run_pushed(p: &mut Plane, w: &World, secs: f64, c: Controls, push: f64) -> (bool, Vec<String>) {
        let dt = 1.0 / 180.0;
        let mut notes = Vec::new();
        for k in 0..(secs / dt) as usize {
            if (k as f64) * dt < 0.8 {
                p.omega.z += push * dt;
            }
            let imp = p.step(dt, &c, w);
            notes.extend(p.notes.drain(..));
            p.shed.clear();
            if imp.is_some() {
                return (true, notes);
            }
        }
        (false, notes)
    }
    /// Rolling down the runway at `kt`, banked over on one main wheel and
    /// still rolling `roll` rad/s (a gust under a wing, a botched crosswind landing).
    fn rolling(kt: f64, roll: f64, seed: u64) -> Plane {
        let mut p = Plane::spawn_ground(C172, 37.8, -122.46, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.rng = Rng(seed);
        p.throttle = 0.0;
        p.charge = 0.0;
        let bank = 0.12 * roll.signum();
        // Pivot about the downhill main wheel so it stays on the ground.
        let wheel = C172.gear[if roll > 0.0 { 1 } else { 2 }].at;
        let before = p.pos + p.rot * wheel;
        p.rot = p.rot * DQuat::from_rotation_z(-bank);
        p.pos += before - (p.pos + p.rot * wheel);
        p.vel = p.forward() * (kt / KT);
        p.omega = DVec3::new(0.0, 0.0, -roll);
        p
    }

    #[test]
    fn tipping_over_at_40_kt_breaks_a_wing_without_fire() {
        let w = flat_world();
        let mut fires = 0;
        for seed in 1..=12u64 {
            let mut p = rolling(40.0, if seed % 2 == 0 { 2.2 } else { -2.2 }, seed * 7919);
            let (boom, notes) = run_pushed(&mut p, &w, 12.0, Controls::default(), if seed % 2 == 0 { -9.0 } else { 9.0 });
            println!("seed {seed}: boom {boom} {}, wings {:?}, folds {:?}, speed {:.1}, wrecked {}: {notes:?}", p.cause, p.dmg.wing, p.dmg.fold.map(|f| f.on), p.vel.length(), p.wrecked());
            fires += boom as usize;
            assert!(p.dmg.wing.iter().any(|&b| b < C172.span) || p.dmg.fold.iter().any(|f| f.on) || !notes.is_empty(), "something breaks");
        }
        assert_eq!(fires, 0, "a slow tip-over never burns");
    }

    #[test]
    fn a_cartwheel_at_70_kt_tears_things_off() {
        let w = flat_world();
        let mut fires = 0;
        for seed in 1..=8u64 {
            let mut p = rolling(70.0, 3.5, seed * 104_729);
            p.vel += p.pos.normalize() * -3.0;
            let (boom, notes) = run(&mut p, &w, 14.0, Controls { roll: 1.0, ..Default::default() });
            println!("seed {seed}: boom {boom} {}, wings {:?}, tail {}, hull {:.2}: {notes:?}", p.cause, p.dmg.wing, p.dmg.tail.is_some(), p.dmg.hull);
            fires += boom as usize;
            assert!(boom || !notes.is_empty());
        }
        assert!(fires < 8, "not every cartwheel burns");
    }

    #[test]
    fn belly_landing_slides_to_a_stop() {
        let w = flat_world();
        let mut p = Plane::spawn(C172, 37.8, -122.46, 2.0, 0.5, 30.0);
        p.t = 2.0;
        p.throttle = 0.0;
        p.charge = 0.0;
        p.vel += p.pos.normalize() * -2.0;
        let (boom, notes) = run(&mut p, &w, 20.0, Controls::default());
        println!("belly: boom {boom}, speed {:.1}, prop {}, {notes:?}", p.vel.length(), p.dmg.prop);
        assert!(!boom);
        assert!(p.vel.length() < 2.0);
        assert!(p.dmg.wing == [C172.span; 2]);
    }

    #[test]
    fn diving_into_the_ground_explodes() {
        let w = flat_world();
        let mut p = Plane::spawn(C172, 37.8, -122.46, 40.0, 0.5, 60.0);
        p.t = 2.0;
        p.rot = p.rot * DQuat::from_rotation_x(-0.7);
        p.vel = p.forward() * 60.0;
        let (boom, _) = run(&mut p, &w, 5.0, Controls { pitch: -1.0, ..Default::default() });
        assert!(boom);
    }

    #[test]
    fn a_buckled_wing_rips_off_in_the_slipstream() {
        let w = empty_world();
        let mut p = Plane::spawn(C172, 37.8, -122.46, 1500.0, 0.5, 50.0);
        p.dmg.apply_spec("rfold=3.0");
        let dt = 1.0 / 180.0;
        let mut gone = None;
        for k in 0..360 {
            p.step(dt, &Controls::default(), &w);
            if !p.dmg.fold[1].on {
                gone = Some(k as f64 * dt);
                break;
            }
        }
        println!("fold let go after {gone:?} s");
        assert!(gone.is_some_and(|t| t < 1.5));
        assert!(!p.shed.is_empty());
    }

    #[test]
    fn breaks_stay_inside_each_other() {
        let mut rng = Rng(99);
        for spec in [C172, B747] {
            let k = spec.span / 5.5;
            for s in 0..2 {
                let mut prev = Tear::NONE;
                let mut st = spec.span;
                while st > 1.0 * k {
                    st -= rng.range(0.4, 1.4) * k;
                    let t = damage::wing_break(spec, s, st.max(0.5 * k), rng.range(-0.8, 0.8) * k, rng.range(-0.3, 0.3), 0.2, &prev);
                    // The new break's mid-chord point is kept by the old stump, the old tip is not kept by the new.
                    let mid = damage::wing_pt(spec, s, st.max(0.5 * k), 0.45, 0.0);
                    assert!(prev.keeps(mid));
                    assert!(!t.keeps(damage::wing_pt(spec, s, st.max(0.5 * k) + 0.3 * k, 0.45, 0.0)));
                    prev = t;
                }
            }
        }
    }

    // ---------------------------------------------------------------- the 747

    /// The Starship stack, released on the pad.
    fn on_the_pad() -> Plane {
        let mut p = Plane::spawn_ground(&crate::aircraft::starship::SPEC, 0.45, -1.7, 0.0, 0.5, 0.0);
        (p.hold, p.t) = (false, 2.0);
        p
    }
    fn height(p: &Plane) -> f64 {
        geo::pos_to_lla(p.pos).2
    }
    /// How high its engines are.
    fn base(p: &Plane) -> f64 {
        height(p) - p.wheel_height()
    }
    /// The angle between two directions (deg).
    fn deg(a: DVec3, b: DVec3) -> f64 {
        a.normalize().dot(b.normalize()).clamp(-1.0, 1.0).acos().to_degrees()
    }

    #[test]
    fn a_rocket_stands_on_its_pad() {
        let w = flat_world();
        let mut p = on_the_pad();
        let (mass, cg, _, _) = p.rocket_mass(p.spec.rocket().unwrap());
        println!("on the pad: {:.0} t, centre of gravity {:.1} m above the engines", mass / 1000.0, 48.0 - cg);
        assert!((mass - 4.975e6).abs() < 1.0);
        let (boom, _) = run(&mut p, &w, 30.0, Controls::default());
        let lean = p.forward().dot(p.pos.normalize()).acos().to_degrees();
        println!("after 30 s: its base {:.2} m into the ground, leaning {lean:.2} deg, {:.3} m/s", 48.0 - height(&p), p.vel.length());
        assert!(!boom && p.on_ground && lean < 0.5 && (height(&p) - 48.0).abs() < 0.3 && p.vel.length() < 0.05);
        // (Nor does it stage where it stands.)
        p.stage_sep();
        run(&mut p, &w, 1.0, Controls::default());
        assert_eq!(p.stage, 0);
    }

    #[test]
    fn a_rocket_lights_the_engines_it_is_told_to() {
        let w = flat_world();
        let burn = |lit: usize| {
            let mut p = on_the_pad();
            p.lit = lit;
            run(&mut p, &w, 4.0, Controls { throttle_set: Some(1.0), ..Default::default() });
            (p.thrust_now(), p.fuel_flow, p.engines().unwrap())
        };
        let (all, inner, centre) = (burn(0), burn(1), burn(2));
        println!("thrust and flow: {:?} {:.1} MN {:.0} kg/s, {:?} {:.1} MN {:.0} kg/s, {:?} {:.1} MN {:.0} kg/s", all.2, all.0 / 1e6, all.1, inner.2, inner.0 / 1e6, inner.1, centre.2, centre.0 / 1e6, centre.1);
        assert!((all.0 / 73.5e6 - 1.0).abs() < 0.02 && (all.1 / 22_920.0 - 1.0).abs() < 0.02);
        assert!((inner.0 / all.0 - 13.0 / 33.0).abs() < 0.01 && (inner.1 / all.1 - 13.0 / 33.0).abs() < 0.01);
        assert!((centre.0 / all.0 - 3.0 / 33.0).abs() < 0.01 && (centre.1 / all.1 - 3.0 / 33.0).abs() < 0.01);
    }

    #[test]
    fn a_rocket_flies_to_space_in_two_stages() {
        let w = flat_world();
        let mut p = on_the_pad();
        let dt = 1.0 / 120.0;
        let up0 = p.pos.normalize();
        let (mut t, mut cleared, mut max_q, mut staged) = (0.0, 0.0, (0.0f64, 0.0), (0.0, 0.0, 0.0));
        let mut top = 0.0f64;
        while t < 520.0 {
            // Straight up for 12 s, then lean over until it is 60 degrees from
            // the vertical, and let it go on turning from there.
            let lean = p.forward().dot(p.pos.normalize()).clamp(-1.0, 1.0).acos();
            let pitch = if t > 12.0 && lean < 60f64.to_radians() { -1.0 } else { 0.0 };
            if t >= 141.0 && p.stage == 0 {
                p.stage_sep();
            }
            let imp = p.step(dt, &Controls { pitch, throttle_set: Some(1.0), ..Default::default() }, &w);
            assert!(imp.is_none(), "blew up at {t:.0} s: {:?} ({})", p.notes, p.cause);
            assert!(p.pos.is_finite() && p.vel.is_finite() && p.rot.is_finite() && p.omega.is_finite(), "NaN at {t:.0} s");
            t += dt;
            let h = base(&p);
            if cleared == 0.0 && h > 146.0 {
                cleared = t;
            }
            let q = 0.5 * density(height(&p)) * p.vel.length_squared();
            if q > max_q.0 {
                max_q = (q, t);
            }
            if p.stage == 1 && staged.0 == 0.0 {
                staged = (t, h, p.vel.length());
                let rk = p.spec.rocket().unwrap();
                let (m, _, it, _) = p.rocket_mass(rk);
                println!("staged at {t:.0} s: {:.0} km up, {:.0} m/s, {:.0} km downrange; the ship alone {:.0} t, pitch inertia {:.2e} kg m2, {:.0} t of propellant", h / 1000.0, p.vel.length(), p.pos.normalize().dot(up0).acos() * geo::R / 1000.0, m / 1000.0, it, p.fuel / 1000.0);
                assert!((m - 1.3e6).abs() < 500.0 && (p.fuel - 1.2e6).abs() < 500.0 && it < 5.0e8 && p.dmg.tail.is_some() && p.shed.len() == 1);
            }
            p.shed.clear();
            p.notes.clear();
            top = top.max(h);
            if p.stage == 1 && p.fuel <= 0.0 && p.vel.dot(p.pos.normalize()) < 0.0 {
                break;
            }
        }
        println!("cleared the tower (146 m) at {cleared:.1} s; max q {:.1} kPa at {:.0} s; ship burnt out {:.0} m/s; highest {:.0} km", max_q.0 / 1000.0, max_q.1, p.vel.length(), top / 1000.0);
        assert!((6.0..11.0).contains(&cleared), "{cleared}");
        assert!((20.0e3..50.0e3).contains(&max_q.0), "{max_q:?}");
        assert!((40.0e3..90.0e3).contains(&staged.1) && (1_200.0..2_700.0).contains(&staged.2), "{staged:?}");
        assert!(top > 100.0e3, "{top}");
    }

    #[test]
    fn a_ship_lands_on_its_engines() {
        let w = flat_world();
        // The ship alone, nearly empty, falling tail first from 400 m.
        let mut p = Plane::spawn(&crate::aircraft::starship::SPEC, 0.45, -1.7, 400.0, 0.5, 0.0);
        p.t = 2.0;
        p.stage_sep();
        while p.stage == 0 {
            p.step(1.0 / 120.0, &Controls { throttle_set: Some(0.0), ..Default::default() }, &w);
        }
        (p.fuel, p.lit, p.vel) = (30_000.0, 1, -p.pos.normalize() * 40.0);
        p.shed.clear();
        assert_eq!((p.stage, p.engines().unwrap().1), (1, 3));
        let dt = 1.0 / 120.0;
        let (mut t, mut down_at, mut hit) = (0.0, 0.0, 0.0f64);
        while t < 60.0 {
            // Three engines at their floor still out-push its weight: burn only
            // when it is falling faster than it can stop from here.
            let base = base(&p);
            let vz = p.vel.dot(p.pos.normalize());
            let burn = !p.on_ground && vz < -(2.0 * 7.0 * (base - 1.0).max(0.0)).sqrt().max(1.5);
            let was = p.on_ground;
            let imp = p.step(dt, &Controls { throttle_set: Some(if burn { 0.03 } else { 0.0 }), ..Default::default() }, &w);
            assert!(imp.is_none(), "blew up at {t:.1} s, {base:.1} m up at {vz:.1} m/s ({})", p.cause);
            if p.on_ground && !was && down_at == 0.0 {
                (down_at, hit) = (t, vz);
            }
            t += dt;
        }
        let lean = deg(p.forward(), p.pos);
        println!("the ship touched down at {down_at:.1} s at {hit:.1} m/s with {:.1} t of propellant left; standing, leaning {lean:.2} deg", p.fuel / 1000.0);
        assert!(p.on_ground && down_at > 0.0 && hit > -6.0 && lean < 1.0 && p.vel.length() < 0.1 && !p.wrecked());
    }

    /// Flown the way a player flies it, with the keys and nothing cleverer:
    /// Shift until the throttle is open, W for a few seconds once it has
    /// cleared the tower, hands off while it turns over on its own, Backspace
    /// as the booster runs dry, hands off again until the ship burns out.
    #[test]
    fn a_rocket_is_flown_by_hand() {
        let w = flat_world();
        let mut p = on_the_pad();
        let dt = 1.0 / 120.0;
        let kick = 16.0;
        let (mut t, mut staged, mut load, mut q_max, mut wander) = (0.0, (0.0, 0.0, 0.0, 0.0), (0.0f64, 0.0), 0.0f64, 0.0f64);
        let mut held = p.forward();
        while t < 600.0 {
            let c = Controls { throttle: if t < 2.5 { 1.0 } else { 0.0 }, pitch: if (12.0..12.0 + kick).contains(&t) { -1.0 } else { 0.0 }, ..Default::default() };
            if p.stage == 0 && p.fuel_frac() < 0.04 {
                p.stage_sep();
            }
            let was = p.stage;
            let imp = p.step(dt, &c, &w);
            assert!(imp.is_none(), "blew up at {t:.0} s: {:?} ({})", p.notes, p.cause);
            t += dt;
            let (h, v) = (base(&p), p.airspeed());
            let q = 0.5 * density(height(&p)) * v.length_squared();
            q_max = q_max.max(q);
            if v.length() > 30.0 {
                let a = deg(p.forward(), v);
                if q * a.to_radians() > load.0 {
                    load = (q * a.to_radians(), a);
                }
            }
            if p.stage != was {
                staged = (t, h, p.vel.length(), 90.0 - deg(p.vel, p.pos));
                held = p.forward();
            }
            // (Out of the air, hands off, it keeps the attitude it had.)
            if p.stage == 1 && p.fuel > 0.0 && height(&p) > 80.0e3 {
                wander = wander.max(deg(p.forward(), held));
            } else {
                held = p.forward();
            }
            p.shed.clear();
            p.notes.clear();
            if p.stage == 1 && p.fuel <= 0.0 {
                break;
            }
        }
        let apo = p.apogee().map_or(f64::INFINITY, |a| a / 1000.0);
        println!("by hand (W held {kick} s): max q {:.1} kPa, worst q-alpha {:.0} Pa rad (at {:.1} deg); staged at {:.0} s, {:.0} km, {:.0} m/s, climbing at {:.0} deg; ship burnt out at {t:.0} s, {:.0} km up, {:.0} m/s, {:.0} km downrange, apogee {apo:.0} km; attitude wandered {wander:.2} deg", q_max / 1000.0, load.0, load.1, staged.0, staged.1 / 1000.0, staged.2, staged.3, base(&p) / 1000.0, p.vel.length(), p.downrange() / 1000.0);
        assert!(load.0 < 4.5e3, "{load:?}");
        assert!((45.0e3..85.0e3).contains(&staged.1) && (1_300.0..2_700.0).contains(&staged.2) && (15.0..60.0).contains(&staged.3), "{staged:?}");
        assert!(p.vel.length() > 6_000.0 && apo > 150.0 && wander < 1.0);
    }

    #[test]
    fn a_rocket_stages_hot() {
        let w = flat_world();
        let mut p = Plane::spawn(&crate::aircraft::starship::SPEC, 0.45, -1.7, 60_000.0, 0.5, 1_500.0);
        p.t = 2.0;
        let dt = 1.0 / 120.0;
        let c = Controls { throttle_set: Some(1.0), ..Default::default() };
        for _ in 0..240 {
            p.step(dt, &c, &w);
        }
        let (all, at) = (p.thrust_now(), p.model_pos());
        assert_eq!((p.stage, p.engines().unwrap().1, p.plume_code().floor()), (0, 33, 0.0));
        p.stage_sep();
        // The booster cuts to its centre three while the ship's six burn on top of it ...
        let mut hot = 0.0;
        while p.stage == 0 {
            p.step(dt, &c, &w);
            if p.stage == 0 {
                assert_eq!((p.engines().unwrap().1, p.plume_code().floor()), (3, 2.0));
                assert!(p.shed.is_empty() && !p.dmg.tail.is_some() && p.thrust_now() < 0.12 * all);
            }
            hot += dt;
        }
        // ... then it is let go, keeping its speed, and the ship is already under way on its own.
        let (thrust, moved) = (p.thrust_now(), (p.model_pos() - at - p.vel * hot).length());
        println!("hot for {hot:.2} s; the ship alone: {:.1} MN at once, its origin {:.1} m up the model, {:.1} m above its engines; the model's origin moved {moved:.1} m off its path", thrust / 1e6, -p.origin, p.wheel_height());
        assert!((0.6..1.2).contains(&hot) && thrust > 11.0e6 && p.engines().unwrap().1 == 6 && p.plume_code().floor() == 1.0);
        assert!(p.shed.len() == 1 && p.shed[0].clean && p.dmg.tail.is_some());
        assert!((p.wheel_height() - 24.875).abs() < 0.01 && (p.centre() - p.pos).length() < 0.01 && moved < 5.0);
        assert!(p.dmg.report().iter().any(|l| l == "BOOSTER SEPARATED"));
        assert_eq!(p.shadow_r(), 0.45 * p.spec.shadow_r);
    }

    /// The ship's way home: belly first from 20 km on its flaps, then upright
    /// on three engines to stand on its skirt.
    #[test]
    fn a_ship_falls_on_its_belly_and_lands() {
        let w = flat_world();
        let mut p = Plane::spawn(&crate::aircraft::starship::SPEC, 0.45, -1.7, 20_000.0, 0.5, 0.0);
        p.t = 2.0;
        p.stage_sep();
        while p.stage == 0 {
            p.step(1.0 / 120.0, &Controls { throttle_set: Some(0.0), ..Default::default() }, &w);
        }
        p.shed.clear();
        // Nearly empty, lying level with its tiles down, falling.
        let up = p.pos.normalize();
        let (e, n, _) = geo::enu(p.pos);
        let f = (n * 0.5f64.cos() + e * 0.5f64.sin()).normalize();
        p.rot = DQuat::from_mat3(&DMat3::from_cols(f.cross(up), up, -f));
        (p.fuel, p.vel, p.omega, p.auto) = (60_000.0, -up * 150.0, DVec3::ZERO, true);
        let dt = 1.0 / 120.0;
        let (mut t, mut term, mut tilt, mut flip_at, mut down_at, mut hit, mut rolled) = (0.0, 0.0, 0.0f64, 0.0, 0.0, 0.0f64, 0.0f64);
        let start = p.pos;
        while t < 400.0 {
            let up = p.pos.normalize();
            let (h, vz) = (base(&p), p.vel.dot(up));
            // Hands off all the way down: it holds itself level on its flaps, then flies its own flip and landing burn. A nudge of roll and back on the way, to see the flaps answer.
            let mut c = Controls::default();
            if flip_at == 0.0 {
                c.roll = if (20.0..22.0).contains(&t) { 1.0 } else if (22.0..24.0).contains(&t) { -1.0 } else { 0.0 };
                rolled = rolled.max(deg(p.up(), up));
                if t > 30.0 {
                    tilt = tilt.max(deg(p.up(), up));
                }
                if h < 3000.0 {
                    term = vz;
                }
                if p.running {
                    flip_at = t;
                }
            }
            let was = p.on_ground;
            let imp = p.step(dt, &c, &w);
            assert!(imp.is_none(), "blew up at {t:.1} s, {h:.0} m up at {vz:.1} m/s ({}): {:?}", p.cause, p.notes);
            if p.on_ground && !was && down_at == 0.0 {
                (down_at, hit) = (t, vz);
            }
            t += dt;
            if down_at > 0.0 && t > down_at + 20.0 {
                break;
            }
        }
        let lean = deg(p.forward(), p.pos);
        println!("belly first: rolled {rolled:.0} deg and back on its flaps, held within {tilt:.1} deg of level after, falling at {:.0} m/s at the bottom; flipped at {flip_at:.0} s, down at {down_at:.0} s at {hit:.1} m/s, {:.0} m from under where it began, {:.1} t left; standing, leaning {lean:.2} deg", -term, (p.pos.normalize() - start.normalize()).length() * geo::R, p.fuel / 1000.0);
        assert!((80.0..105.0).contains(&-term), "{term}");
        assert!(rolled > 8.0 && tilt < 3.0, "{rolled} {tilt}");
        assert!(p.on_ground && down_at > 0.0 && hit > -6.0 && lean < 1.5 && p.vel.length() < 0.1 && !p.wrecked());
    }

    #[test]
    fn performance_is_a_747() {
        let w = empty_world();
        let s = B747;
        let fe = match &s.engine {
            Engine::Fan(f) => f,
            _ => unreachable!(),
        };
        // Cruise at FL350, Mach 0.85, about 90% N1 or a little less.
        let h = 10_668.0;
        let v = 0.85 * sound_speed(h);
        let mut p = Plane::spawn(s, 0.6, -2.1, h, 0.0, v);
        println!("cruise trim: alpha {:.1} deg, throttle {:.2}, N1 {:.0}%", p.trim.to_degrees(), p.throttle, p.n1 * 100.0);
        assert!(p.trim.to_degrees() > 0.5 && p.trim.to_degrees() < 4.5, "{}", p.trim.to_degrees());
        assert!(p.n1 > 0.78 && p.n1 < 0.97, "{}", p.n1);
        let thr = p.throttle;
        let (va, vs) = fly(&mut p, &w, 120.0, thr, hold_alt(h));
        println!("cruise: M {:.3}, {:.0} kt true, {:.0} kt indicated, vs {vs:.1} m/s", va / sound_speed(h), va * KT, p.ias() * KT);
        assert!((va / sound_speed(h) - 0.85).abs() < 0.03);
        // Flat out at FL350 it is held back by the shock waves before Mach 0.95.
        let mut p = Plane::spawn(s, 0.6, -2.1, h, 0.0, v);
        let (vm, _) = fly(&mut p, &w, 300.0, 1.0, hold_alt(h));
        println!("flat out at FL350: M {:.3}", vm / sound_speed(h));
        assert!(vm / sound_speed(h) > 0.87 && vm / sound_speed(h) < 0.96);

        // Stall at 290 t: about 155 kt clean, about 125 kt with flaps 30.
        let vs_at = |flap_cl: f64, a: f64| (2.0 * s.mass * 9.81 / (RHO0 * s.wing_area * (s.cl0 + flap_cl + s.cl_a * a))).sqrt() * KT;
        let clean = vs_at(0.0, s.stall_alpha);
        let full = vs_at(s.flap_cl, s.stall_alpha - s.stall_flap + s.stall_slat);
        println!("stall {clean:.0} kt clean, {full:.0} kt flaps 30 (Vref ~{:.0})", full * 1.23);
        assert!(clean > 145.0 && clean < 170.0 && full > 115.0 && full < 135.0);

        // Static thrust a quarter of the weight; the fans take several seconds to spool up.
        let t0 = Plane::fan_thrust(fe, 1.0, 0.0, 0.0);
        println!("static thrust {:.0} kN = {:.2} W", t0 / 1000.0, t0 / (s.mass * 9.81));
        assert!(t0 / (s.mass * 9.81) > 0.3 && t0 / (s.mass * 9.81) < 0.4);
    }

    #[test]
    fn a_747_takes_off_and_climbs() {
        let w = flat_world();
        let s = B747;
        let mut p = Plane::spawn_ground(s, 37.6, -122.37, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.flap_set = 4; // flaps 20
        p.flaps = s.flap_frac(4);
        p.slats = 1.0;
        let dt = 1.0 / 120.0;
        let (mut lifted, mut t) = (None, 0.0);
        let start = p.pos;
        let mut v_rot = 0.0;
        while t < 90.0 {
            let v = p.vel.length();
            // Rotate at 155 kt to about 12 degrees nose up, then hold the climb.
            let pitch_up = p.up().dot(p.pos.normalize()).acos();
            let pitch_deg = (p.forward().dot(p.pos.normalize())).asin().to_degrees();
            let _ = pitch_up;
            let pitch = if v * KT > 155.0 { ((12.0 - pitch_deg) * 0.12).clamp(-1.0, 1.0) } else { 0.0 };
            let c = Controls { throttle: 1.0, pitch, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            if lifted.is_none() && !p.on_ground && t > 5.0 {
                lifted = Some(((p.pos - start).length(), v));
            }
            if v * KT > 155.0 && v_rot == 0.0 {
                v_rot = t;
            }
            t += dt;
        }
        let (dist, vlo) = lifted.expect("it never left the ground");
        let (_, _, h) = geo::pos_to_lla(p.pos);
        println!("rotate after {v_rot:.0} s, lift-off after {dist:.0} m at {:.0} kt; at 90 s: {h:.0} m, {:.0} kt, N1 {:.0}%", vlo * KT, p.vel.length() * KT, p.n1 * 100.0);
        assert!(dist > 1200.0 && dist < 3200.0, "{dist}");
        assert!(h > 300.0, "{h}");
        assert!(p.notes.is_empty() && p.dmg.gear.iter().all(|g| *g));
    }

    #[test]
    fn a_747_settles_on_its_gear() {
        let w = flat_world();
        for (spec, wind) in [(C172, 0.0), (B747, 0.0), (DR400, 0.0), (C172, 6.0), (B747, 6.0), (DR400, 6.0), (G650, 0.0), (G650, 6.0)] {
            let mut p = Plane::spawn_ground(spec, 37.6, -122.37, 0.0, 0.5, 0.0);
            // A tailwind, parked: no airflow worth the name over the wing.
            p.wind = p.forward() * wind;
            p.hold = false;
            p.t = 2.0;
            let dt = 1.0 / 180.0;
            let mut worst_vn: f64 = 0.0;
            for _ in 0..(10.0 / dt) as usize {
                assert!(p.step(dt, &Controls::default(), &w).is_none());
                worst_vn = worst_vn.min(p.vel.dot(p.pos.normalize()));
            }
            let pitch = p.forward().dot(p.pos.normalize()).asin().to_degrees();
            println!("{}: notes {:?}, compress {:?}, pitch {pitch:.2} deg, worst sink {worst_vn:.2} m/s, speed {:.2}", spec.name, p.notes, &p.compress[..spec.gear.len()], p.vel.length());
            assert!(p.dmg.gear.iter().all(|g| *g), "{:?}", p.notes);
            assert!(pitch.abs() < 2.0);
        }
    }

    #[test]
    fn a_747_lands_on_its_gear() {
        let w = flat_world();
        let s = B747;
        // Over the threshold at 150 kt, flaps 30, sinking 3 m/s.
        let mut p = Plane::spawn(s, 37.6, -122.37, 12.0, 0.5, 150.0 / KT);
        p.t = 2.0;
        p.configure(6, true, 3.0);
        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        let mut touched = false;
        while t < 60.0 {
            let c = Controls { throttle: -1.0, brake: touched, reverse: touched, pitch: if touched { -0.1 } else { 0.05 }, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            touched |= p.on_ground;
            t += dt;
        }
        println!("after 60 s: {:.1} m/s, notes {:?}, spoilers {:.2}, reversers {:.2}", p.vel.length(), p.notes, p.spoilers, p.reverser);
        assert!(touched);
        assert!(p.vel.length() < 3.0, "{}", p.vel.length());
        assert!(p.dmg.gear.iter().all(|g| *g));
    }

    // ---------------------------------------------------------------- the Gulfstream G650
    // Targets and their sources are in aircraft/g650/mod.rs ("The flight model").

    const G650: &Spec = &crate::aircraft::g650::SPEC;
    /// G650 weights (kg): maximum take-off and landing weight (TCDS).
    const G650_MTOW: f64 = 45_177.0;
    const G650_MLW: f64 = 37_874.0;

    /// The fuel that makes a G650 (with its usual payload) weigh `kg`.
    fn g650_fuel_for(kg: f64) -> f64 {
        let fuel = kg - G650.mass + 0.5 * G650.fuel_kg;
        assert!(fuel >= 0.0 && fuel <= G650.fuel_kg, "{kg} kg needs {fuel} kg of fuel");
        fuel
    }

    /// Its 1 g stall (m/s, sea level) at `kg` with the flaps at `flaps` (fraction of full travel).
    fn g650_vs(kg: f64, flaps: f64) -> f64 {
        let s = G650;
        let cl = s.cl0 + s.flap_cl * flaps + s.cl_a * (s.stall_alpha - s.stall_flap * flaps);
        (2.0 * kg * 9.81 / (RHO0 * s.wing_area * cl)).sqrt()
    }

    #[test]
    fn performance_is_a_g650() {
        let w = empty_world();
        let s = G650;
        let fe = match &s.engine {
            Engine::Fan(f) => f,
            _ => unreachable!(),
        };
        // Cruise at FL450, Mach 0.85 (its long-range cruise), at 35.3 t: N1 in the high 80s.
        let h = 13_716.0;
        let v = 0.85 * sound_speed(h);
        let mut p = Plane::spawn(s, 0.6, -2.1, h, 0.0, v);
        println!("cruise trim: alpha {:.1} deg, throttle {:.2}, N1 {:.0}%", p.trim.to_degrees(), p.throttle, p.n1 * 100.0);
        assert!(p.trim.to_degrees() > 1.5 && p.trim.to_degrees() < 4.5, "{}", p.trim.to_degrees());
        assert!(p.n1 > 0.80 && p.n1 < 0.96, "{}", p.n1);
        let thr = p.throttle;
        let (va, vs) = fly(&mut p, &w, 120.0, thr, hold_alt(h));
        let burn = p.fuel_flow;
        let range = s.fuel_kg / burn * va / 1852.0;
        println!("cruise: M {:.3}, {:.0} kt true, vs {vs:.1} m/s, {:.0} kg/h ({:.0} lb/h); all its fuel at that rate: {range:.0} nmi", va / sound_speed(h), va * KT, burn * 3600.0, burn * 3600.0 * 2.2046);
        assert!((va / sound_speed(h) - 0.85).abs() < 0.03 && p.notes.is_empty(), "{} {:?}", va / sound_speed(h), p.notes);
        // (some 1,250-1,450 kg/h at this weight; 480 US gal/h, 1,460 kg/h, on average)
        assert!(burn * 3600.0 > 1_100.0 && burn * 3600.0 < 1_550.0, "{}", burn * 3600.0);
        // (7,000 nmi at Mach 0.85 with eight passengers, after the climb and with reserves left)
        assert!(range > 6_500.0 && range < 9_000.0, "{range}");
        // Its high-speed cruise, Mach 0.90, burns more for the same weight and height.
        let mut p = Plane::spawn(s, 0.6, -2.1, h, 0.0, 0.90 * sound_speed(h));
        let thr = p.throttle;
        let (va90, _) = fly(&mut p, &w, 60.0, thr, hold_alt(h));
        let ratio = p.fuel_flow / burn;
        println!("high-speed cruise: M {:.3}, {:.0} kg/h, {ratio:.2} times the long-range cruise's", va90 / sound_speed(h), p.fuel_flow * 3600.0);
        assert!((va90 / sound_speed(h) - 0.90).abs() < 0.03 && ratio > 1.03 && ratio < 1.35, "{ratio}");

        // Flat out at FL410: up to its MMO of 0.925 (its maximum cruise), and
        // held back there by the drag rise, short of where it would flutter.
        let h = 12_497.0;
        let mut p = Plane::spawn(s, 0.6, -2.1, h, 0.0, 0.85 * sound_speed(h));
        let (vm, _) = fly(&mut p, &w, 300.0, 1.0, hold_alt(h));
        let top = top_mach(s, h, false, 0.5);
        println!("flat out at FL410: M {:.3} after 300 s, M {top:.3} where thrust meets drag; notes {:?}", vm / sound_speed(h), p.notes);
        assert!(vm / sound_speed(h) > 0.915 && vm / sound_speed(h) < 0.97 && p.notes.is_empty(), "{}", vm / sound_speed(h));
        assert!(top > 0.915 && top < 0.97, "{top}");

        // How high it goes: the climb left at full thrust, Mach 0.85 (m/s).
        let climb = |kg: f64, alt: f64| {
            let mut p = Plane::spawn(s, 0.6, -2.1, alt, 0.0, 200.0);
            p.fuel = g650_fuel_for(kg);
            let v = 0.85 * sound_speed(alt);
            (p.thrust(1.0, v, alt) - p.drag_level(v, alt, 0.0, 0.0)) * v / (kg * 9.81)
        };
        // Straight to FL410 at its maximum weight, less what the climb burns
        // (its initial cruise altitude, Aviation Week), up to its 51,000 ft
        // ceiling once light, but not at FL510 heavy.
        let (c410, c510, c510_full) = (climb(44_000.0, 12_497.0), climb(33_000.0, 15_545.0), climb(G650_MTOW, 15_545.0));
        println!("climb left at M0.85: {:.0} fpm at FL410 at 44 t, {:.0} fpm at FL510 at 33 t, {:.0} fpm at FL510 at MTOW", c410 * 196.85, c510 * 196.85, c510_full * 196.85);
        assert!(c410 * 196.85 > 300.0 && c510 * 196.85 > 100.0 && c510_full < 0.0, "{c410} {c510} {c510_full}");

        // Stall at the maximum landing weight: about 109 kt at flaps 39 (Vref
        // 134), 123 clean; at 65,500 lb its Vref is 120 kt (a flight evaluation).
        let (full, clean, vref_light) = (g650_vs(G650_MLW, 1.0) * KT, g650_vs(G650_MLW, 0.0) * KT, 1.23 * g650_vs(29_710.0, 1.0) * KT);
        println!("stall at MLW: {full:.0} kt flaps 39, {clean:.0} kt clean; Vref at 65,500 lb {vref_light:.0} kt; V2 at MTOW, flaps 20: {:.0} kt", 1.13 * g650_vs(G650_MTOW, 20.0 / 39.0) * KT);
        assert!(full > 104.0 && full < 114.0 && clean > 115.0 && clean < 130.0, "{full} {clean}");
        assert!(vref_light > 115.0 && vref_light < 125.0, "{vref_light}");

        // A third of its maximum weight in static thrust; the ICAO fuel flows (two engines).
        let t0 = Plane::fan_thrust(fe, 1.0, 0.0, 0.0);
        println!("static thrust {:.1} kN = {:.3} of MTOW; fuel flows {:?} kg/s", t0 / 1000.0, t0 / (G650_MTOW * 9.81), s.fuel_flow);
        assert!(t0 / (G650_MTOW * 9.81) > 0.33 && t0 / (G650_MTOW * 9.81) < 0.35);
        assert!(s.fuel_flow[0] > 0.15 && s.fuel_flow[0] < 0.20 && s.fuel_flow[1] > 1.5 && s.fuel_flow[1] < 1.65);

        // Go-around thrust: from flight idle to 90% of full thrust in 4-8 s (CS 25.119 allows 8).
        let (h, v) = (300.0, 70.0);
        let mut p = Plane::spawn(s, 0.6, -2.1, h, 0.0, v);
        p.throttle = 0.0;
        p.settle_engine(v, h);
        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        while p.power * fe.n as f64 * fe.thrust < 0.9 * p.thrust(1.0, p.airspeed().length(), h) && t < 20.0 {
            assert!(p.step(dt, &Controls { throttle_set: Some(1.0), ..Default::default() }, &w).is_none());
            t += dt;
        }
        println!("idle to 90% thrust: {t:.1} s");
        assert!(t > 4.0 && t < 8.0, "{t}");

        // Full aileron at 250 kt: a business jet's roll rate, not an airliner's.
        let mut p = Plane::spawn(s, 0.6, -2.1, 1500.0, 0.0, 250.0 / KT * (RHO0 / density(1500.0)).sqrt());
        let mut best: f64 = 0.0;
        for _ in 0..(4.0 / dt) as usize {
            assert!(p.step(dt, &Controls { roll: 1.0, ..Default::default() }, &w).is_none());
            best = best.max(p.omega.z.abs());
        }
        println!("roll rate at 250 kt: {:.0} deg/s", best.to_degrees());
        assert!(best.to_degrees() > 15.0 && best.to_degrees() < 35.0, "{}", best.to_degrees());
    }

    #[test]
    fn a_g650_takes_off_and_climbs() {
        let w = flat_world();
        let s = G650;
        // At its maximum take-off weight, flaps 20 (its usual setting), full thrust.
        let mut p = Plane::spawn_ground(s, 37.6, -122.37, 0.0, 0.5, 0.0);
        p.hold = false;
        p.t = 2.0;
        p.fuel = g650_fuel_for(G650_MTOW);
        p.flap_set = 2;
        p.flaps = s.flap_frac(2);
        let dt = 1.0 / 120.0;
        let (mut lifted, mut t) = (None, 0.0);
        let start = p.pos;
        let (mut want, mut last, mut most): (f64, f64, f64) = (0.0, 0.0, 0.0);
        let (mut want_climb, mut last_ias) = (12.0f64, 0.0f64);
        let mut fl410 = None;
        let pitch_of = |p: &Plane| p.forward().dot(p.pos.normalize()).asin().to_degrees();
        while t < 2400.0 && fl410.is_none() {
            let v = p.vel.length();
            let pitch_deg = pitch_of(&p);
            let rate = (pitch_deg - last) / dt;
            last = pitch_deg;
            let (_, _, hh) = geo::pos_to_lla(p.pos);
            let ias = p.ias() * KT;
            if p.on_ground {
                most = most.max(pitch_deg);
            }
            // Rotate at 138 kt, 2.5 degrees a second (pulling firmly: the aim leads the nose by 3), to 10 until the tail is
            // clear, then on to 15; once climbing away, fly the speed: 250 kt
            // below 10,000 ft, then 300 kt to Mach 0.80, then Mach 0.80.
            if lifted.is_some() && hh > 300.0 {
                let target = if hh < 3048.0 { 250.0 } else if p.mach() < 0.80 { 300.0 } else { ias * 0.80 / p.mach() };
                let accel = (ias - last_ias) / dt;
                want_climb = (want_climb + ((ias - target) * 0.04 + accel * 0.6) * dt).clamp(1.0, 18.0);
                want = want_climb;
            } else if v * KT > 138.0 {
                want = (want.max(pitch_deg + 3.0) + 2.5 * dt).min(if hh > p.wheel_height() + 6.0 { 15.0 } else { 10.0 });
                want_climb = want;
            }
            last_ias = ias;
            // Flaps up at 180 kt, out of VFE's way.
            if ias > 180.0 && lifted.is_some() {
                p.flap_set = 0;
            }
            let pitch = if v * KT > 138.0 { ((want - pitch_deg) * 0.25 - rate * 0.08).clamp(-1.0, 1.0) } else { 0.0 };
            let roll_err = -p.right().dot(p.pos.normalize());
            let c = Controls { throttle: 1.0, pitch, roll: (-roll_err * 3.0 + p.omega.z * 0.5).clamp(-1.0, 1.0), ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            if lifted.is_none() && !p.on_ground && t > 5.0 {
                lifted = Some(((p.pos - start).length(), v, t));
                p.toggle_gear();
            }
            if hh > 12_497.0 {
                fl410 = Some(t);
            }
            t += dt;
        }
        let (dist, vlo, tlo) = lifted.expect("it never left the ground");
        println!("take-off at {:.0} t: off after {dist:.0} m and {tlo:.0} s at {:.0} kt, at most {most:.1} deg nose up on the runway", G650_MTOW / 1000.0, vlo * KT);
        // (5,858 ft, 1,786 m, of balanced field at MTOW, sea level, ISA, flaps 20: an
        // engine-out figure; with both, the wheels leave well before that)
        assert!(dist > 900.0 && dist < 1600.0, "{dist}");
        assert!(vlo * KT > 130.0 && vlo * KT < 155.0, "{}", vlo * KT);
        // (the tail cone meets the runway at about 16 degrees)
        assert!(most < 14.0, "{most}");
        let up = fl410.expect("it never reached FL410");
        println!("FL410 after {:.1} min, burning {:.0} kg on the way; M {:.2}, {:.0} kg/h", up / 60.0, g650_fuel_for(G650_MTOW) - p.fuel, p.mach(), p.fuel_flow * 3600.0);
        // (to its cruise altitude in as little as 22 minutes, BJT)
        assert!(up > 15.0 * 60.0 && up < 30.0 * 60.0, "{}", up / 60.0);
        assert!(p.notes.is_empty() && p.dmg.gear.iter().all(|g| *g), "{:?}", p.notes);
    }

    #[test]
    fn a_g650_approaches_and_lands() {
        let w = flat_world();
        let s = G650;
        let vref = 1.23 * g650_vs(G650_MLW, 1.0);
        // On a 3 degree glide at its maximum landing weight, gear down, flaps 39, Vref + 5 kt.
        let v = vref + 5.0 / KT;
        let mut p = Plane::spawn(s, 0.6, -2.1, 600.0, 0.0, v);
        p.fuel = g650_fuel_for(G650_MLW);
        let sink = v * 3f64.to_radians().sin();
        p.configure(3, true, sink);
        let thr = p.throttle;
        let (va, vs) = fly(&mut p, &w, 60.0, thr, |_| 0.0);
        let ld = p.mass() * 9.81 / p.drag_level(va, 600.0, p.flaps, 1.0);
        let pitch = p.forward().dot(p.pos.normalize()).asin().to_degrees();
        println!("approach at {:.1} t: {:.0} kt (Vref {:.0}), {:.0} fpm, alpha {:.1} deg, pitch {pitch:.1} deg, N1 {:.0}%, lift over drag {ld:.1}", p.mass() / 1000.0, va * KT, vref * KT, vs * 196.85, p.alpha.to_degrees(), p.n1 * 100.0);
        assert!((va - v).abs() < 3.0 && (vs + sink).abs() < 1.0, "{va} {vs}");
        assert!(p.alpha.to_degrees() > 3.0 && p.alpha.to_degrees() < 9.0 && !p.stalled && p.notes.is_empty());
        assert!(p.n1 > 0.45 && p.n1 < 0.75 && ld > 5.5 && ld < 9.0, "{} {ld}", p.n1);

        // Over the threshold at 50 ft and Vref, then down: spoilers, brakes and reversers.
        let mut p = Plane::spawn(s, 37.6, -122.37, 15.24, 0.5, vref);
        p.t = 2.0;
        p.fuel = g650_fuel_for(G650_MLW);
        p.configure(3, true, vref * 3f64.to_radians().sin());
        let dt = 1.0 / 120.0;
        let start = p.pos;
        let (mut t, mut down) = (0.0, None);
        while t < 60.0 {
            let touched = down.is_some();
            let c = Controls { throttle: -1.0, brake: touched, reverse: touched, pitch: if touched { -0.1 } else { 0.08 }, ..Default::default() };
            assert!(p.step(dt, &c, &w).is_none(), "{:?}", p.notes);
            if down.is_none() && p.on_ground {
                down = Some(p.pos);
            }
            t += dt;
        }
        let down = down.expect("it never touched down");
        let (air, roll) = ((down - start).length(), (p.pos - down).length());
        println!("landing at {:.1} t from 50 ft at {:.0} kt: {air:.0} m to touch down, {roll:.0} m on the runway; {:.1} m/s after 60 s, spoilers {:.2}, reversers {:.2}, notes {:?}", p.mass() / 1000.0, vref * KT, p.vel.length(), p.spoilers, p.reverser, p.notes);
        // (2,680 ft, 817 m, from 50 ft at MLW, sea level, ISA)
        assert!(p.vel.length() < 1.0 && roll < 1_000.0 && air + roll < 1_300.0, "{} {air} {roll}", p.vel.length());
        assert!(p.notes.is_empty() && p.dmg.gear.iter().all(|g| *g), "{:?}", p.notes);
    }
}
