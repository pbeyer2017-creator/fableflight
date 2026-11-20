# This fork

A personal fork of [Rolandjg/fableflight](https://github.com/Rolandjg/fableflight) for look-and-feel tweaks and
extra aircraft. The original is MIT-licensed; keep `LICENSE` and its copyright line.

## Branches and remotes

| Name | What it is |
|---|---|
| `upstream` remote | The author's repo. Read-only here (push is disabled). |
| `origin` remote | This fork on GitHub. |
| `main` branch | An exact copy of upstream's `main`. Never commit to it. |
| `custom` branch | Our changes. Work here. |

## Getting the author's updates

```powershell
powershell -ExecutionPolicy Bypass -File scripts\sync-upstream.ps1
cargo run --release
```

To keep merges painless, prefer adding new files (a new `src/aircraft/<name>/` folder, new assets) over rewriting
the author's big shared files such as `src/main.rs`.

## Our changes (on `custom`)

| Change | Files | Checked |
|---|---|---|
| FlightGear imports wear their default livery. The importer runs no Nasal, so it now reads the livery file the set names (`sim/model/livery/file`) and applies `material` animation textures itself. | `src/fgmodel.rs`, `src/hangar.rs` | 3 new unit tests; full `cargo test` (246 pass). Before/after screenshots: P-51D went from blank metal to its "Delivery Day" markings; F-14B unchanged (its default was already right). |
| FlightGear jet nozzles follow the engine. `n1`/`n2` animations got the propeller's spin angle (0..2π, looping), so petals sat splayed and flickered (B-1B); now they get fan speed in percent. Every engine's throttle (`engine[1]`...) moves, not just engine 0's (MiG-29 right nozzle). Interpolation tables are sorted and held at their ends, as FlightGear does. | `src/fgmodel.rs`, `src/airframe.rs`, `src/plane.rs` | New unit test; full `cargo test` (282 pass). Before/after screenshots: B-1B petals no longer splayed; MiG-29 at full throttle, both nozzles now match. |
| FlightGear `select` animations on things we model (gear position, flaps, speedbrake, throttle, fan speed) are checked every frame (`Vis::When`, `Model::conds`), instead of once at load with everything at zero. A wheel shown "when the gear is out" used to be dropped from the model (MiG-29 had no gear). Properties we don't model keep their at-rest value inside the condition. Such parts keep their gear group and role. | `src/fgmodel.rs`, `src/airframe.rs`, `src/main.rs`, `src/plane.rs` | New unit test; full `cargo test` (283 pass). Screenshots: MiG-29 parked now stands on its gear, and hides it in flight with the gear up; B-1B and F-14B unchanged on the ground. |
| Downloaded jets get afterburner plumes: the Hornet's own (its sheath, core and discs, which plane.wgsl lights from `burner`), placed on each nozzle (`Model::placed`) and scaled to it. Engine positions come from YASim `<jet>` / JSBSim thruster locations; each nozzle exit is found on the 3D model (the aftmost skin around the engine's line). Left engines take the Hornet's left plume. The heat-haze layer is left out: it is one shell across both engines and redraws the scene behind it, so moved it painted over the neighbouring flame. | `src/fgmodel.rs`, `src/aircraft/fa18/mod.rs`, `src/airframe.rs`, `src/main.rs`, `src/hangar.rs`, `src/yasim.rs`, `src/jsbsim.rs` | New unit test; full `cargo test` (284 pass). Screenshots in afterburner (`--boost-at`): MiG-29 (both flames from both sides), F-14B, B-1B (four). |

Not done yet: choosing among an aircraft's other liveries; each aircraft's own FlightGear exhaust effects (we draw the Hornet's flame instead, and no dry-power heat haze); Nasal-driven animations (F-14 wing sweep); the at-rest check treats text compares (`sim/current-view/name` equals "Pilot View") as true, which hides pilot figures (left as is: they may block the cockpit camera).

## Contributing a change back

Make a branch from `main` (not `custom`) holding only that change, push it to `origin`, and open a pull request
to `Rolandjg/fableflight`. Follow the repo's `AGENTS.md`.
