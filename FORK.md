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

Not done yet: choosing among an aircraft's other liveries; gear `select` conditions (evaluated once at load, so gear doors and wheels can stick); exhaust and afterburner effects (skipped); Nasal-driven animations (F-14 wing sweep).

## Contributing a change back

Make a branch from `main` (not `custom`) holding only that change, push it to `origin`, and open a pull request
to `Rolandjg/fableflight`. Follow the repo's `AGENTS.md`.
