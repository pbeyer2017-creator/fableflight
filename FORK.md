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

## Contributing a change back

Make a branch from `main` (not `custom`) holding only that change, push it to `origin`, and open a pull request
to `Rolandjg/fableflight`. Follow the repo's `AGENTS.md`.
