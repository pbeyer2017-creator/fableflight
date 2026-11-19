# Pull the original author's latest FableFlight into this fork.
#
#   main    = an exact copy of upstream (Rolandjg/fableflight). Never commit here.
#   custom  = our changes. upstream's updates are merged into it.
#
# Run from anywhere:  powershell -ExecutionPolicy Bypass -File scripts\sync-upstream.ps1
# If the merge hits a conflict, the script stops and says which files need a hand fix.

$ErrorActionPreference = 'Continue'
Set-Location (Split-Path $PSScriptRoot -Parent)

function Git-Step([string]$what) {
    Write-Host "> git $what" -ForegroundColor Cyan
    git @($what -split ' ')
    if ($LASTEXITCODE -ne 0) { throw "git $what failed (exit $LASTEXITCODE)" }
}

if (git status --porcelain --untracked-files=no) {
    throw 'You have uncommitted changes. Commit them on the custom branch first, then rerun.'
}

Git-Step 'fetch upstream'
Git-Step 'switch main'
Git-Step 'merge --ff-only upstream/main'
Git-Step 'push origin main'
Git-Step 'switch custom'

$new = git rev-list --count custom..main
if ([int]$new -eq 0) {
    Write-Host 'custom already has everything from upstream.' -ForegroundColor Green
} else {
    Write-Host "Merging $new new upstream commit(s) into custom..." -ForegroundColor Cyan
    git merge main --no-edit
    if ($LASTEXITCODE -ne 0) {
        Write-Host 'Merge conflict. These files changed on both sides:' -ForegroundColor Yellow
        git diff --name-only --diff-filter=U
        Write-Host 'Fix them (or ask Claude to), then: git add <files>; git commit; git push origin custom'
        exit 1
    }
    Write-Host 'Merged. Rebuild with: cargo run --release' -ForegroundColor Green
}

Git-Step 'push origin custom'
