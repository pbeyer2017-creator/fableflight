@echo off
rem Start FableFlight from this fork, rebuilding first if anything changed.
rem (A desktop shortcut points here. The first start after an update takes a few minutes.)
cd /d "%~dp0.."
title FableFlight
cargo run --release
if errorlevel 1 (
    echo.
    echo FableFlight stopped with an error. The lines above say why.
    pause
)
