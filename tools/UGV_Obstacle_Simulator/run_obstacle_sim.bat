@echo off
title UGV Obstacle Simulator
cd /d "%~dp0"
where python >nul 2>nul && (python --version >nul 2>nul) && (python obstacle_sim.py & goto end)
where py >nul 2>nul && (py -3 obstacle_sim.py & goto end)
echo.
echo  Python 3 was not found.
echo  Install it from https://www.python.org/downloads/  (tick "Add python.exe to PATH"
echo  and keep the default "tcl/tk and IDLE" option), then run this file again.
echo.
:end
pause
