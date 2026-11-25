@echo off
REM Starts MATLAB, opens the competition project and starts the simulation.
REM Double-click this file instead of opening MATLAB by hand.
start "" "C:\Program Files\MATLAB\R2026b\bin\matlab.exe" -r "cd('%~dp0'); OPEN_PROJECT; set_param('PX4MultiCopterPerceptionAndControl','SimulationCommand','start')"
