@echo off
REM Deletes the old MATLAB Projects copy so PX4MultiCopterPerceptionAndControlStart
REM can unpack a fresh default example again.
REM Close MATLAB and close Cursor's workspace on that folder first.

set TARGET=C:\Users\rikki\MATLAB\Projects\examples\PX4MultiCopterPerceptionAndControl
if not exist "%TARGET%" (
  echo Already gone: %TARGET%
  goto done
)

echo Removing "%TARGET%" ...
rmdir /s /q "%TARGET%"
if exist "%TARGET%" (
  echo FAILED - folder still locked. Close MATLAB + Cursor on that path, then rerun.
  exit /b 1
)

echo Removed. Now in MATLAB run:
echo   PX4MultiCopterPerceptionAndControlStart
:done
pause
