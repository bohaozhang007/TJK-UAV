@echo off
setlocal
if not defined TRACKER_ENV set "TRACKER_ENV=sam2"
where conda.exe >nul 2>nul
if errorlevel 1 (
    if exist "%USERPROFILE%\anaconda3\Scripts\conda.exe" set "PATH=%USERPROFILE%\anaconda3\Scripts;%PATH%"
)
where conda.exe >nul 2>nul
if errorlevel 1 (
    echo Error: conda.exe was not found on PATH.
    exit /b 1
)
conda.exe run --no-capture-output -n "%TRACKER_ENV%" python "%~dp0tracker\server.py" %*
exit /b %ERRORLEVEL%
