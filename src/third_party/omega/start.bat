@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "PROJECT_ROOT=%SCRIPT_DIR%..\..\.."
set "VIS_DIR=%~1"
if "%VIS_DIR%"=="" set "VIS_DIR=%PROJECT_ROOT%\logs\omega"

cd /d "%PROJECT_ROOT%"
call conda activate gsam2_vggt

python "%SCRIPT_DIR%server.py" --vis-dir "%VIS_DIR%"
