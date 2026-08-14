@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
set "SAM2_ROOT=%SCRIPT_DIR%..\sam2"
set "SAM3_ROOT=%SCRIPT_DIR%..\sam3"
set "DA3_ROOT=%SCRIPT_DIR%..\depth-anything-3"
if not defined DA3_ENV set "DA3_ENV=da3"
if not defined SAM3_ENV set "SAM3_ENV=sam3"
if not defined AGENT_ENV set "AGENT_ENV=sam2"
set "MPLCONFIGDIR=%SCRIPT_DIR%.cache\matplotlib"
set "PYTHONPATH=%SAM2_ROOT%;%DA3_ROOT%\src;%SCRIPT_DIR%src;%PYTHONPATH%"
set "USE_DA3=0"
set "USE_SAM3=0"
set "AGENT_ARGS="

where conda.exe >nul 2>nul
if errorlevel 1 (
    echo Error: conda.exe was not found on PATH.
    exit /b 1
)
if not exist "%MPLCONFIGDIR%" mkdir "%MPLCONFIGDIR%"

:parse_args
if "%~1"=="" goto run
if /I "%~1"=="--use_da3" (
    set "USE_DA3=1"
) else (
    if /I "%~1"=="--det" if /I "%~2"=="sam3" set "USE_SAM3=1"
    if /I "%~1"=="--det=sam3" set "USE_SAM3=1"
    set "AGENT_ARGS=!AGENT_ARGS! "%~1""
)
shift
goto parse_args

:run
if "%USE_DA3%"=="1" (
    curl.exe --noproxy "*" -fsS http://127.0.0.1:8770/health >nul 2>nul
    if errorlevel 1 (
        echo [START] Starting DA3 service...
        start "TJK-UAV DA3" /MIN conda.exe run --no-capture-output -n "%DA3_ENV%" python "%SCRIPT_DIR%src\third_party\da3\server.py" --da3-root "%DA3_ROOT%" --model-dir "%DA3_ROOT%\checkpoints\DA3NESTED-GIANT-LARGE"
    )

    echo [START] Waiting for DA3 service...
    for /L %%I in (1,1,120) do (
        curl.exe --noproxy "*" -fsS http://127.0.0.1:8770/health >nul 2>nul
        if not errorlevel 1 goto da3_ready
        timeout /t 1 /nobreak >nul
    )
    echo Error: DA3 service did not become ready within 120 seconds.
    exit /b 1
)

:da3_ready
if "%USE_SAM3%"=="1" (
    curl.exe --noproxy "*" -fsS http://127.0.0.1:8780/health >nul 2>nul
    if errorlevel 1 (
        echo [START] Starting SAM3 service...
        start "TJK-UAV SAM3" /MIN conda.exe run --no-capture-output -n "%SAM3_ENV%" python "%SCRIPT_DIR%src\third_party\sam3\server.py" --sam3-root "%SAM3_ROOT%" --checkpoint "%SAM3_ROOT%\sam3.pt"
    )

    echo [START] Waiting for SAM3 service...
    for /L %%I in (1,1,300) do (
        curl.exe --noproxy "*" -fsS http://127.0.0.1:8780/health >nul 2>nul
        if not errorlevel 1 goto sam3_ready
        timeout /t 1 /nobreak >nul
    )
    echo Error: SAM3 service did not become ready within 300 seconds.
    exit /b 1
)

:sam3_ready
echo [START] Starting Agent v20...
conda.exe run --no-capture-output -n "%AGENT_ENV%" python "%SCRIPT_DIR%src\agent\tjk\v20.py" !AGENT_ARGS!
exit /b %ERRORLEVEL%
