@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
set "CONDA_EXE=%USERPROFILE%\anaconda3\Scripts\conda.exe"
set "USE_DA3=0"
set "USE_SAM3=0"
set "AGENT_ARGS="

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
        start "TJK-UAV DA3" /MIN "%CONDA_EXE%" run --no-capture-output -n gsam2_vggt python "%SCRIPT_DIR%src\third_party\da3\server.py"
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
        start "TJK-UAV SAM3" /MIN "%CONDA_EXE%" run --no-capture-output -n sam3 python "%SCRIPT_DIR%src\third_party\sam3\server.py"
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
echo [START] Starting Agent v18...
"%CONDA_EXE%" run --no-capture-output -n yolo python "%SCRIPT_DIR%src\agent\tjk\v18.py" !AGENT_ARGS!
exit /b %ERRORLEVEL%
