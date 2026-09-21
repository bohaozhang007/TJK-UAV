@echo off
setlocal
if not defined DEPTH_ENV set "DEPTH_ENV=da3"
where conda.exe >nul 2>nul
if errorlevel 1 (
    if exist "%USERPROFILE%\anaconda3\Scripts\conda.exe" set "PATH=%USERPROFILE%\anaconda3\Scripts;%PATH%"
)
where conda.exe >nul 2>nul
if errorlevel 1 (
    echo Error: conda.exe was not found on PATH.
    exit /b 1
)
conda.exe run --no-capture-output -n "%DEPTH_ENV%" python "%~dp0depth\server.py" %*
exit /b %ERRORLEVEL%
