@echo off
setlocal

set "VIS_DIR=%~1"
if "%VIS_DIR%"=="" set "VIS_DIR=C:\Users\colab999\Desktop\project\picoStar\vis_omega"

cd /d C:\Users\colab999\Desktop\project\picoStar
call conda activate gsam2_vggt

python omega_proj_server.py --vis-dir "%VIS_DIR%"