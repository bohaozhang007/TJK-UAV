@echo off
setlocal
python "%~dp0scripts\record_fpv.py" %*
exit /b %ERRORLEVEL%
