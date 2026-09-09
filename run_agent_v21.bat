@echo off
setlocal
set "TJK_AGENT_VERSION=21"
call "%~dp0run_agent.bat" %*
exit /b %ERRORLEVEL%
