@echo off
setlocal enabledelayedexpansion
for /L %%i in (1,1,30) do (
    ping -n 1 -w 500 10.87.27.%%i >nul 2>&1
    if !errorlevel!==0 echo 10.87.27.%%i is alive
)