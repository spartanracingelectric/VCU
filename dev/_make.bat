@echo off

set batpath=%~dp0

:doitagain

cls
make

pause >nul

goto doitagain

CHOICE /C qwertyuiopasdfghjklzxcvbnm1234567890 /N /M "[Q]uit?"
IF ERRORLEVEL 1 SET M=1
IF ERRORLEVEL 2 SET M=2
IF %M%==1 GOTO quit
IF %M%==2 GOTO doitagain

:quit