@echo off

set batpath=%~dp0

cd %batpath%
echo SRE-make at %cd%

make %*

:quit