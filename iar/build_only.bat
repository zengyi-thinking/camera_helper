@echo off
setlocal

set "IAR_ROOT=D:\software\IAR Systems\Embedded Workbench 9.2"
set "IAR_BIN=%IAR_ROOT%\common\bin"
set "PROJ_DIR=%~dp0project_config"

echo.
echo [BUILD] Camera helper CM7_0...
echo.

"%IAR_BIN%\iarbuild.exe" "%PROJ_DIR%\cyt4bb7_cm_7_0.ewp" -make Debug -log warnings
if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Camera helper CM7_0 build failed!
    exit /b 1
)

echo [OK] Camera helper CM7_0 build success
echo.

echo [BUILD] Camera helper CM7_1 empty core...
"%IAR_BIN%\iarbuild.exe" "%PROJ_DIR%\cyt4bb7_cm_7_1.ewp" -make Debug -log warnings
if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Camera helper CM7_1 build failed!
    exit /b 1
)

echo [OK] Camera helper CM7_1 build success
echo.
echo [OK] Camera helper build success
