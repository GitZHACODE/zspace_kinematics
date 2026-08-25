@echo off
setlocal EnableExtensions

set "ROOT=%~dp0"
for %%I in ("%ROOT%.") do set "ROOT=%%~fI"
if not defined BUILD_DIR set "BUILD_DIR=%ROOT%\build"
set "LLVM_BIN=C:\Program Files\LLVM\bin"
set "VSDEV=C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat"
set "VS_NINJA=C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"

if not exist "%LLVM_BIN%\clang++.exe" (
  echo Clang was not found at "%LLVM_BIN%".
  exit /b 1
)
if not exist "%VSDEV%" (
  echo Visual Studio developer environment was not found at "%VSDEV%".
  exit /b 1
)

call "%VSDEV%" -arch=x64 -host_arch=x64
if errorlevel 1 exit /b %errorlevel%

set "PATH=%LLVM_BIN%;%PATH%"
set "NINJA=ninja"
where ninja >nul 2>nul
if errorlevel 1 set "NINJA=%VS_NINJA%"
if not exist "%NINJA%" if not "%NINJA%"=="ninja" (
  echo Ninja was not found. Install Ninja or update VS_NINJA in this script.
  exit /b 1
)

cmake -G Ninja -S "%ROOT%" -B "%BUILD_DIR%" ^
  -DCMAKE_CXX_COMPILER=clang++ ^
  -DCMAKE_MAKE_PROGRAM="%NINJA%"
if errorlevel 1 exit /b %errorlevel%

cmake --build "%BUILD_DIR%" --parallel
if errorlevel 1 exit /b %errorlevel%

ctest --test-dir "%BUILD_DIR%" --output-on-failure
exit /b %errorlevel%
