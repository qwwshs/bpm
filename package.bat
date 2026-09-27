@echo off
setlocal EnableExtensions
pushd "%~dp0"
if errorlevel 1 (
    echo Error: cannot open the project directory.
    exit /b 1
)

set "GCC="
for /f "delims=" %%G in ('where gcc.exe 2^>nul') do if not defined GCC set "GCC=%%G"
if not defined GCC if exist "C:\msys64\mingw64\bin\gcc.exe" set "GCC=C:\msys64\mingw64\bin\gcc.exe"
if not defined GCC if exist "C:\msys64\ucrt64\bin\gcc.exe" set "GCC=C:\msys64\ucrt64\bin\gcc.exe"
if not defined GCC (
    echo Error: GCC was not found. Install MinGW-w64 or MSYS2 GCC and add gcc.exe to PATH.
    popd
    exit /b 1
)

set "DIST_DIR=%CD%\dist"
set "PACKAGE_DIR=%DIST_DIR%\bpm-windows-x64"
set "ZIP_PATH=%DIST_DIR%\bpm-windows-x64.zip"

if not exist "%DIST_DIR%" mkdir "%DIST_DIR%"
if errorlevel 1 goto :failed
if exist "%PACKAGE_DIR%" rmdir /s /q "%PACKAGE_DIR%"
if errorlevel 1 goto :failed
mkdir "%PACKAGE_DIR%"
if errorlevel 1 goto :failed

echo Compiling bpm.c with "%GCC%"...
"%GCC%" -std=c11 -O2 -Wall -Wextra -Wpedantic -o "%PACKAGE_DIR%\bpm.exe" bpm.c -lm -lwinmm -lshell32
if errorlevel 1 goto :failed

copy /y README.md "%PACKAGE_DIR%\README.md" >nul
if errorlevel 1 goto :failed

echo Creating "%ZIP_PATH%"...
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "Compress-Archive -Path '.\dist\bpm-windows-x64\*' -DestinationPath '.\dist\bpm-windows-x64.zip' -Force"
if errorlevel 1 goto :failed

echo Package created: %ZIP_PATH%
popd
exit /b 0

:failed
echo Error: package creation failed.
popd
exit /b 1
