@echo off
REM ========================================
REM Build Script for RSLidar TouchDesigner Plugin
REM ========================================

echo.
echo ========================================
echo RSLidar TouchDesigner Plugin Builder
echo ========================================
echo.

REM Check if MSBuild exists
set MSBUILD="C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe"
if not exist %MSBUILD% (
    echo ERROR: MSBuild not found at:
    echo %MSBUILD%
    echo.
    echo Please install Visual Studio 2022 Community or update the path in this script.
    echo.
    pause
    exit /b 1
)

REM Check if solution exists
set SOLUTION="build\RSLidarTouchDesigner.sln"
if not exist %SOLUTION% (
    echo ERROR: Solution file not found at:
    echo %SOLUTION%
    echo.
    echo Please run CMake to generate the solution first.
    echo.
    pause
    exit /b 1
)

REM Warning about TouchDesigner
echo WARNING: Make sure TouchDesigner is CLOSED before building!
echo The build will fail if the DLL files are locked.
echo.
choice /C YN /M "Is TouchDesigner closed? Continue with build"
if errorlevel 2 (
    echo Build cancelled.
    pause
    exit /b 0
)

echo.
echo Building RSLidarTOP (Release x64)...
echo.

REM Build RSLidarTOP
%MSBUILD% "build\RSLidarTOP.vcxproj" -p:Configuration=Release -p:Platform=x64 -v:minimal

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ========================================
    echo BUILD FAILED for RSLidarTOP!
    echo ========================================
    echo.
    echo Common issues:
    echo - TouchDesigner is still running (DLL locked)
    echo - Missing dependencies
    echo - Compilation errors
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo BUILD SUCCESSFUL!
echo ========================================
echo.
echo Output file:
echo   build\bin\Release\RSLidarTOP.dll
echo.
echo Note: RSLidarCHOP has been removed. All functionality (point cloud + IMU)
echo       is now available in RSLidarTOP via TOP output and Info CHOP.
echo.
echo You can now load these plugins in TouchDesigner.
echo.
pause
