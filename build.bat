@echo off
REM Production build script for Physical AI & Robotics Platform

echo ========================================
echo Physical AI & Robotics Platform
echo Production Build
echo ========================================
echo.

REM Check if .env exists
if not exist ".env" (
    echo [ERROR] .env file not found!
    echo Please create a .env file in the root directory with required variables.
    pause
    exit /b 1
)

echo [1/3] Building Auth Server...
cd auth
call npm install
call npm run build
if errorlevel 1 (
    echo [ERROR] Auth build failed!
    cd ..
    pause
    exit /b 1
)
cd ..
echo [OK] Auth server built successfully
echo.

echo [2/3] Building Web Frontend...
cd web
call npm install
call npm run build
if errorlevel 1 (
    echo [ERROR] Web build failed!
    cd ..
    pause
    exit /b 1
)
cd ..
echo [OK] Web frontend built successfully
echo.

echo [3/3] Preparing API...
cd api
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)
call venv\Scripts\activate.bat
pip install -r requirements.txt
cd ..
echo [OK] API prepared successfully
echo.

echo ========================================
echo Build completed successfully!
echo ========================================
echo.
echo Built artifacts:
echo - Auth: auth/dist/
echo - Web: web/build/
echo - API: Ready to run with uvicorn
echo.
echo To run in production mode:
echo 1. Set NODE_ENV=production
echo 2. Run: node auth/dist/index.js
echo 3. Serve web/build/ with a static file server
echo 4. Run: uvicorn src.main:app --host 0.0.0.0 --port 8000
echo.

pause
