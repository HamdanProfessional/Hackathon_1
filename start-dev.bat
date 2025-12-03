@echo off
REM Development startup script for Physical AI & Robotics Platform
REM This script sets up and starts all three services in parallel

echo ========================================
echo Physical AI & Robotics Platform
echo Development Environment Setup
echo ========================================
echo.

REM Check if .env exists
if not exist ".env" (
    echo [ERROR] .env file not found!
    echo Please create a .env file in the root directory with required variables.
    echo See .env.example for reference.
    pause
    exit /b 1
)

echo [1/4] Checking Python installation...
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is not installed or not in PATH
    echo Please install Python 3.10+ from https://www.python.org/
    pause
    exit /b 1
)
echo [OK] Python is installed
echo.

echo [2/4] Checking Node.js installation...
node --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Node.js is not installed or not in PATH
    echo Please install Node.js 18+ from https://nodejs.org/
    pause
    exit /b 1
)
echo [OK] Node.js is installed
echo.

echo [3/4] Installing dependencies...
echo.

REM Install API dependencies
echo [API] Installing Python dependencies...
cd api
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)
call venv\Scripts\activate.bat
pip install -r requirements.txt --quiet
cd ..
echo [OK] API dependencies installed
echo.

REM Install Auth dependencies
echo [AUTH] Installing Node.js dependencies...
cd auth
if not exist "node_modules" (
    call npm install
) else (
    echo node_modules exists, skipping install...
)
cd ..
echo [OK] Auth dependencies installed
echo.

REM Install Web dependencies
echo [WEB] Installing Node.js dependencies...
cd web
if not exist "node_modules" (
    call npm install
) else (
    echo node_modules exists, skipping install...
)
cd ..
echo [OK] Web dependencies installed
echo.

echo [4/4] Starting services...
echo.
echo Starting all services in new terminal windows...
echo - API (FastAPI): http://localhost:8000
echo - Auth Server: http://localhost:3001
echo - Web Frontend: http://localhost:3000
echo.
echo Press Ctrl+C in each window to stop the services
echo.

REM Start API server in new window
start "API Server (Port 8000)" cmd /k "cd /d %~dp0api && call venv\Scripts\activate.bat && uvicorn src.main:app --reload --host 0.0.0.0 --port 8000"

REM Wait a bit before starting next service
timeout /t 2 /nobreak >nul

REM Start Auth server in new window
start "Auth Server (Port 3001)" cmd /k "cd /d %~dp0auth && npm start"

REM Wait a bit before starting next service
timeout /t 2 /nobreak >nul

REM Start Web frontend in new window
start "Web Frontend (Port 3000)" cmd /k "cd /d %~dp0web && npm start"

echo.
echo ========================================
echo All services started successfully!
echo ========================================
echo.
echo Services:
echo - API: http://localhost:8000
echo - API Docs: http://localhost:8000/docs
echo - Auth: http://localhost:3001
echo - Web: http://localhost:3000
echo.
echo Check the individual terminal windows for logs
echo Close this window to keep services running
echo.

pause
