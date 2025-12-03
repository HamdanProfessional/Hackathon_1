#!/bin/bash

# Development startup script for Physical AI & Robotics Platform
# This script sets up and starts all three services in parallel

echo "========================================"
echo "Physical AI & Robotics Platform"
echo "Development Environment Setup"
echo "========================================"
echo ""

# Check if .env exists
if [ ! -f ".env" ]; then
    echo "[ERROR] .env file not found!"
    echo "Please create a .env file in the root directory with required variables."
    echo "See .env.example for reference."
    exit 1
fi

echo "[1/4] Checking Python installation..."
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3 is not installed or not in PATH"
    echo "Please install Python 3.10+ from https://www.python.org/"
    exit 1
fi
echo "[OK] Python is installed: $(python3 --version)"
echo ""

echo "[2/4] Checking Node.js installation..."
if ! command -v node &> /dev/null; then
    echo "[ERROR] Node.js is not installed or not in PATH"
    echo "Please install Node.js 18+ from https://nodejs.org/"
    exit 1
fi
echo "[OK] Node.js is installed: $(node --version)"
echo ""

echo "[3/4] Installing dependencies..."
echo ""

# Install API dependencies
echo "[API] Installing Python dependencies..."
cd api
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi
source venv/bin/activate
pip install -r requirements.txt --quiet
cd ..
echo "[OK] API dependencies installed"
echo ""

# Install Auth dependencies
echo "[AUTH] Installing Node.js dependencies..."
cd auth
if [ ! -d "node_modules" ]; then
    npm install
else
    echo "node_modules exists, skipping install..."
fi
cd ..
echo "[OK] Auth dependencies installed"
echo ""

# Install Web dependencies
echo "[WEB] Installing Node.js dependencies..."
cd web
if [ ! -d "node_modules" ]; then
    npm install
else
    echo "node_modules exists, skipping install..."
fi
cd ..
echo "[OK] Web dependencies installed"
echo ""

echo "[4/4] Starting services..."
echo ""
echo "Starting all services in background..."
echo "- API (FastAPI): http://localhost:8000"
echo "- Auth Server: http://localhost:3001"
echo "- Web Frontend: http://localhost:3000"
echo ""

# Create a log directory
mkdir -p logs

# Start API server in background
echo "Starting API server..."
cd api
source venv/bin/activate
nohup uvicorn src.main:app --reload --host 0.0.0.0 --port 8000 > ../logs/api.log 2>&1 &
API_PID=$!
cd ..
echo "[OK] API server started (PID: $API_PID)"

# Wait a bit before starting next service
sleep 2

# Start Auth server in background
echo "Starting Auth server..."
cd auth
nohup npm start > ../logs/auth.log 2>&1 &
AUTH_PID=$!
cd ..
echo "[OK] Auth server started (PID: $AUTH_PID)"

# Wait a bit before starting next service
sleep 2

# Start Web frontend in background
echo "Starting Web frontend..."
cd web
nohup npm start > ../logs/web.log 2>&1 &
WEB_PID=$!
cd ..
echo "[OK] Web frontend started (PID: $WEB_PID)"

# Save PIDs to file for stop script
echo "$API_PID" > .pids
echo "$AUTH_PID" >> .pids
echo "$WEB_PID" >> .pids

echo ""
echo "========================================"
echo "All services started successfully!"
echo "========================================"
echo ""
echo "Services:"
echo "- API: http://localhost:8000"
echo "- API Docs: http://localhost:8000/docs"
echo "- Auth: http://localhost:3001"
echo "- Web: http://localhost:3000"
echo ""
echo "Process IDs:"
echo "- API: $API_PID"
echo "- Auth: $AUTH_PID"
echo "- Web: $WEB_PID"
echo ""
echo "Logs are being written to the logs/ directory"
echo "- API: logs/api.log"
echo "- Auth: logs/auth.log"
echo "- Web: logs/web.log"
echo ""
echo "To stop all services, run: ./stop-dev.sh"
echo "Or manually kill processes: kill $API_PID $AUTH_PID $WEB_PID"
echo ""
