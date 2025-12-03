#!/bin/bash

# Stop all development services

echo "========================================"
echo "Stopping all services..."
echo "========================================"
echo ""

if [ ! -f ".pids" ]; then
    echo "[WARNING] .pids file not found"
    echo "Attempting to find and kill processes by port..."

    # Kill processes on specific ports
    echo "Killing process on port 8000 (API)..."
    lsof -ti:8000 | xargs kill -9 2>/dev/null && echo "[OK] API stopped" || echo "[INFO] No process on port 8000"

    echo "Killing process on port 3001 (Auth)..."
    lsof -ti:3001 | xargs kill -9 2>/dev/null && echo "[OK] Auth stopped" || echo "[INFO] No process on port 3001"

    echo "Killing process on port 3000 (Web)..."
    lsof -ti:3000 | xargs kill -9 2>/dev/null && echo "[OK] Web stopped" || echo "[INFO] No process on port 3000"
else
    # Read PIDs from file
    API_PID=$(sed -n '1p' .pids)
    AUTH_PID=$(sed -n '2p' .pids)
    WEB_PID=$(sed -n '3p' .pids)

    echo "Stopping API (PID: $API_PID)..."
    kill $API_PID 2>/dev/null && echo "[OK] API stopped" || echo "[INFO] API process not found"

    echo "Stopping Auth (PID: $AUTH_PID)..."
    kill $AUTH_PID 2>/dev/null && echo "[OK] Auth stopped" || echo "[INFO] Auth process not found"

    echo "Stopping Web (PID: $WEB_PID)..."
    kill $WEB_PID 2>/dev/null && echo "[OK] Web stopped" || echo "[INFO] Web process not found"

    # Remove PID file
    rm .pids
fi

echo ""
echo "========================================"
echo "All services stopped"
echo "========================================"
echo ""
