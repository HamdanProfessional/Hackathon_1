# Development Scripts

This directory contains automated scripts to simplify running the Physical AI & Robotics Platform.

## Quick Start

### Windows

```bash
# Start all services (API, Auth, Web)
start-dev.bat

# Build for production
build.bat
```

### Linux/macOS

```bash
# Make scripts executable (first time only)
chmod +x start-dev.sh stop-dev.sh

# Start all services
./start-dev.sh

# Stop all services
./stop-dev.sh
```

## Scripts Overview

### `start-dev.bat` (Windows)
- Checks for Python and Node.js installation
- Creates Python virtual environment if needed
- Installs all dependencies (API, Auth, Web)
- Starts all three services in separate terminal windows
- Services:
  - **API**: http://localhost:8000 (FastAPI + Uvicorn)
  - **Auth**: http://localhost:3001 (Better-Auth server)
  - **Web**: http://localhost:3000 (Docusaurus frontend)

### `start-dev.sh` (Linux/macOS)
- Same functionality as Windows version
- Runs services in background
- Logs output to `logs/` directory
- Saves process IDs to `.pids` file for easy cleanup

### `stop-dev.sh` (Linux/macOS)
- Stops all running services
- Uses saved PIDs or finds processes by port
- Cleans up PID file

### `build.bat` (Windows)
- Builds all components for production
- Creates optimized bundles:
  - Auth: `auth/dist/`
  - Web: `web/build/`
  - API: Prepares virtual environment

## Prerequisites

### Required Software

1. **Python 3.10+**
   - Download: https://www.python.org/downloads/
   - Verify: `python --version` or `python3 --version`

2. **Node.js 18+**
   - Download: https://nodejs.org/
   - Verify: `node --version`

3. **npm** (comes with Node.js)
   - Verify: `npm --version`

### Environment Setup

1. Create `.env` file in the root directory:
   ```bash
   # Database
   DATABASE_URL=postgresql://...

   # API Keys
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key

   # Auth
   BETTER_AUTH_SECRET=your_secret_key
   BETTER_AUTH_URL=http://localhost:3001

   # Frontend
   FRONTEND_URL=http://localhost:3000
   ```

2. (Optional) Create `.env.local` files in subdirectories for local overrides

## Manual Setup (Alternative)

If you prefer to run services manually:

### API (FastAPI)
```bash
cd api
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate.bat
pip install -r requirements.txt
uvicorn src.main:app --reload --port 8000
```

### Auth Server
```bash
cd auth
npm install
npm start  # Development
npm run build  # Production build
```

### Web Frontend
```bash
cd web
npm install
npm start  # Development
npm run build  # Production build
```

## Service Ports

| Service | Development Port | Description |
|---------|-----------------|-------------|
| API | 8000 | FastAPI backend with RAG |
| API Docs | 8000/docs | Swagger UI documentation |
| Auth | 3001 | Better-Auth authentication server |
| Web | 3000 | Docusaurus frontend |

## Troubleshooting

### Port Already in Use

**Windows:**
```bash
# Find process using port 8000
netstat -ano | findstr :8000

# Kill process (replace PID)
taskkill /PID <PID> /F
```

**Linux/macOS:**
```bash
# Find and kill process on port 8000
lsof -ti:8000 | xargs kill -9
```

### Dependencies Not Installing

**Python:**
```bash
# Upgrade pip
python -m pip install --upgrade pip

# Clear cache and reinstall
pip cache purge
pip install -r requirements.txt --no-cache-dir
```

**Node.js:**
```bash
# Clear npm cache
npm cache clean --force

# Delete node_modules and reinstall
rm -rf node_modules package-lock.json
npm install
```

### Database Connection Issues

1. Verify `DATABASE_URL` in `.env` is correct
2. Check if database is accessible
3. Run migrations:
   ```bash
   cd auth
   node run-migration.js
   ```

### Auth Server Not Starting

1. Check if TypeScript is compiled:
   ```bash
   cd auth
   npm run build
   ```

2. Verify port 3001 is not in use
3. Check `auth/dist/index.js` exists

## Development Tips

### Hot Reload

All services support hot reload in development mode:
- **API**: Uvicorn auto-reloads on file changes
- **Auth**: Nodemon watches TypeScript files
- **Web**: Webpack dev server hot-reloads React components

### Viewing Logs

**Windows:** Check individual terminal windows

**Linux/macOS:**
```bash
# View logs
tail -f logs/api.log
tail -f logs/auth.log
tail -f logs/web.log

# View all logs together
tail -f logs/*.log
```

### Testing API Endpoints

1. Open browser: http://localhost:8000/docs
2. Use Swagger UI to test endpoints
3. Or use curl:
   ```bash
   curl http://localhost:8000/health
   ```

## Production Deployment

### Build Process

```bash
# Windows
build.bat

# Linux/macOS
chmod +x build.sh
./build.sh
```

### Running in Production

1. **Auth Server:**
   ```bash
   cd auth
   NODE_ENV=production node dist/index.js
   ```

2. **Web Frontend:**
   - Serve `web/build/` with nginx, Apache, or Node.js static server
   - Example with serve:
     ```bash
     npm install -g serve
     serve -s web/build -p 3000
     ```

3. **API:**
   ```bash
   cd api
   source venv/bin/activate
   uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
   ```

### Environment Variables for Production

Update `.env` with production values:
- Use production database URL
- Set secure `BETTER_AUTH_SECRET`
- Update `FRONTEND_URL` to production domain
- Enable HTTPS URLs

## Support

For issues or questions:
1. Check logs in `logs/` directory (Linux/macOS)
2. Review individual terminal windows (Windows)
3. Verify all prerequisites are installed
4. Check `.env` configuration
5. Consult main README.md for detailed documentation
