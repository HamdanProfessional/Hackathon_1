@echo off
REM ========================================
REM GitHub Pages Deployment Script (Windows)
REM Physical AI & Robotics Platform
REM ========================================

setlocal enabledelayedexpansion

echo =========================================
echo GitHub Pages Deployment Script
echo Physical AI ^& Robotics Platform
echo =========================================
echo.

REM Check if we're in a git repository
if not exist ".git" (
    echo [WARNING] Git repository not initialized
    echo Initializing Git repository...
    git init

    REM Create comprehensive .gitignore
    echo Creating .gitignore...
    (
        echo # Dependencies
        echo node_modules/
        echo venv/
        echo __pycache__/
        echo *.pyc
        echo.
        echo # Build outputs
        echo dist/
        echo build/
        echo *.egg-info/
        echo.
        echo # Environment variables
        echo .env
        echo .env.local
        echo .env.*.local
        echo.
        echo # Logs
        echo *.log
        echo npm-debug.log*
        echo yarn-debug.log*
        echo yarn-error.log*
        echo logs/
        echo.
        echo # IDE
        echo .vscode/
        echo .idea/
        echo *.swp
        echo *.swo
        echo *~
        echo.
        echo # OS
        echo .DS_Store
        echo Thumbs.db
        echo.
        echo # Process IDs
        echo .pids
        echo.
        echo # Docusaurus
        echo .docusaurus/
        echo .cache-loader/
        echo.
        echo # Misc
        echo .temp/
        echo temp/
        echo *.tmp
        echo coverage/
    ) > .gitignore

    echo [OK] .gitignore created
) else (
    echo [OK] Git repository already initialized
)

REM Check if docusaurus config has been updated
echo.
echo Checking Docusaurus configuration...
findstr /C:"<YOUR_GITHUB_USERNAME>" web\docusaurus.config.js >nul
if !errorlevel! == 0 (
    echo [ERROR] Docusaurus config not updated!
    echo.
    echo Please edit web\docusaurus.config.js and replace:
    echo   - ^<YOUR_GITHUB_USERNAME^> with your GitHub username
    echo   - ^<REPO_NAME^> with your repository name
    echo.
    echo Example:
    echo   url: 'https://johndoe.github.io',
    echo   baseUrl: '/my-robotics-project/',
    echo   organizationName: 'johndoe',
    echo   projectName: 'my-robotics-project',
    echo.
    pause
    exit /b 1
)

echo [OK] Docusaurus config appears to be updated

REM Get git user info
for /f "tokens=*" %%i in ('git config user.name') do set GIT_USER=%%i
for /f "tokens=*" %%i in ('git config user.email') do set GIT_EMAIL=%%i

if "!GIT_USER!"=="" (
    echo [ERROR] Git user not configured
    echo Please configure git:
    echo   git config --global user.name "Your Name"
    echo   git config --global user.email "your.email@example.com"
    pause
    exit /b 1
)

echo [OK] Git user configured: !GIT_USER! ^<!GIT_EMAIL!^>

REM Check if there are uncommitted changes
git status --porcelain > nul 2>&1
if not errorlevel 1 (
    echo.
    echo [WARNING] You have uncommitted changes
    echo Would you like to commit them now? (y/n)
    set /p response=

    if /i "!response!"=="y" (
        echo.
        echo Enter commit message (or press Enter for default):
        set /p commit_message=

        if "!commit_message!"=="" (
            set commit_message=Prepare for GitHub Pages deployment
        )

        git add .
        git commit -m "!commit_message!"
        echo [OK] Changes committed
    )
)

REM Check if remote origin exists
git remote get-url origin >nul 2>&1
if errorlevel 1 (
    echo.
    echo [WARNING] No remote origin configured
    echo.
    echo Please:
    echo   1. Create a repository on GitHub
    echo   2. Add it as remote: git remote add origin ^<url^>
    echo   3. Run this script again
    echo.
    echo Or use GitHub CLI (gh) if installed
    pause
    exit /b 1
)

echo [OK] Remote origin configured

REM Get current branch
for /f "tokens=*" %%i in ('git branch --show-current') do set current_branch=%%i

REM Push current branch to remote
echo.
echo Pushing current branch to remote...
git push -u origin !current_branch! || echo [WARNING] Push failed or branch already up to date

REM Deploy to GitHub Pages
echo.
echo =========================================
echo Deploying to GitHub Pages...
echo =========================================
echo.

cd web

REM Install dependencies if needed
if not exist "node_modules" (
    echo Installing dependencies...
    call npm install
)

REM Build and deploy
echo Building and deploying...
set USE_SSH=true
call npm run deploy

if errorlevel 0 (
    echo.
    echo =========================================
    echo [OK] Deployment successful!
    echo =========================================
    echo.
    echo Your site will be available at:
    echo   Check your repository settings for the exact URL
    echo.
    echo Note: It may take a few minutes for GitHub Pages to build and deploy
    echo.
    echo To check deployment status:
    echo   1. Go to your GitHub repository
    echo   2. Click on 'Settings' -^> 'Pages'
    echo.
) else (
    echo.
    echo [ERROR] Deployment failed
    echo Please check the error messages above
    pause
    exit /b 1
)

cd ..
pause
