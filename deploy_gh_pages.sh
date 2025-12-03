#!/bin/bash

# ========================================
# GitHub Pages Deployment Script
# Physical AI & Robotics Platform
# ========================================

set -e  # Exit on error

echo "========================================="
echo "GitHub Pages Deployment Script"
echo "Physical AI & Robotics Platform"
echo "========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in a git repository
if [ ! -d ".git" ]; then
    echo -e "${YELLOW}⚠️  Git repository not initialized${NC}"
    echo "Initializing Git repository..."
    git init

    # Create comprehensive .gitignore
    echo "Creating .gitignore..."
    cat > .gitignore << 'EOF'
# Dependencies
node_modules/
venv/
__pycache__/
*.pyc

# Build outputs
dist/
build/
*.egg-info/

# Environment variables
.env
.env.local
.env.*.local

# Logs
*.log
npm-debug.log*
yarn-debug.log*
yarn-error.log*
logs/

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Process IDs
.pids

# Docusaurus
.docusaurus/
.cache-loader/

# Misc
.temp/
temp/
*.tmp
coverage/
EOF

    echo -e "${GREEN}✓ .gitignore created${NC}"
else
    echo -e "${GREEN}✓ Git repository already initialized${NC}"
fi

# Check if docusaurus config has been updated
echo ""
echo "Checking Docusaurus configuration..."
if grep -q "<YOUR_GITHUB_USERNAME>" web/docusaurus.config.js; then
    echo -e "${RED}❌ ERROR: Docusaurus config not updated!${NC}"
    echo ""
    echo "Please edit web/docusaurus.config.js and replace:"
    echo "  - <YOUR_GITHUB_USERNAME> with your GitHub username"
    echo "  - <REPO_NAME> with your repository name"
    echo ""
    echo "Example:"
    echo "  url: 'https://johndoe.github.io',"
    echo "  baseUrl: '/my-robotics-project/',"
    echo "  organizationName: 'johndoe',"
    echo "  projectName: 'my-robotics-project',"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓ Docusaurus config appears to be updated${NC}"

# Get git user info
GIT_USER=$(git config user.name)
GIT_EMAIL=$(git config user.email)

if [ -z "$GIT_USER" ] || [ -z "$GIT_EMAIL" ]; then
    echo -e "${RED}❌ Git user not configured${NC}"
    echo "Please configure git:"
    echo "  git config --global user.name \"Your Name\""
    echo "  git config --global user.email \"your.email@example.com\""
    exit 1
fi

echo -e "${GREEN}✓ Git user configured: $GIT_USER <$GIT_EMAIL>${NC}"

# Check if there are uncommitted changes
if [ -n "$(git status --porcelain)" ]; then
    echo ""
    echo -e "${YELLOW}⚠️  You have uncommitted changes${NC}"
    echo "Would you like to commit them now? (y/n)"
    read -r response

    if [ "$response" = "y" ] || [ "$response" = "Y" ]; then
        echo ""
        echo "Enter commit message (or press Enter for default):"
        read -r commit_message

        if [ -z "$commit_message" ]; then
            commit_message="Prepare for GitHub Pages deployment"
        fi

        git add .
        git commit -m "$commit_message"
        echo -e "${GREEN}✓ Changes committed${NC}"
    fi
fi

# Check if remote origin exists
if ! git remote get-url origin > /dev/null 2>&1; then
    echo ""
    echo -e "${YELLOW}⚠️  No remote origin configured${NC}"

    # Check if GitHub CLI is available
    if command -v gh &> /dev/null; then
        echo ""
        echo "GitHub CLI detected. Would you like to create a new repository? (y/n)"
        read -r create_repo

        if [ "$create_repo" = "y" ] || [ "$create_repo" = "Y" ]; then
            echo ""
            echo "Enter repository name (default: physical-ai-textbook):"
            read -r repo_name

            if [ -z "$repo_name" ]; then
                repo_name="physical-ai-textbook"
            fi

            echo ""
            echo "Make repository public? (y/n, default: y):"
            read -r is_public

            if [ "$is_public" = "n" ] || [ "$is_public" = "N" ]; then
                visibility="--private"
            else
                visibility="--public"
            fi

            echo "Creating GitHub repository..."
            gh repo create "$repo_name" $visibility --source=. --remote=origin
            echo -e "${GREEN}✓ Repository created and remote added${NC}"
        else
            echo ""
            echo "Please create a repository manually on GitHub and add it as remote:"
            echo "  git remote add origin https://github.com/<username>/<repo>.git"
            exit 1
        fi
    else
        echo ""
        echo "GitHub CLI (gh) not found."
        echo "Please:"
        echo "  1. Create a repository on GitHub"
        echo "  2. Add it as remote: git remote add origin <url>"
        echo "  3. Run this script again"
        echo ""
        echo "Or install GitHub CLI: https://cli.github.com/"
        exit 1
    fi
fi

echo ""
echo -e "${GREEN}✓ Remote origin configured${NC}"

# Check if on main/master branch
current_branch=$(git branch --show-current)
if [ "$current_branch" != "main" ] && [ "$current_branch" != "master" ]; then
    echo -e "${YELLOW}⚠️  Not on main/master branch (current: $current_branch)${NC}"
    echo "It's recommended to deploy from main/master. Continue anyway? (y/n)"
    read -r continue_deploy

    if [ "$continue_deploy" != "y" ] && [ "$continue_deploy" != "Y" ]; then
        echo "Deployment cancelled"
        exit 0
    fi
fi

# Push current branch to remote
echo ""
echo "Pushing current branch to remote..."
git push -u origin "$current_branch" || echo -e "${YELLOW}⚠️  Push failed or branch already up to date${NC}"

# Deploy to GitHub Pages
echo ""
echo "========================================="
echo "Deploying to GitHub Pages..."
echo "========================================="
echo ""

cd web

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    npm install
fi

# Build and deploy
echo "Building and deploying..."
GIT_USER="$GIT_USER" npm run deploy

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo -e "${GREEN}✓ Deployment successful!${NC}"
    echo "========================================="
    echo ""

    # Extract values from config
    org_name=$(grep "organizationName:" docusaurus.config.js | sed "s/.*'\(.*\)'.*/\1/")
    project_name=$(grep "projectName:" docusaurus.config.js | sed "s/.*'\(.*\)'.*/\1/")

    echo "Your site will be available at:"
    echo "  https://$org_name.github.io/$project_name/"
    echo ""
    echo "Note: It may take a few minutes for GitHub Pages to build and deploy"
    echo ""
    echo "To check deployment status:"
    echo "  1. Go to https://github.com/$org_name/$project_name"
    echo "  2. Click on 'Settings' → 'Pages'"
    echo ""
else
    echo ""
    echo -e "${RED}❌ Deployment failed${NC}"
    echo "Please check the error messages above"
    exit 1
fi
