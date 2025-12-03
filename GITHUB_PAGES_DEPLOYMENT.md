# GitHub Pages Deployment Guide

This guide explains how to deploy the Physical AI & Robotics Platform frontend to GitHub Pages.

## Prerequisites

1. **Git** installed and configured
   ```bash
   git config --global user.name "Your Name"
   git config --global user.email "your.email@example.com"
   ```

2. **Node.js 18+** and npm installed

3. **GitHub Account** and repository created

4. **(Optional) GitHub CLI** for easier repository creation
   - Install: https://cli.github.com/

## Quick Start

### Step 1: Update Docusaurus Configuration

Edit `web/docusaurus.config.js` and replace `<REPO_NAME>` with your repository name:

```javascript
// Current configuration:
url: 'https://HamdanProfessional.github.io',  // ✓ Already set
baseUrl: '/<REPO_NAME>/',  // ← Replace with your repo name
organizationName: 'HamdanProfessional',  // ✓ Already set
projectName: '<REPO_NAME>',  // ← Replace with your repo name
```

**Example (if your repo is named "robotics-platform"):**
```javascript
url: 'https://HamdanProfessional.github.io',
baseUrl: '/robotics-platform/',
organizationName: 'HamdanProfessional',
projectName: 'robotics-platform',
```

### Step 2: Run Deployment Script

**Windows:**
```bash
scripts\deploy_gh_pages.bat
```

**Linux/macOS:**
```bash
chmod +x scripts/deploy_gh_pages.sh
scripts/deploy_gh_pages.sh
```

The script will:
1. Initialize git repository (if needed)
2. Create `.gitignore` file
3. Validate Docusaurus configuration
4. Commit any uncommitted changes
5. Push to GitHub
6. Build and deploy to GitHub Pages

### Step 3: Enable GitHub Pages

1. Go to your GitHub repository
2. Click **Settings** → **Pages**
3. Under "Source", select the `gh-pages` branch
4. Click **Save**

Your site will be available at: `https://YOUR_USERNAME.github.io/REPO_NAME/`

## Manual Deployment

If you prefer to deploy manually:

### 1. Configure Git Remote

```bash
# Create repository on GitHub, then:
git remote add origin https://github.com/username/repo-name.git
git branch -M main
git push -u origin main
```

### 2. Deploy to GitHub Pages

```bash
cd web
GIT_USER=your-github-username npm run deploy
```

On Windows:
```bash
cd web
set USE_SSH=true
npm run deploy
```

## Deployment Options

### Using SSH (Recommended for frequent deploys)

1. Set up SSH keys: https://docs.github.com/en/authentication/connecting-to-github-with-ssh

2. Use SSH URL for remote:
   ```bash
   git remote set-url origin git@github.com:username/repo.git
   ```

3. Deploy:
   ```bash
   cd web
   USE_SSH=true npm run deploy
   ```

### Using Personal Access Token

1. Create token: https://github.com/settings/tokens

2. Deploy with token:
   ```bash
   cd web
   GIT_USER=username GIT_PASS=your_token npm run deploy
   ```

## Troubleshooting

### Error: "Please tell me who you are"

Configure git user:
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Error: "Permission denied (publickey)"

Use HTTPS instead of SSH or set up SSH keys:
```bash
git remote set-url origin https://github.com/username/repo.git
```

### Error: "Config not updated"

Make sure you've replaced ALL placeholders in `web/docusaurus.config.js`:
- `<YOUR_GITHUB_USERNAME>`
- `<REPO_NAME>`

### Deployment Succeeds but Site Shows 404

1. Check GitHub Pages settings (Settings → Pages)
2. Ensure `gh-pages` branch is selected as source
3. Wait 2-5 minutes for GitHub to build the site
4. Verify the URL matches your baseUrl in config

### Styles Not Loading / Broken Links

This usually means `baseUrl` is incorrect. It should match your repository name exactly:
```javascript
// If repo is: https://github.com/user/my-project
baseUrl: '/my-project/',  // Include leading and trailing slashes!
```

### Want to Use Custom Domain?

1. Add a `static/CNAME` file in the `web/` directory:
   ```
   your-custom-domain.com
   ```

2. Update `docusaurus.config.js`:
   ```javascript
   url: 'https://your-custom-domain.com',
   baseUrl: '/',
   ```

3. Configure DNS: https://docs.github.com/en/pages/configuring-a-custom-domain-for-your-github-pages-site

## Automated Deployment with GitHub Actions

To automatically deploy on push to main:

1. Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: write

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm
          cache-dependency-path: web/package-lock.json

      - name: Install dependencies
        working-directory: web
        run: npm ci

      - name: Build website
        working-directory: web
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./web/build
          publish_branch: gh-pages
```

2. Push to GitHub - deployment will happen automatically on each push to main

## Important Notes

### Backend Services (API & Auth)

**GitHub Pages only hosts static files** - it cannot run the backend services (API and Auth server).

For a fully functional deployment, you need to:

1. **Deploy backend services separately:**
   - **Option A:** Use a cloud platform (Railway, Render, Fly.io, AWS, Google Cloud)
   - **Option B:** Use a VPS with Docker
   - **Option C:** Use serverless functions (Vercel, Netlify Functions)

2. **Update frontend configuration:**
   - Create `web/.env.production` with production API URLs:
     ```
     REACT_APP_API_URL=https://your-api-domain.com
     REACT_APP_AUTH_URL=https://your-auth-domain.com
     ```

3. **Update API client:**
   - Modify `web/src/lib/auth-client.ts` to use production URLs
   - Update CORS settings in backend to allow your GitHub Pages domain

### Environment Variables

GitHub Pages builds are public. **Never commit sensitive data:**
- API keys should be server-side only
- Use environment variables in backend
- Frontend should only have public configuration

### Limitations

- No server-side rendering (SSR)
- No API endpoints
- No database connections
- No authentication server (backend required)
- Static site only

## Complete Production Architecture

For a production deployment, you'd typically have:

```
┌─────────────────────┐
│  GitHub Pages       │  ← Static frontend (Docusaurus)
│  (your-site.io)     │
└──────────┬──────────┘
           │
           │ HTTPS API calls
           │
           ↓
┌─────────────────────┐
│  Backend Server     │  ← FastAPI + Auth server
│  (api.your-site.io) │  ← Hosted on cloud platform
└──────────┬──────────┘
           │
           ↓
┌─────────────────────┐
│  Database           │  ← Neon Postgres
│  (cloud hosted)     │
└─────────────────────┘
```

## Next Steps

After deploying to GitHub Pages:

1. **Deploy backend services** (see `DEPLOYMENT.md` for backend deployment options)
2. **Set up custom domain** (optional)
3. **Configure CI/CD** for automatic deployments
4. **Monitor performance** with GitHub Pages analytics
5. **Set up error tracking** (Sentry, LogRocket, etc.)

## Resources

- [Docusaurus Deployment Guide](https://docusaurus.io/docs/deployment)
- [GitHub Pages Documentation](https://docs.github.com/en/pages)
- [Custom Domain Setup](https://docs.github.com/en/pages/configuring-a-custom-domain-for-your-github-pages-site)
- [GitHub Actions for Pages](https://github.com/marketplace/actions/github-pages-action)

## Support

For deployment issues:
1. Check the troubleshooting section above
2. Review GitHub Pages build logs (Settings → Pages)
3. Check browser console for errors
4. Verify all URLs in configuration match your setup
