// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Hardware-Adaptive Learning Platform',
  favicon: 'img/favicon.ico',

  // Vercel deployment configuration
  url: 'https://web-chwgs5l23-hamdanprofessionals-projects.vercel.app',
  baseUrl: '/',

  organizationName: 'HamdanProfessional', // GitHub username
  projectName: 'Hackathon_1', // Repository name
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'warn',

  // Custom fields for backend API URLs
  customFields: {
    apiUrl: process.env.API_URL || 'https://api-257cf7htk-hamdanprofessionals-projects.vercel.app',
    authUrl: process.env.AUTH_URL || 'https://auth-8gydz6z5y-hamdanprofessionals-projects.vercel.app',
  },

  // Folder-based routing (NOT i18n plugin per research.md)
  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Edit URL removed for now
        },
        blog: false, // Disable blog feature
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI Textbook',
        items: [
          {
            type: 'doc',
            docId: 'en/intro',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'custom-authNavbarItem',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `Physical AI & Humanoid Robotics Textbook Platform. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;
