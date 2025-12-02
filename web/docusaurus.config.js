// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Hardware-Adaptive Learning Platform',
  favicon: 'img/favicon.ico',

  url: 'https://your-site.vercel.app',
  baseUrl: '/',

  organizationName: 'ai-robotics',
  projectName: 'textbook-platform',

  onBrokenLinks: 'warn',

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
