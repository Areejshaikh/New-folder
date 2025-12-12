// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Test Book',
  tagline: 'Test site',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://test.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/test/',

  // GitHub pages deployment config.
  organizationName: 'test', 
  projectName: 'test', 

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: false,
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
        title: 'Test Site',
        logo: {
          alt: 'Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Docs',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `Copyright © ${new Date().getFullYear()} Test.`,
      },
    }),
};

module.exports = config;