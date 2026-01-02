import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'HemiHex Docs',
  tagline: 'Dinosaurs are cool',
  favicon: 'img/favicon.ico',
  future: { v4: true },

  url: 'https://docs.hemihex.com',
  baseUrl: '/',
  trailingSlash: false,

  organizationName: 'matinmlk',
  projectName: 'hemihex-docs',
  deploymentBranch: 'gh-pages',
  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // ✅ MUST be top-level (not inside themeConfig)
  markdown: { mermaid: true },
  themes: ['@docusaurus/theme-mermaid'],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'hh101',
        path: 'docs/hh101',
        routeBasePath: 'hh-101',          // URL: /hh-101/...
        sidebarPath: require.resolve('./sidebars-hh101.ts'),
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'hhbot',
        path: 'docs/hhbot',
        routeBasePath: 'hh-bot',         // URL: /hh-bot/...
        sidebarPath: require.resolve('./sidebars-hhbot.ts'),
      },
    ],
  ],

  presets: [
    [
      'classic',
      {
        docs: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],


  themeConfig: {
  image: 'img/logo.png',
  colorMode: { respectPrefersColorScheme: true },
  navbar: {
    logo: {
      alt: 'HemiHex',
      src: 'img/logo.png',
      href: '/docs/intro',
    },
    items: [
      { type: 'docSidebar', sidebarId: 'hh101Sidebar', docsPluginId: 'hh101', label: 'HH-101', position: 'left' },
      { type: 'docSidebar', sidebarId: 'hhbotSidebar', docsPluginId: 'hhbot', label: 'HH-Bot', position: 'left' },
    ],
  },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [{ label: 'HH-101 Jetson', to: '/docs/intro' }],
        },
        {
          title: 'Community',
          items: [
            { label: 'Matin Tech Lab', href: 'https://matintechlab.com' },
            { label: 'Matin Engineer', href: 'https://matin.engineer' },
          ],
        },
        {
          title: 'More',
          items: [{ label: 'GitHub', href: 'https://github.com/matinmlk/hemihex' }],
        },
      ],
      copyright: `Copyright HemiHex© ${new Date().getFullYear()}.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
