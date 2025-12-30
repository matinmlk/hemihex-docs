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

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/matinmlk/hemihex-docs',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: { respectPrefersColorScheme: true },
    navbar: {
      title: '',
      logo: { alt: 'HemiHex Logo', src: 'img/logo.png' },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Tutorial',
        },
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
