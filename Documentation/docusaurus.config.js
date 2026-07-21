const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;
const migrationManifest = require('./migration/manifest.json');

const normalizeRoute = (route) => {
  if (route === '/') {
    return route;
  }
  return `/${route.replace(/^\/|\/$/g, '')}/`;
};

const legacyPageByRoute = new Map(
  migrationManifest.pages.map((page) => [
    normalizeRoute(page.route),
    page.source.replace('.', '/'),
  ]),
);

async function createConfig() {
  const remarkMath = (await import('remark-math')).default;
  const rehypeKatex = (await import('rehype-katex')).default;

  return {
    title: 'GNC Tools',
    tagline: 'Guidance, navigation, and control engineering reference',
    url: process.env.SITE_URL || 'https://jonathancurrie.github.io',
    baseUrl: process.env.BASE_URL || '/gnc-tools/',
    organizationName: 'jonathancurrie',
    projectName: 'gnc-tools',
    deploymentBranch: 'gh-pages',
    trailingSlash: true,
    onBrokenLinks: 'throw',
    onBrokenAnchors: 'throw',
    onDuplicateRoutes: 'throw',

    markdown: {
      format: 'detect',
    },

    presets: [
      [
        'classic',
        {
          docs: {
            routeBasePath: '/',
            sidebarPath: require.resolve('./sidebars.js'),
            breadcrumbs: true,
            remarkPlugins: [remarkMath],
            rehypePlugins: [rehypeKatex],
          },
          blog: false,
          pages: false,
          theme: {
            customCss: require.resolve('./src/css/custom.css'),
          },
          gtag: {
            trackingID: 'G-9GBVJ653GH',
          },
          sitemap: {
            changefreq: 'monthly',
            priority: 0.5,
          },
        },
      ],
    ],

    plugins: [
      [
        '@docusaurus/plugin-client-redirects',
        {
          createRedirects(existingPath) {
            const legacyPage = legacyPageByRoute.get(normalizeRoute(existingPath));
            return legacyPage ? [`/pmwiki.php/${legacyPage}`] : undefined;
          },
        },
      ],
      [
        require.resolve('@cmfcmf/docusaurus-search-local'),
        {
          indexDocs: true,
          indexBlog: false,
          indexPages: false,
          language: 'en',
          maxSearchResults: 8,
        },
      ],
    ],

    themeConfig: {
      metadata: [
        {
          name: 'description',
          content:
            'Guidance, navigation, and control engineering tutorials, reference material, and tools.',
        },
      ],
      colorMode: {
        defaultMode: 'light',
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'GNC Tools',
        hideOnScroll: true,
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            to: '/control/pid/',
            label: 'PID control',
            position: 'left',
          },
          {
            to: '/control/filtering/',
            label: 'Filtering',
            position: 'left',
          },
          {
            href: 'https://github.com/jonathancurrie/gnc-tools',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'GNC Tools',
            items: [
              {label: 'What is GNC?', to: '/overview/what-is-gnc/'},
              {label: 'PID control', to: '/control/pid/'},
              {label: 'Digital filtering', to: '/control/filtering/'},
            ],
          },
          {
            title: 'Resources',
            items: [
              {label: 'Software', to: '/resources/software/'},
              {label: 'Nomenclature', to: '/overview/nomenclature/'},
              {
                label: 'GitHub',
                href: 'https://github.com/jonathancurrie/gnc-tools',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} <a href="https://www.controlengineering.co.nz/">Control Engineering</a>.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['matlab'],
      },
    },
  };
}

module.exports = createConfig;
