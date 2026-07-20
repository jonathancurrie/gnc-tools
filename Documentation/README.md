# GNC Tools documentation

This is the Docusaurus 3 replacement for the former GNC Tools PmWiki. It
contains the current public documentation and public images, offline search,
and client-side fallback redirects for legacy PmWiki page paths.

## Local development

Requires Node.js 20 or newer and pnpm 11.9.0.

```sh
cd Documentation
pnpm install --frozen-lockfile
pnpm start
```

Create the same production output used by GitHub Pages:

```sh
pnpm build
pnpm serve
```

## GitHub Pages

The default configuration targets
`https://jonathancurrie.github.io/gnc-tools/`. Keep this Docusaurus project in
the `Documentation/` directory of the `jonathancurrie/gnc-tools` repository
and select **GitHub Actions** as the Pages source. The repository-root workflow
at `.github/workflows/deploy-pages.yml` builds from `Documentation/` and
deploys pushes to either `master` or `main`.

Set `SITE_URL` and `BASE_URL` in the build environment to override the hosting
URL without editing `docusaurus.config.js`. `BASE_URL` should include leading
and trailing slashes.

## Migration records

`migration/manifest.json` records every source page, Markdown file, public
route, converted construct count, and migration warning. `migration/page-map.json`
records the curated destination hierarchy.

The public project deliberately excludes the downloaded PmWiki runtime,
configuration, page-revision metadata, logs, credentials, and private keys.
Redirects generated in this static site are browser redirects; configure HTTP
301 redirects on the former web server when retiring the original URLs.
