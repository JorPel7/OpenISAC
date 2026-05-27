import { defineConfig } from "astro/config";

const site = process.env.OPENISAC_SITE_URL ?? "https://openisac.zzw123app.top";
const base = process.env.OPENISAC_SITE_BASE ?? "/";

export default defineConfig({
  site,
  base,
  build: {
    format: "file",
  },
  trailingSlash: "never",
});
