import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import { svelteSVG } from 'rollup-plugin-svelte-svg'
import * as dotenv from 'dotenv'
import { resolve } from 'path'
import legacy from '@rollup/plugin-legacy';

dotenv.config({ path: resolve(__dirname, '../.env') });

// https://vitejs.dev/config/
export default defineConfig({
  optimizeDeps: {
    force: process.env.VITE_FORCE_REBUILD === "true",
    exclude: ['leader-line']
  },
  server: {
    watch: {
      usePolling: true
    },
    host: process.env.IP_ADDRESS || 'localhost',
    port: parseInt(process.env.PORT) || 2137
  },
  define: {
    'process.env.IP_ADDRESS': JSON.stringify(process.env.IP_ADDRESS)
  },
  plugins: [
    legacy({ 'node_modules/leader-line/leader-line.min.js': 'LeaderLine' }),
    svelte(),
    svelteSVG({
      svgo: {},
      enforce: "pre",
    })
  ],
})
