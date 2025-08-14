import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import * as dotenv from 'dotenv'
import { resolve } from 'path'

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
    port: parseInt(process.env.PORT) || 2137,
    timezone: process.env.TIMEZONE || 'UTC',
  },
  define: {
    'process.env.IP_ADDRESS': JSON.stringify(process.env.IP_ADDRESS),
    'process.env.TIMEZONE': JSON.stringify(process.env.TIMEZONE)
  },
  plugins: [
    legacy({ 'node_modules/leader-line/leader-line.min.js': 'LeaderLine' }),
    svelte(),
  ],
})
