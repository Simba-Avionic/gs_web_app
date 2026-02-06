import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import * as dotenv from 'dotenv'
import { resolve } from 'path'
import { visualizer } from "rollup-plugin-visualizer";

dotenv.config({ path: resolve(__dirname, '../.env') });

export default defineConfig({
  server: {
    host: process.env.IP_ADDRESS || 'localhost',
    port: process.env.SERVER_PORT || 2137,
    timezone: process.env.TIMEZONE || 'UTC',
  },
  define: {
    'process.env.IP_ADDRESS': JSON.stringify(process.env.IP_ADDRESS),
    'process.env.SERVER_PORT': JSON.stringify(process.env.SERVER_PORT),
    'process.env.TIMEZONE': JSON.stringify(process.env.TIMEZONE),
  },
  plugins: [
    svelte(),
    // visualizer({ 
    //   open: true, 
    //   filename: 'bundle-stats.html' 
    // }),
  ],
  build: {
    minify: 'esbuild',
    rollupOptions: {
      output: {
        manualChunks(id) {
          if (id.includes('plotly.js-basic-dist-min')) {
            return 'vendor-plotly';
          }
          if (id.includes('leaflet')) {
            return 'vendor-leaflet';
          }
          if (id.includes('hls.js')) {
            return 'vendor-hls';
          }
          if (id.includes('node_modules')) {
            return 'vendor-others';
          }
        }
      }
    }
  }
})