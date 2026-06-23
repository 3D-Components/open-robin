import { defineConfig } from 'vitest/config'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), tailwindcss()],
  test: {
    environment: 'jsdom',
    setupFiles: './src/test/setup.ts',
    coverage: {
      provider: 'v8',
      reporter: ['text', 'lcovonly'],
      reportsDirectory: 'coverage',
      include: [
        'src/utils/helpers.ts',
        'src/config/aiInputFeatures.ts',
        'src/hooks/useRobinAPI.ts',
        'src/components/ui/**/*.{ts,tsx}',
      ],
      exclude: [
        'src/components/ui/index.ts',
      ],
      thresholds: {
        lines: 90,
        statements: 90,
        functions: 90,
      },
    },
  },
  server: {
    port: 5174,        // avoid conflict with wme-ui container on 5173
    strictPort: true,  // fail instead of silently picking another port
    proxy: {
      '/api': {
        target: 'http://localhost:8001',
        changeOrigin: true,
        rewrite: (path: string) => path.replace(/^\/api/, ''),
      },
    },
  },
})
