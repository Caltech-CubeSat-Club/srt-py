import tailwindcss from '@tailwindcss/vite';
import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig, type Plugin } from 'vite';
import { execSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const REPO_ROOT = path.resolve(fileURLToPath(import.meta.url), '../../');

/**
 * Regenerates frontend/src/lib/generated/types.ts from the Pydantic
 * models in backend/app/models/ before dev server start and before
 * build. This makes "the TS types are stale" structurally impossible
 * to forget — it runs as a side effect of just starting work.
 *
 * If you ever need to skip it (e.g. CI without Python available),
 * set SKIP_TYPE_GEN=1.
 */
function pydanticTypeGen(): Plugin {
	return {
		name: 'pydantic-type-gen',
		buildStart() {
			if (process.env.SKIP_TYPE_GEN) return;
			try {
				execSync('python3 scripts/generate_ts_types.py', {
					cwd: REPO_ROOT,
					stdio: 'inherit'
				});
			} catch (err) {
				// Fail the build/dev-start loudly rather than silently
				// serving stale types.
				throw new Error(
					'Failed to regenerate TS types from Pydantic models. ' +
						'See output above. (Set SKIP_TYPE_GEN=1 to bypass.)'
				);
			}
		}
	};
}

export default defineConfig({ 
	plugins: [tailwindcss(), pydanticTypeGen(), sveltekit()],
	server: {
		// Dev-only: pnpm run dev serves the Svelte app on its own origin
		// (typically localhost:5173), separate from FastAPI/uvicorn. In
		// production, Caddy + app.frontend() serve everything from one
		// origin, so the app's relative fetch('/auth/token') and
		// `${window.location.host}` WebSocket URLs just work without any
		// proxy. This config exists ONLY to make those same relative
		// URLs work during local dev, by having Vite's dev server
		// transparently forward matching requests to the real FastAPI
		// backend running on 127.0.0.1:8080 (per DASHBOARD_HOST/PORT).
		proxy: {
			'/auth': {
				target: 'http://127.0.0.1:8080',
				changeOrigin: true
			},
			'/ws': {
				// Deliberately http:// here, not ws:// — there's a known
				// Vite issue (vitejs/vite#20223) where some configurations
				// with a ws:// target fail to upgrade the connection at
				// all and silently fall back to plain HTTP. Using an
				// http:// target with ws: true lets Vite's own proxy
				// middleware detect and perform the upgrade itself, which
				// is the documented reliable form.
				target: 'http://127.0.0.1:8080',
				ws: true,
				changeOrigin: true
			}
		}
	}
 });
