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
				execSync('python3 ../scripts/generate_ts_types.py', {
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

export default defineConfig({ plugins: [tailwindcss(), pydanticTypeGen(), sveltekit()] });
