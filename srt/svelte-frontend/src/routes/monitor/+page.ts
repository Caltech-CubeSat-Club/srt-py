import { error } from '@sveltejs/kit';
import { fetchAuthToken } from '$lib/stores/auth';
import type { PageLoad } from './$types';

/**
 * Runs before /monitor renders. Forces the Basic Auth challenge (see
 * fetchAuthToken) to resolve before the page mounts, so the WebSocket
 * connection in +page.svelte always has a token ready rather than
 * racing the auth check.
 */
export const load: PageLoad = async ({ fetch, params }) => {
	try {
		const token = await fetchAuthToken(fetch);
		return { token };
	} catch (e) {
		// Browser prompt was cancelled, or credentials were wrong and
		// the user gave up retrying — show a clear error rather than a
		// half-loaded page.
		throw error(401, 'Login required to view this page.');
	}
};