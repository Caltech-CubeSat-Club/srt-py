import { writable } from 'svelte/store';

/** Holds the current JWT, or null if not yet authenticated this session. */
export const authToken = writable<string | null>(null);

/**
 * Hits GET /auth/token. If the browser hasn't already sent Basic Auth
 * credentials for this origin, the 401 + WWW-Authenticate response
 * triggers the browser's native username/password prompt, the browser
 * retries automatically with the Authorization header attached, and
 * this resolves once that retry succeeds — no custom login form needed.
 *
 * Throws on failure (wrong credentials, user cancelled the prompt) so
 * callers can decide how to handle that (e.g. SvelteKit's `load` can
 * throw a redirect/error).
 */
export async function fetchAuthToken(customFetch: typeof window.fetch): Promise<string> {
	const res = await customFetch('/auth/token');

	if (!res.ok) {
		throw new Error(`Authentication failed (${res.status})`);
	}

	const data = await res.json();
	authToken.set(data.access_token);
	return data.access_token;
}