// See https://svelte.dev/docs/kit/types#app.d.ts
// for information about these interfaces
import type { InteractivityProps } from '@threlte/extras';
declare global {
	namespace Threlte {
		interface UserProps extends InteractivityProps {}
	}
	namespace App {
		// interface Error {}
		// interface Locals {}
		// interface PageData {}
		// interface PageState {}
		// interface Platform {}
	}
}

export {};
