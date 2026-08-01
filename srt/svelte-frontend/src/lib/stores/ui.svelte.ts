// ui.svelte.ts
import { AzEl } from '$lib/coordinates.svelte';

// AzEl's own fields are $state (see coordinates.svelte.ts) so this instance
// is already reactive per-field without needing an outer $state() wrapper.
export const cursorAzEl = new AzEl(0, 0);

export interface UIState {
    raDecGridVisible?: boolean;
    azElGridVisible?: boolean;
    horizonTextureVisible?: boolean;
    azElLimitsVisible?: boolean;
    observationBand?: 'UHF' | 'L' | 'S' | 'C';
}

export const uiState = $state<UIState>({
    raDecGridVisible: true,
    azElGridVisible: false,
    horizonTextureVisible: true,
    azElLimitsVisible: true,
    observationBand: 'L',
});
