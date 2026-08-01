// ui.svelte.ts
import { AzEl } from '$lib/coordinates.svelte';

export const cursorAzEl = $state(new AzEl(0, 0));

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
