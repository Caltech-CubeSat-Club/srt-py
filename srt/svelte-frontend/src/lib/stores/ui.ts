import { writable } from 'svelte/store';

export const cursorAzEl = writable<[number, number]>([0, 0]);
