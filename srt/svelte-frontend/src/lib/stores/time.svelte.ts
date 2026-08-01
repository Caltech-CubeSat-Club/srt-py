import { daemonStatus } from './daemonStatus.svelte';

// Daemon status pushes arrive roughly every 500ms; ticking the browser-time
// fallback at the same rate keeps it just as smooth without waking up
// reactive consumers (e.g. a UI clock readout) any more often than needed.
const TICK_INTERVAL_MS = 500;

// A single exported const object, not exported `let`/`$derived` bindings -
// Svelte's cross-module reactivity for reassigned exports is unreliable, so
// mutate this object's own fields instead of reassigning the export itself
// (same pattern as daemonStatus/uiState).
export const timeState = $state({
    mode: 'live' as 'live' | 'custom',
    customTime: Date.now() / 1000,
    tick: Date.now() / 1000,

    // Seconds since epoch: daemon time when connected, else ticking browser
    // time, unless a custom time has been explicitly set for previewing.
    get time(): number {
        return this.mode === 'custom' ? this.customTime : (daemonStatus.time ?? this.tick);
    },

    // Julian Date (UTC) form of `time` - what coordinates.svelte's Az/El and
    // sidereal-time conversions expect.
    get jd_ut(): number {
        return this.time / 86400 + 2440587.5;
    },
});

setInterval(() => {
    timeState.tick = Date.now() / 1000;
}, TICK_INTERVAL_MS);

export function setCustomTime(t: number) {
    timeState.mode = 'custom';
    timeState.customTime = t;
}

export function goLive() {
    timeState.mode = 'live';
}
