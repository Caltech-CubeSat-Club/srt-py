export const SKY_DOME_RADIUS = 500.0;
export const PI = Math.PI;
export const LATITUDE_DEG = 34.140;
export const LONGITUDE_DEG = -118.124;
export const BASE_FOV_DEG = 160;

export const HORIZON_TEXTURE = '/moore_roof.webp';
export const HORIZON_TEXTURE_ROTATION_DEG = 250.3826;
export const HORIZON_TEXTURE_VERTICAL_OFFSET = 0.02;

export const STOCKERT_SURVEY_TEXTURE = '/stockert.jpg';

export const OBSERVATION_BAND_LAMBDAS: Record<string, number> = {
    'UHF': 0.5,
    'L': 0.21,
    'S': 0.1,
    'C': 0.05,
};

export const GRID_RING_THICKNESS = 0.3;

// Single source of truth for "this UI toggle controls that rendered thing" -
// shared between AntennaStatePanel's toolbar button styling and the actual
// shader/ring colors, so a button's color always matches what it toggles.
export const UI_COLORS = {
    azElGrid: '#f97316',
    raDecGrid: '#9ca3af',
    elLimits: '#ef4444',
    skyline: '#c2a878',
    beam: '#22c55e',
} as const;