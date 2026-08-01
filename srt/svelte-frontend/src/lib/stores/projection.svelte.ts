// projection.svelte.ts
import { Vector3 } from 'three';
import type { Camera } from 'three';
import { BASE_FOV_DEG } from '$lib/constants';

function fovScaleFromDegrees(fovDeg: number): number {
  const halfFovRad = (fovDeg * Math.PI / 180) / 2;
  const edgeRadius = 2 * Math.sin(halfFovRad) / (1 + Math.cos(halfFovRad));
  return 1 / edgeRadius;
}

let fovDeg = $state(BASE_FOV_DEG);
let aspect = $state(1);

export const uFovScale = {
  get value() {
    return fovScaleFromDegrees(fovDeg);
  }
};

// Camera aspect ratio (width/height) - see chunks/stereographic.glsl for why
// every projection needs this to avoid stretching circles into ellipses on
// a non-square canvas. Kept in sync with the real camera via setAspect(),
// called each frame alongside setFov() (see ControlPanel.svelte's useTask).
export const uAspect = {
  get value() {
    return aspect;
  }
};

export function setFov(deg: number) {
  fovDeg = deg;
}

export function setAspect(a: number) {
  aspect = a;
}

/**
 * JS mirror of stereographicProject() in chunks/stereographic.glsl (the
 * point/billboard variant), for forward-projecting a world position into
 * the same screen-space NDC coordinates the shader renders it at. Needed
 * for anything that has to compare screen positions (e.g. cursor-to-marker
 * selection) since Three's own Raycaster/projectionMatrix-based unprojection
 * doesn't know about this custom projection at all.
 */
export function projectToScreenNDC(worldPos: Vector3, camera: Camera): { x: number; y: number } {
  const viewPos = worldPos.clone().applyMatrix4(camera.matrixWorldInverse);
  const dir = viewPos.normalize();
  const denom = Math.max(1 - dir.z, 1e-4);
  const scale = (2 / denom) * fovScaleFromDegrees(fovDeg);
  return { x: (dir.x * scale) / aspect, y: dir.y * scale };
}

/**
 * Inverse of stereographicProject(): given a screen-space NDC coordinate,
 * returns the view-space direction it corresponds to. Algebraically solves
 * screenXY = dir.xy * (2/(1-dir.z)) * fovScale together with |dir| = 1 for
 * dir - closed form, not an approximation. Needed anywhere a real screen
 * position (a mouse event) has to be turned into a sky direction - Three's
 * own Raycaster/camera.projectionMatrixInverse doesn't know about this
 * custom projection, so raycasting against real geometry gives an
 * increasingly wrong answer as the effective FOV widens away from the real
 * camera's actual FOV/zoom.
 */
export function inverseStereographicToViewDirection(ndcX: number, ndcY: number): Vector3 {
  const fovScale = fovScaleFromDegrees(fovDeg);
  const sx = (ndcX * aspect) / fovScale;
  const sy = ndcY / fovScale;
  const r2 = sx * sx + sy * sy;
  const z = (r2 - 4) / (r2 + 4);
  const k = 4 / (r2 + 4);
  return new Vector3(sx * k, sy * k, z);
}