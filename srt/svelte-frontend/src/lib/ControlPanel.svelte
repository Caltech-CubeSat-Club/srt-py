<script lang="ts">
  import { T, useThrelte, useTask } from '@threlte/core'
  import { interactivity, CameraControls, type CameraControlsRef } from '@threlte/extras'
  import * as THREE from 'three'
  import { SKY_DOME_RADIUS as RADIUS, PI, BASE_FOV_DEG } from '$lib/constants';
  import { setFov, setAspect, uFovScale } from '$lib/stores/projection.svelte';
  import { uiState } from '$lib/stores/ui.svelte';

  import SkyGrid from '$lib/SkyGrid.svelte';
  import AzElGrid from '$lib/AzElGrid.svelte';
  import ShaderObjects from '$lib/ShaderObjects.svelte';
  import HorizonTexture from '$lib/HorizonTexture.svelte';
  import SkyMaps from '$lib/SkyMaps.svelte';
  import TelescopeBeam from '$lib/TelescopeBeam.svelte';
  import HorizonRings from '$lib/HorizonRings.svelte';

  interface Props {
    controls?: CameraControlsRef;
  }

  let { controls = $bindable() }: Props = $props();
  let camera = $state.raw<THREE.PerspectiveCamera>();

  const threlteCtx = useThrelte();
  const { scene } = threlteCtx;
  scene.background = new THREE.Color('#0f172a'); // Dark background for the sky
  interactivity();

  // CameraControls' ZOOM action (mouseButtons.wheel below) writes directly to
  // camera.zoom, which normally only feeds camera.projectionMatrix - but our
  // stereographic-projection materials bypass that matrix entirely and read
  // uFovScale instead (see projection.svelte.ts). Without this, scroll-to-zoom
  // would move the real camera.zoom while every custom-shader layer stayed put.
  useTask(() => {
    if (!camera) return;
    const baseHalfFovRad = (BASE_FOV_DEG * PI / 180) / 2;
    const effectiveHalfFovRad = Math.atan(Math.tan(baseHalfFovRad) / camera.zoom);
    setFov((effectiveHalfFovRad * 2 * 180) / PI);
    // Camera.aspect is real THREE.PerspectiveCamera state, kept up to date by
    // Threlte on canvas resize - our custom shaders bypass projectionMatrix
    // entirely, so they need this mirrored into uAspect the same way as FOV.
    setAspect(camera.aspect);
  });
</script>

<T.PerspectiveCamera
  bind:ref={camera}
  makeDefault
  fov={BASE_FOV_DEG}
  minPolarAngle={PI / 2 - 0.1}
  near={0.01}
  far={RADIUS * 2}
/>

<CameraControls
  bind:ref={controls}
  mouseButtons.wheel={32} // CameraControls.ACTION.ZOOM
  minZoom={.05}
  maxZoom={250}
  azimuthRotateSpeed={-0.5 / uFovScale.value}
  polarRotateSpeed={-0.5 / uFovScale.value}
  oncreate={(ref) => {
    ref.setPosition(0, 0, 1e-5);
    ref.rotateTo(0, PI, false);
  }}
/>

<T.AmbientLight intensity={0.5} />

<!-- Sky -->
<SkyGrid />
<AzElGrid />
<HorizonTexture />
<SkyMaps />
<!-- <Objects /> -->
<ShaderObjects />

<!-- Horizon: mount elevation tracking limits -->
{#if uiState.azElLimitsVisible}
  <HorizonRings />
{/if}

<!-- Telescope position -->
<TelescopeBeam />