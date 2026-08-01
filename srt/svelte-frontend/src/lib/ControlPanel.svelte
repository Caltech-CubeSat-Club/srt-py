<script lang="ts">
  import { T, useThrelte, useTask } from '@threlte/core'
  import { interactivity, CameraControls, type CameraControlsRef } from '@threlte/extras'
  import * as THREE from 'three'
  import { SKY_DOME_RADIUS as RADIUS, PI, BASE_FOV_DEG } from '$lib/constants';
  import { setFov, setAspect, uFovScale, uAspect } from '$lib/stores/projection.svelte';
  import gridVert from '$lib/shaders/grid.vert';
  import gridFrag from '$lib/shaders/grid.frag';

  import SkyGrid from '$lib/SkyGrid.svelte';
  import ShaderObjects from '$lib/ShaderObjects.svelte';
  import BackgroundSphere from '$lib/BackgroundSphere.svelte';
  import TelescopeBeam from '$lib/TelescopeBeam.svelte';

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
<BackgroundSphere />
<!-- <Objects /> -->
<ShaderObjects />

<!-- Horizon: fixed at 15deg elevation -->
<T.Mesh position={[0, RADIUS*Math.sin(15*PI/180), 0]} rotation={[PI/2, 0, 0]} renderOrder={-1}>
  <T.TorusGeometry args={[RADIUS*Math.cos(15*PI/180), 1]} />
  <T.ShaderMaterial
    vertexShader={gridVert}
    fragmentShader={gridFrag}
    uniforms={{ uColor: { value: new THREE.Color('#f00') }, uOpacity: { value: 0.8 }, uFovScale, uAspect }}
    side={THREE.DoubleSide}
    transparent
  />
</T.Mesh>
<T.Mesh position={[0, RADIUS*Math.sin(81*PI/180), 0]} rotation={[PI/2, 0, 0]} renderOrder={-1}>
  <T.TorusGeometry args={[RADIUS*Math.cos(81*PI/180), 1]} />
  <T.ShaderMaterial
    vertexShader={gridVert}
    fragmentShader={gridFrag}
    uniforms={{ uColor: { value: new THREE.Color('#f00') }, uOpacity: { value: 0.8 }, uFovScale, uAspect }}
    side={THREE.DoubleSide}
    transparent
  />
</T.Mesh>
<T.Mesh position={[0, RADIUS/1.1, 0]} renderOrder={-1}>
  <T.SphereGeometry args={[2, 16, 16]} />
  <T.ShaderMaterial
    vertexShader={gridVert}
    fragmentShader={gridFrag}
    uniforms={{ uColor: { value: new THREE.Color('#f00') }, uOpacity: { value: 0.8 }, uFovScale, uAspect }}
    side={THREE.DoubleSide}
    transparent
  />
</T.Mesh>

<!-- Telescope position -->
<TelescopeBeam />