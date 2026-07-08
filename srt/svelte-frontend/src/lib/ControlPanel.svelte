<script lang="ts">
  import { T, useThrelte } from '@threlte/core'
  import { interactivity, CameraControls, type CameraControlsRef } from '@threlte/extras'
  import { Text } from '@threlte/extras'
  import * as THREE from 'three'
  import { azEltoVector3 } from '$lib/coordinates'
  import { liveTelescopeData } from '$lib/stores/telescope_state';

  import SkyGrid from '$lib/SkyGrid.svelte';

  let PI = Math.PI;

  interface Props {
    controls?: CameraControlsRef;
  }

  let { controls = $bindable() }: Props = $props();

  const RADIUS = 500; // Size of the sky dome

  let telescopeAzEl = $derived(
    [liveTelescopeData.rotor.az, liveTelescopeData.rotor.el]
  )

  const { scene } = useThrelte();
  scene.background = new THREE.Color('#0f172a'); // Dark background for the sky
  interactivity();
</script>

<T.PerspectiveCamera
  makeDefault
  fov={60}
  minPolarAngle={Math.PI / 2 - 0.1}
  near={0.01}
  far={RADIUS * 2}
/>

<CameraControls
  bind:ref={controls}
  mouseButtons.wheel={32} // CameraControls.ACTION.ZOOM
  minZoom={1}
  maxZoom={10}
  azimuthRotateSpeed={-0.5}
  polarRotateSpeed={-0.5}
  oncreate={(ref) => {
    ref.setPosition(0, 0, 1e-5);
    ref.rotateTo(Math.PI, 3*Math.PI / 4, false);
  }}
/>

<T.AmbientLight intensity={0.5} />

<!-- Sky -->
<SkyGrid latitude_deg={37} />
<T.Mesh position={[0, 0, 0]}>
  <T.SphereGeometry args={[RADIUS, 128, 128]} />
</T.Mesh>

<!-- Ground -->
<T.Mesh position={[0, -1, 0]} rotation={[-Math.PI / 2, 0, 0]}>
  <T.CircleGeometry args={[RADIUS, 64]} />
  <T.MeshBasicMaterial color="#efffff" transparent opacity={0.8} side={THREE.DoubleSide} />
</T.Mesh>

<!-- Objects -->
{#each Object.entries(liveTelescopeData.object_locs) as [name, [az, el]]}
  {@const pos = azEltoVector3(az, el, RADIUS)}
  <T.Mesh position={[pos.x, pos.y, pos.z]}>
    <T.SphereGeometry args={[2, 16, 16]} />
    <T.MeshBasicMaterial color="#f59e0b" toneMapped={false} />
  </T.Mesh>
  <Text
    text={name}
    position={[pos.x/1.1, pos.y/1.1, pos.z/1.1]}
    fontSize={15}
    color={'#0f0'}
    anchorX={'left'}
    anchorY={'bottom'}
    onadded={(e) => {
        e.target.lookAt(0, 0, 0);
    }}
/>
{/each}

<!-- Horizon -->
<T.Mesh position={[0, RADIUS*Math.sin(PI/12)/2, 0]} rotation={[0, 0, 0]}>
  <T.CylinderGeometry args={[RADIUS, RADIUS, RADIUS*Math.sin(PI/12), 64, 1, true]} />
  <T.MeshBasicMaterial color="#f00" transparent opacity={0.8} side={THREE.DoubleSide} />
</T.Mesh>