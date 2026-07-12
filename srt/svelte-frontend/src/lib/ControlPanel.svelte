<script lang="ts">
  import { T, useThrelte } from '@threlte/core'
  import { interactivity, CameraControls, type CameraControlsRef } from '@threlte/extras'
  import * as THREE from 'three'
  import { liveTelescopeData, telescopeState } from '$lib/stores/telescope_state';
  import { azEltoVector3 } from '$lib/coordinates';

  import SkyGrid from '$lib/SkyGrid.svelte';
  import Objects from '$lib/Objects.svelte';
  import BackgroundSphere from '$lib/BackgroundSphere.svelte';

  const PI = Math.PI;
  const RADIUS = 500; // Size of the sky dome
  interface Props {
    controls?: CameraControlsRef;
  }

  let { controls = $bindable() }: Props = $props();

  // Update values from nonreactive high frequency store every frame
  // And other frame-based updates
  let telescopeAzEl = $state([0, 0]);
	$effect(() => {
		let frameId: number;
		function tick() {
      telescopeAzEl = [liveTelescopeData.rotor.az ?? 0, liveTelescopeData.rotor.el ?? 0];
      frameId = requestAnimationFrame(tick);
    }
    frameId = requestAnimationFrame(tick);
		return () => cancelAnimationFrame(frameId);
	});

  // Update values from reactive telescopeState store
  let beamwidth = $state(0); // Beamwidth in degrees
  let beamwidth_on_sky = $state(0); // Convert beamwidth in degrees to a width on the sky sphere of RADIUS
  let beam_vector = $state(new THREE.Vector3(0, 0, 0));
  telescopeState.subscribe((state) => {
    if (state.beamwidth !== undefined) {
      beamwidth = state.beamwidth;
      beamwidth_on_sky = 2*Math.tan((beamwidth/2) * (PI/180)) * RADIUS;
      beam_vector = azEltoVector3(telescopeAzEl[0], telescopeAzEl[1], RADIUS/1.15);
    }
  });


  const { scene } = useThrelte();
  scene.background = new THREE.Color('#0f172a'); // Dark background for the sky
  interactivity();
</script>

<T.PerspectiveCamera
  makeDefault
  fov={60}
  minPolarAngle={PI / 2 - 0.1}
  near={0.01}
  far={RADIUS * 2}
/>

<CameraControls
  bind:ref={controls}
  mouseButtons.wheel={32} // CameraControls.ACTION.ZOOM
  minZoom={.2}
  maxZoom={50}
  azimuthRotateSpeed={-0.5}
  polarRotateSpeed={-0.5}
  oncreate={(ref) => {
    ref.setPosition(0, 0, 1e-5);
    ref.rotateTo(PI, 3*PI / 4, false);
  }}
/>

<T.AmbientLight intensity={0.5} />

<!-- Sky -->
<SkyGrid latitude_deg={37} />
<BackgroundSphere radius={RADIUS}  />

<!-- Ground -->
<!-- <T.Mesh position={[0, -1, 0]} rotation={[-Math.PI / 2, 0, 0]}>
  <T.CircleGeometry args={[RADIUS, 64]} />
  <T.MeshBasicMaterial color="#efffff" transparent opacity={0.8} side={THREE.DoubleSide} />
</T.Mesh> -->

<Objects radius={RADIUS} />

<!-- Horizon -->
<T.Mesh position={[0, RADIUS*Math.sin(PI/12), 0]} rotation={[PI/2, 0, 0]}>
  <T.TorusGeometry args={[RADIUS, 5]} />
  <T.MeshBasicMaterial color="#f00" transparent opacity={0.8} side={THREE.DoubleSide} />
</T.Mesh>

<!-- Telescope position -->
<T.Mesh 
  position={[beam_vector.x, beam_vector.y, beam_vector.z]} 
  onadded={(e) => {
    e.target.lookAt(0, 0, 0);
  }}
  >
  <T.CircleGeometry args={[beamwidth_on_sky/2]} />
  <T.MeshBasicMaterial color="#0f0" toneMapped={false} />
</T.Mesh>
