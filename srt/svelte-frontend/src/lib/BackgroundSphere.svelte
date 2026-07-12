<script lang="ts">
	import { T } from '@threlte/core';
    import { interactivity } from '@threlte/extras';
	import * as THREE from 'three';
    import { pointToAzEl } from '$lib/coordinates';
    import { cursorAzEl } from '$lib/stores/ui';

	let { radius }: { radius: number } = $props();

	let geometry = $state.raw<THREE.SphereGeometry>();

	$effect(() => {
		if (geometry) {
			updateUVs();
		}
	});

	function updateUVs() {
		if (!geometry) return;
		const uvAttribute = geometry.attributes.uv;

		for (let i = 0; i < uvAttribute.count; i++) {
			// Get current horizontal (u) and vertical (v) values
			let u = uvAttribute.getX(i);
			let v = uvAttribute.getY(i);

			// Apply the transformation to the vertical coordinate
			v = v + 0.18;
			u = 1 - u;

			// Update the attribute array
			uvAttribute.setXY(i, u, v);
		}

		// Tell Three.js that the UV data has changed and needs an update
		uvAttribute.needsUpdate = true;
	}

	const texture = new THREE.TextureLoader().load('/moore_roof.jpg');
	texture.wrapS = THREE.ClampToEdgeWrapping;
	texture.wrapT = THREE.ClampToEdgeWrapping;
	texture.colorSpace = THREE.SRGBColorSpace; // Ensure correct color space
	const material = new THREE.MeshBasicMaterial({ map: texture, side: THREE.BackSide });

    interactivity();
</script>

<T.Mesh 
    rotation={[0, 4.37, 0]} 
    position={[0, 0, 0]}
    onpointermove={(e) => {
        const point = e.point;
        const azEl = pointToAzEl(point, radius * 1.1);
        cursorAzEl.set(azEl);
    }}>
	<T.SphereGeometry args={[radius * 1.1, 64, 64]} bind:ref={geometry} />
	<T is={material} />
</T.Mesh>
