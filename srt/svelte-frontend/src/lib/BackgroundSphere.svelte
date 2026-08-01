<script lang="ts">
	import { T } from '@threlte/core';
    import { interactivity } from '@threlte/extras';
	import * as THREE from 'three';
    import { AzEl } from '$lib/coordinates.svelte';
    import { cursorAzEl } from '$lib/stores/ui.svelte';
    import { SKY_DOME_RADIUS as radius, HORIZON_TEXTURE, HORIZON_TEXTURE_ROTATION_DEG, HORIZON_TEXTURE_VERTICAL_OFFSET } from '$lib/constants';
    import { uFovScale, uAspect, inverseStereographicToViewDirection } from '$lib/stores/projection.svelte';
    import skyDomeVert from '$lib/shaders/skyDome.vert';
    import skyDomeFrag from '$lib/shaders/skyDome.frag';

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
			v = v + HORIZON_TEXTURE_VERTICAL_OFFSET;
			u = 1 - u;

			// Update the attribute array
			uvAttribute.setXY(i, u, v);
		}

		// Tell Three.js that the UV data has changed and needs an update
		uvAttribute.needsUpdate = true;
	}

	const texture = new THREE.TextureLoader().load(HORIZON_TEXTURE);
	texture.wrapS = THREE.ClampToEdgeWrapping;
	texture.wrapT = THREE.ClampToEdgeWrapping;
	texture.colorSpace = THREE.SRGBColorSpace; // Ensure correct color space
	// Every stereographic-projected material writes a fixed gl_Position.z (no
	// real depth concept in this projection - see chunks/stereographic.glsl),
	// so normal depth testing against it is meaningless and, worse, this sphere
	// covers the whole screen and would occlude anything real-perspective-
	// projected behind it. Layering between these sky layers is instead
	// controlled explicitly via renderOrder (most-background first).
	const material = new THREE.ShaderMaterial({
		vertexShader: skyDomeVert,
		fragmentShader: skyDomeFrag,
		uniforms: { map: { value: texture }, uFovScale, uAspect },
		side: THREE.BackSide,
		depthTest: false,
		depthWrite: false,
	});

    interactivity();
</script>

<T.Mesh
    rotation={[0, HORIZON_TEXTURE_ROTATION_DEG * Math.PI / 180, 0]}
    position={[0, 0, 0]}
    renderOrder={-2}
    onpointermove={(e) => {
        // e.point is a real 3D raycast hit against this sphere's actual
        // geometry, using the real perspective camera - unrelated to what's
        // visually on screen via the custom stereographic shader. That
        // mismatch grows with zoom (effective FOV vs. the camera's real
        // FOV/zoom diverge more), so we invert the same projection the
        // shader uses instead, from the raw NDC pointer position.
        const viewDir = inverseStereographicToViewDirection(e.pointer.x, e.pointer.y);
        const worldDir = viewDir.applyQuaternion(e.camera.quaternion);
        const worldPoint = worldDir.multiplyScalar(radius * 1.1);
        const azEl = AzEl.fromVector3(worldPoint, radius * 1.1);
        cursorAzEl.az_deg = azEl.az_deg;
        cursorAzEl.el_deg = azEl.el_deg;
    }}>
	<T.SphereGeometry args={[radius * 1.1, 64, 64]} bind:ref={geometry} />
	<T is={material} />
</T.Mesh>
