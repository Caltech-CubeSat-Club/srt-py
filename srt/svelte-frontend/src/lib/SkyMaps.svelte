<script lang="ts">
	import { T } from '@threlte/core';
    import { interactivity } from '@threlte/extras';
	import * as THREE from 'three';
    import { SKY_DOME_RADIUS as radius, STOCKERT_SURVEY_TEXTURE, PI, LATITUDE_DEG } from '$lib/constants';
    import { uFovScale, uAspect } from '$lib/stores/projection.svelte';
    import { Galactic } from '$lib/coordinates.svelte'
    import { timeState } from '$lib/stores/time.svelte';
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
			u = 1 - u;

			// Update the attribute array
			uvAttribute.setXY(i, u, v);
		}

		// Tell Three.js that the UV data has changed and needs an update
		uvAttribute.needsUpdate = true;
	}

	// THREE.SphereGeometry's default UV parametrization, after the u = 1 - u
	// flip above, puts three fixed reference points exactly on this mesh's
	// local +X/+Y/+Z axes: (glon=0, glat=0) on +X, (any glon, glat=+90, the
	// galactic pole) on +Y, and (glon=-90, glat=0) on +Z. That last one
	// encodes the same left/right ambiguity as the u flip above - if the
	// map ends up mirrored (east/west swapped), change it to glon=+90.
	// Mapping each of those three points to where it actually belongs right
	// now (via Galactic -> RaDec -> AzEl, exactly like SkyGrid's RA/Dec
	// rings) and reading the result off as a basis matrix gives the whole
	// sphere's orientation in one shot - not just one calibrated point, but
	// the map's roll too, and it updates live with time.
	let rotation = $derived.by((): [number, number, number] => {
		const jd_ut = timeState.jd_ut;
		const xAxis = new Galactic(0, 0).toAzEl(jd_ut).toVector3(1);
		const yAxis = new Galactic(0, 90).toAzEl(jd_ut).toVector3(1);
		const zAxis = new Galactic(-90, 0).toAzEl(jd_ut).toVector3(1);
		const basis = new THREE.Matrix4().makeBasis(xAxis, yAxis, zAxis);
		const euler = new THREE.Euler().setFromRotationMatrix(basis);
		return [euler.x, euler.y, euler.z];
	});

	const texture = new THREE.TextureLoader().load(STOCKERT_SURVEY_TEXTURE);
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
		uniforms: { map: { value: texture }, uFovScale, uAspect, uOpacity: { value: 0.4 } },
		side: THREE.BackSide,
		depthTest: false,
		depthWrite: false,
		// Needed for skyDome.frag's alpha fade to actually blend against
		// whatever's behind it (the scene's own clear color, since this
		// sphere draws first/furthest-back) instead of writing fully opaque
		// regardless of the alpha it outputs.
		transparent: true,
	});

    interactivity();
</script>

<T.Mesh
    {rotation}
    position={[0, 0, 0]}
    renderOrder={-3}>
	<T.SphereGeometry args={[radius * 1.1, 64, 64]} bind:ref={geometry} />
	<T is={material} />
</T.Mesh>
