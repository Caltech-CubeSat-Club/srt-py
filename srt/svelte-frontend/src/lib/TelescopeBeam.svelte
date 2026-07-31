<script lang="ts">
    import { T } from '@threlte/core'
	import * as THREE from 'three';
    import { azEltoVector3 } from '$lib/coordinates'
    import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
    import { uFovScale } from '$lib/stores/projection.svelte';
    import beamVert from '$lib/shaders/beam.vert';
    import beamFrag from '$lib/shaders/beam.frag';

    import { SKY_DOME_RADIUS as radius } from '$lib/constants';

    // Same physical constants as scripts/plot_beam_pattern.py's Airy pattern
    // for a uniformly-illuminated circular aperture. Live-computed in
    // beam.frag instead of sampling a static texture, so the pattern stays
    // physically calibrated. $state (not consts) so these are tunable at
    // runtime rather than requiring a shader edit.
    let lambdaMeters = $state(0.21);
    let apertureMeters = $state(6.0);
    // Billboard quad edge (local position +/-0.5) sits at this many degrees
    // from the beam center - matches the script's plotted domain radius.
    let edgeThetaDeg = $state(2.5);

    // Same unitless "edge radius" convention as fovScaleFromDegrees /
    // ShaderObjects' angularSize - the stereographic-projected NDC radius
    // for a point at edgeThetaDeg from view center, before the shared
    // uFovScale multiplier.
    let beamAngularSize = $derived.by(() => {
        const edgeThetaRad = (edgeThetaDeg * Math.PI) / 180;
        const edgeRadius = (2 * Math.sin(edgeThetaRad)) / (1 + Math.cos(edgeThetaRad));
        return 2 * edgeRadius;
    });

    const uLambdaMeters = { get value() { return lambdaMeters; } };
    const uApertureMeters = { get value() { return apertureMeters; } };
    const uEdgeThetaDeg = { get value() { return edgeThetaDeg; } };
    const uAngularSize = { get value() { return beamAngularSize; } };

    let telescopeAzEl = $derived(
        daemonStatus.rotor
        ? [daemonStatus.rotor.az ?? 0, daemonStatus.rotor.el ?? 0]
        : [0, 0]
    );

    let beam_vector = $derived(azEltoVector3(telescopeAzEl[0], telescopeAzEl[1], radius / 1.15));

	const material = new THREE.ShaderMaterial({
		vertexShader: beamVert,
		fragmentShader: beamFrag,
		uniforms: { uLambdaMeters, uApertureMeters, uEdgeThetaDeg, uAngularSize, uFovScale },
		transparent: true,
		depthTest: false,
		depthWrite: false,
	});
</script>

<T.Mesh position={[beam_vector.x, beam_vector.y, beam_vector.z]} renderOrder={1}>
    <T.PlaneGeometry args={[1, 1]} />
    <T is={material} />
</T.Mesh>