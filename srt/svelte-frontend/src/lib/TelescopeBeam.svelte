<script lang="ts">
    import { T } from '@threlte/core'
	import * as THREE from 'three';
    import { AzEl } from '$lib/coordinates.svelte'
    import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
    import { uFovScale, uAspect } from '$lib/stores/projection.svelte';
    import { uiState } from '$lib/stores/ui.svelte';
    import beamVert from '$lib/shaders/beam.vert';
    import beamFrag from '$lib/shaders/beam.frag';

    import { SKY_DOME_RADIUS as radius, OBSERVATION_BAND_LAMBDAS, UI_COLORS } from '$lib/constants';

    let lambdaMeters = $derived(OBSERVATION_BAND_LAMBDAS[uiState.observationBand || 'L']);
    let apertureMeters = $state(6.0);
    // Billboard quad edge (local position +/-0.5) sits at this many degrees
    // from the beam center - matches the script's plotted domain radius.
    let edgeThetaRad = $derived(
        2.44 * (lambdaMeters / apertureMeters)
    );

    // Same unitless "edge radius" convention as fovScaleFromDegrees /
    // ShaderObjects' angularSize - the stereographic-projected NDC radius
    // for a point at edgeThetaDeg from view center, before the shared
    // uFovScale multiplier.
    let beamAngularSize = $derived.by(() => {
        const edgeRadius = (2 * Math.sin(edgeThetaRad)) / (1 + Math.cos(edgeThetaRad));
        return 2 * edgeRadius;
    });

    const uLambdaMeters = { get value() { return lambdaMeters; } };
    const uApertureMeters = { get value() { return apertureMeters; } };
    const uEdgeThetaRad = { get value() { return edgeThetaRad; } };
    const uAngularSize = { get value() { return beamAngularSize; } };
    const uColor = { value: new THREE.Color(UI_COLORS.beam) };

    let telescopeAzEl = $derived(
        daemonStatus.rotor
        ? [daemonStatus.rotor.az ?? 0, daemonStatus.rotor.el ?? 0]
        : [0, 0]
    );

    let beam_vector = $derived(new AzEl(telescopeAzEl[0], telescopeAzEl[1]).toArray(radius / 1.15));

	const material = new THREE.ShaderMaterial({
		vertexShader: beamVert,
		fragmentShader: beamFrag,
		uniforms: { uLambdaMeters, uApertureMeters, uEdgeThetaRad, uAngularSize, uColor, uFovScale, uAspect },
		transparent: true,
		depthTest: false,
		depthWrite: false,
	});
</script>

<T.Mesh position={beam_vector} renderOrder={1} frustumCulled={false}>
    <T.PlaneGeometry args={[1, 1]} />
    <T is={material} />
</T.Mesh>