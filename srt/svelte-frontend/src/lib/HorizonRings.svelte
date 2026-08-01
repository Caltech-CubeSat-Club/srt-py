<script lang="ts">
    import * as THREE from 'three'
    import { T } from '@threlte/core'
    import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
    import { uFovScale, uAspect } from '$lib/stores/projection.svelte';
    import gridVert from '$lib/shaders/grid.vert';
    import gridFrag from '$lib/shaders/grid.frag';
    import { SKY_DOME_RADIUS as RADIUS, PI } from '$lib/constants';

    // daemonStatus.el_limits is [lowerBoundDeg, upperBoundDeg] - the mount's
    // enforced elevation tracking limits, shown as two flat rings on the sky.
    // Not to be confused with BackgroundSphere's HORIZON_TEXTURE (a
    // photographic panorama) or daemonStatus.horizon_points (a real terrain
    // obstruction profile) - this is just the mount's own safety limits.
    const DEFAULT_EL_LIMITS: [number, number] = [15, 81];
    let elLimits = $derived(daemonStatus.el_limits ?? DEFAULT_EL_LIMITS);
</script>

{#snippet limitRing(elevationDeg: number)}
    {@const elRad = elevationDeg * PI / 180}
    <T.Mesh position={[0, RADIUS * Math.sin(elRad), 0]} rotation={[PI / 2, 0, 0]} renderOrder={-1}>
        <T.TorusGeometry args={[RADIUS * Math.cos(elRad), 1]} />
        <T.ShaderMaterial
            vertexShader={gridVert}
            fragmentShader={gridFrag}
            uniforms={{ uColor: { value: new THREE.Color('#f00') }, uOpacity: { value: 0.8 }, uFovScale, uAspect }}
            side={THREE.DoubleSide}
            transparent
        />
    </T.Mesh>
{/snippet}

{@render limitRing(elLimits[0])}
{@render limitRing(elLimits[1])}