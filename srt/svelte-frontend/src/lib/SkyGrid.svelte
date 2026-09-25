<script lang="ts">
    import * as THREE from 'three'
    import { T } from '@threlte/core'
    import { uiState } from '$lib/stores/ui.svelte'
    import { RaDec, lst_radians } from '$lib/coordinates.svelte'
    import { timeState } from '$lib/stores/time.svelte';
    import { uFovScale, uAspect } from '$lib/stores/projection.svelte';
    import gridVert from '$lib/shaders/grid.vert';
    import gridFrag from '$lib/shaders/grid.frag';
    import { SKY_DOME_RADIUS as RADIUS, PI, LATITUDE_DEG, GRID_RING_THICKNESS, UI_COLORS } from '$lib/constants';

    const uOpacity = {
        get value() {
            return uiState.raDecGridVisible ? 0.5 : 0.0;
        }
    }

    // Every ring below gets its OWN renderOrder (the `+ i * 0.01`), not one
    // shared value for the layer. gl_Position.z is fixed at 0 under this
    // projection so there's no real depth to sort by - Three falls back to
    // object/material id to break ties between equal renderOrders, and that
    // order isn't stable across recompiles/HMR, which showed up as rings
    // flickering against each other where they cross on screen.
    //
    // frustumCulled={false} for the reason spelled out in
    // StereographicText.svelte: Three tests local geometry bounds against the
    // REAL camera frustum, which knows nothing about our vertex shader, so
    // on-screen rings get culled before the shader ever runs.
    //
    // TODO: these ranges collide with AzElGrid's (dec rings -1.90..-1.49 vs
    // its el rings -1.90..-1.56), so with both grids visible some pairs share
    // a renderOrder again and the tiebreak problem comes back.
</script>

{#snippet dec_ring(dec_deg: number, ringRenderOrder: number)}
    {@const dec = dec_deg * PI / 180}
    {@const lat = LATITUDE_DEG * PI / 180}
    {@const ringRadius = RADIUS * Math.cos(dec)}
    {@const ringCenterVectorLength = RADIUS * Math.sin(dec)}
    <!-- Get North Pole vector -->
    <!-- +Z = North -->
    {@const dz = ringCenterVectorLength * Math.cos(lat) }
    {@const dy = ringCenterVectorLength * Math.sin(lat) }
    <T.Mesh position={[0, dy, dz]} rotation={[-lat, 0, 0]} renderOrder={ringRenderOrder} frustumCulled={false}>
        <T.TorusGeometry args={[ringRadius, GRID_RING_THICKNESS / uFovScale.value, 8, 64]} />
        <T.ShaderMaterial
            vertexShader={gridVert}
            fragmentShader={gridFrag}
            uniforms={{ uColor: { value: new THREE.Color(UI_COLORS.raDecGrid) }, uOpacity, uFovScale, uAspect }}
            side={THREE.DoubleSide}
            transparent
        />
    </T.Mesh>
{/snippet}

{#snippet ra_ring(ra_deg: number, color: string, ringRenderOrder: number)}
    {@const lat = LATITUDE_DEG * PI / 180}
    {@const ra = ra_deg * PI / 180}
    <!-- Get North Pole vector -->
    <!-- +Z = North -->
    <T.Mesh position={[0, 0, 0]} rotation={[PI/2 - lat, - ra - lst_radians(timeState.jd_ut), PI/2]} renderOrder={ringRenderOrder} frustumCulled={false}>
        <T.TorusGeometry args={[
            RADIUS, //radius of torus
            GRID_RING_THICKNESS / uFovScale.value, //radius of tube
            8, //radial segments
            64, // tubular segments
            PI, // arc length
            PI/2, // theta start
            PI // theta length
        ]} />
        <T.ShaderMaterial
            vertexShader={gridVert}
            fragmentShader={gridFrag}
            uniforms={{ uColor: { value: new THREE.Color(color) }, uOpacity, uFovScale, uAspect }}
            side={THREE.DoubleSide}
            transparent
        />
    </T.Mesh>
{/snippet}


{#each Array.from({ length: 42 }) as _, i}
    {@const declination = (i + 1) * 5 - 90}
    {@render dec_ring(declination, -1.9 + i * 0.01)}
{/each}

{#each Array.from({ length: 24 }) as _, i}
    {@const rightAscension = (i + 1) * 15}
    {@render ra_ring(rightAscension, UI_COLORS.raDecGrid, -1.4 + i * 0.01)}
{/each}

<!-- <StereographicText
    text={'N'}
    position={[northPoleVector.x/1.1, northPoleVector.y/1.1, northPoleVector.z/1.1]}
    fontSize={30}
    color={'#f00'}
    anchorX={'left'}
    anchorY={'bottom'}
/> -->
