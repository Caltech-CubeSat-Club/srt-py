<script lang="ts">
    import * as THREE from 'three'
    import { T } from '@threlte/core'
    import { uFovScale, uAspect } from '$lib/stores/projection.svelte';
    import { uiState } from '$lib/stores/ui.svelte';
    import gridVert from '$lib/shaders/grid.vert';
    import gridFrag from '$lib/shaders/grid.frag';
    import { SKY_DOME_RADIUS as RADIUS, PI, GRID_RING_THICKNESS, UI_COLORS } from '$lib/constants';

    const uOpacity = {
        get value() {
            return uiState.azElGridVisible ? 1.0 : 0.0;
        }
    }
</script>

{#snippet el_ring(el_deg: number, ringRenderOrder: number)}
    {@const elRad = el_deg * PI / 180}
    <T.Mesh position={[0, RADIUS * Math.sin(elRad), 0]} rotation={[PI / 2, 0, 0]} renderOrder={ringRenderOrder} frustumCulled={false}>
        <T.TorusGeometry args={[RADIUS * Math.cos(elRad), GRID_RING_THICKNESS / uFovScale.value, 8, 64]} />
        <T.ShaderMaterial
            vertexShader={gridVert}
            fragmentShader={gridFrag}
            uniforms={{ uColor: { value: new THREE.Color(UI_COLORS.azElGrid) }, uOpacity, uFovScale, uAspect }}
            side={THREE.DoubleSide}
            transparent
        />
    </T.Mesh>
{/snippet}

{#snippet az_ring(az_deg: number, color: string, ringRenderOrder: number)}
    {@const az = az_deg * PI / 180}
    <T.Mesh position={[0, 0, 0]} rotation={[0, -az, PI / 2]} renderOrder={ringRenderOrder} frustumCulled={false}>
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


{#each Array.from({ length: 35 }) as _, i}
    {@const elevation = (i + 1) * 5 - 90}
    {@render el_ring(elevation, -1.9 + i * 0.01)}
{/each}

{#each Array.from({ length: 24 }) as _, i}
    {@const azimuth = i * 15}
    {@render az_ring(azimuth, UI_COLORS.azElGrid, -1.4 + i * 0.01)}
{/each}
