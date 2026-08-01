<script lang="ts">
    import * as THREE from 'three'
    import { T } from '@threlte/core'
    import StereographicText from '$lib/StereographicText.svelte'
    import { RaDec, lst_radians } from '$lib/coordinates.svelte'
    import { timeState } from '$lib/stores/time.svelte';
    import { uFovScale } from '$lib/stores/projection.svelte';
    import gridVert from '$lib/shaders/grid.vert';
    import gridFrag from '$lib/shaders/grid.frag';
    import { SKY_DOME_RADIUS as RADIUS, PI, LATITUDE_DEG } from '$lib/constants';

    let northPoleVector = $derived(new RaDec(0, 90).toAzEl(timeState.jd_ut).toVector3()); // North Pole is at ra=0, dec=90
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
    <T.Mesh position={[0, dy, dz]} rotation={[-lat, 0, 0]} renderOrder={ringRenderOrder}>
        <T.TorusGeometry args={[ringRadius, .75 / uFovScale.value, 8, 64]} />
        <T.ShaderMaterial
            vertexShader={gridVert}
            fragmentShader={gridFrag}
            uniforms={{ uColor: { value: new THREE.Color('#bbb') }, uOpacity: { value: 1.0 }, uFovScale }}
            side={THREE.DoubleSide}
        />
    </T.Mesh>
{/snippet}

{#snippet ra_ring(ra_deg: number, color: string, ringRenderOrder: number)}
    {@const lat = LATITUDE_DEG * PI / 180}
    {@const ra = ra_deg * PI / 180}
    <!-- Get North Pole vector -->
    <!-- +Z = North -->
    <T.Mesh position={[0, 0, 0]} rotation={[PI/2 - lat, - ra - lst_radians(timeState.jd_ut), PI/2]} renderOrder={ringRenderOrder}>
        <T.TorusGeometry args={[
            RADIUS, //radius of torus
            .75 / uFovScale.value, //radius of tube
            8, //radial segments
            64, // tubular segments
            PI, // arc length
            PI/2, // theta start
            PI // theta length
        ]} />
        <T.ShaderMaterial
            vertexShader={gridVert}
            fragmentShader={gridFrag}
            uniforms={{ uColor: { value: new THREE.Color(color) }, uOpacity: { value: 1.0 }, uFovScale }}
            side={THREE.DoubleSide}
        />
    </T.Mesh>
{/snippet}


{#each Array.from({ length: 42 }) as _, i}
    {@const declination = (i + 1) * 5 - 90}
    {@render dec_ring(declination, -1.9 + i * 0.01)}
{/each}

{#each Array.from({ length: 24 }) as _, i}
    {@const rightAscension = (i + 1) * 15}
    {@render ra_ring(rightAscension, i === 0 ? '#f00' : '#555', -1.4 + i * 0.01)}
{/each}

<StereographicText
    text={'N'}
    position={[northPoleVector.x/1.1, northPoleVector.y/1.1, northPoleVector.z/1.1]}
    fontSize={30}
    color={'#f00'}
    anchorX={'left'}
    anchorY={'bottom'}
/>
