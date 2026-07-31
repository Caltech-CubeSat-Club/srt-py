<script lang="ts">
    import * as THREE from 'three'
    import { T } from '@threlte/core'
    import { Text } from '@threlte/extras'
    import { createDerivedMaterial } from 'troika-three-utils'
    import { raDecToAzElDegrees, localSiderealTime, azEltoVector3 } from '$lib/coordinates'
    import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
    import { uFovScale } from '$lib/stores/projection.svelte';
    import gridVert from '$lib/shaders/grid.vert';
    import gridFrag from '$lib/shaders/grid.frag';
    import stereographicChunk from '$lib/shaders/chunks/stereographic.glsl';

    const RADIUS = 500; // Size of the sky dome

    const PI = Math.PI;

    let { latitude_deg }: { latitude_deg: number } = $props();

    let time = $derived(daemonStatus.time ?? Date.now() / 1000);

    function raDecToVector3(ra_deg: number, dec_deg: number) {
        const [az_deg, el_deg] = raDecToAzElDegrees(ra_deg, dec_deg, latitude_deg, -118.129, time / 86400 + 2440587.5);
        return azEltoVector3(az_deg, el_deg, RADIUS);
    }

    let northPoleVector = raDecToVector3(0, 90); // North Pole is at az=0, el=90

    let northLabelMesh: any = $state();

    // troika-three-text's own vertex shader still ends by computing gl_Position
    // via the standard projectionMatrix*modelViewMatrix chain, so labels drift
    // out of sync with everything else once the real camera's FOV/zoom diverges
    // from our stereographic view. troika always derives whatever material you
    // give it through its own createDerivedMaterial call (adding its SDF glyph
    // logic on top), so we can't inject our override into the material we hand
    // it - it has to be layered on top of troika's *own* already-derived output
    // instead, in a vertexMainOutro (which runs after the standard projection
    // chain, so it gets the final say on gl_Position). position.xy here is
    // troika's own glyph-quad-shaped local position (already anchor-adjusted),
    // which is exactly the "offset from center" our billboard convention wants
    // (compare stars.vert/beam.vert, which do the same split with an actual
    // instanced/uniform translate instead of the mesh's own transform).
    const TEXT_BILLBOARD_SCALE = 0.04; // unitless like MARKER_ANGULAR_SIZE - retune once visible

    function wireStereographicText(textMesh: any) {
        if (textMesh.material?.isStereographicText) return;
        textMesh.material = createDerivedMaterial(textMesh.material, {
            chained: true,
            uniforms: { uFovScale, uTextBillboardScale: { value: TEXT_BILLBOARD_SCALE } },
            vertexDefs: `
                ${stereographicChunk}
                uniform float uTextBillboardScale;
            `,
            vertexMainOutro: `
                vec4 centerView = modelViewMatrix * vec4(0.0, 0.0, 0.0, 1.0);
                vec4 projectedCenter = stereographicProject(centerView.xyz);
                gl_Position = projectedCenter + vec4(position.xy * uTextBillboardScale * uFovScale, 0.0, 0.0);
            `,
        });
        (textMesh.material as any).isStereographicText = true;
    }
</script>

{#snippet dec_ring(dec_deg: number, ringRenderOrder: number)}
    {@const dec = dec_deg * PI / 180}
    {@const lat = latitude_deg * PI / 180}
    {@const ringRadius = RADIUS * Math.cos(dec)}
    {@const ringCenterVectorLength = RADIUS * Math.sin(dec)}
    <!-- Get North Pole vector -->
    <!-- +Z = North -->
    {@const dz = ringCenterVectorLength * Math.cos(lat) }
    {@const dy = ringCenterVectorLength * Math.sin(lat) }
    <T.Mesh position={[0, dy, dz]} rotation={[-lat, 0, 0]} renderOrder={ringRenderOrder}>
        <T.TorusGeometry args={[ringRadius, .25, 8, 64]} />
        <T.ShaderMaterial
            vertexShader={gridVert}
            fragmentShader={gridFrag}
            uniforms={{ uColor: { value: new THREE.Color('#bbb') }, uOpacity: { value: 1.0 }, uFovScale }}
            side={THREE.DoubleSide}
        />
    </T.Mesh>
{/snippet}

{#snippet ra_ring(ra_deg: number, color: string, ringRenderOrder: number)}
    {@const lat = latitude_deg * PI / 180}
    {@const ra = ra_deg * PI / 180}
    {@const lst = localSiderealTime(time / 86400 + 2440587.5, -118.129 * PI / 180)}
    <!-- Get North Pole vector -->
    <!-- +Z = North -->
    <T.Mesh position={[0, 0, 0]} rotation={[PI/2 - lat, - ra - lst, PI/2]} renderOrder={ringRenderOrder}>
        <T.TorusGeometry args={[
            RADIUS, //radius of torus
            .25, //radius of tube
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

<Text
    bind:ref={northLabelMesh}
    text={'N'}
    position={[northPoleVector.x/1.1, northPoleVector.y/1.1, northPoleVector.z/1.1]}
    fontSize={30}
    color={'#f00'}
    anchorX={'left'}
    anchorY={'bottom'}
    onsync={() => wireStereographicText(northLabelMesh)}
/>
