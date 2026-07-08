<script lang="ts">
    import * as THREE from 'three'
    import { T } from '@threlte/core'
    import { Text } from '@threlte/extras'
    import { raDecToAzElDegrees, localSiderealTime, azEltoVector3 } from '$lib/coordinates'
    import { liveTelescopeData } from '$lib/stores/telescope_state';

    const RADIUS = 500; // Size of the sky dome

    const PI = Math.PI;

    let { latitude_deg }: { latitude_deg: number } = $props();

    function raDecToVector3(ra_deg: number, dec_deg: number) {
        const [az_deg, el_deg] = raDecToAzElDegrees(ra_deg, dec_deg, latitude_deg, -118.129, liveTelescopeData.time / 86400 + 2440587.5);
        return azEltoVector3(az_deg, el_deg, RADIUS);
    }

    let northPoleVector = raDecToVector3(0, 90); // North Pole is at az=0, el=90

</script>

{#snippet dec_ring(dec_deg: number)}
    {@const dec = dec_deg * PI / 180}
    {@const lat = latitude_deg * PI / 180}
    {@const ringRadius = RADIUS * Math.cos(dec)}
    {@const ringCenterVectorLength = RADIUS * Math.sin(dec)}
    <!-- Get North Pole vector -->
    <!-- +Z = North -->
    {@const dz = ringCenterVectorLength * Math.cos(lat) }
    {@const dy = ringCenterVectorLength * Math.sin(lat) }
    <T.Mesh position={[0, dy, dz]} rotation={[-lat, 0, 0]}>
        <T.TorusGeometry args={[ringRadius, .5, 8, 64]} />
        <T.MeshBasicMaterial color="#bbb" side={THREE.DoubleSide} />
    </T.Mesh>
{/snippet}

{#snippet ra_ring(ra_deg: number, color: string = '#555')}
    {@const lat = latitude_deg * PI / 180}
    {@const ra = ra_deg * PI / 180}
    {@const lst = localSiderealTime(liveTelescopeData.time / 86400 + 2440587.5, -118.129 * PI / 180)}
    <!-- Get North Pole vector -->
    <!-- +Z = North -->
    <T.Mesh position={[0, 0, 0]} rotation={[PI/2 - lat, - ra - lst, PI/2]}>
        <T.TorusGeometry args={[
            RADIUS, //radius of torus
            .5, //radius of tube
            8, //radial segments
            64, // tubular segments
            PI, // arc length
            PI/2, // theta start
            PI // theta length
        ]} />
        <T.MeshBasicMaterial color={color} side={THREE.DoubleSide} />
    </T.Mesh>
{/snippet}


{#each Array.from({ length: 42 }) as _, i}
    {@const declination = (i + 1) * 5 - 90}
    {@render dec_ring(declination)}
{/each}

{#each Array.from({ length: 24 }) as _, i}
    {@const rightAscension = (i + 1) * 15}
    {@render ra_ring(rightAscension, i === 0 ? '#f00' : '#555')}
{/each}

<Text
    text={'N'}
    position={[northPoleVector.x/1.1, northPoleVector.y/1.1, northPoleVector.z/1.1]}
    fontSize={30}
    color={'#f00'}
    anchorX={'left'}
    anchorY={'bottom'}
    onadded={(e) => {
        e.target.lookAt(0, 0, 0);
    }}
/>
