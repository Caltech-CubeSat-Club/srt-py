<script lang="ts">
  import { T } from '@threlte/core'
  import { azEltoVector3 } from '$lib/coordinates'
  import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
  import { Text } from '@threlte/extras'
  import { cursorAzEl } from '$lib/stores/ui.svelte';

  let object_locs = $derived(daemonStatus.object_locs ?? {});

  import { SKY_DOME_RADIUS as radius } from '$lib/constants';

  let cursorPos = $derived(azEltoVector3(cursorAzEl[0], cursorAzEl[1], radius));

  let selectedObject = $derived.by( () => {
    let closestObject: string | null = null;
    let closestDistance = 10; // Threshold distance in the same units as the radius

    for (const [name, [az, el]] of Object.entries(object_locs)) {
      const pos = azEltoVector3(az, el, radius);
      const distance = pos.distanceTo(cursorPos);

      if (distance < closestDistance) {
        closestDistance = distance;
        closestObject = name;
      }
    }

    return closestObject;
  });
</script>


{#each Object.entries(object_locs) as [name, [az, el]]}
  {@const pos = azEltoVector3(az, el, radius)}
  {@const ball_radius = (name === selectedObject) ? 10 : 2}
  <T.Mesh position={[pos.x, pos.y, pos.z]}>
    <T.SphereGeometry args={[ball_radius, 16, 16]} />
    <T.MeshBasicMaterial color={(name === selectedObject) ? "#0f0" : "#f59e0b"} toneMapped={false} />
  </T.Mesh>
  <Text
    text={name}
    position={[pos.x/1.1, pos.y/1.1, pos.z/1.1]}
    fontSize={15}
    color={'#0f0'}
    anchorX={'left'}
    anchorY={'bottom'}
    onadded={(e) => {
        e.target.lookAt(0, 0, 0);
    }}
    
/>
{/each}

<!-- Cursor position indicator -->
{#if !selectedObject}}
  <T.Mesh position={[cursorPos.x, cursorPos.y, cursorPos.z]}>
    <T.SphereGeometry args={[10, 16, 16]} />
    <T.MeshBasicMaterial color="#0000ff" toneMapped={false} />
  </T.Mesh>
{/if}