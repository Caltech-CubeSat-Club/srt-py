<script lang="ts">
  import { T, useThrelte } from '@threlte/core'
  import { azEltoVector3 } from '$lib/coordinates'
  import { liveTelescopeData } from '$lib/stores/telescope_state';
  import { Text } from '@threlte/extras'
  import { cursorAzEl } from '$lib/stores/ui';


  let object_locs = $state(liveTelescopeData.object_locs);

  let { radius }: { radius: number } = $props();

  $effect(() => {
    const interval = setInterval(() => {
      object_locs = liveTelescopeData.object_locs;
    }, 500);
    return () => clearInterval(interval);
  });
</script>


{#each Object.entries(object_locs) as [name, [az, el]]}
  {@const pos = azEltoVector3(az, el, radius)}
  <T.Mesh position={[-pos.x, pos.y, pos.z]}>
    <T.SphereGeometry args={[2, 16, 16]} />
    <T.MeshBasicMaterial color="#f59e0b" toneMapped={false} />
  </T.Mesh>
  <Text
    text={name}
    position={[-pos.x/1.1, pos.y/1.1, pos.z/1.1]}
    fontSize={15}
    color={'#0f0'}
    anchorX={'left'}
    anchorY={'bottom'}
    onadded={(e) => {
        e.target.lookAt(0, 0, 0);
    }}
/>
{/each}