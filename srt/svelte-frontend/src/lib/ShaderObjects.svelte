<script lang="ts">
    import { T, useThrelte } from '@threlte/core'
    import * as THREE from 'three'
    import starsVert from '$lib/shaders/stars.vert'
    import starsFrag from '$lib/shaders/stars.frag'
    import StereographicText from '$lib/StereographicText.svelte'

    import { AzEl } from '$lib/coordinates.svelte'
    import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
    import { cursorAzEl } from '$lib/stores/ui.svelte';
    import { uFovScale, uAspect, projectToScreenNDC } from '$lib/stores/projection.svelte';
    import { SKY_DOME_RADIUS as radius } from '$lib/constants';

    const { camera } = useThrelte();

    // Screen-space (NDC) selection radius - roughly how close the cursor
    // needs to be to a marker's actual rendered position to select it.
    const SELECT_THRESHOLD_NDC = 0.05;

    const SELECTED_COLOR = new THREE.Color('#00ff00');
    const DEFAULT_COLOR = new THREE.Color('#f59e0b');
    // stars.vert scales the base [-0.5,0.5] quad by angularSize * uFovScale directly
    // in clip space, so this is NDC half-width per unit, not a real angular/pixel size.
    const MARKER_ANGULAR_SIZE_1DEG = 0.0175;

    let object_locs = $derived(daemonStatus.object_locs ?? {});
    let objectNames = $derived(Object.keys(object_locs));

    let cursorPos = $derived(cursorAzEl.toVector3(radius));

    // Screen-space (NDC) selection: mirrors what stars.vert actually renders
    // (via projectToScreenNDC), rather than comparing real 3D world-space
    // distances, which distorts badly near the FOV edge under stereographic
    // projection - two markers that are visually close on screen can be far
    // apart in world space, and vice versa.
    let selectedObject = $derived.by( () => {
        const cam = camera.current;
        if (!cam) return null;

        let closestObject: string | null = null;
        let closestDistance = SELECT_THRESHOLD_NDC;

        const cursorNDC = projectToScreenNDC(cursorPos, cam);

        for (const [name, [az, el]] of Object.entries(object_locs)) {
            const pos = new AzEl(az, el).toVector3(radius);
            const ndc = projectToScreenNDC(pos, cam);
            const dx = ndc.x - cursorNDC.x;
            const dy = ndc.y - cursorNDC.y;
            const distance = Math.sqrt(dx * dx + dy * dy);

            if (distance < closestDistance) {
                closestDistance = distance;
                closestObject = name;
            }
        }

        return closestObject;
    });

    // Base billboard quad: a unit square in local XY. stars.vert applies it as a
    // post-projection clip-space offset (see chunks/stereographic.glsl), so this
    // shape is shared by every instance and never itself gets projected.
    //
    // Three.js caches an InstancedBufferGeometry's max instance count from the
    // first-ever bind of an instanced attribute on that geometry INSTANCE, and
    // never recomputes it afterward (WebGLBindingStates.js). Rather than reach
    // into that private bookkeeping, we build a fresh geometry whenever the
    // instance count changes, so Three's cache starts clean each time.
    function createGeometry(count: number): THREE.InstancedBufferGeometry {
        const geo = new THREE.InstancedBufferGeometry();
        geo.setIndex([0, 1, 2, 0, 2, 3]);
        geo.setAttribute('position', new THREE.Float32BufferAttribute(
            [-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0], 3
        ));
        geo.setAttribute('uv', new THREE.Float32BufferAttribute(
            [0, 0, 1, 0, 1, 1, 0, 1], 2
        ));
        geo.setAttribute('translate', new THREE.InstancedBufferAttribute(new Float32Array(count * 3), 3));
        geo.setAttribute('angularSize', new THREE.InstancedBufferAttribute(new Float32Array(count), 1));
        geo.setAttribute('objectColor', new THREE.InstancedBufferAttribute(new Float32Array(count * 3), 3));
        geo.instanceCount = count;
        return geo;
    }

    let geometry = $state.raw<THREE.InstancedBufferGeometry>(createGeometry(0));
    let instanceCount = -1;

    $effect(() => {
        const names = objectNames;
        const count = names.length;

        if (count !== instanceCount) {
            instanceCount = count;
            geometry = createGeometry(count);
        }

        const translateAttr = geometry.attributes.translate as THREE.InstancedBufferAttribute;
        const sizeAttr = geometry.attributes.angularSize as THREE.InstancedBufferAttribute;
        const colorAttr = geometry.attributes.objectColor as THREE.InstancedBufferAttribute;

        names.forEach((name, i) => {
            const [az, el] = object_locs[name];
            const pos = new AzEl(az, el).toVector3(radius);
            translateAttr.setXYZ(i, pos.x, pos.y, pos.z);
            sizeAttr.setX(i, (name === "Sun" || name === "Moon") ? MARKER_ANGULAR_SIZE_1DEG * 0.5 : MARKER_ANGULAR_SIZE_1DEG * 0.25);

            const color = name === selectedObject ? SELECTED_COLOR : DEFAULT_COLOR;
            colorAttr.setXYZ(i, color.r, color.g, color.b);
        });

        translateAttr.needsUpdate = true;
        sizeAttr.needsUpdate = true;
        colorAttr.needsUpdate = true;
    });

    const material = new THREE.ShaderMaterial({
        vertexShader: starsVert,
        fragmentShader: starsFrag,
        uniforms: { uFovScale, uAspect },
        transparent: true,
    });
</script>

<T.Mesh frustumCulled={false}>
    <T is={geometry} />
    <T is={material} />
</T.Mesh>

{#each objectNames as name}
    {@const [az, el] = object_locs[name]}
    {@const pos = new AzEl(az, el).toArray(radius)}
    <StereographicText
        text={name}
        position={pos}
        fontSize={15}
        color={name === selectedObject ? '#00ff00' : '#f59e0b'}
        anchorX={'left'}
        anchorY={'bottom'}
    />
{/each}