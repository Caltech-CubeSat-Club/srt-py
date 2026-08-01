<script lang="ts">
    import { Text } from '@threlte/extras';
    import { createDerivedMaterial } from 'troika-three-utils';
    import { uFovScale, uAspect } from '$lib/stores/projection.svelte';
    import stereographicChunk from '$lib/shaders/chunks/stereographic.glsl';

    interface Props {
        // Unitless like MARKER_ANGULAR_SIZE in ShaderObjects.svelte - troika's
        // local glyph units (which scale with fontSize) times this give the
        // clip-space billboard offset.
        billboardScale?: number;
        [key: string]: any;
    }

    let { billboardScale = 0.008, ...rest }: Props = $props();

    const uTextBillboardScale = {
        get value() {
            return uFovScale.value < 1.5
                ? billboardScale * uFovScale.value / 1.5
                : billboardScale;
        },
    };

    let textMesh: any = $state();

    // troika-three-text's own vertex shader still ends by computing gl_Position
    // via the standard projectionMatrix*modelViewMatrix chain, so labels drift
    // out of sync with everything else once the real camera's FOV/zoom diverges
    // from our stereographic view. troika always derives whatever material you
    // give it through its own createDerivedMaterial call (adding its SDF glyph
    // logic on top), so we can't inject our override into the material we hand
    // it - it has to be layered on top of troika's *own* already-derived output
    // instead, in a vertexMainOutro (which runs after the standard projection
    // chain, so it gets the final say on gl_Position).
    //
    // position.xy is NOT usable here for per-glyph placement, even though it
    // looks like it should be: troika's own vertexTransform (VERTEX_TRANSFORM
    // in TextDerivedMaterial.js) globally renames every use of position/normal/uv
    // inside its derived shader to a uniquely-keyed local copy, mutates *that*,
    // and never copies it back - so from out here position.xy is still the raw,
    // shared 0..1 unit quad, identical for every glyph in the string (this is
    // what caused every letter to collapse onto the same spot). Instead we
    // recompute troika's own per-glyph placement formula ourselves, directly
    // from aTroikaGlyphBounds - the attribute troika's placement is actually
    // driven by, which (not being position/normal/uv) survives untouched.
    // This intentionally skips troika's outline-padding/clipRect expansion
    // (a cosmetic detail we don't use here), not just the core per-glyph box.
    function wireStereographicText() {
        if (!textMesh || textMesh.material?.isStereographicText) return;
        textMesh.material = createDerivedMaterial(textMesh.material, {
            chained: true,
            uniforms: { uFovScale, uAspect, uTextBillboardScale },
            vertexDefs: `
                ${stereographicChunk}
                uniform float uTextBillboardScale;
            `,
            vertexMainOutro: `
                vec2 glyphLocalXY = mix(aTroikaGlyphBounds.xy, aTroikaGlyphBounds.zw, position.xy) + uTextBillboardScale;
                vec4 centerView = modelViewMatrix * vec4(0.0, 0.0, 0.0, 1.0);
                vec4 projectedCenter = stereographicProject(centerView.xyz);
                vec2 offset = glyphLocalXY * uTextBillboardScale;
                offset.x /= uAspect;
                gl_Position = projectedCenter + vec4(offset, 0.0, 0.0);
            `,
        });
        (textMesh.material as any).isStereographicText = true;
    }
</script>

<!--
  frustumCulled defaults to true, and Three's CPU-side culling check is
  entirely blind to our custom vertex shader - it tests the mesh's local
  geometry bounding volume against the REAL camera's frustum (real fov/zoom/
  aspect), not what our stereographic override actually puts on screen. Our
  effective view is often much wider than the real camera's frustum, so
  labels that are clearly on-screen via the shader still get silently culled
  before the vertex shader even runs (see the same fix on ShaderObjects'
  marker mesh).
-->
<Text bind:ref={textMesh} {...rest} frustumCulled={false} onsync={wireStereographicText} />
