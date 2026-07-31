#include "chunks/stereographic"

// position, uv, and uFovScale are already provided: position/uv are auto-injected
// by THREE.ShaderMaterial, uFovScale is declared inside chunks/stereographic.glsl.
attribute vec3 translate;      // per-object sky position (RA/Dec -> xyz on sky dome)
attribute float angularSize;   // per-object custom attribute
attribute vec3 objectColor;    // per-object custom attribute

varying vec2 vUv;
varying vec3 vColor;

void main() {
  vec4 centerView = modelViewMatrix * vec4( translate, 1.0 );
  vec4 projectedCenter = stereographicProject( centerView.xyz );

  // billboard offset: apply the sprite shape directly in clip/NDC space,
  // scaled by angularSize and a pixel-size factor, since we're now past
  // the nonlinear projection step rather than before it like the example
  gl_Position = projectedCenter + vec4( position.xy * angularSize * uFovScale, 0.0, 0.0 );

  vUv = uv;
  vColor = objectColor;
}