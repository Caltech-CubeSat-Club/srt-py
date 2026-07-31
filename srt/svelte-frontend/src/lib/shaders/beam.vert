#include "chunks/stereographic"

uniform float uAngularSize;

varying vec2 vUv;

void main() {
  // Mesh position (not the local quad geometry) carries the beam's world
  // location - same separation of "where" vs "billboard shape" as
  // ShaderObjects' translate attribute, just via the mesh transform instead
  // of an instanced attribute since there's only one beam.
  vec4 centerView = modelViewMatrix * vec4( 0.0, 0.0, 0.0, 1.0 );
  vec4 projectedCenter = stereographicProject( centerView.xyz );
  gl_Position = projectedCenter + vec4( position.xy * uAngularSize * uFovScale, 0.0, 0.0 );
  vUv = uv;
}
