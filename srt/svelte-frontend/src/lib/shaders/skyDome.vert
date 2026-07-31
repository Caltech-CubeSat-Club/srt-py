#include "chunks/stereographic"

varying vec2 vUv;
varying float vDenom;

void main() {
  vec4 viewPos = modelViewMatrix * vec4( position, 1.0 );
  float denom;
  gl_Position = stereographicProjectClipped( viewPos.xyz, denom );
  vDenom = denom;
  vUv = uv;
}
