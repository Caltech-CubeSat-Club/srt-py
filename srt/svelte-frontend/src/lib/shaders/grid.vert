#include "chunks/stereographic"

varying float vDenom;

void main() {
  vec4 viewPos = modelViewMatrix * vec4( position, 1.0 );
  float denom;
  gl_Position = stereographicProjectClipped( viewPos.xyz, denom );
  vDenom = denom;
}
