uniform vec3 uColor;
uniform float uOpacity;

varying float vDenom;

void main() {
  // Blind spot directly opposite the view direction - see
  // stereographicProjectClipped in chunks/stereographic.glsl for why this
  // can't be handled by clipping alone.
  if ( vDenom < 0.02 ) discard;

  // see skyDome.frag - linearToOutputTexel() must be called explicitly here too.
  gl_FragColor = linearToOutputTexel( vec4( uColor, uOpacity ) );
}
