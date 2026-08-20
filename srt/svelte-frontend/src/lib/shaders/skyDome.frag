uniform sampler2D map;
uniform float uOpacity;

varying vec2 vUv;
varying float vDenom;

void main() {
  // Blind spot directly opposite the view direction - see
  // stereographicProjectClipped in chunks/stereographic.glsl for why this
  // can't be handled by clipping alone.
  if ( vDenom < 0.02 ) discard;

  // linearToOutputTexel() is auto-injected by THREE.ShaderMaterial's fragment
  // prefix (colorspace_pars_fragment) but, unlike built-in materials, never
  // auto-called - without it linear values write straight to an sRGB-output
  // canvas and look washed out.
  gl_FragColor = linearToOutputTexel( texture2D( map, vUv ) );
  gl_FragColor.a *= uOpacity;
}
