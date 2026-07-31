varying vec2 vUv;

// Same physical constants and field-pattern equation as scripts/plot_beam_pattern.py:
//   E = (2*lambda/(pi*D)) * J1((pi*D/lambda)*sin(theta)) / sin(theta)
// for a uniformly-illuminated circular aperture (Airy pattern). uEdgeThetaDeg
// matches that script's plotted domain radius and must stay in sync with
// TelescopeBeam.svelte's uAngularSize (used there to size the billboard quad).
uniform float uLambdaMeters;
uniform float uApertureMeters;
uniform float uEdgeThetaDeg;
const float PI = 3.14159265358979;

// Numerical Recipes' rational-polynomial approximation of the Bessel
// function J1(x) (accurate to ~1e-8 over this range) - GLSL has no built-in
// Bessel function.
float besselJ1( float x ) {
  float ax = abs( x );
  if ( ax < 8.0 ) {
    float y = x * x;
    float ans1 = x * ( 72362614232.0 + y * ( -7895059235.0 + y * ( 242396853.1 + y * ( -2972611.439 + y * ( 15704.48260 + y * ( -30.16036606 ) ) ) ) ) );
    float ans2 = 144725228442.0 + y * ( 2300535178.0 + y * ( 18583304.74 + y * ( 99447.43394 + y * ( 376.9991397 + y ) ) ) );
    return ans1 / ans2;
  } else {
    float z = 8.0 / ax;
    float y = z * z;
    float xx = ax - 2.356194491;
    float ans1 = 1.0 + y * ( 0.183105e-2 + y * ( -0.3516396496e-4 + y * ( 0.2457520174e-5 + y * ( -0.240337019e-6 ) ) ) );
    float ans2 = 0.04687499995 + y * ( -0.2002690873e-3 + y * ( 0.8449199096e-5 + y * ( -0.88228987e-6 + y * 0.105787412e-6 ) ) );
    float ans = sqrt( 0.636619772 / ax ) * ( cos( xx ) * ans1 - z * sin( xx ) * ans2 );
    if ( x < 0.0 ) ans = -ans;
    return ans;
  }
}

void main() {
  float r = length( vUv - 0.5 ); // 0 at center, 0.5 at the quad edge
  if ( r > 0.5 ) discard; // outside the quad, don't render
  float thetaRad = radians( 2.0 * r * uEdgeThetaDeg ); // small-angle: UV radius scales linearly with real angle here

  float x = ( PI * uApertureMeters / uLambdaMeters ) * sin( thetaRad );

  // field = 2*J1(x)/x is algebraically identical to the script's
  // (2*lambda/(pi*D))*J1(x)/sin(theta) form, but avoids a separate division
  // by sin(theta) and handles the on-axis limit (x -> 0, J1(x)/x -> 0.5)
  // cleanly, since that limit is exactly the field's normalized peak of 1.0.
  float field = ( abs( x ) < 1e-4 ) ? 1.0 : ( 2.0 * besselJ1( x ) / x );

  float intensity = field * field; // power pattern
  if ( intensity < 0.001 ) discard;

  // see skyDome.frag - linearToOutputTexel() must be called explicitly here too.
  gl_FragColor = linearToOutputTexel( vec4( 0.0, 1.0, 0.0, clamp( intensity, 0.0, 1.0 ) ) );
}
