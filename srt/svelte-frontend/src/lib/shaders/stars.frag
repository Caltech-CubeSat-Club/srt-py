varying vec2 vUv;
varying vec3 vColor;

void main() {
    vec2 centered = vUv - 0.5;
    if (dot(centered, centered) > 0.25) discard;
    // see skyDome.frag - linearToOutputTexel() must be called explicitly here too.
    gl_FragColor = linearToOutputTexel(vec4(vColor, 1.0));
}