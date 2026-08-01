uniform float uFovScale;
// Camera aspect ratio (width/height). NDC space always spans -1..1 in both
// axes regardless of the canvas's actual dimensions, so without this, the
// same NDC displacement covers different physical pixel distances
// horizontally vs vertically on a non-square canvas - stretching circles
// into ellipses and breaking cursor math the same way. FOV is treated as
// vertical (matching THREE.PerspectiveCamera.fov), so X gets the extra
// division - same convention as a standard perspective projection matrix.
uniform float uAspect;

// For points/billboards only: divides manually and fixes w=1 so a later
// vec4 offset (e.g. a billboard's screen-space size) added to the result
// isn't itself re-divided by the hardware perspective divide. This forces
// w=1 for every vertex, which defeats the GPU's own near-plane clipping -
// fine for isolated points (an off-screen marker just doesn't show up), but
// NEVER use this for continuous surfaces/lines (grid rings, sky dome): a
// triangle with one vertex near the camera direction and another near the
// opposite (antipodal) direction will interpolate between a normal
// coordinate and a diverging one with no clipping to stop it, producing a
// giant stretched flash across the screen as it rotates through view. Use
// stereographicProjectClipped for those instead.
vec4 stereographicProject(vec3 viewPos) {
    vec3 dir = normalize(viewPos);
    float denom = max(1.0 - dir.z, 1e-4);
    vec2 screenXY = dir.xy * (2.0 / denom) * uFovScale;
    screenXY.x /= uAspect;
    return vec4(screenXY, 0.0, 1.0);
}

// For continuous surfaces/lines. Encodes the same denominator into w
// instead of dividing manually, so the hardware perspective divide produces
// an identical on-screen result, but the GPU's real clip-space culling can
// now see and reject triangles that are provably outside the frustum before
// rasterization - the same mechanism standard perspective projection relies
// on. This does NOT fully solve behind-camera rendering by itself though:
// the point exactly opposite the view direction (denom == 0) is a genuine
// mathematical singularity of this projection - points near it are valid
// but arbitrarily magnified, which is "on screen" as far as clipping is
// concerned, not something clipping can reject. denomOut is exposed so the
// caller can pass it to the fragment shader and discard that small blind
// spot explicitly (see grid.frag) - every full-sky conformal projection has
// an unavoidable pinhole directly behind the viewer for this reason.
vec4 stereographicProjectClipped(vec3 viewPos, out float denomOut) {
    vec3 dir = normalize(viewPos);
    denomOut = 1.0 - dir.z;
    vec2 screenXY = dir.xy * 2.0 * uFovScale;
    screenXY.x /= uAspect;
    return vec4(screenXY, 0.0, denomOut);
}