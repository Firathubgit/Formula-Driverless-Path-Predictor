// The road and its edges: the geometry's colour near the car, fading into the backdrop with distance, as a driver's
// display lets the road ahead fall away into the dark. A surface with clarity is seen through by that much, to the
// car's reflection beneath it, and brightened so that over the bare backdrop it composites to exactly its own colour:
// the reflection adds only what it is brighter than the backdrop, scaled by the clarity.
VARYING vec3 vWorld;
VARYING vec4 vColor;

// Unshaded output is written as it is, so the linear colours the geometry stores are encoded for display here.
vec3 encode(vec3 c)
{
    return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(vec3(0.0031308), c));
}

void MAIN()
{
    float fade = 1.0 - smoothstep(fadeNear, fadeFar, distance(vWorld.xz, focus.xz));
    vec3 surface = mix(horizon, encode(vColor.rgb) * gain, fade);
    float alpha = 1.0 - clarity;
    FRAGCOLOR = vec4(max(surface - clarity * horizon, vec3(0.0)) / alpha, alpha);
}
