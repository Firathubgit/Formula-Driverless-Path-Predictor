// Lines and ribbons drawn over the road, blended: the geometry's colour and its own alpha, scaled by a strength and
// faded out with distance from the car.
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
    FRAGCOLOR = vec4(encode(vColor.rgb), vColor.a * strength * fade);
}
