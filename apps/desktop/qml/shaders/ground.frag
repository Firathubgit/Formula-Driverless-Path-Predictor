// The road and its edges: the geometry's colour near the car, fading into the backdrop with distance, as a driver's
// display lets the road ahead fall away into the dark. A surface with clarity is seen through by that much, to the
// car's reflection beneath it, and brightened so that over the bare backdrop it composites to exactly its own colour:
// the reflection adds only what it is brighter than the backdrop, scaled by the clarity.
//
// With gloss, the floor of the kart hall (decision 0051): polished, so the lamps under the roof stand in it as long streaks
// toward the eye, and never one flat colour. Without gloss nothing below MAIN's first lines runs, and the surface is exactly
// what it was.
VARYING vec3 vWorld;
VARYING vec4 vColor;

// Unshaded output is written as it is, so the linear colours the geometry stores are encoded for display here.
vec3 encode(vec3 c)
{
    return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(vec3(0.0031308), c));
}

float hash(vec2 p)
{
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

float grain(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x), mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
}

// How much of the lamps a point of the floor mirrors toward the eye. The lamps hang on a grid at one height (lampGrid: the
// first lamp's x and z, the spacing in x and in z). The mirrored ray leaves the floor away from the eye at the elevation it
// arrived at; a polished floor that is not a mirror spreads it up and down by glossSpread, in radians, and hardly at all
// sideways, which is what draws each lamp out into a streak toward the eye. So the ray's track along the ceiling is followed
// across the grid's lines, whichever family it crosses more squarely, and each crossing counts by how near it passes a lamp
// and how near that lamp's elevation is to the mirrored one.
float lampStreaks(vec3 world, vec3 eye)
{
    vec3 toEye = eye - world;
    float reach = max(length(toEye.xz), 0.001);
    float theta0 = atan(max(toEye.y, 0.05), reach);
    vec2 dir = -toEye.xz / reach;
    float rise = max(lampHeight - world.y, 0.5);
    bool acrossX = abs(dir.x) >= abs(dir.y);
    float dc = acrossX ? dir.x : dir.y;
    float dOther = acrossX ? dir.y : dir.x;
    float pc = acrossX ? world.x : world.z;
    float pOther = acrossX ? world.z : world.x;
    float originC = acrossX ? lampGrid.x : lampGrid.y;
    float spacingC = acrossX ? lampGrid.z : lampGrid.w;
    float originOther = acrossX ? lampGrid.y : lampGrid.x;
    float spacingOther = acrossX ? lampGrid.w : lampGrid.z;
    float lowC = acrossX ? hallBounds.x : hallBounds.y;
    float highC = acrossX ? hallBounds.z : hallBounds.w;
    float lowOther = acrossX ? hallBounds.y : hallBounds.x;
    float highOther = acrossX ? hallBounds.w : hallBounds.z;
    float way = dc >= 0.0 ? 1.0 : -1.0;
    // The nearest crossing worth counting is the one whose lamp stands highest above the mirrored elevation.
    float nearest = rise / tan(min(theta0 + 2.2 * glossSpread, 1.45));
    float first = (pc + dc * nearest - originC) / spacingC;
    float index = way > 0.0 ? ceil(first) : floor(first);
    float total = 0.0;
    for (int k = 0; k < 12; ++k) {
        float c = originC + (index + way * float(k)) * spacingC;
        float along = (c - pc) / dc;
        float other = pOther + dOther * along;
        float cell = (other - originOther) / spacingOther;
        float miss = (cell - floor(cell + 0.5)) * spacingOther * abs(dc);
        float lobe = (atan(rise, along) - theta0) / glossSpread;
        float inside = step(lowC, c) * step(c, highC) * step(lowOther, other) * step(other, highOther);
        total += inside * exp(-lobe * lobe - miss * miss / 0.9);
    }
    return total;
}

void MAIN()
{
    float fade = 1.0 - smoothstep(fadeNear, fadeFar, distance(vWorld.xz, focus.xz));
    vec3 colour = encode(vColor.rgb) * gain;
    if (gloss > 0.0) {
        float worn = 0.6 * grain(vWorld.xz * 0.9) + 0.4 * grain(vWorld.xz * 4.3);
        // A hall's asphalt under white lamps is a neutral grey, not a road's blue at night: dark surfaces lose their tint,
        // paint keeps its colour.
        float level = dot(colour, vec3(0.333));
        float painted = smoothstep(0.05, 0.3, max(max(colour.r, colour.g), colour.b) - min(min(colour.r, colour.g), colour.b));
        colour = mix(vec3(level), colour, painted);
        colour *= 0.86 + 0.24 * worn;
        vec3 toEye = CAMERA_POSITION - vWorld;
        float grazing = 1.0 - clamp(toEye.y / max(length(toEye), 0.001), 0.0, 1.0);
        float sheen = grazing * grazing;
        float streaks = lampStreaks(vWorld, CAMERA_POSITION);
        // The streaks of the lamps, and beneath them the lit roof the whole floor mirrors faintly, more the lower the eye looks
        // along it: far floor is a pale grey, the floor at the kart's own wheels stays dark.
        colour += gloss * (streaks * (0.14 + 0.36 * sheen) * (0.75 + 0.5 * worn) * vec3(0.9, 0.9, 0.91) + vec3(0.07) * sheen * grazing);
    }
    vec3 surface = mix(horizon, colour, fade);
    float alpha = 1.0 - clarity;
    FRAGCOLOR = vec4(max(surface - clarity * horizon, vec3(0.0)) / alpha, alpha);
}
