// Shared by the scene's track materials: the fragment stages need the world position, to fade with distance from the car,
// and the geometry's own colour.
VARYING vec3 vWorld;
VARYING vec4 vColor;

void MAIN()
{
    vWorld = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
    vColor = COLOR;
    POSITION = MODELVIEWPROJECTION_MATRIX * vec4(VERTEX, 1.0);
}
