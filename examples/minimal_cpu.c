// Minimal headless rigid-body demo: drop a box on a floor, print height.
// No graphics, no CUDA. Build with -DPUFFYSICS_BUILD_EXAMPLES=ON.
#include <stdio.h>

#include "puffysics.cuh"

int main(void) {
    B3World w;
    b3_world_init(&w);
    B3ShapeDef sd = b3_default_shape();
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, -0.5f, 0.0f);
    int g = b3_create_body(&w, &gd);
    if (g < 0 || b3_create_box(&w, g, b3_v(50.0f, 0.5f, 50.0f), &sd) < 0) {
        fprintf(stderr, "ground setup failed\n");
        return 1;
    }
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 5.0f, 0.0f);
    int b = b3_create_body(&w, &bd);
    if (b < 0 || b3_create_box(&w, b, b3_v(0.5f, 0.5f, 0.5f), &sd) < 0) {
        fprintf(stderr, "box setup failed\n");
        return 1;
    }
    b3_finalize_mass(&w, b);
    for (int i = 0; i <= 120; i++) {
        if (i % 30 == 0) {
            printf("t=%.2f y=%.4f\n", (double)i / 60.0,
                (double)w.bodies[b].position.y);
        }
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    return 0;
}
