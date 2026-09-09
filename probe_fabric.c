/* Headless well-depth probe using the same masses as play_fabric. */
#include <math.h>
#include <stdio.h>
#include "dat_cloth.h"

static int add_ball(B3World* w, float r, float density, B3Vec3 pos) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    int body = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(w, body, b3_v(0, 0, 0), r, &sd);
    float vol = (4.0f / 3.0f) * B3_PI * r * r * r;
    float m = density * vol;
    B3Vec3 inertia = b3_v(0.4f * m * r * r, 0.4f * m * r * r,
        0.4f * m * r * r);
    b3_set_inertial(w, body, m, b3_v(0, 0, 0), inertia);
    printf("ball r=%.3f density=%.1f mass=%.4f kg inv_mass=%.4f\n",
        r, density, m, w->bodies[body].inv_mass);
    return body;
}

int main(void) {
    const int W = 25, H = 25;
    const float spacing = 0.1f;
    const float half = 0.5f * spacing * (float)(W - 1);
    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    DatCloth c;
    dat_cloth_init(&c, W, H, spacing, b3_v(-half, 0.0f, -half));
    c.gravity = 0.0f;
    c.node_mass = 0.04f;
    c.relax = 0.03f;
    c.iters = 4;
    c.damping = 0.0f;
    int bodies[1];
    float radii[1];
    B3Vec3 snap[1];
    bodies[0] = add_ball(&w, 0.16f, 23.0f, b3_v(0.0f, 0.35f, 0.0f));
    radii[0] = 0.16f;
    for (int frame = 1; frame <= 360; frame++) {
        for (int s = 0; s < subs; s++) {
            b3_step(&w, h, 1);
            dat_cloth_step(&c, h);
            snap[0] = w.bodies[bodies[0]].center;
            dat_couple(&c, &w, snap, bodies, radii, 1, h);
            dat_couple_velocities(&w, snap, bodies, 1, h);
        }
        if (frame == 60 || frame == 180 || frame == 360) {
            float miny = 1e9f, maxy = -1e9f;
            int mid = (H / 2) * W + (W / 2);
            int contacts = 0;
            for (int k = 0; k < c.n; k++) {
                if (c.pos[k].y < miny) miny = c.pos[k].y;
                if (c.pos[k].y > maxy) maxy = c.pos[k].y;
                B3Vec3 d = b3_sub(c.pos[k], w.bodies[bodies[0]].center);
                if (b3_len(d) < radii[0] + 0.02f) contacts++;
            }
            printf("t=%.1fs well=%.3f center=%.3f ball=%.3f n=%d\n",
                frame / 60.0, maxy - miny, c.pos[mid].y,
                w.bodies[bodies[0]].center.y, contacts);
        }
    }
    dat_cloth_free(&c);
    return 0;
}
