#include <math.h>
#include <stdio.h>
#include "puffysics.cuh"
#include "b3_loose.cuh"

static void stats(B3Loose* s, const char* tag) {
    float ymin = 1e9f, ymax = -1e9f, ke = 0.0f, xmax = 0.0f;
    int n = 0;
    for (int b = 0; b < s->n_bodies; b++) {
        if (s->bodies[b].type != B3_DYNAMIC) continue;
        B3Vec3 p = s->bodies[b].position;
        if (p.y < ymin) ymin = p.y;
        if (p.y > ymax) ymax = p.y;
        float dx = p.x - 0.35f;
        if (fabsf(dx) > xmax) xmax = fabsf(dx);
        B3Vec3 v = s->bodies[b].lin_vel;
        B3Vec3 w = s->bodies[b].ang_vel;
        float mass = s->bodies[b].inv_mass > 0.0f ? 1.0f / s->bodies[b].inv_mass : 0.0f;
        ke += 0.5f * mass * (v.x * v.x + v.y * v.y + v.z * v.z);
        ke += 0.5f * (w.x * w.x + w.y * w.y + w.z * w.z);
        n += 1;
    }
    printf("%s  n=%d contacts=%d y=[%.4f, %.4f] |dx|=%.4f ke=%.6f\n",
        tag, n, s->n_contacts, ymin, ymax, xmax, ke);
}

static void run(const char* name, float gap, float mu, int settle, int steps) {
    B3Loose s;
    b3_loose_init(&s, 64, 64, 512);
    b3_loose_add_ground(&s);
    b3_loose_spawn_grid(&s, 1, 5, 5, 0.03f, gap, 250.0f, b3_v(0.35f, 0.0f, 0.0f));
    for (int i = 0; i < s.n_shapes; i++) {
        if (s.bodies[s.shapes[i].body].type == B3_DYNAMIC) {
            s.shapes[i].friction = mu;
        }
    }
    printf("== %s gap=%.4f mu=%.2f ==\n", name, gap, mu);
    stats(&s, "  t=0.00");
    for (int i = 0; i < settle; i++) {
        b3_loose_step(&s, 0.02f, 4);
    }
    if (settle) {
        stats(&s, "  after settle");
        for (int b = 0; b < s.n_bodies; b++) {
            s.bodies[b].lin_vel = b3_v(0, 0, 0);
            s.bodies[b].ang_vel = b3_v(0, 0, 0);
        }
    }
    for (int i = 1; i <= steps; i++) {
        b3_loose_step(&s, 0.02f, 4);
        if (i == 10 || i == 50 || i == 100 || i == steps) {
            char tag[32];
            snprintf(tag, sizeof(tag), "  t=%.2f", i * 0.02f);
            stats(&s, tag);
        }
    }
    b3_loose_free(&s);
}

int main(void) {
    run("current", 0.0005f, 0.45f, 0, 150);
    run("gap0 mu1", 0.0f, 1.0f, 0, 150);
    run("gap0 mu1 settle60", 0.0f, 1.0f, 60, 150);
    run("gap0 mu1 settle180", 0.0f, 1.0f, 180, 150);
    return 0;
}
