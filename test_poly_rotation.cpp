// g++ -O3 test_poly_rotation.cpp -o /tmp/test-poly-rotation
#define B3_POLY_ROTATION 1
#include "puffysics.cuh"
#include <stdio.h>
#include <stdlib.h>

static void require(bool ok) {
    if (!ok) { fprintf(stderr, "rotation accuracy check failed\n"); exit(1); }
}

int main() {
    double worst = 0.0, euler_worst = 0.0;
    // Sweep capped increments and the public helper's uncapped fallback.
    // Nonidentity initial orientation checks multiplication order as well.
    B3Quat q = b3_qnorm(b3_q(0.2f, -0.3f, 0.4f, 0.8f));
    double qn = sqrt((double)q.v.x*q.v.x + (double)q.v.y*q.v.y
        + (double)q.v.z*q.v.z + (double)q.s*q.s);
    for (int i = 0; i <= 100000; ++i) {
        float a = (float)(2.0 * B3_MAX_ROTATION * i / 100000.0);
        B3Vec3 dw = b3_v(a * 0.36f, a * -0.48f, a * 0.8f);
        double angle = sqrt((double)dw.x*dw.x + (double)dw.y*dw.y
            + (double)dw.z*dw.z);
        double s = angle == 0 ? 0.5 : sin(angle/2) / angle;
        double x = s*dw.x, y = s*dw.y, z = s*dw.z, w = cos(angle/2);
        double expected[4] = {
            (w*q.v.x + x*q.s + y*q.v.z - z*q.v.y)/qn,
            (w*q.v.y + y*q.s + z*q.v.x - x*q.v.z)/qn,
            (w*q.v.z + z*q.s + x*q.v.y - y*q.v.x)/qn,
            (w*q.s - x*q.v.x - y*q.v.y - z*q.v.z)/qn};
        B3Quat p = b3_q_integrate(q, dw);
        double got[4] = {p.v.x, p.v.y, p.v.z, p.s};
        for (int j = 0; j < 4; ++j) {
            double err = fabs(got[j] - expected[j]);
            require(isfinite(err) && err < 3e-7);
            if (err > worst) worst = err;
        }
        if (angle <= B3_MAX_ROTATION) {
            double err = angle - 2 * atan(angle/2);
            if (err > euler_worst) euler_worst = err;
        }
    }
    B3Quat p = b3_q_id();
    float increment = 0.001f;
    for (int i = 0; i < 100000; ++i)
        p = b3_q_integrate(p, b3_v(0, 0, increment));
    double half = 100000.0 * increment / 2;
    double drift = hypot(p.v.z - sin(half), p.s - cos(half));
    require(drift < 1e-3 && fabs(b3_qdot(p, p) - 1) < 3e-7);
    printf("max quaternion component error %.9g; capped Euler angular error %.9g rad; 100k-step drift %.9g\n",
        worst, euler_worst, drift);
}
