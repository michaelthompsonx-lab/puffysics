// Selecting box B as the reference face must select the facing side of A.
#include <stdio.h>
#ifndef B3_TEST_HEADER
#define B3_TEST_HEADER "puffysics.cuh"
#endif
#include B3_TEST_HEADER
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while (0)

int main() {
    for (int axis=0;axis<3;axis++) for (int sign=-1;sign<=1;sign+=2) {
        B3Vec3 n=b3_v(axis==0,axis==1,axis==2);
        B3Vec3 half=b3_v(axis==0 ? 0.5f:20,axis==1 ? 0.5f:20,axis==2 ? 0.5f:20);
        B3Obb floor=b3_obb(b3_mul(n,-0.5f),b3_q_id(),half);
        B3Vec3 tilt=b3_v(axis==2,axis==0,axis==1);
        B3Obb box=b3_obb(b3_mul(n,0.5f),b3_q_axis_angle(tilt,sign*0.005f),b3_v(0.5f,0.5f,0.5f));
        B3Mani ab,ba;
        b3_collide_boxes(&ab,&box,&floor);
        b3_collide_boxes(&ba,&floor,&box);
        CHECK(ab.count==4 && ba.count==4);
        CHECK(b3_len(b3_add(ab.normal,ba.normal))<1e-6f);
        CHECK(b3_dot(ba.normal,n)>0.999f);
        for (int p=0;p<ab.count;p++) {
            CHECK(fabsf(ab.sep[p])<0.003f);
            CHECK(fabsf(b3_dot(ab.p_b[p],n))<1e-6f);
            bool match=false;
            for (int q=0;q<ba.count;q++) {
                if (b3_len(b3_sub(ab.p_a[p],ba.p_b[q]))<1e-6f
                    && b3_len(b3_sub(ab.p_b[p],ba.p_a[q]))<1e-6f
                    && fabsf(ab.sep[p]-ba.sep[q])<1e-6f) match=true;
            }
            CHECK(match);
        }
    }
    puts("box face orientation passed: all axes, both tilts, both shape orders");
}
