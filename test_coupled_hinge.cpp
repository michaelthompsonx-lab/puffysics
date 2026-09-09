// One unbiased hinge solve should remove all five constrained velocity residuals.
#include <stdio.h>
#define B3_PACKED_GS 1
#define B3_REVOLUTE_ONLY 1
#define B3_JOINT_ITERS 1
#define B3_MAX_BODIES 2
#define B3_MAX_SHAPES 2
#define B3_MAX_JOINTS 1
#define B3_MAX_CONTACTS 1
#ifndef B3_COUPLED_HINGE
#define B3_COUPLED_HINGE 1
#endif
#include "puffysics.cuh"

int main() {
    float worst=0;
    for(int trial=0;trial<24;trial++) {
        B3World w; b3_world_init(&w);
        for(int i=0;i<2;i++) {
            B3BodyDef bd=b3_default_body();
            bd.type=(trial%3==0 && i==0)?B3_STATIC:B3_DYNAMIC;
            bd.position=b3_v(i*0.4f,0,0);
            bd.rotation=b3_q_axis_angle(b3_norm(b3_v(1,2,3)),0.13f*(trial+i));
            bd.lin_vel=bd.type==B3_DYNAMIC?b3_v(i?0.2f:-0.7f,0.3f*i,-0.5f):b3_v(0,0,0);
            bd.ang_vel=bd.type==B3_DYNAMIC?b3_v(0.2f,-0.3f*(i+1),0.8f*i):b3_v(0,0,0);
            int b=b3_create_body(&w,&bd);
            B3ShapeDef sd=b3_default_shape(); sd.density=i?1.0f:1.0f+trial%10;
            b3_create_box(&w,b,b3_v(0.2f,0.3f,0.15f),&sd); b3_finalize_mass(&w,b);
        }
        b3_create_revolute(&w,0,1,b3_v(0.2f,0.15f,-0.1f),b3_v(-0.2f,-0.12f,0.1f),b3_norm(b3_v(1,1,2)));
        b3_prepare_joints(&w,0.005f);
        B3GsBody bodies[2]; B3GsJoint joints[1]; B3GsContact contacts[1];
        b3_gs_load(&w,bodies,joints,contacts);
        b3_gs_solve(&w,bodies,joints,contacts,0.005f,200,0);
        B3GsJoint* j=&joints[0]; B3GsBody* a=&bodies[0]; B3GsBody* b=&bodies[1];
        B3Vec3 dv=b3_sub(b3_add(b->lin_vel,b3_cross(b->ang_vel,j->cache_rb)),
            b3_add(a->lin_vel,b3_cross(a->ang_vel,j->cache_ra)));
        B3Vec3 dw=b3_sub(b->ang_vel,a->ang_vel);
        float residual=fmaxf(b3_len(dv),fmaxf(fabsf(b3_dot(dw,j->perp_x)),fabsf(b3_dot(dw,j->perp_y))));
        if(!isfinite(residual)||residual>2e-5f) {
            fprintf(stderr,"FAIL trial=%d hinge velocity residual=%g\n",trial,residual); return 1;
        }
        worst=fmaxf(worst,residual);
    }
    printf("coupled hinge: 24 configurations, maximum velocity residual=%g\n",worst);
}
