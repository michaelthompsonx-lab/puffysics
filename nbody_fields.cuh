#ifndef NBODY_FIELDS_CUH
#define NBODY_FIELDS_CUH
// Non-gravitational force demonstrations, independent of rendering and RL.
// Spring forces couple neighbouring lattice displacements. Vortex and charged
// particles respond to prescribed external fields, not pairwise Coulomb forces.
#include "nbody.cuh"
enum NbodyFieldKind { NBODY_VORTEX, NBODY_SPRINGS, NBODY_CHARGES };
struct NbodyFieldConfig {
    int kind, width;
    float strength, drag, confinement, spin, spacing, wave_speed;
    NbodyVec electric, magnetic;
};
static NbodyFieldConfig nbody_field_default(int kind,int n) {
    NbodyFieldConfig c={};c.kind=kind;c.strength=1;
    c.width=(int)ceil(sqrt((double)n));if(c.width<2)c.width=2;
    c.spacing=12.0f/(c.width-1);c.wave_speed=.9f;
    if(kind==NBODY_VORTEX){c.drag=1.8f;c.confinement=.36f;c.spin=2.1f;}
    else if(kind==NBODY_SPRINGS){c.drag=.04f;c.confinement=.10f;}
    else {c.confinement=.18f;c.electric={.2f,0,0};c.magnetic={0,1.8f,0};}
    return c;
}
static int nbody_field_cfg_ok(NbodyFieldConfig c) {
    return c.kind>=NBODY_VORTEX&&c.kind<=NBODY_CHARGES&&c.width>=2
        &&isfinite(c.strength)&&c.strength>0&&c.strength<=4
        &&isfinite(c.drag)&&c.drag>=0&&isfinite(c.confinement)&&c.confinement>=0
        &&isfinite(c.spin)&&isfinite(c.spacing)&&c.spacing>0
        &&isfinite(c.wave_speed)&&c.wave_speed>=0
        &&isfinite(c.electric.x)&&isfinite(c.electric.y)&&isfinite(c.electric.z)
        &&isfinite(c.magnetic.x)&&isfinite(c.magnetic.y)&&isfinite(c.magnetic.z);
}
static NBODY_HD NbodyVec nbody_field_cross(NbodyVec a,NbodyVec b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
static NBODY_HD bool nbody_field_pinned(int i,int n,int width) {
    return i<width||i+width>=n||i%width==0||i%width==width-1;
}
static NBODY_HD NbodyVec nbody_field_spring_acceleration(const NbodyPoint *p,
        const NbodyVec *rest,int n,int i,NbodyFieldConfig c) {
    if(nbody_field_pinned(i,n,c.width))return {0,0,0};
    NbodyVec u={p[i].x-rest[i].x,p[i].y-rest[i].y,p[i].z-rest[i].z};
    NbodyVec a={-c.confinement*u.x,-c.confinement*u.y,-c.confinement*u.z};
    float k=c.wave_speed*c.wave_speed/(c.spacing*c.spacing);
    int adjacent[4]={i-1,i+1,i-c.width,i+c.width};
    for(int j=0;j<4;++j){int b=adjacent[j];
        a.x+=k*((p[b].x-rest[b].x)-u.x);
        a.y+=k*((p[b].y-rest[b].y)-u.y);
        a.z+=k*((p[b].z-rest[b].z)-u.z);
    }
    return {c.strength*a.x,c.strength*a.y,c.strength*a.z};
}
static NBODY_HD NbodyVec nbody_field_flow(NbodyPoint p,NbodyFieldConfig c) {
    float omega=c.spin/(1+.1f*(p.x*p.x+p.z*p.z));
    return {-omega*p.z,.9f*sinf(.6f*p.x)*cosf(.6f*p.z),omega*p.x};
}
static NBODY_HD void nbody_field_vortex_step(NbodyPoint *p,NbodyVec *v,
        NbodyFieldConfig c,float dt) {
    // Exact drag half-steps around a conservative harmonic Verlet step.
    float decay=expf(-.5f*c.drag*c.strength*dt),k=c.confinement*c.strength;
    NbodyVec flow=nbody_field_flow(*p,c);
    v->x=flow.x+(v->x-flow.x)*decay;v->y=flow.y+(v->y-flow.y)*decay;v->z=flow.z+(v->z-flow.z)*decay;
    v->x-=.5f*dt*k*p->x;v->y-=dt*k*p->y;v->z-=.5f*dt*k*p->z;
    p->x+=dt*v->x;p->y+=dt*v->y;p->z+=dt*v->z;
    v->x-=.5f*dt*k*p->x;v->y-=dt*k*p->y;v->z-=.5f*dt*k*p->z;
    flow=nbody_field_flow(*p,c);
    v->x=flow.x+(v->x-flow.x)*decay;v->y=flow.y+(v->y-flow.y)*decay;v->z=flow.z+(v->z-flow.z)*decay;
}
static NBODY_HD void nbody_field_boris_step(NbodyPoint *p,NbodyVec *v,
        float charge_to_mass,NbodyFieldConfig c,float dt) {
    // Symmetric drift/Boris/drift; integer-time positions and velocities.
    // The magnetic rotation preserves speed when electric/confinement = 0.
    p->x+=.5f*dt*v->x;p->y+=.5f*dt*v->y;p->z+=.5f*dt*v->z;
    float h=.5f*dt*c.strength;
    NbodyVec kick={h*(charge_to_mass*c.electric.x-c.confinement*p->x),
        h*(charge_to_mass*c.electric.y-c.confinement*p->y),h*(charge_to_mass*c.electric.z-c.confinement*p->z)};
    NbodyVec minus={v->x+kick.x,v->y+kick.y,v->z+kick.z};
    NbodyVec t={h*charge_to_mass*c.magnetic.x,h*charge_to_mass*c.magnetic.y,h*charge_to_mass*c.magnetic.z};
    float f=2/(1+t.x*t.x+t.y*t.y+t.z*t.z);
    NbodyVec s={f*t.x,f*t.y,f*t.z},cross=nbody_field_cross(minus,t);
    NbodyVec prime={minus.x+cross.x,minus.y+cross.y,minus.z+cross.z};
    cross=nbody_field_cross(prime,s);
    *v={minus.x+cross.x+kick.x,minus.y+cross.y+kick.y,minus.z+cross.z+kick.z};
    p->x+=.5f*dt*v->x;p->y+=.5f*dt*v->y;p->z+=.5f*dt*v->z;
}
static NBODY_HD void nbody_field_spring_kick(NbodyPoint *p,NbodyVec *v,NbodyVec a,
        NbodyVec rest,int i,int n,NbodyFieldConfig c,float dt,bool drift) {
    if(nbody_field_pinned(i,n,c.width)){p->x=rest.x;p->y=rest.y;p->z=rest.z;*v={0,0,0};return;}
    float decay=expf(-.5f*c.drag*c.strength*dt);
    if(drift){v->x*=decay;v->y*=decay;v->z*=decay;}
    v->x+=.5f*dt*a.x;v->y+=.5f*dt*a.y;v->z+=.5f*dt*a.z;
    if(drift){p->x+=dt*v->x;p->y+=dt*v->y;p->z+=dt*v->z;}
    else {v->x*=decay;v->y*=decay;v->z*=decay;}
}
static int nbody_fields_step(NbodyPoint *p,NbodyVec *v,NbodyVec *a,const NbodyVec *rest,
        int n,NbodyFieldConfig c,float dt,int steps) {
    if(n<0||steps<0||!nbody_field_cfg_ok(c)||!isfinite(dt)||dt<0
            ||(n&&(!p||!v||!a||(c.kind==NBODY_SPRINGS&&!rest))))return 0;
    if(n==0||steps==0||dt==0)return 1;
    if(c.kind!=NBODY_SPRINGS){
        for(int i=0;i<n;++i)for(int step=0;step<steps;++step)
            if(c.kind==NBODY_VORTEX)nbody_field_vortex_step(p+i,v+i,c,dt);
            else nbody_field_boris_step(p+i,v+i,(i&1)?-1.f:1.f,c,dt);
    }else{
        for(int i=0;i<n;++i)a[i]=nbody_field_spring_acceleration(p,rest,n,i,c);
        for(int step=0;step<steps;++step){
            for(int i=0;i<n;++i)nbody_field_spring_kick(p+i,v+i,a[i],rest[i],i,n,c,dt,true);
            for(int i=0;i<n;++i)a[i]=nbody_field_spring_acceleration(p,rest,n,i,c);
            for(int i=0;i<n;++i)nbody_field_spring_kick(p+i,v+i,a[i],rest[i],i,n,c,dt,false);
        }
    }
    return 1;
}
#ifdef __CUDACC__
static __global__ void nbody_fields_points_k(NbodyPoint *p,NbodyVec *v,int n,NbodyFieldConfig c,float dt,int steps){
    int i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=n)return;
    NbodyPoint point=p[i];NbodyVec velocity=v[i];
    for(int step=0;step<steps;++step)
        if(c.kind==NBODY_VORTEX)nbody_field_vortex_step(&point,&velocity,c,dt);
        else nbody_field_boris_step(&point,&velocity,(i&1)?-1.f:1.f,c,dt);
    p[i]=point;v[i]=velocity;
}
static __global__ void nbody_fields_springs_k(const NbodyPoint *p,NbodyVec *a,const NbodyVec *rest,int n,NbodyFieldConfig c){
    int i=blockIdx.x*blockDim.x+threadIdx.x;if(i<n)a[i]=nbody_field_spring_acceleration(p,rest,n,i,c);
}
static __global__ void nbody_fields_kick_k(NbodyPoint *p,NbodyVec *v,const NbodyVec *a,
        const NbodyVec *rest,int n,NbodyFieldConfig c,float dt,bool drift){
    int i=blockIdx.x*blockDim.x+threadIdx.x;if(i<n)nbody_field_spring_kick(p+i,v+i,a[i],rest[i],i,n,c,dt,drift);
}
static cudaError_t nbody_fields_step_gpu(NbodyPoint *p,NbodyVec *v,NbodyVec *a,const NbodyVec *rest,
        int n,NbodyFieldConfig c,float dt,int steps,cudaStream_t stream){
    if(n<0||steps<0||!nbody_field_cfg_ok(c)||!isfinite(dt)||dt<0
            ||(n&&(!p||!v||!a||(c.kind==NBODY_SPRINGS&&!rest))))return cudaErrorInvalidValue;
    if(n==0||steps==0||dt==0)return cudaSuccess;
    int blocks=(n+255)/256;
    if(c.kind!=NBODY_SPRINGS)nbody_fields_points_k<<<blocks,256,0,stream>>>(p,v,n,c,dt,steps);
    else {
        nbody_fields_springs_k<<<blocks,256,0,stream>>>(p,a,rest,n,c);
        cudaError_t e=cudaGetLastError();if(e!=cudaSuccess)return e;
        for(int step=0;step<steps;++step){
            nbody_fields_kick_k<<<blocks,256,0,stream>>>(p,v,a,rest,n,c,dt,true);
            e=cudaGetLastError();if(e!=cudaSuccess)return e;
            nbody_fields_springs_k<<<blocks,256,0,stream>>>(p,a,rest,n,c);
            e=cudaGetLastError();if(e!=cudaSuccess)return e;
            nbody_fields_kick_k<<<blocks,256,0,stream>>>(p,v,a,rest,n,c,dt,false);
            e=cudaGetLastError();if(e!=cudaSuccess)return e;
        }
    }
    return cudaGetLastError();
}
#endif
#endif
