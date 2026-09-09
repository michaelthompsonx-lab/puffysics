// Independent dense velocity-response oracle for branched, reversed and rooted
// hinge forests. Also checks soft GS fixed-point residuals and atomic fallback.
#define B3_PACKED_GS 1
#define B3_REVOLUTE_ONLY 1
#define B3_TREE_DUAL 1
#define B3_MAX_BODIES 10
#define B3_MAX_JOINTS 10
#define B3_MAX_SHAPES 10
#define B3_MAX_CONTACTS 1
#include "puffysics.cuh"
#include <stdio.h>

static void read_rows(const B3GsBody* b,const B3GsJoint* j,double* out) {
    const B3GsBody &a=b[j->body_a], &c=b[j->body_b];
    B3Vec3 p=b3_sub(b3_add(c.lin_vel,b3_cross(c.ang_vel,j->cache_rb)),
        b3_add(a.lin_vel,b3_cross(a.ang_vel,j->cache_ra)));
    B3Vec3 v=b3_sub(c.ang_vel,a.ang_vel);
    out[0]=p.x;out[1]=p.y;out[2]=p.z;
    out[3]=b3_dot(v,j->perp_x);out[4]=b3_dot(v,j->perp_y);
}
static void impulse(B3GsBody* b,const B3GsJoint* j,int row) {
    B3Vec3 p=b3_v(row==0,row==1,row==2);
    B3Vec3 a=row==3?j->perp_x:row==4?j->perp_y:b3_v(0,0,0);
    for(int side=0;side<2;side++) {
        B3GsBody* v=&b[side?j->body_b:j->body_a];
        float s=side?1.f:-1.f;
        v->lin_vel=b3_madd(v->lin_vel,s*v->inv_mass,p);
        v->ang_vel=b3_madd(v->ang_vel,s,b3_mv(v->inv_i,
            b3_add(b3_cross(side?j->cache_rb:j->cache_ra,p),a)));
    }
}
int main() {
    double worst=0; int failures=0;
    for(int fixture=0;fixture<5;fixture++) for(int root=0;root<3;root++) for(int bias=0;bias<2;bias++) {
        B3World w; b3_world_init(&w);
        for(int i=0;i<8;i++) {
            B3BodyDef bd=b3_default_body(); bd.type=i==0?root:B3_DYNAMIC;
            bd.position=b3_v(.17f*i,.13f*(i%3),.1f*i);
            bd.rotation=b3_q_axis_angle(b3_norm(b3_v(1,2,3)),.13f*i);
            int id=b3_create_body(&w,&bd); B3ShapeDef sd=b3_default_shape(); sd.density=1+i;
            b3_create_box(&w,id,b3_v(.2f,.3f,.15f),&sd); b3_finalize_mass(&w,id);
        }
        for(int edge=1;edge<8;edge++) {
            int i=fixture==3 ? 8-edge:edge;
            if(fixture==4 && i==4) continue; // disconnected forest
            int pa=fixture==0?0:fixture==1?(i-1)/2:i-1;
            if(fixture==4) pa=i==5?4:pa;
            int a=(i&1)?pa:i, b=(i&1)?i:pa;
            b3_create_revolute(&w,a,b,b3_v(.05f*i,.07f,-.03f),b3_v(-.1f,.03f*i,.08f),b3_norm(b3_v(1,i+1,2)));
        }
        b3_prepare_joints(&w,.005f);
        B3GsBody bl[10],zero[10]; B3GsJoint jl[10]; B3GsContact cl[1];
        b3_gs_load(&w,bl,jl,cl);
        for(int i=0;i<8;i++) {
            bl[i].lin_vel=b3_v(.03f*i,-.02f*i,.07f);
            bl[i].ang_vel=b3_v(.02f,.01f*i,-.04f*i);
        }
        for(int i=0;i<w.joint_count;i++) {
            b3_cache_revolute_gs(&jl[i],&w.joints[i],&bl[jl[i].body_a],&bl[jl[i].body_b]);
            jl[i].linear_impulse=b3_v(.0001f*i,-.0002f,.0003f);
            jl[i].perp_impulse={.0001f,-.0002f};
        }
        B3TreeFactor f;
        if(!b3_tree_factor(&w,bl,jl,bias,&f)) { failures++; continue; }
        int n=5*w.joint_count; double a[50][50]={{0}}, rhs[50], old[50], before[50];
        for(int j=0;j<w.joint_count;j++) {
            read_rows(bl,&jl[j],before+5*j);
            old[5*j]=jl[j].linear_impulse.x;old[5*j+1]=jl[j].linear_impulse.y;old[5*j+2]=jl[j].linear_impulse.z;
            old[5*j+3]=jl[j].perp_impulse.x;old[5*j+4]=jl[j].perp_impulse.y;
        }
        for(int c=0;c<n;c++) {
            memcpy(zero,bl,sizeof(bl));
            for(int b=0;b<8;b++) zero[b].lin_vel=zero[b].ang_vel=b3_v(0,0,0);
            impulse(zero,&jl[c/5],c%5);
            for(int j=0;j<w.joint_count;j++) { double row[5];read_rows(zero,&jl[j],row);for(int r=0;r<5;r++) a[5*j+r][c]=row[r]; }
        }
        double physical[50][50]; memcpy(physical,a,sizeof(a));
        for(int r=0;r<n;r++) {
            B3GsJoint* j=&jl[r/5];
            B3Vec3 sep=b3_add(b3_sub(j->cache_rb,j->cache_ra),j->delta_center);
            float err[5]={sep.x,sep.y,sep.z,j->cache_rel_x,j->cache_rel_y};
            rhs[r]=-before[r]-(bias?j->softness.bias_rate*err[r%5]:0);
            for(int c=5*(r/5);c<5*(r/5+1);c++) if(B3_COUPLED_HINGE || ((r%5<3)==(c%5<3))) {
                double reg=bias?a[r][c]*j->softness.impulse_scale/j->softness.mass_scale:0;
                rhs[r]-=reg*old[c]; a[r][c]+=reg;
            }
        }
        b3_tree_project(&w,bl,jl,bias,&f);
        double x[50];
        for(int j=0;j<w.joint_count;j++) {
            x[5*j]=jl[j].linear_impulse.x-old[5*j];x[5*j+1]=jl[j].linear_impulse.y-old[5*j+1];x[5*j+2]=jl[j].linear_impulse.z-old[5*j+2];
            x[5*j+3]=jl[j].perp_impulse.x-old[5*j+3];x[5*j+4]=jl[j].perp_impulse.y-old[5*j+4];
        }
        double residual=0,scale=0;
        for(int r=0;r<n;r++) { double v=-rhs[r];for(int c=0;c<n;c++) v+=a[r][c]*x[c];residual=fmax(residual,fabs(v));scale=fmax(scale,fabs(rhs[r])); }
        for(int j=0;j<w.joint_count;j++) {
            double after[5];read_rows(bl,&jl[j],after);
            for(int i=0;i<5;i++) {
                int r=5*j+i;double expected=before[r];
                for(int c=0;c<n;c++) expected+=physical[r][c]*x[c];
                residual=fmax(residual,fabs(expected-after[i]));
            }
        }
        residual/=fmax(1.,scale);worst=fmax(worst,residual);
        if(residual>2e-4) {printf("FAIL fixture=%d root=%d bias=%d residual=%g\n",fixture,root,bias,residual);failures++;}
        // Close a cycle, or request fixed rotation: factorization must decline.
        b3_create_revolute(&w,0,7,b3_v(0,0,0),b3_v(0,0,0),b3_v(0,0,1));
        if(fixture!=4 && b3_tree_order(&w,&f)) failures++;
        w.joints[0].fixed_rotation=1;if(b3_tree_order(&w,&f)) failures++;
        B3GsBody saved[10];B3GsJoint saved_j[10];
        memcpy(saved,bl,sizeof(bl));memcpy(saved_j,jl,sizeof(jl));
        if(b3_tree_solve(&w,bl,jl,cl,.005f,200,bias) ||
            memcmp(saved,bl,sizeof(bl)) || memcmp(saved_j,jl,sizeof(jl))) failures++;
        w.joints[0].fixed_rotation=0;w.joint_count--;
        for(int b=0;b<8;b++) { bl[b].inv_mass=0;bl[b].inv_i=b3_mat0(); }
        memcpy(saved,bl,sizeof(bl));
        if(b3_tree_solve(&w,bl,jl,cl,.005f,200,bias) ||
            memcmp(saved,bl,sizeof(bl)) || memcmp(saved_j,jl,sizeof(jl))) failures++;
    }
    printf("tree dual: 30 configurations, worst normalized dense residual=%g, failures=%d\n",worst,failures);
    return failures?1:0;
}
