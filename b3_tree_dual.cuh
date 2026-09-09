// Packed rigid-body adaptation of M-ABD's topology-aware KKT condensation
// (doi:10.1145/3811276, section 5). This is not an affine-body integrator.
// Eliminate leaf joints by updating the parent's 6x6 mobility. This retains
// sibling couplings without storing the dense joint-space matrix. Pose-dependent
// factors are rebuilt per pass; multiple contact/axial iterations reuse them.
// Contacts: Delassus via one B3Art bind per tree solve (independent 1/m if
// B3_ART_CONTACTS=0 or the bind fails).
// B3_TREE_SPRINGS adds the implicit axial row (six rows per joint). Torque
// saturation is a box-constrained QP active set, not post-solve clipping.
// At a leaf: D = Gc Pc Gc^T + Gp Pp Gp^T + R, T = D^-1 Gp Pp.
// Condense into the parent: Pp -= (Gp Pp)^T T, zp += T^T rhs.
// Reverse the elimination with lambda = D^-1 rhs - T * parent_impulses.
// R is the original GS block softness; using the condensed D for R would
// change the fixed point. Only factorization workspace changes on failure.
#pragma once

#define B3_TREE_ROWS (5 + B3_TREE_SPRINGS)
#define B3_TR B3_TREE_ROWS

typedef struct B3TreeFactor {
    int order[B3_MAX_JOINTS], parent[B3_MAX_JOINTS], child[B3_MAX_JOINTS];
    signed char active[B3_MAX_JOINTS];
    float invd[B3_MAX_JOINTS][B3_TR*B3_TR];
    float regularizer[B3_MAX_JOINTS][B3_TR*B3_TR];
    float transfer[B3_MAX_JOINTS][B3_TR*6]; // D^-1 G_parent P_parent
} B3TreeFactor;

// Accept a forest; unrelated free bodies are harmless. Peeling also detects
// cycles without changing velocities or accumulated impulses on fallback.
static B3_HD B3_INL int b3_tree_order(const B3World* w, B3TreeFactor* f) {
    int deg[B3_MAX_BODIES] = {0};
    unsigned char used[B3_MAX_JOINTS] = {0};
    if (w->joint_count < 1) return 0;
    for (int k=0;k<w->joint_count;k++) {
        const B3Joint* j=&w->joints[k];
        if (j->fixed_rotation || j->body_a==j->body_b) return 0;
#ifndef B3_REVOLUTE_ONLY
        if (j->type!=B3_JOINT_REVOLUTE || j->enable_motor) return 0;
#endif
        deg[j->body_a]++; deg[j->body_b]++;
    }
    for (int t=0;t<w->joint_count;t++) {
        int found=-1;
        for (int k=0;k<w->joint_count;k++) {
            const B3Joint* j=&w->joints[k];
            if (!used[k] && (deg[j->body_a]==1 || deg[j->body_b]==1)) {
                found=k; break;
            }
        }
        if (found<0) return 0;
        const B3Joint* j=&w->joints[found];
        int c=deg[j->body_a]==1 ? j->body_a:j->body_b;
        f->order[t]=found; f->child[t]=c;
        f->parent[t]=c==j->body_a ? j->body_b:j->body_a;
        used[found]=1; deg[j->body_a]--; deg[j->body_b]--;
    }
    return 1;
}

static B3_HD B3_INL void b3_tree_gradient(const B3GsJoint* j, int body, float* g) {
    float sign=body==j->body_a ? -1.0f:1.0f;
    B3Vec3 r=body==j->body_a ? j->cache_ra:j->cache_rb;
    for(int i=0;i<B3_TR*6;i++) g[i]=0;
    for(int i=0;i<B3_TR;i++) {
        B3Vec3 a;
        if(i<3) {
            B3Vec3 e=b3_v(i==0, i==1, i==2);
            g[i*6+i]=sign; a=b3_cross(r,e);
        } else if(i<5) a=i==3 ? j->perp_x:j->perp_y;
        else a=(j->bits & B3_GS_SPRING) ? j->rotation_axis:b3_v(0,0,0);
        g[i*6+3]=sign*a.x; g[i*6+4]=sign*a.y; g[i*6+5]=sign*a.z;
    }
}
static B3_HD B3_INL void b3_tree_mask_gradient(float* g, int active) {
#if B3_TREE_SPRINGS
    if(active) for(int q=0;q<6;q++) g[5*6+q]=0;
#endif
}
static B3_HD B3_INL void b3_tree_mobility(const B3GsBody* b, float* p) {
    for(int i=0;i<36;i++) p[i]=0;
    p[0]=p[7]=p[14]=b->inv_mass;
    B3Vec3 col[3]={b->inv_i.cx,b->inv_i.cy,b->inv_i.cz};
    for(int c=0;c<3;c++) {
        p[3*6+3+c]=col[c].x; p[4*6+3+c]=col[c].y; p[5*6+3+c]=col[c].z;
    }
}
static B3_HD B3_INL void b3_tree_gp(const float* g,const float* p,float* out) {
    for(int i=0;i<B3_TR;i++) for(int c=0;c<6;c++) {
        float v=i<3 ? g[i*6+i]*p[i*6+c]:0;
        for(int q=3;q<6;q++) v+=g[i*6+q]*p[q*6+c];
        out[i*6+c]=v;
    }
}
static B3_HD B3_INL void b3_tree_gram(const float* gp,const float* g,float* d) {
    for(int i=0;i<B3_TR;i++) for(int c=0;c<B3_TR;c++) {
        float v=c<3 ? gp[i*6+c]*g[c*6+c]:0;
        for(int q=3;q<6;q++) v+=gp[i*6+q]*g[c*6+q];
        d[i*B3_TR+c]+=v;
    }
}
// SPD Cholesky, with a relative pivot check. Failure precedes any state writes.
static B3_HD B3_INL int b3_tree_inverse(float* d,float* out) {
    for(int i=0;i<B3_TR;i++) {
        float scale=d[i*B3_TR+i];
        for(int c=0;c<=i;c++) {
            float v=d[i*B3_TR+c];
            for(int q=0;q<c;q++) v-=d[i*B3_TR+q]*d[c*B3_TR+q];
            if(i==c) {
                if(!(v>1e-7f*scale) || !isfinite(v)) return 0;
                d[i*B3_TR+c]=sqrtf(v);
            } else d[i*B3_TR+c]=v/d[c*B3_TR+c];
        }
    }
    for(int c=0;c<B3_TR;c++) {
        float x[B3_TR];
        for(int i=0;i<B3_TR;i++) {
            float v=i==c ? 1.0f:0.0f;
            for(int q=0;q<i;q++) v-=d[i*B3_TR+q]*x[q];
            x[i]=v/d[i*B3_TR+i];
        }
        for(int i=B3_TR-1;i>=0;i--) {
            float v=x[i]; for(int q=i+1;q<B3_TR;q++) v-=d[q*B3_TR+i]*x[q];
            x[i]=v/d[i*B3_TR+i]; out[i*B3_TR+c]=x[i];
        }
    }
    return 1;
}
// Legacy uncoupled GS regularizes anchor and alignment blocks separately.
// A coupled-hinge build regularizes the full joint block instead.
static B3_HD B3_INL void b3_tree_regularizer(const B3GsJoint* j,
        const B3GsBody* bl,int use_bias,float* r) {
    for(int i=0;i<B3_TR*B3_TR;i++) r[i]=0;
    if(use_bias) {
        float g[B3_TR*6],p[36],gp[B3_TR*6];
        for(int side=0;side<2;side++) {
            int b=side ? j->body_b:j->body_a;
            b3_tree_gradient(j,b,g); b3_tree_mobility(&bl[b],p);
            b3_tree_gp(g,p,gp); b3_tree_gram(gp,g,r);
        }
        float s=j->softness.impulse_scale/j->softness.mass_scale;
        for(int i=0;i<B3_TR;i++) for(int c=0;c<B3_TR;c++)
            r[i*B3_TR+c]*=(i<5 && c<5 &&
                (B3_COUPLED_HINGE || ((i<3)==(c<3)))) ? s:0.0f;
    }
#if B3_TREE_SPRINGS
    // The axial spring remains soft in BOTH biased and relaxation passes.
    // Its scalar fixed point is Cdot+bias+R*lambda=0.
    r[5*B3_TR+5]=(j->bits & B3_GS_SPRING) ?
        j->spring_softness.impulse_scale /
            (j->spring_softness.mass_scale*j->axial_mass):1.0f;
#endif
}
static B3_HD B3_INL int b3_tree_factor_active(const B3World* w,const B3GsBody* bl,
        const B3GsJoint* jl,int use_bias,B3TreeFactor* f,const signed char* active) {
    if(!b3_tree_order(w,f)) return 0;
    for(int k=0;k<w->joint_count;k++) f->active[k]=active ? active[k]:0;
    float p[B3_MAX_BODIES][36];
    for(int b=0;b<w->body_count;b++) b3_tree_mobility(&bl[b],p[b]);
    for(int t=0;t<w->joint_count;t++) {
        const B3GsJoint* j=&jl[f->order[t]];
        int pa=f->parent[t],ch=f->child[t];
        float gp[B3_TR*6],gc[B3_TR*6],u[B3_TR*6],v[B3_TR*6],d[B3_TR*B3_TR];
        b3_tree_gradient(j,pa,gp); b3_tree_gradient(j,ch,gc);
        b3_tree_mask_gradient(gp,f->active[f->order[t]]);
        b3_tree_mask_gradient(gc,f->active[f->order[t]]);
        b3_tree_gp(gp,p[pa],u); b3_tree_gp(gc,p[ch],v);
        b3_tree_regularizer(j,bl,use_bias,f->regularizer[t]);
#if B3_TREE_SPRINGS
        if(f->active[f->order[t]]) f->regularizer[t][5*B3_TR+5]=1.0f;
#endif
        for(int i=0;i<B3_TR*B3_TR;i++) d[i]=f->regularizer[t][i];
        b3_tree_gram(u,gp,d); b3_tree_gram(v,gc,d);
        if(!b3_tree_inverse(d,f->invd[t])) return 0;
        for(int i=0;i<B3_TR;i++) for(int c=0;c<6;c++) {
            float x=0; for(int q=0;q<B3_TR;q++) x+=f->invd[t][i*B3_TR+q]*u[q*6+c];
            f->transfer[t][i*6+c]=x;
        }
        for(int i=0;i<6;i++) for(int c=0;c<6;c++) {
            float x=0; for(int q=0;q<B3_TR;q++) x+=u[q*6+i]*f->transfer[t][q*6+c];
            p[pa][i*6+c]-=x;
        }
    }
    return 1;
}
static B3_HD B3_INL int b3_tree_factor(const B3World* w,const B3GsBody* bl,
        const B3GsJoint* jl,int use_bias,B3TreeFactor* f) {
    return b3_tree_factor_active(w,bl,jl,use_bias,f,0);
}
typedef struct B3TreeSolution {
    float delta[B3_MAX_JOINTS][B3_TR]; // original joint order
    float force[B3_MAX_BODIES][6];
} B3TreeSolution;
static B3_HD B3_INL void b3_tree_compute(const B3World* w,const B3GsBody* bl,
        const B3GsJoint* jl,int use_bias,const B3TreeFactor* f,
        const float* fixed,B3TreeSolution* sol) {
    float z[B3_MAX_BODIES][6]={{0}}, y[B3_MAX_JOINTS][B3_TR];
#if B3_TREE_SPRINGS
    if(fixed) for(int k=0;k<w->joint_count;k++) {
        const B3GsJoint* j=&jl[k];
        for(int side=0;side<2;side++) {
            int b=side?j->body_b:j->body_a;
            B3Vec3 dv=b3_mul(b3_mv(bl[b].inv_i,j->rotation_axis),
                (side?1.f:-1.f)*fixed[k]);
            z[b][3]+=dv.x;z[b][4]+=dv.y;z[b][5]+=dv.z;
        }
    }
#endif
    for(int t=0;t<w->joint_count;t++) {
        const B3GsJoint* j=&jl[f->order[t]];
        int pa=f->parent[t],ch=f->child[t];
        float gp[B3_TR*6],gc[B3_TR*6],rhs[B3_TR];
        const float* r=f->regularizer[t];
        b3_tree_gradient(j,pa,gp); b3_tree_gradient(j,ch,gc);
        b3_tree_mask_gradient(gp,f->active[f->order[t]]);
        b3_tree_mask_gradient(gc,f->active[f->order[t]]);
        float old[B3_TR]={j->linear_impulse.x,j->linear_impulse.y,j->linear_impulse.z,
            j->perp_impulse.x,j->perp_impulse.y};
#if B3_TREE_SPRINGS
        old[5]=j->spring_impulse;
#endif
        B3Vec3 sep=b3_add(b3_add(b3_sub(bl[j->body_b].delta_pos,bl[j->body_a].delta_pos),
            b3_sub(j->cache_rb,j->cache_ra)),j->delta_center);
        float bias[B3_TR]={sep.x,sep.y,sep.z,j->cache_rel_x,j->cache_rel_y};
        float vp[6]={bl[pa].lin_vel.x,bl[pa].lin_vel.y,bl[pa].lin_vel.z,
            bl[pa].ang_vel.x,bl[pa].ang_vel.y,bl[pa].ang_vel.z};
        float vc[6]={bl[ch].lin_vel.x,bl[ch].lin_vel.y,bl[ch].lin_vel.z,
            bl[ch].ang_vel.x,bl[ch].ang_vel.y,bl[ch].ang_vel.z};
        for(int i=0;i<B3_TR;i++) {
            float x=use_bias ? -j->softness.bias_rate*bias[i]:0;
            for(int q=0;q<6;q++) x-=gp[i*6+q]*(vp[q]+z[pa][q])+gc[i*6+q]*(vc[q]+z[ch][q]);
            for(int q=0;q<B3_TR;q++) x-=r[i*B3_TR+q]*old[q];
#if B3_TREE_SPRINGS
            if(i==5) {
                // Replace the hinge bias, which is disabled for relaxation,
                // with the spring's always-on target-position bias.
                if(!(j->bits & B3_GS_SPRING) || f->active[f->order[t]]) x=0;
                else x-=j->spring_softness.bias_rate*(j->cache_twist-j->target_angle);
            }
#endif
            rhs[i]=x;
        }
        for(int i=0;i<B3_TR;i++) {
            float x=0; for(int q=0;q<B3_TR;q++) x+=f->invd[t][i*B3_TR+q]*rhs[q];
            y[t][i]=x;
        }
        // P G^T D^-1 rhs = transfer^T rhs (symmetric D).
        for(int i=0;i<6;i++) for(int q=0;q<B3_TR;q++) z[pa][i]+=f->transfer[t][q*6+i]*rhs[q];
    }
    // z now holds accumulated generalized impulses from later eliminations.
    for(int b=0;b<w->body_count;b++) for(int q=0;q<6;q++) z[b][q]=0;
    for(int t=w->joint_count-1;t>=0;t--) {
        const B3GsJoint* j=&jl[f->order[t]];
        int pa=f->parent[t],ch=f->child[t];
        float gp[B3_TR*6],gc[B3_TR*6],lambda[B3_TR];
        b3_tree_gradient(j,pa,gp); b3_tree_gradient(j,ch,gc);
        b3_tree_mask_gradient(gp,f->active[f->order[t]]);
        b3_tree_mask_gradient(gc,f->active[f->order[t]]);
        for(int i=0;i<B3_TR;i++) {
            float x=y[t][i]; for(int q=0;q<6;q++) x-=f->transfer[t][i*6+q]*z[pa][q];
            lambda[i]=x;
        }
        for(int q=0;q<6;q++) for(int i=0;i<B3_TR;i++) {
            z[pa][q]+=gp[i*6+q]*lambda[i]; z[ch][q]+=gc[i*6+q]*lambda[i];
        }
        for(int i=0;i<B3_TR;i++) sol->delta[f->order[t]][i]=lambda[i];
    }
#if B3_TREE_SPRINGS
    if(fixed) for(int k=0;k<w->joint_count;k++) if(f->active[k]) {
        const B3GsJoint* j=&jl[k]; sol->delta[k][5]=fixed[k];
        for(int side=0;side<2;side++) {
            int b=side?j->body_b:j->body_a;
            B3Vec3 a=b3_mul(j->rotation_axis,(side?1.f:-1.f)*fixed[k]);
            z[b][3]+=a.x;z[b][4]+=a.y;z[b][5]+=a.z;
        }
    }
#endif
    for(int b=0;b<w->body_count;b++) for(int q=0;q<6;q++) sol->force[b][q]=z[b][q];
}
static B3_HD B3_INL void b3_tree_apply(const B3World* w,B3GsBody* bl,
        B3GsJoint* jl,const B3TreeSolution* sol) {
    for(int k=0;k<w->joint_count;k++) {
        B3GsJoint* j=&jl[k];const float* d=sol->delta[k];
        j->linear_impulse=b3_add(j->linear_impulse,b3_v(d[0],d[1],d[2]));
        j->perp_impulse.x+=d[3];j->perp_impulse.y+=d[4];
#if B3_TREE_SPRINGS
        j->spring_impulse+=d[5];
#endif
    }
    for(int b=0;b<w->body_count;b++) if(bl[b].flags & B3_FLAG_DYNAMIC) {
        const float* z=sol->force[b];
        bl[b].lin_vel=b3_madd(bl[b].lin_vel,bl[b].inv_mass,b3_v(z[0],z[1],z[2]));
        bl[b].ang_vel=b3_add(bl[b].ang_vel,b3_mv(bl[b].inv_i,b3_v(z[3],z[4],z[5])));
    }
}
static B3_HD B3_INL void b3_tree_project(const B3World* w,B3GsBody* bl,
        B3GsJoint* jl,int use_bias,const B3TreeFactor* f) {
    B3TreeSolution sol;b3_tree_compute(w,bl,jl,use_bias,f,0,&sol);
    b3_tree_apply(w,bl,jl,&sol);
}

#if B3_TREE_SPRINGS
// Bounded spring QP: change one active bound at a time, refactor, and check
// both primal feasibility and KKT signs. No trial changes physical state.
// An iteration cap or singular factor returns to the legacy projected rows.
#ifndef B3_TREE_EVENT
#define B3_TREE_EVENT(event) ((void)0)
#endif
static B3_HD B3_INL int b3_tree_spring_project(const B3World* w,B3GsBody* bl,
        B3GsJoint* jl,float h,int use_bias,B3TreeFactor* f) {
    B3TreeSolution sol;
    signed char active[B3_MAX_JOINTS];float fixed[B3_MAX_JOINTS];
    for(int k=0;k<w->joint_count;k++) active[k]=f->active[k];
    for(int pass=0;pass<4*w->joint_count+4;pass++) {
        for(int k=0;k<w->joint_count;k++) fixed[k]=active[k] ?
            active[k]*jl[k].max_motor_torque*h-jl[k].spring_impulse:0;
        b3_tree_compute(w,bl,jl,use_bias,f,fixed,&sol);
        int change=-1; signed char state=0;
        for(int k=0;k<w->joint_count;k++) {
            const B3GsJoint* j=&jl[k];float next=j->spring_impulse+sol.delta[k][5];
            if(!isfinite(next)) return 0;
            if(!(j->bits & B3_GS_SPRING) || j->max_motor_torque<=0) continue;
            float bound=j->max_motor_torque*h;
            if(!active[k] && (next>bound || next<-bound)) {
                change=k;state=next>bound ? 1:-1;break;
            }
        }
        if(change<0) for(int k=0;k<w->joint_count;k++) if(active[k]) {
            const B3GsJoint* j=&jl[k];int a=j->body_a,b=j->body_b;
            const float* fa=sol.force[a];const float* fb=sol.force[b];
            B3Vec3 wa=b3_add(bl[a].ang_vel,b3_mv(bl[a].inv_i,b3_v(fa[3],fa[4],fa[5])));
            B3Vec3 wb=b3_add(bl[b].ang_vel,b3_mv(bl[b].inv_i,b3_v(fb[3],fb[4],fb[5])));
            float bias=j->spring_softness.bias_rate*(j->cache_twist-j->target_angle);
            float reg=j->spring_softness.impulse_scale /
                (j->spring_softness.mass_scale*j->axial_mass);
            float g=b3_dot(b3_sub(wb,wa),j->rotation_axis)+bias+
                reg*(j->spring_impulse+sol.delta[k][5]);
            if(!isfinite(g)) return 0;
            // At upper bound g<=0; at lower bound g>=0.
            if(active[k]*g>1e-5f*(1+fabsf(bias))) { change=k;state=0;break; }
        }
        if(change<0) {
            for(int k=0;k<w->joint_count;k++) for(int q=0;q<B3_TR;q++)
                if(!isfinite(sol.delta[k][q])) return 0;
            B3_TREE_EVENT(0);b3_tree_apply(w,bl,jl,&sol);return 1;
        }
        active[change]=state;B3_TREE_EVENT(1);
        if(!b3_tree_factor_active(w,bl,jl,use_bias,f,active)) return 0;
    }
    return 0;
}
#endif
#if defined(B3_INTERLEAVE_CONTACTS) && !defined(B3_ABLATE_NO_CONTACT)
#if B3_ART_CONTACTS
#define B3_TREE_CONTACTS(artp) \
    b3_solve_contacts_gs_w((B3World*)(w), (artp), cl, w->contact_count, bl, \
        inv_h, w->contact_speed, use_bias)
#else
#define B3_TREE_CONTACTS(artp) \
    b3_solve_contacts_gs(cl,w->contact_count,bl,inv_h,w->contact_speed,use_bias)
#endif
#else
#define B3_TREE_CONTACTS(artp) ((void)0)
#endif

static B3_HD B3_INL int b3_tree_solve(const B3World* w,B3GsBody* bl,
        B3GsJoint* jl,B3GsContact* cl,float h,float inv_h,int use_bias) {
    B3TreeFactor f;
    if(!b3_tree_factor(w,bl,jl,use_bias,&f)) {
#if B3_TREE_SPRINGS
        B3_TREE_EVENT(2);
#endif
        return 0;
    }
#if B3_ART_CONTACTS
    B3Art art;
    B3Art* artp = b3_art_bind(&art, w) ? &art : 0;
#endif
    for(int iter=0;iter<B3_TREE_OUTER_ITERS;iter++) {
#if B3_ART_CONTACTS
        B3_TREE_CONTACTS(artp);
#else
        B3_TREE_CONTACTS(0);
#endif
#if B3_TREE_SPRINGS
        // Keep unilateral limit rows; the spring is handled by the tree solve.
        for(int i=0;i<w->joint_count;i++) {
            int bits=jl[i].bits;jl[i].bits &= ~B3_GS_SPRING;
            b3_solve_axial_gs(&jl[i],&bl[jl[i].body_a],&bl[jl[i].body_b],h,inv_h,use_bias);
            jl[i].bits=bits;
        }
        if(!b3_tree_spring_project(w,bl,jl,h,use_bias,&f)) {
            B3_TREE_EVENT(2);
            int iters=use_bias ? B3_JOINT_ITERS:B3_RELAX_ITERS;
            for(int k=0;k<iters;k++) {
#if B3_ART_CONTACTS
                B3_TREE_CONTACTS(artp);
#else
                B3_TREE_CONTACTS(0);
#endif
                for(int i=0;i<w->joint_count;i++) b3_solve_revolute_gs(&jl[i],
                    &bl[jl[i].body_a],&bl[jl[i].body_b],h,inv_h,use_bias);
            }
            return 1;
        }
#else
        for(int i=0;i<w->joint_count;i++) b3_solve_axial_gs(&jl[i],
            &bl[jl[i].body_a],&bl[jl[i].body_b],h,inv_h,use_bias);
        b3_tree_project(w,bl,jl,use_bias,&f);
#endif
    }
    return 1;
}
