// Contact churn and caller-reordered caches must preserve impulse matching.
#include <algorithm>
#include <vector>
#include <stdio.h>
#include "puffysics.cuh"
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while (0)

int main() {
    B3World w;
    b3_world_init(&w);
    B3BodyDef bd=b3_default_body();
    bd.position=b3_v(0,-0.5f,0);
    int ground=b3_create_body(&w,&bd);
    B3ShapeDef sd=b3_default_shape();
    sd.density=0;
    b3_create_box(&w,ground,b3_v(50,0.5f,50),&sd);
    b3_finalize_mass(&w,ground);
    for (int i=0;i<12;i++) {
        bd=b3_default_body(); bd.type=B3_DYNAMIC;
        bd.position=b3_v((i%4)*0.98f,0.5f+(i/4)*0.98f,0);
        int b=b3_create_body(&w,&bd);
        sd.density=1;
        b3_create_box(&w,b,b3_v(0.5f,0.5f,0.5f),&sd);
        b3_finalize_mass(&w,b);
    }
    int matched=0, fresh=0;
    for (int trial=0;trial<120;trial++) {
        b3_find_contacts(&w);
        for (int c=0;c<w.contact_count;c++) {
            B3Contact& q=w.contacts[c];
            q.friction_impulse={c+0.25f,c+0.5f};
            q.twist_impulse=c+0.75f;
            q.rolling_impulse=b3_v(c+1,c+2,c+3);
            for (int p=0;p<q.point_count;p++) q.points[p].normal_impulse=100*c+p+1;
        }
        if (trial%3==1) std::reverse(w.contacts,w.contacts+w.contact_count);
        if (trial%3==2 && w.contact_count>0 && w.contact_count<B3_MAX_CONTACTS) {
            w.contacts[w.contact_count]=w.contacts[0];
            w.contacts[w.contact_count].twist_impulse=999;
            w.contact_count++;
        }
        if (trial%10==0) w.contact_count=0;
        std::vector<B3Contact> old(w.contacts,w.contacts+w.contact_count);
        // Deterministic changes introduce and remove contacts at different keys.
        for (int i=0;i<12;i++) {
            B3Body& b=w.bodies[i+1];
            b.position=b3_v((i%4)*0.98f,0.5f+(i/4)*0.98f+((trial+i)%5==0 ? 4.0f:0),0);
            b.center=b.position;
        }
        b3_find_contacts(&w);
        for (int c=0;c<w.contact_count;c++) {
            B3Contact& q=w.contacts[c];
            B3Vec2 friction={0,0}; B3Vec3 rolling=b3_v(0,0,0);
            float twist=0, normal[B3_MAX_MANIFOLD]={0};
            bool found=false;
            for (const B3Contact& prev:old) {
                if (prev.shape_a!=q.shape_a || prev.shape_b!=q.shape_b) continue;
                found=true; friction=prev.friction_impulse; rolling=prev.rolling_impulse;
                twist=prev.twist_impulse;
                for (int p=0;p<q.point_count;p++) for (int r=0;r<prev.point_count;r++)
                    if (q.points[p].feature==prev.points[r].feature) normal[p]=prev.points[r].normal_impulse;
            }
            matched+=found; fresh+=!found;
            CHECK(q.friction_impulse.x==friction.x && q.friction_impulse.y==friction.y);
            CHECK(q.twist_impulse==twist);
            CHECK(q.rolling_impulse.x==rolling.x && q.rolling_impulse.y==rolling.y && q.rolling_impulse.z==rolling.z);
            for (int p=0;p<q.point_count;p++) CHECK(q.points[p].normal_impulse==normal[p]);
        }
    }
    CHECK(matched>0 && fresh>0);
    printf("warm cache passed: %d persistent, %d new; sorted, reversed, duplicate and empty caches\n",matched,fresh);
}
