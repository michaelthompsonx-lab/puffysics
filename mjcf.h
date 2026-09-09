// Minimal MJCF (MuJoCo XML) loader for Puffysics env sims.
// Host-only. Visuals are STL meshes; collision meshes become OBB boxes.
#pragma once

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "puffysics.cuh"
#include "stl.h"

#ifndef MJCF_MAX_BODIES
#define MJCF_MAX_BODIES B3_MAX_BODIES
#endif
#ifndef MJCF_MAX_JOINTS
#define MJCF_MAX_JOINTS B3_MAX_JOINTS
#endif
#ifndef MJCF_MAX_GEOMS
#define MJCF_MAX_GEOMS 256
#endif
#ifndef MJCF_MAX_MESHES
#define MJCF_MAX_MESHES 64
#endif
#ifndef MJCF_MAX_MATS
#define MJCF_MAX_MATS 64
#endif
#ifndef MJCF_MAX_CLASSES
#define MJCF_MAX_CLASSES 32
#endif
#ifndef MJCF_MAX_NAME
#define MJCF_MAX_NAME 64
#endif
#ifndef MJCF_MAX_PATH
#define MJCF_MAX_PATH 512
#endif
#ifndef MJCF_MAX_KEYS
#define MJCF_MAX_KEYS 8
#endif
#ifndef MJCF_MAX_QPOS
#define MJCF_MAX_QPOS 32
#endif

#define MJCF_GEOM_MESH 0
#define MJCF_GEOM_BOX 1
#define MJCF_GEOM_SPHERE 2
#define MJCF_GEOM_CAPSULE 3
#define MJCF_GEOM_PLANE 4

typedef struct MjcfClass {
    char name[MJCF_MAX_NAME];
    int parent;
    int geom_type;
    int geom_contype;
    int geom_conaffinity;
    int geom_group;
    float geom_friction;
    float joint_damping;
    float joint_armature;
    int has_geom_contype;
    int has_joint_damping;
} MjcfClass;

typedef struct MjcfMeshAsset {
    char name[MJCF_MAX_NAME];
    char file[MJCF_MAX_PATH];
    StlMesh stl;
    int loaded;
} MjcfMeshAsset;

typedef struct MjcfMaterial {
    char name[MJCF_MAX_NAME];
    float rgba[4];
} MjcfMaterial;

typedef struct MjcfGeom {
    char name[MJCF_MAX_NAME];
    char mesh[MJCF_MAX_NAME];
    char material[MJCF_MAX_NAME];
    char klass[MJCF_MAX_NAME];
    int body;
    int type;
    int mesh_id;
    int collide;
    int visual;
    B3Vec3 pos;
    B3Quat rot;
    B3Vec3 size;
    float radius;
    float rgba[4];
    int contype;
} MjcfGeom;

typedef struct MjcfJoint {
    char name[MJCF_MAX_NAME];
    int parent;
    int child;
    int type; /* 0 hinge */
    B3Vec3 axis_parent;
    B3Vec3 anchor_parent;
    B3Vec3 anchor_child;
    float lower;
    float upper;
    int has_limit;
    float damping;
    float armature;
} MjcfJoint;

typedef struct MjcfBody {
    char name[MJCF_MAX_NAME];
    int parent;
    int world_id;
    B3Vec3 rel_pos;
    B3Quat rel_rot;
    B3Vec3 world_pos;
    B3Quat world_rot;
    B3Vec3 com;
    B3Vec3 inertia;
    float mass;
    int has_inertial;
    int freejoint;
} MjcfBody;

typedef struct MjcfKey {
    char name[MJCF_MAX_NAME];
    float qpos[MJCF_MAX_QPOS];
    int nq;
} MjcfKey;

typedef struct MjcfModel {
    char path[MJCF_MAX_PATH];
    char dir[MJCF_MAX_PATH];
    char meshdir[MJCF_MAX_PATH];
    int z_up;
    int body_count;
    int joint_count;
    int geom_count;
    int mesh_count;
    int mat_count;
    int class_count;
    int key_count;
    int root;
    int has_floor;
    MjcfBody bodies[MJCF_MAX_BODIES];
    MjcfJoint joints[MJCF_MAX_JOINTS];
    MjcfGeom geoms[MJCF_MAX_GEOMS];
    MjcfMeshAsset meshes[MJCF_MAX_MESHES];
    MjcfMaterial mats[MJCF_MAX_MATS];
    MjcfClass classes[MJCF_MAX_CLASSES];
    MjcfKey keys[MJCF_MAX_KEYS];
    char error[256];
} MjcfModel;

typedef struct MjcfSpawn {
    int n_bodies;
    int n_joints;
    int n_shapes;
    int root;
    int ground;
    int body_map[MJCF_MAX_BODIES];
    int joint_map[MJCF_MAX_JOINTS];
} MjcfSpawn;

#ifdef RAYLIB_H
typedef struct MjcfRenderGeom {
    Model model;
    int loaded;
    int geom;
} MjcfRenderGeom;

typedef struct MjcfRenderer {
    MjcfRenderGeom geoms[MJCF_MAX_GEOMS];
    int count;
} MjcfRenderer;
#endif

static inline B3Vec3 mjcf_zup_vec(B3Vec3 p) {
    /* MuJoCo Z-up -> engine/Raylib Y-up: (x,y,z) -> (x,z,-y) */
    return b3_v(p.x, p.z, -p.y);
}

static inline B3Quat mjcf_zup_quat(B3Quat q) {
    B3Quat r = b3_q_axis_angle(b3_v(1.0f, 0.0f, 0.0f), -0.5f * B3_PI);
    return b3_qnorm(b3_qmul(r, b3_qmul(q, b3_qconj(r))));
}

static inline B3Quat mjcf_quat_wxyz(float w, float x, float y, float z) {
    return b3_qnorm(b3_q(x, y, z, w));
}

static inline void mjcf_dirname(const char* path, char* out, int n) {
    snprintf(out, (size_t)n, "%s", path);
    char* slash = strrchr(out, '/');
    if (slash) {
        *slash = 0;
    } else {
        snprintf(out, (size_t)n, ".");
    }
}

static inline void mjcf_join(char* out, int n, const char* a, const char* b) {
    if (b[0] == '/' || (b[0] && b[1] == ':')) {
        snprintf(out, (size_t)n, "%s", b);
        return;
    }
    if (a[0] == 0 || strcmp(a, ".") == 0) {
        snprintf(out, (size_t)n, "%s", b);
        return;
    }
    snprintf(out, (size_t)n, "%s/%s", a, b);
}

static inline char* mjcf_read_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) {
        return 0;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);
    if (sz <= 0) {
        fclose(f);
        return 0;
    }
    char* s = (char*)malloc((size_t)sz + 1);
    if (!s) {
        fclose(f);
        return 0;
    }
    size_t rd = fread(s, 1, (size_t)sz, f);
    fclose(f);
    s[rd] = 0;
    return s;
}

static inline void mjcf_strip_comments(char* s) {
    char* w = s;
    for (char* r = s; *r; ) {
        if (r[0] == '<' && r[1] == '!' && r[2] == '-' && r[3] == '-') {
            r += 4;
            while (*r && !(r[0] == '-' && r[1] == '-' && r[2] == '>')) {
                r++;
            }
            if (*r) {
                r += 3;
            }
            continue;
        }
        *w++ = *r++;
    }
    *w = 0;
}

static inline int mjcf_attr(const char* tag, const char* key, char* out, int n) {
    const char* p = tag;
    size_t klen = strlen(key);
    while (*p && *p != '>') {
        if ((p == tag || isspace((unsigned char)p[-1])) &&
                strncmp(p, key, klen) == 0 && p[klen] == '=') {
            p += klen + 1;
            char q = *p;
            if (q == '"' || q == '\'') {
                p++;
                int i = 0;
                while (*p && *p != q && i < n - 1) {
                    out[i++] = *p++;
                }
                out[i] = 0;
                return 1;
            }
        }
        p++;
    }
    if (n > 0) {
        out[0] = 0;
    }
    return 0;
}

static inline int mjcf_floats(const char* s, float* o, int maxn) {
    int n = 0;
    while (s && *s && n < maxn) {
        char* end = 0;
        o[n] = strtof(s, &end);
        if (end == s) {
            break;
        }
        n++;
        s = end;
    }
    return n;
}

static inline B3Vec3 mjcf_vec3_attr(const char* tag, const char* key,
        B3Vec3 fallback) {
    char buf[128];
    if (!mjcf_attr(tag, key, buf, (int)sizeof(buf))) {
        return fallback;
    }
    float v[3] = {0, 0, 0};
    mjcf_floats(buf, v, 3);
    return b3_v(v[0], v[1], v[2]);
}

static inline B3Quat mjcf_quat_attr(const char* tag, const char* key) {
    char buf[128];
    if (!mjcf_attr(tag, key, buf, (int)sizeof(buf))) {
        return b3_q_id();
    }
    float v[4] = {1, 0, 0, 0};
    mjcf_floats(buf, v, 4);
    return mjcf_quat_wxyz(v[0], v[1], v[2], v[3]);
}

static inline const char* mjcf_read_tag(const char* p, char* tag, int n) {
    int i = 0;
    while (*p && *p != '>' && i < n - 2) {
        tag[i++] = *p++;
    }
    if (*p == '>') {
        tag[i++] = '>';
        p++;
    }
    tag[i] = 0;
    return p;
}

static inline const char* mjcf_tag_name(const char* tag) {
    const char* p = tag;
    if (*p == '<') {
        p++;
    }
    if (*p == '/') {
        p++;
    }
    return p;
}

static inline int mjcf_tag_is(const char* tag, const char* name) {
    const char* p = mjcf_tag_name(tag);
    size_t n = strlen(name);
    return strncmp(p, name, n) == 0
        && (p[n] == 0 || p[n] == ' ' || p[n] == '\t' || p[n] == '/'
            || p[n] == '>');
}

static inline int mjcf_tag_close(const char* tag) {
    const char* p = tag;
    if (*p == '<') {
        p++;
    }
    return *p == '/';
}

static inline int mjcf_self_close(const char* tag) {
    size_t n = strlen(tag);
    return n >= 2 && tag[n - 2] == '/';
}

static inline int mjcf_find_class(const MjcfModel* m, const char* name) {
    if (!name || !name[0]) {
        return -1;
    }
    for (int i = 0; i < m->class_count; i++) {
        if (strcmp(m->classes[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static inline int mjcf_find_mesh(const MjcfModel* m, const char* name) {
    if (!name || !name[0]) {
        return -1;
    }
    for (int i = 0; i < m->mesh_count; i++) {
        if (strcmp(m->meshes[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static inline int mjcf_find_mat(const MjcfModel* m, const char* name) {
    if (!name || !name[0]) {
        return -1;
    }
    for (int i = 0; i < m->mat_count; i++) {
        if (strcmp(m->mats[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static inline int mjcf_find_key(const MjcfModel* m, const char* name) {
    if (!name || !name[0]) {
        return -1;
    }
    /* Last match: scene_walk.xml has a leftover STAND then STAND2. */
    int found = -1;
    for (int i = 0; i < m->key_count; i++) {
        if (strcmp(m->keys[i].name, name) == 0) {
            found = i;
        }
    }
    return found;
}

static inline int mjcf_geom_type_id(const char* s) {
    if (!s || !s[0]) {
        return MJCF_GEOM_MESH;
    }
    if (strcmp(s, "box") == 0) {
        return MJCF_GEOM_BOX;
    }
    if (strcmp(s, "sphere") == 0) {
        return MJCF_GEOM_SPHERE;
    }
    if (strcmp(s, "capsule") == 0) {
        return MJCF_GEOM_CAPSULE;
    }
    if (strcmp(s, "plane") == 0) {
        return MJCF_GEOM_PLANE;
    }
    return MJCF_GEOM_MESH;
}

static inline void mjcf_class_init(MjcfClass* c) {
    memset(c, 0, sizeof(*c));
    c->parent = -1;
    c->geom_type = MJCF_GEOM_MESH;
    c->geom_friction = 0.8f;
    c->geom_contype = 1;
    c->geom_conaffinity = 1;
}

static inline int mjcf_add_class(MjcfModel* m, const char* name, int parent) {
    if (m->class_count >= MJCF_MAX_CLASSES) {
        return -1;
    }
    int id = m->class_count++;
    MjcfClass* c = &m->classes[id];
    mjcf_class_init(c);
    snprintf(c->name, sizeof(c->name), "%s", name ? name : "");
    c->parent = parent;
    if (parent >= 0) {
        *c = m->classes[parent];
        snprintf(c->name, sizeof(c->name), "%s", name ? name : "");
        c->parent = parent;
    }
    return id;
}

static inline const MjcfClass* mjcf_class_of(const MjcfModel* m,
        const char* name) {
    int id = mjcf_find_class(m, name);
    return id >= 0 ? &m->classes[id] : NULL;
}

static inline int mjcf_geom_collides(const MjcfGeom* g) {
    if (strcmp(g->klass, "collision") == 0
            || strcmp(g->klass, "self_collision_only") == 0) {
        return 1;
    }
    return g->contype != 0 && strcmp(g->klass, "visual") != 0;
}

static inline int mjcf_geom_visual(const MjcfGeom* g) {
    if (g->type == MJCF_GEOM_PLANE) {
        return 0;
    }
    if (strcmp(g->klass, "visual") == 0) {
        return 1;
    }
    return g->contype == 0 && strcmp(g->klass, "collision") != 0
        && strcmp(g->klass, "self_collision_only") != 0;
}

static inline void mjcf_apply_class_geom(MjcfGeom* g, const MjcfClass* c) {
    if (!c) {
        return;
    }
    if (!g->klass[0]) {
        snprintf(g->klass, sizeof(g->klass), "%s", c->name);
    }
    if (c->has_geom_contype) {
        g->contype = c->geom_contype;
    }
}

static const char* mjcf_parse_xml(MjcfModel* m, const char* xml,
        const char* dir, int depth);

static inline const char* mjcf_skip_to(const char* p, const char* name) {
    char tag[2048];
    while (*p) {
        if (*p != '<') {
            p++;
            continue;
        }
        p = mjcf_read_tag(p, tag, (int)sizeof(tag));
        if (mjcf_tag_close(tag) && mjcf_tag_is(tag, name)) {
            return p;
        }
    }
    return p;
}

static inline void mjcf_parse_compiler(MjcfModel* m, const char* tag,
        const char* dir) {
    char buf[MJCF_MAX_PATH];
    if (mjcf_attr(tag, "meshdir", buf, (int)sizeof(buf))) {
        mjcf_join(m->meshdir, (int)sizeof(m->meshdir), dir, buf);
    }
}

static inline void mjcf_parse_mesh_asset(MjcfModel* m, const char* tag,
        const char* dir) {
    if (m->mesh_count >= MJCF_MAX_MESHES) {
        return;
    }
    MjcfMeshAsset* a = &m->meshes[m->mesh_count++];
    memset(a, 0, sizeof(*a));
    mjcf_attr(tag, "name", a->name, (int)sizeof(a->name));
    char file[MJCF_MAX_PATH];
    if (mjcf_attr(tag, "file", file, (int)sizeof(file))) {
        const char* base = m->meshdir[0] ? m->meshdir : dir;
        mjcf_join(a->file, (int)sizeof(a->file), base, file);
        if (!a->name[0]) {
            const char* slash = strrchr(file, '/');
            const char* basef = slash ? slash + 1 : file;
            snprintf(a->name, sizeof(a->name), "%s", basef);
            char* dot = strrchr(a->name, '.');
            if (dot) {
                *dot = 0;
            }
        }
    }
}

static inline void mjcf_parse_material(MjcfModel* m, const char* tag) {
    if (m->mat_count >= MJCF_MAX_MATS) {
        return;
    }
    MjcfMaterial* mat = &m->mats[m->mat_count++];
    memset(mat, 0, sizeof(*mat));
    mat->rgba[0] = mat->rgba[1] = mat->rgba[2] = 0.7f;
    mat->rgba[3] = 1.0f;
    mjcf_attr(tag, "name", mat->name, (int)sizeof(mat->name));
    char buf[64];
    if (mjcf_attr(tag, "rgba", buf, (int)sizeof(buf))) {
        mjcf_floats(buf, mat->rgba, 4);
    }
}

static inline void mjcf_parse_default_block(MjcfModel* m, const char** pp,
        int parent) {
    char tag[2048];
    const char* p = *pp;
    while (*p) {
        if (*p != '<') {
            p++;
            continue;
        }
        p = mjcf_read_tag(p, tag, (int)sizeof(tag));
        if (mjcf_tag_close(tag) && mjcf_tag_is(tag, "default")) {
            break;
        }
        if (mjcf_tag_is(tag, "default") && !mjcf_tag_close(tag)) {
            char name[MJCF_MAX_NAME];
            mjcf_attr(tag, "class", name, (int)sizeof(name));
            int id = mjcf_add_class(m, name, parent);
            if (!mjcf_self_close(tag)) {
                const char* inner = p;
                mjcf_parse_default_block(m, &inner, id);
                p = inner;
            }
            continue;
        }
        if (parent < 0) {
            continue;
        }
        MjcfClass* c = &m->classes[parent];
        if (mjcf_tag_is(tag, "geom")) {
            char buf[64];
            if (mjcf_attr(tag, "type", buf, (int)sizeof(buf))) {
                c->geom_type = mjcf_geom_type_id(buf);
            }
            if (mjcf_attr(tag, "contype", buf, (int)sizeof(buf))) {
                c->geom_contype = atoi(buf);
                c->has_geom_contype = 1;
            }
            if (mjcf_attr(tag, "conaffinity", buf, (int)sizeof(buf))) {
                c->geom_conaffinity = atoi(buf);
            }
            if (mjcf_attr(tag, "group", buf, (int)sizeof(buf))) {
                c->geom_group = atoi(buf);
            }
            if (mjcf_attr(tag, "friction", buf, (int)sizeof(buf))) {
                float f[3] = {0};
                mjcf_floats(buf, f, 3);
                c->geom_friction = f[0];
            }
        } else if (mjcf_tag_is(tag, "joint")) {
            char buf[64];
            if (mjcf_attr(tag, "damping", buf, (int)sizeof(buf))) {
                c->joint_damping = strtof(buf, NULL);
                c->has_joint_damping = 1;
            }
            if (mjcf_attr(tag, "armature", buf, (int)sizeof(buf))) {
                c->joint_armature = strtof(buf, NULL);
            }
        }
    }
    *pp = p;
}

static inline void mjcf_parse_key(MjcfModel* m, const char* tag) {
    if (m->key_count >= MJCF_MAX_KEYS) {
        return;
    }
    MjcfKey* k = &m->keys[m->key_count++];
    memset(k, 0, sizeof(*k));
    mjcf_attr(tag, "name", k->name, (int)sizeof(k->name));
    char buf[1024];
    if (mjcf_attr(tag, "qpos", buf, (int)sizeof(buf))) {
        k->nq = mjcf_floats(buf, k->qpos, MJCF_MAX_QPOS);
    }
}

static inline void mjcf_add_geom(MjcfModel* m, int body, const char* tag,
        const char* klass) {
    if (m->geom_count >= MJCF_MAX_GEOMS) {
        return;
    }
    MjcfGeom* g = &m->geoms[m->geom_count++];
    memset(g, 0, sizeof(*g));
    g->body = body;
    g->mesh_id = -1;
    g->rgba[0] = g->rgba[1] = g->rgba[2] = 0.7f;
    g->rgba[3] = 1.0f;
    g->contype = 1;
    g->rot = b3_q_id();
    mjcf_attr(tag, "name", g->name, (int)sizeof(g->name));
    mjcf_attr(tag, "mesh", g->mesh, (int)sizeof(g->mesh));
    mjcf_attr(tag, "material", g->material, (int)sizeof(g->material));
    char cls[MJCF_MAX_NAME];
    if (mjcf_attr(tag, "class", cls, (int)sizeof(cls))) {
        snprintf(g->klass, sizeof(g->klass), "%s", cls);
    } else if (klass && klass[0]) {
        snprintf(g->klass, sizeof(g->klass), "%s", klass);
    }
    const MjcfClass* c = mjcf_class_of(m, g->klass);
    mjcf_apply_class_geom(g, c);
    char buf[64];
    if (mjcf_attr(tag, "type", buf, (int)sizeof(buf))) {
        g->type = mjcf_geom_type_id(buf);
    } else if (c) {
        g->type = c->geom_type;
    } else if (g->mesh[0]) {
        g->type = MJCF_GEOM_MESH;
    }
    if (mjcf_attr(tag, "contype", buf, (int)sizeof(buf))) {
        g->contype = atoi(buf);
    }
    g->pos = mjcf_vec3_attr(tag, "pos", b3_v(0, 0, 0));
    g->rot = mjcf_quat_attr(tag, "quat");
    g->size = mjcf_vec3_attr(tag, "size", b3_v(0, 0, 0));
    if (g->type == MJCF_GEOM_SPHERE) {
        g->radius = g->size.x;
    } else if (g->type == MJCF_GEOM_CAPSULE) {
        g->radius = g->size.x;
    }
    if (mjcf_attr(tag, "rgba", buf, (int)sizeof(buf))) {
        mjcf_floats(buf, g->rgba, 4);
    }
    if (g->type == MJCF_GEOM_PLANE) {
        m->has_floor = 1;
        g->collide = 0;
        g->visual = 0;
        return;
    }
    g->collide = mjcf_geom_collides(g);
    g->visual = mjcf_geom_visual(g);
}

static inline void mjcf_add_joint(MjcfModel* m, int parent, int child,
        const char* tag, const char* klass) {
    if (m->joint_count >= MJCF_MAX_JOINTS) {
        return;
    }
    MjcfJoint* j = &m->joints[m->joint_count++];
    memset(j, 0, sizeof(*j));
    j->parent = parent;
    j->child = child;
    j->axis_parent = b3_v(0, 0, 1);
    mjcf_attr(tag, "name", j->name, (int)sizeof(j->name));
    j->axis_parent = mjcf_vec3_attr(tag, "axis", b3_v(0, 0, 1));
    /* MuJoCo joint axis is child-local. Puffysics revolute axis is
     * parent-local. neck quat makes child +Z lateral (pitch); treating
     * XML (0,0,1) as parent-local turns it into world-up yaw. */
    j->axis_parent = b3_rotate(m->bodies[child].rel_rot, j->axis_parent);
    char buf[128];
    if (mjcf_attr(tag, "range", buf, (int)sizeof(buf))) {
        float r[2] = {0, 0};
        if (mjcf_floats(buf, r, 2) >= 2) {
            j->lower = r[0];
            j->upper = r[1];
            j->has_limit = 1;
        }
    }
    char cls[MJCF_MAX_NAME];
    const char* use = klass;
    if (mjcf_attr(tag, "class", cls, (int)sizeof(cls))) {
        use = cls;
    }
    const MjcfClass* c = mjcf_class_of(m, use);
    if (c && c->has_joint_damping) {
        j->damping = c->joint_damping;
    }
    if (c && c->joint_armature > 0.0f) {
        j->armature = c->joint_armature;
    }
    if (mjcf_attr(tag, "armature", buf, (int)sizeof(buf))) {
        j->armature = strtof(buf, NULL);
    }
    /* Joint sits at the child origin in the parent frame. */
    j->anchor_parent = m->bodies[child].rel_pos;
    j->anchor_child = b3_v(0, 0, 0);
}

static inline void mjcf_parse_inertial(MjcfBody* b, const char* tag) {
    b->has_inertial = 1;
    b->com = mjcf_vec3_attr(tag, "pos", b3_v(0, 0, 0));
    char buf[128];
    if (mjcf_attr(tag, "mass", buf, (int)sizeof(buf))) {
        b->mass = strtof(buf, NULL);
    }
    if (mjcf_attr(tag, "diaginertia", buf, (int)sizeof(buf))) {
        float v[3] = {0, 0, 0};
        mjcf_floats(buf, v, 3);
        b->inertia = b3_v(v[0], v[1], v[2]);
    } else if (mjcf_attr(tag, "fullinertia", buf, (int)sizeof(buf))) {
        float v[6] = {0};
        mjcf_floats(buf, v, 6);
        b->inertia = b3_v(v[0], v[1], v[2]);
    }
}

static const char* mjcf_parse_body(MjcfModel* m, const char* p, int parent,
        const char* inherited_class);

static const char* mjcf_parse_body(MjcfModel* m, const char* p, int parent,
        const char* inherited_class) {
    char tag[2048];
    /* Caller has already consumed the opening <body ...> into the last tag
     * via the worldbody/body walker; this function is used after that. */
    (void)inherited_class;
    while (*p) {
        if (*p != '<') {
            p++;
            continue;
        }
        p = mjcf_read_tag(p, tag, (int)sizeof(tag));
        if (mjcf_tag_close(tag) && mjcf_tag_is(tag, "body")) {
            return p;
        }
        if (mjcf_tag_is(tag, "inertial")) {
            if (parent >= 0) {
                mjcf_parse_inertial(&m->bodies[parent], tag);
            }
            continue;
        }
        if (mjcf_tag_is(tag, "freejoint")) {
            if (parent >= 0) {
                m->bodies[parent].freejoint = 1;
            }
            continue;
        }
        if (mjcf_tag_is(tag, "joint") && !mjcf_tag_close(tag)) {
            if (parent >= 0 && m->bodies[parent].parent >= 0) {
                mjcf_add_joint(m, m->bodies[parent].parent, parent, tag,
                    inherited_class);
            }
            continue;
        }
        if (mjcf_tag_is(tag, "geom") && !mjcf_tag_close(tag)) {
            if (parent >= 0) {
                mjcf_add_geom(m, parent, tag, inherited_class);
            }
            continue;
        }
        if (mjcf_tag_is(tag, "body") && !mjcf_tag_close(tag)) {
            if (m->body_count >= MJCF_MAX_BODIES) {
                if (!mjcf_self_close(tag)) {
                    p = mjcf_skip_to(p, "body");
                }
                continue;
            }
            int id = m->body_count++;
            MjcfBody* b = &m->bodies[id];
            memset(b, 0, sizeof(*b));
            b->parent = m->body_count > 1 ? parent : -1;
            /* parent of this new body is the body we were inside */
            b->parent = parent;
            b->world_id = -1;
            b->rel_rot = b3_q_id();
            b->world_rot = b3_q_id();
            mjcf_attr(tag, "name", b->name, (int)sizeof(b->name));
            b->rel_pos = mjcf_vec3_attr(tag, "pos", b3_v(0, 0, 0));
            b->rel_rot = mjcf_quat_attr(tag, "quat");
            char childclass[MJCF_MAX_NAME];
            const char* cc = inherited_class;
            if (mjcf_attr(tag, "childclass", childclass,
                    (int)sizeof(childclass))) {
                cc = childclass;
            }
            if (parent >= 0) {
                const MjcfBody* pa = &m->bodies[parent];
                b->world_rot = b3_qnorm(b3_qmul(pa->world_rot, b->rel_rot));
                b->world_pos = b3_add(pa->world_pos,
                    b3_rotate(pa->world_rot, b->rel_pos));
            } else {
                b->world_pos = b->rel_pos;
                b->world_rot = b->rel_rot;
            }
            if (!mjcf_self_close(tag)) {
                p = mjcf_parse_body(m, p, id, cc);
            }
            continue;
        }
        if (mjcf_tag_is(tag, "site") && !mjcf_self_close(tag)
                && !mjcf_tag_close(tag)) {
            p = mjcf_skip_to(p, "site");
        }
    }
    return p;
}

static const char* mjcf_parse_worldbody(MjcfModel* m, const char* p,
        const char* dir) {
    char tag[2048];
    (void)dir;
    while (*p) {
        if (*p != '<') {
            p++;
            continue;
        }
        p = mjcf_read_tag(p, tag, (int)sizeof(tag));
        if (mjcf_tag_close(tag) && mjcf_tag_is(tag, "worldbody")) {
            return p;
        }
        if (mjcf_tag_is(tag, "geom") && !mjcf_tag_close(tag)) {
            mjcf_add_geom(m, -1, tag, NULL);
            continue;
        }
        if (mjcf_tag_is(tag, "body") && !mjcf_tag_close(tag)) {
            if (m->body_count >= MJCF_MAX_BODIES) {
                if (!mjcf_self_close(tag)) {
                    p = mjcf_skip_to(p, "body");
                }
                continue;
            }
            int id = m->body_count++;
            if (m->root < 0) {
                m->root = id;
            }
            MjcfBody* b = &m->bodies[id];
            memset(b, 0, sizeof(*b));
            b->parent = -1;
            b->world_id = -1;
            mjcf_attr(tag, "name", b->name, (int)sizeof(b->name));
            b->rel_pos = mjcf_vec3_attr(tag, "pos", b3_v(0, 0, 0));
            b->rel_rot = mjcf_quat_attr(tag, "quat");
            b->world_pos = b->rel_pos;
            b->world_rot = b->rel_rot;
            char childclass[MJCF_MAX_NAME];
            const char* cc = NULL;
            if (mjcf_attr(tag, "childclass", childclass,
                    (int)sizeof(childclass))) {
                cc = childclass;
            }
            if (!mjcf_self_close(tag)) {
                p = mjcf_parse_body(m, p, id, cc);
            }
            continue;
        }
        if ((mjcf_tag_is(tag, "light") || mjcf_tag_is(tag, "camera")
                    || mjcf_tag_is(tag, "site"))
                && !mjcf_self_close(tag) && !mjcf_tag_close(tag)) {
            /* skip unknown containers with the same name */
        }
    }
    return p;
}

static const char* mjcf_parse_xml(MjcfModel* m, const char* xml,
        const char* dir, int depth) {
    if (depth > 8) {
        return xml;
    }
    char tag[2048];
    const char* p = xml;
    while (*p) {
        if (*p != '<') {
            p++;
            continue;
        }
        p = mjcf_read_tag(p, tag, (int)sizeof(tag));
        if (mjcf_tag_is(tag, "include")) {
            char file[MJCF_MAX_PATH];
            char path[MJCF_MAX_PATH];
            if (mjcf_attr(tag, "file", file, (int)sizeof(file))) {
                mjcf_join(path, (int)sizeof(path), dir, file);
                char* inc = mjcf_read_file(path);
                if (inc) {
                    mjcf_strip_comments(inc);
                    char idir[MJCF_MAX_PATH];
                    mjcf_dirname(path, idir, (int)sizeof(idir));
                    mjcf_parse_xml(m, inc, idir, depth + 1);
                    free(inc);
                }
            }
            continue;
        }
        if (mjcf_tag_is(tag, "compiler")) {
            mjcf_parse_compiler(m, tag, dir);
            continue;
        }
        if (mjcf_tag_is(tag, "default") && !mjcf_tag_close(tag)) {
            char name[MJCF_MAX_NAME];
            int parent = -1;
            if (mjcf_attr(tag, "class", name, (int)sizeof(name))) {
                parent = mjcf_add_class(m, name, -1);
            }
            if (!mjcf_self_close(tag)) {
                mjcf_parse_default_block(m, &p, parent);
            }
            continue;
        }
        if (mjcf_tag_is(tag, "mesh")) {
            mjcf_parse_mesh_asset(m, tag, dir);
            continue;
        }
        if (mjcf_tag_is(tag, "material")) {
            mjcf_parse_material(m, tag);
            continue;
        }
        if (mjcf_tag_is(tag, "worldbody") && !mjcf_tag_close(tag)) {
            p = mjcf_parse_worldbody(m, p, dir);
            continue;
        }
        if (mjcf_tag_is(tag, "key") && !mjcf_tag_close(tag)) {
            mjcf_parse_key(m, tag);
            continue;
        }
        if (mjcf_tag_is(tag, "sensor") && !mjcf_self_close(tag)
                && !mjcf_tag_close(tag)) {
            p = mjcf_skip_to(p, "sensor");
            continue;
        }
        if (mjcf_tag_is(tag, "actuator") && !mjcf_self_close(tag)
                && !mjcf_tag_close(tag)) {
            p = mjcf_skip_to(p, "actuator");
            continue;
        }
        if (mjcf_tag_is(tag, "contact") && !mjcf_self_close(tag)
                && !mjcf_tag_close(tag)) {
            p = mjcf_skip_to(p, "contact");
            continue;
        }
        if (mjcf_tag_is(tag, "tendon") && !mjcf_self_close(tag)
                && !mjcf_tag_close(tag)) {
            p = mjcf_skip_to(p, "tendon");
            continue;
        }
        if (mjcf_tag_is(tag, "equality") && !mjcf_self_close(tag)
                && !mjcf_tag_close(tag)) {
            p = mjcf_skip_to(p, "equality");
            continue;
        }
    }
    return p;
}

static inline void mjcf_apply_zup(MjcfModel* m) {
    if (!m->z_up) {
        return;
    }
    for (int i = 0; i < m->body_count; i++) {
        MjcfBody* b = &m->bodies[i];
        b->rel_pos = mjcf_zup_vec(b->rel_pos);
        b->rel_rot = mjcf_zup_quat(b->rel_rot);
        b->world_pos = mjcf_zup_vec(b->world_pos);
        b->world_rot = mjcf_zup_quat(b->world_rot);
        b->com = mjcf_zup_vec(b->com);
        b->inertia = b3_v(b->inertia.x, b->inertia.z, b->inertia.y);
    }
    for (int i = 0; i < m->joint_count; i++) {
        MjcfJoint* j = &m->joints[i];
        j->axis_parent = mjcf_zup_vec(j->axis_parent);
        j->anchor_parent = mjcf_zup_vec(j->anchor_parent);
        j->anchor_child = mjcf_zup_vec(j->anchor_child);
    }
    for (int i = 0; i < m->geom_count; i++) {
        MjcfGeom* g = &m->geoms[i];
        g->pos = mjcf_zup_vec(g->pos);
        g->rot = mjcf_zup_quat(g->rot);
        if (g->type == MJCF_GEOM_BOX) {
            g->size = b3_v(g->size.x, g->size.z, g->size.y);
        }
    }
    for (int i = 0; i < m->key_count; i++) {
        MjcfKey* k = &m->keys[i];
        if (k->nq >= 7) {
            B3Vec3 p = mjcf_zup_vec(b3_v(k->qpos[0], k->qpos[1], k->qpos[2]));
            B3Quat q = mjcf_zup_quat(mjcf_quat_wxyz(
                k->qpos[3], k->qpos[4], k->qpos[5], k->qpos[6]));
            k->qpos[0] = p.x;
            k->qpos[1] = p.y;
            k->qpos[2] = p.z;
            k->qpos[3] = q.s;
            k->qpos[4] = q.v.x;
            k->qpos[5] = q.v.y;
            k->qpos[6] = q.v.z;
        }
    }
    for (int i = 0; i < m->mesh_count; i++) {
        if (m->meshes[i].loaded) {
            stl_apply_zup(&m->meshes[i].stl);
        }
    }
}

static inline void mjcf_load_meshes(MjcfModel* m) {
    for (int i = 0; i < m->mesh_count; i++) {
        MjcfMeshAsset* a = &m->meshes[i];
        if (!a->file[0]) {
            continue;
        }
        a->loaded = stl_load(&a->stl, a->file);
    }
    for (int i = 0; i < m->geom_count; i++) {
        MjcfGeom* g = &m->geoms[i];
        if (g->mesh[0]) {
            g->mesh_id = mjcf_find_mesh(m, g->mesh);
        }
        int mi = mjcf_find_mat(m, g->material);
        if (mi >= 0) {
            memcpy(g->rgba, m->mats[mi].rgba, sizeof(g->rgba));
        }
    }
}

static inline void mjcf_free(MjcfModel* m) {
    for (int i = 0; i < m->mesh_count; i++) {
        if (m->meshes[i].loaded) {
            stl_free(&m->meshes[i].stl);
            m->meshes[i].loaded = 0;
        }
    }
}

static inline int mjcf_load(MjcfModel* m, const char* path, int z_up) {
    memset(m, 0, sizeof(*m));
    m->root = -1;
    m->z_up = z_up;
    snprintf(m->path, sizeof(m->path), "%s", path);
    mjcf_dirname(path, m->dir, (int)sizeof(m->dir));
    snprintf(m->meshdir, sizeof(m->meshdir), "%s", m->dir);
    char* xml = mjcf_read_file(path);
    if (!xml) {
        snprintf(m->error, sizeof(m->error), "cannot open %s", path);
        return 0;
    }
    mjcf_strip_comments(xml);
    mjcf_parse_xml(m, xml, m->dir, 0);
    free(xml);
    if (m->body_count < 1) {
        snprintf(m->error, sizeof(m->error), "no bodies in %s", path);
        return 0;
    }
    if (m->root < 0) {
        m->root = 0;
    }
    mjcf_load_meshes(m);
    mjcf_apply_zup(m);
    return 1;
}

static inline void mjcf_mesh_obb(const StlMesh* stl, B3Vec3* center,
        B3Vec3* half) {
    B3Vec3 mn = b3_v(stl->min[0], stl->min[1], stl->min[2]);
    B3Vec3 mx = b3_v(stl->max[0], stl->max[1], stl->max[2]);
    if (stl->positions == NULL || stl->tri_count < 1) {
        *center = b3_v(0, 0, 0);
        *half = b3_v(0.005f, 0.005f, 0.005f);
        return;
    }
    *center = b3_mul(b3_add(mn, mx), 0.5f);
    *half = b3_mul(b3_sub(mx, mn), 0.5f);
    if (half->x < 1.0e-4f) {
        half->x = 1.0e-4f;
    }
    if (half->y < 1.0e-4f) {
        half->y = 1.0e-4f;
    }
    if (half->z < 1.0e-4f) {
        half->z = 1.0e-4f;
    }
}

/* Walk MJCF: only *_collision soles touch terrain (mjlab condim=3, mu=1).
 * self_collision_only is contype=2/conaffinity=2 — robot vs robot, not floor. */
#define MJCF_COL_GROUND 1ull
#define MJCF_COL_SELF   2ull
#define MJCF_COL_FEET   4ull

static inline void mjcf_shape_layers(B3ShapeDef* sd, const MjcfGeom* g) {
    int self_only = (strcmp(g->klass, "self_collision_only") == 0);
    int foot = (strcmp(g->klass, "collision") == 0)
        || (strstr(g->name, "foot") != NULL);
    if (self_only || !foot) {
        sd->category = MJCF_COL_SELF;
        sd->mask = MJCF_COL_SELF;
    } else {
        sd->category = MJCF_COL_FEET;
        sd->mask = MJCF_COL_GROUND;
        sd->friction = 1.0f;
    }
}

static inline int mjcf_spawn_ground(B3World* w) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_STATIC;
    bd.position = b3_v(0.0f, -0.05f, 0.0f);
    int id = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 1.0f;
    sd.restitution = 0.0f;
    sd.category = MJCF_COL_GROUND;
    sd.mask = MJCF_COL_FEET;
    b3_create_box(w, id, b3_v(50.0f, 0.05f, 50.0f), &sd);
    return id;
}

/* Puffysics stores diagonal local I. Axis-aligned add misses hinges whose
 * axis is not a principal axis (knees stayed at Iax~1e-5). Isotropic
 * add on both bodies gives reduced I ~ armature/2 for every hinge. */
static inline void mjcf_add_body_armature(B3Body* b, float armature) {
    if (b->type != B3_DYNAMIC || armature <= 0.0f) {
        return;
    }
    B3Vec3 I = b3_v(
        b->inv_inertia.x > 0.0f ? 1.0f / b->inv_inertia.x : 0.0f,
        b->inv_inertia.y > 0.0f ? 1.0f / b->inv_inertia.y : 0.0f,
        b->inv_inertia.z > 0.0f ? 1.0f / b->inv_inertia.z : 0.0f);
    I.x += armature;
    I.y += armature;
    I.z += armature;
    b->inv_inertia = b3_v(
        I.x > 0.0f ? 1.0f / I.x : 0.0f,
        I.y > 0.0f ? 1.0f / I.y : 0.0f,
        I.z > 0.0f ? 1.0f / I.z : 0.0f);
    b->inv_i_world = b3_world_inv_i(b->rotation, b->inv_inertia);
}

static inline void mjcf_add_armature(B3World* w, int jid, float armature) {
    B3Joint* j = &w->joints[jid];
    mjcf_add_body_armature(&w->bodies[j->body_a], armature);
    mjcf_add_body_armature(&w->bodies[j->body_b], armature);
}

static inline int mjcf_spawn(const MjcfModel* m, B3World* w, MjcfSpawn* s,
        int add_ground) {
    memset(s, 0, sizeof(*s));
    s->root = -1;
    s->ground = -1;
    for (int i = 0; i < MJCF_MAX_BODIES; i++) {
        s->body_map[i] = -1;
    }
    for (int i = 0; i < MJCF_MAX_JOINTS; i++) {
        s->joint_map[i] = -1;
    }
    if (add_ground || m->has_floor) {
        s->ground = mjcf_spawn_ground(w);
    }
    for (int i = 0; i < m->body_count; i++) {
        const MjcfBody* mb = &m->bodies[i];
        B3BodyDef bd = b3_default_body();
        bd.type = B3_DYNAMIC;
        bd.position = mb->world_pos;
        bd.rotation = mb->world_rot;
        int id = b3_create_body(w, &bd);
        s->body_map[i] = id;
        if (i == m->root) {
            s->root = id;
        }
        if (mb->has_inertial && mb->mass > 0.0f) {
            b3_set_inertial(w, id, mb->mass, mb->com, mb->inertia);
        }
    }
    s->n_bodies = m->body_count;

    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.8f;
    sd.restitution = 0.0f;
    for (int i = 0; i < m->geom_count; i++) {
        const MjcfGeom* g = &m->geoms[i];
        if (!g->collide || g->body < 0) {
            continue;
        }
        int bid = s->body_map[g->body];
        if (bid < 0) {
            continue;
        }
        B3ShapeDef gsd = sd;
        mjcf_shape_layers(&gsd, g);
        if (g->type == MJCF_GEOM_BOX) {
            b3_create_box_local(w, bid, g->size, g->pos, g->rot, &gsd);
        } else if (g->type == MJCF_GEOM_SPHERE) {
            b3_create_sphere(w, bid, g->pos, g->radius > 0 ? g->radius : 0.01f,
                &gsd);
        } else if (g->type == MJCF_GEOM_CAPSULE) {
            float half = g->size.y > 0 ? g->size.y : 0.01f;
            b3_add_shape(w, bid, B3_CAPSULE, g->pos, g->rot,
                g->radius > 0 ? g->radius : 0.01f, b3_v(0, half, 0), &gsd);
        } else if (g->mesh_id >= 0 && m->meshes[g->mesh_id].loaded) {
            B3Vec3 center, half;
            mjcf_mesh_obb(&m->meshes[g->mesh_id].stl, &center, &half);
            B3Vec3 lp = b3_add(g->pos, b3_rotate(g->rot, center));
            b3_create_box_local(w, bid, half, lp, g->rot, &gsd);
        }
    }
    s->n_shapes = w->shape_count;

    for (int i = 0; i < m->joint_count; i++) {
        const MjcfJoint* mj = &m->joints[i];
        int pa = s->body_map[mj->parent];
        int pb = s->body_map[mj->child];
        int jid = b3_create_revolute(w, pa, pb,
            mj->anchor_parent, mj->anchor_child, mj->axis_parent);
        s->joint_map[i] = jid;
        if (mj->has_limit) {
            b3_joint_enable_limit(w, jid, 1);
            b3_joint_set_limits(w, jid, mj->lower, mj->upper);
        }
        if (mj->armature > 0.0f) {
            mjcf_add_armature(w, jid, mj->armature);
        }
    }
    s->n_joints = m->joint_count;
    return s->n_bodies > 0;
}

static inline void mjcf_sync_body(B3World* w, int id, B3Vec3 pos, B3Quat rot) {
    B3Body* b = &w->bodies[id];
    b->position = pos;
    b->rotation = b3_qnorm(rot);
    b->center = b3_xf_point(b->position, b->rotation, b->local_center);
    b->lin_vel = b3_v(0, 0, 0);
    b->ang_vel = b3_v(0, 0, 0);
    b->delta_pos = b3_v(0, 0, 0);
    b->delta_rot = b3_q_id();
}

static inline void mjcf_rotate_subtree(const MjcfModel* m, B3World* w,
        const MjcfSpawn* s, int body, B3Vec3 origin, B3Vec3 axis, float ang) {
    B3Quat q = b3_q_axis_angle(axis, ang);
    for (int i = 0; i < m->body_count; i++) {
        int cur = i;
        int hit = 0;
        while (cur >= 0) {
            if (cur == body) {
                hit = 1;
                break;
            }
            cur = m->bodies[cur].parent;
        }
        if (!hit) {
            continue;
        }
        int id = s->body_map[i];
        B3Body* b = &w->bodies[id];
        B3Vec3 p = b3_add(origin, b3_rotate(q, b3_sub(b->position, origin)));
        B3Quat r = b3_qnorm(b3_qmul(q, b->rotation));
        mjcf_sync_body(w, id, p, r);
    }
}

static inline void mjcf_set_joint_angles(B3World* w, const MjcfModel* m,
        const MjcfSpawn* s, const float* q) {
    for (int i = 0; i < m->body_count; i++) {
        mjcf_sync_body(w, s->body_map[i],
            m->bodies[i].world_pos, m->bodies[i].world_rot);
    }
    for (int i = 0; i < m->joint_count; i++) {
        float ang = q ? q[i] : 0.0f;
        if (fabsf(ang) < 1.0e-8f) {
            continue;
        }
        const MjcfJoint* j = &m->joints[i];
        int pa = s->body_map[j->parent];
        const B3Body* ba = &w->bodies[pa];
        B3Vec3 origin = b3_xf_point(ba->position, ba->rotation, j->anchor_parent);
        B3Vec3 axis = b3_rotate(ba->rotation, j->axis_parent);
        float al = b3_len(axis);
        if (al < 1.0e-8f) {
            continue;
        }
        axis = b3_mul(axis, 1.0f / al);
        mjcf_rotate_subtree(m, w, s, j->child, origin, axis, ang);
    }
}

static inline int mjcf_apply_key(B3World* w, const MjcfModel* m,
        const MjcfSpawn* s, const char* name) {
    int ki = mjcf_find_key(m, name);
    if (ki < 0) {
        return 0;
    }
    const MjcfKey* k = &m->keys[ki];
    if (k->nq >= 7) {
        int root = s->root >= 0 ? s->root : s->body_map[m->root];
        B3Vec3 p = b3_v(k->qpos[0], k->qpos[1], k->qpos[2]);
        B3Quat q = mjcf_quat_wxyz(k->qpos[3], k->qpos[4], k->qpos[5], k->qpos[6]);
        /* Temporarily write root bind, then set_joint_angles overwrites from
         * model bind and we re-apply root translation/rotation after. */
        mjcf_set_joint_angles(w, m, s, k->qpos + 7);
        B3Vec3 bind = m->bodies[m->root].world_pos;
        B3Quat bind_q = m->bodies[m->root].world_rot;
        B3Quat dq = b3_qnorm(b3_qmul(q, b3_qconj(bind_q)));
        B3Vec3 dp = b3_sub(p, bind);
        for (int i = 0; i < m->body_count; i++) {
            int id = s->body_map[i];
            B3Body* b = &w->bodies[id];
            B3Vec3 np = b3_add(p, b3_rotate(dq, b3_sub(b->position, bind)));
            B3Quat nr = b3_qnorm(b3_qmul(dq, b->rotation));
            (void)dp;
            mjcf_sync_body(w, id, np, nr);
        }
        (void)root;
    } else {
        mjcf_set_joint_angles(w, m, s, k->qpos);
    }
    return 1;
}

static inline void mjcf_hold_pose(B3World* w, const MjcfSpawn* s, int n,
        float hertz, float damp, float torque) {
    int nj = n < s->n_joints ? n : s->n_joints;
    for (int i = 0; i < nj; i++) {
        int j = s->joint_map[i];
        if (j < 0) {
            continue;
        }
        float a = b3_joint_angle(w, j);
        b3_joint_enable_spring(w, j, 1);
        b3_joint_set_spring(w, j, a, hertz, damp);
        b3_joint_enable_motor(w, j, 1);
        b3_joint_set_motor(w, j, 0.0f, torque);
    }
}

#ifdef RAYLIB_H
static inline float mjcf_clip01(float x) {
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

static inline Color mjcf_color(const float rgba[4]) {
    Color c;
    c.r = (unsigned char)(mjcf_clip01(rgba[0]) * 255.0f);
    c.g = (unsigned char)(mjcf_clip01(rgba[1]) * 255.0f);
    c.b = (unsigned char)(mjcf_clip01(rgba[2]) * 255.0f);
    c.a = (unsigned char)(mjcf_clip01(rgba[3]) * 255.0f);
    return c;
}

static inline Quaternion mjcf_rq(B3Quat q) {
    Quaternion r;
    r.x = q.v.x;
    r.y = q.v.y;
    r.z = q.v.z;
    r.w = q.s;
    return r;
}

static inline Vector3 mjcf_rv(B3Vec3 v) {
    return (Vector3){v.x, v.y, v.z};
}

static inline void mjcf_renderer_load(MjcfRenderer* r, const MjcfModel* m) {
    memset(r, 0, sizeof(*r));
    for (int i = 0; i < m->geom_count && r->count < MJCF_MAX_GEOMS; i++) {
        const MjcfGeom* g = &m->geoms[i];
        if (!g->visual || g->mesh_id < 0) {
            continue;
        }
        const MjcfMeshAsset* a = &m->meshes[g->mesh_id];
        if (!a->loaded) {
            continue;
        }
        MjcfRenderGeom* rg = &r->geoms[r->count++];
        rg->geom = i;
        Color tint = mjcf_color(g->rgba);
        Mesh mesh = stl_to_raylib_mesh(&a->stl);
        if (mesh.vertexCount <= 0) {
            rg->loaded = 0;
            continue;
        }
        rg->model = LoadModelFromMesh(mesh);
        rg->loaded = rg->model.meshCount > 0;
        if (rg->loaded && rg->model.materialCount > 0) {
            rg->model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = tint;
        }
    }
}

static inline void mjcf_renderer_free(MjcfRenderer* r) {
    for (int i = 0; i < r->count; i++) {
        if (r->geoms[i].loaded) {
            UnloadModel(r->geoms[i].model);
            r->geoms[i].loaded = 0;
        }
    }
    r->count = 0;
}

static inline void mjcf_renderer_draw(const MjcfRenderer* r,
        const MjcfModel* m, const B3World* w, const MjcfSpawn* s) {
    for (int i = 0; i < r->count; i++) {
        const MjcfRenderGeom* rg = &r->geoms[i];
        if (!rg->loaded) {
            continue;
        }
        const MjcfGeom* g = &m->geoms[rg->geom];
        if (g->body < 0) {
            continue;
        }
        int bid = s->body_map[g->body];
        const B3Body* b = &w->bodies[bid];
        B3Vec3 p = b3_xf_point(b->position, b->rotation, g->pos);
        B3Quat q = b3_qmul(b->rotation, g->rot);
        DrawModelEx(rg->model, mjcf_rv(p), (Vector3){q.v.x, q.v.y, q.v.z},
            2.0f * acosf(b3_clamp(q.s, -1.0f, 1.0f)) * (180.0f / B3_PI),
            (Vector3){1, 1, 1}, mjcf_color(g->rgba));
    }
}
#endif
