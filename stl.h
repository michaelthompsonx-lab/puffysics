// Binary + ASCII STL loader. Raylib has no STL reader.
// Host-only. Optional Raylib Mesh upload when raylib.h is included first.
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef STL_MAX_PATH
#define STL_MAX_PATH 512
#endif

typedef struct StlMesh {
    char path[STL_MAX_PATH];
    int tri_count;
    float* positions; /* 9 floats per triangle: p0,p1,p2 */
    float* normals;   /* 3 floats per triangle */
    float min[3];
    float max[3];
} StlMesh;

static inline void stl_free(StlMesh* m) {
    free(m->positions);
    free(m->normals);
    m->positions = 0;
    m->normals = 0;
    m->tri_count = 0;
}

static inline void stl_grow_aabb(StlMesh* m, float x, float y, float z) {
    if (x < m->min[0]) {
        m->min[0] = x;
    }
    if (y < m->min[1]) {
        m->min[1] = y;
    }
    if (z < m->min[2]) {
        m->min[2] = z;
    }
    if (x > m->max[0]) {
        m->max[0] = x;
    }
    if (y > m->max[1]) {
        m->max[1] = y;
    }
    if (z > m->max[2]) {
        m->max[2] = z;
    }
}

/* MuJoCo Z-up -> engine/Raylib Y-up: (x,y,z) -> (x,z,-y). Same as mjcf_zup_vec. */
static inline void stl_apply_zup(StlMesh* m) {
    if (!m->positions || m->tri_count < 1) {
        return;
    }
    for (int i = 0; i < m->tri_count; i++) {
        if (m->normals) {
            float ny = m->normals[i * 3 + 1];
            m->normals[i * 3 + 1] = m->normals[i * 3 + 2];
            m->normals[i * 3 + 2] = -ny;
        }
        for (int v = 0; v < 3; v++) {
            float* p = m->positions + i * 9 + v * 3;
            float y = p[1];
            p[1] = p[2];
            p[2] = -y;
        }
    }
    m->min[0] = m->min[1] = m->min[2] = 1.0e30f;
    m->max[0] = m->max[1] = m->max[2] = -1.0e30f;
    int nv = m->tri_count * 3;
    for (int i = 0; i < nv; i++) {
        stl_grow_aabb(m, m->positions[i * 3 + 0],
            m->positions[i * 3 + 1], m->positions[i * 3 + 2]);
    }
}

static inline uint32_t stl_u32le(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline float stl_f32le(const unsigned char* p) {
    uint32_t u = stl_u32le(p);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static inline int stl_load_ascii(StlMesh* m, const char* text) {
    int cap = 256;
    int n = 0;
    m->positions = (float*)malloc((size_t)cap * 9 * sizeof(float));
    m->normals = (float*)malloc((size_t)cap * 3 * sizeof(float));
    if (!m->positions || !m->normals) {
        return 0;
    }
    const char* p = text;
    float nx = 0, ny = 0, nz = 1;
    float verts[9];
    int vi = 0;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
            p++;
        }
        if (strncmp(p, "facet", 5) == 0) {
            if (sscanf(p, "facet normal %f %f %f", &nx, &ny, &nz) != 3) {
                nx = 0;
                ny = 0;
                nz = 1;
            }
            vi = 0;
        } else if (strncmp(p, "vertex", 6) == 0) {
            float x, y, z;
            if (sscanf(p, "vertex %f %f %f", &x, &y, &z) == 3 && vi < 3) {
                verts[vi * 3 + 0] = x;
                verts[vi * 3 + 1] = y;
                verts[vi * 3 + 2] = z;
                vi++;
            }
        } else if (strncmp(p, "endfacet", 8) == 0 && vi == 3) {
            if (n >= cap) {
                cap *= 2;
                float* np = (float*)realloc(m->positions,
                    (size_t)cap * 9 * sizeof(float));
                float* nn = (float*)realloc(m->normals,
                    (size_t)cap * 3 * sizeof(float));
                if (!np || !nn) {
                    free(np);
                    free(nn);
                    return 0;
                }
                m->positions = np;
                m->normals = nn;
            }
            memcpy(m->positions + n * 9, verts, 9 * sizeof(float));
            m->normals[n * 3 + 0] = nx;
            m->normals[n * 3 + 1] = ny;
            m->normals[n * 3 + 2] = nz;
            for (int i = 0; i < 3; i++) {
                stl_grow_aabb(m, verts[i * 3], verts[i * 3 + 1],
                    verts[i * 3 + 2]);
            }
            n++;
            vi = 0;
        }
        while (*p && *p != '\n') {
            p++;
        }
        if (*p == '\n') {
            p++;
        }
    }
    m->tri_count = n;
    return n > 0;
}

static inline int stl_load_binary(StlMesh* m, const unsigned char* buf,
        size_t n) {
    if (n < 84) {
        return 0;
    }
    uint32_t tri = stl_u32le(buf + 80);
    if (84ull + (uint64_t)tri * 50ull > (uint64_t)n) {
        return 0;
    }
    if (tri == 0 || tri > 20000000u) {
        return 0;
    }
    m->positions = (float*)malloc((size_t)tri * 9 * sizeof(float));
    m->normals = (float*)malloc((size_t)tri * 3 * sizeof(float));
    if (!m->positions || !m->normals) {
        return 0;
    }
    const unsigned char* p = buf + 84;
    for (uint32_t i = 0; i < tri; i++) {
        m->normals[i * 3 + 0] = stl_f32le(p + 0);
        m->normals[i * 3 + 1] = stl_f32le(p + 4);
        m->normals[i * 3 + 2] = stl_f32le(p + 8);
        for (int v = 0; v < 3; v++) {
            float x = stl_f32le(p + 12 + v * 12 + 0);
            float y = stl_f32le(p + 12 + v * 12 + 4);
            float z = stl_f32le(p + 12 + v * 12 + 8);
            m->positions[i * 9 + v * 3 + 0] = x;
            m->positions[i * 9 + v * 3 + 1] = y;
            m->positions[i * 9 + v * 3 + 2] = z;
            stl_grow_aabb(m, x, y, z);
        }
        p += 50;
    }
    m->tri_count = (int)tri;
    return 1;
}

static inline int stl_load(StlMesh* m, const char* path) {
    memset(m, 0, sizeof(*m));
    m->min[0] = m->min[1] = m->min[2] = 1.0e30f;
    m->max[0] = m->max[1] = m->max[2] = -1.0e30f;
    snprintf(m->path, STL_MAX_PATH, "%s", path);
    FILE* f = fopen(path, "rb");
    if (!f) {
        return 0;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return 0;
    }
    long sz = ftell(f);
    if (sz <= 0) {
        fclose(f);
        return 0;
    }
    rewind(f);
    unsigned char* buf = (unsigned char*)malloc((size_t)sz + 1);
    if (!buf) {
        fclose(f);
        return 0;
    }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[rd] = 0;
    int ascii = 0;
    if (rd >= 6 && (memcmp(buf, "solid ", 6) == 0
            || memcmp(buf, "solid\n", 6) == 0
            || memcmp(buf, "solid\r", 6) == 0)) {
        /* Many binary STLs still start with "solid". Require a facet keyword. */
        size_t lim = rd < 1024 ? rd : 1024;
        for (size_t i = 0; i + 5 < lim; i++) {
            if (memcmp(buf + i, "facet", 5) == 0) {
                ascii = 1;
                break;
            }
        }
    }
    int ok = ascii ? stl_load_ascii(m, (const char*)buf)
        : stl_load_binary(m, buf, rd);
    free(buf);
    if (!ok) {
        stl_free(m);
        return 0;
    }
    return 1;
}

#ifdef RAYLIB_H
static inline Mesh stl_to_raylib_mesh(const StlMesh* m) {
    Mesh mesh = { 0 };
    if (m->tri_count <= 0) {
        return mesh;
    }
    int vc = m->tri_count * 3;
    mesh.vertexCount = vc;
    mesh.triangleCount = m->tri_count;
    mesh.vertices = (float*)MemAlloc((unsigned int)vc * 3 * sizeof(float));
    mesh.normals = (float*)MemAlloc((unsigned int)vc * 3 * sizeof(float));
    mesh.texcoords = (float*)MemAlloc((unsigned int)vc * 2 * sizeof(float));
    mesh.indices = 0;
    memcpy(mesh.vertices, m->positions, (size_t)vc * 3 * sizeof(float));
    for (int t = 0; t < m->tri_count; t++) {
        float nx = m->normals[t * 3 + 0];
        float ny = m->normals[t * 3 + 1];
        float nz = m->normals[t * 3 + 2];
        for (int v = 0; v < 3; v++) {
            int i = t * 3 + v;
            mesh.normals[i * 3 + 0] = nx;
            mesh.normals[i * 3 + 1] = ny;
            mesh.normals[i * 3 + 2] = nz;
            mesh.texcoords[i * 2 + 0] = 0.0f;
            mesh.texcoords[i * 2 + 1] = 0.0f;
        }
    }
    UploadMesh(&mesh, 0);
    return mesh;
}

static inline Model stl_load_model(const char* path, Color tint) {
    StlMesh stl;
    Model model = { 0 };
    if (!stl_load(&stl, path)) {
        return model;
    }
    Mesh mesh = stl_to_raylib_mesh(&stl);
    stl_free(&stl);
    if (mesh.vertexCount <= 0) {
        return model;
    }
    model = LoadModelFromMesh(mesh);
    if (model.materialCount > 0) {
        model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = tint;
    }
    return model;
}
#endif
