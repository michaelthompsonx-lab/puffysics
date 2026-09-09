#ifndef PLAY_DRAW_H
#define PLAY_DRAW_H
#include "raylib.h"
#include <string.h>

#define float3 play_raymath_float3
#include "raymath.h"
#undef float3
#include "rlgl.h"

typedef struct PlayLook {
    Shader lit;
    Shader water;
    int lit_view;
    int water_view;
    int lit_color;
    int water_color;
    int hover;
    int poke;
    float poke_age;
} PlayLook;

static PlayLook* play_look_cur;

static void play_fill_shader(Shader sh, int loc, Color c) {
    if (loc < 0) {
        return;
    }
    float v[4] = {
        (float)c.r / 255.0f,
        (float)c.g / 255.0f,
        (float)c.b / 255.0f,
        (float)c.a / 255.0f
    };
    SetShaderValue(sh, loc, v, SHADER_UNIFORM_VEC4);
}

static void play_fill(Color c) {
    if (!play_look_cur) {
        return;
    }
    play_fill_shader(play_look_cur->lit, play_look_cur->lit_color, c);
}

static Vector3 play_v3(B3Vec3 p) {
    return (Vector3){p.x, p.y, p.z};
}

static Matrix play_quat_matrix(B3Quat q) {
    B3Vec3 x = b3_rotate(q, b3_v(1.0f, 0.0f, 0.0f));
    B3Vec3 y = b3_rotate(q, b3_v(0.0f, 1.0f, 0.0f));
    B3Vec3 z = b3_rotate(q, b3_v(0.0f, 0.0f, 1.0f));
    Matrix m = {0};
    m.m0 = x.x; m.m1 = x.y; m.m2 = x.z;
    m.m4 = y.x; m.m5 = y.y; m.m6 = y.z;
    m.m8 = z.x; m.m9 = z.y; m.m10 = z.z;
    m.m15 = 1.0f;
    return m;
}

static const char* play_vs = R"GLSL(#version 330
layout (location = 0) in vec3 vertexPosition;
layout (location = 1) in vec2 vertexTexCoord;
layout (location = 2) in vec3 vertexNormal;
layout (location = 3) in vec4 vertexColor;
uniform mat4 mvp;
uniform mat4 matModel;
out vec3 worldPos;
out vec3 worldN;
out vec4 vertColor;
void main() {
    worldPos = vec3(matModel * vec4(vertexPosition, 1.0));
    worldN = mat3(matModel) * vertexNormal;
    vertColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)GLSL";

static const char* play_fs_lit = R"GLSL(#version 330
in vec3 worldPos;
in vec3 worldN;
in vec4 vertColor;
uniform vec4 colDiffuse;
uniform vec3 viewPos;
out vec4 finalColor;
void main() {
    /* Draw* tints vertexColor; colDiffuse is white unless we set it. */
    vec4 albedo = colDiffuse;
    if (dot(vertColor.rgb, vertColor.rgb) > 1.0e-6 &&
        dot(colDiffuse.rgb, colDiffuse.rgb) > 2.99) {
        albedo = vertColor;
    } else if (dot(vertColor.rgb, vertColor.rgb) > 1.0e-6) {
        albedo *= vertColor;
    }
    vec3 n = worldN;
    float nlen = length(n);
    if (nlen < 1.0e-6) {
        finalColor = albedo;
        return;
    }
    n = normalize(n);
    if (!gl_FrontFacing) n = -n;
    vec3 light = normalize(vec3(-0.42, 0.86, 0.38));
    vec3 fillL = normalize(vec3(0.55, 0.18, -0.65));
    vec3 v = normalize(viewPos - worldPos);
    float nd = max(dot(n, light), 0.0);
    float fill = max(dot(n, fillL), 0.0);
    float spec = pow(max(dot(n, normalize(light + v)), 0.0), 52.0);
    float rim = pow(1.0 - max(dot(n, v), 0.0), 3.0);
    float fog = clamp((length(worldPos.xz) - 2.0) / 16.0, 0.0, 0.55);
    vec3 c = albedo.rgb * (0.20 + 0.70 * nd + 0.18 * fill);
    c += vec3(1.00, 0.97, 0.90) * spec * 0.38;
    c += vec3(0.10, 0.13, 0.18) * rim;
    c = mix(c, vec3(0.055, 0.065, 0.080), fog);
    finalColor = vec4(c, albedo.a);
}
)GLSL";

static const char* play_fs_water = R"GLSL(#version 330
in vec3 worldPos;
in vec3 worldN;
in vec4 vertColor;
uniform vec4 colDiffuse;
uniform vec3 viewPos;
out vec4 finalColor;
void main() {
    vec3 n = worldN;
    float nlen = length(n);
    if (nlen > 1.0e-6) n = normalize(n);
    if (!gl_FrontFacing) n = -n;
    vec3 v = normalize(viewPos - worldPos);
    float fres = pow(1.0 - max(dot(n, v), 0.0), 2.4);
    vec3 deep = vec3(0.07, 0.26, 0.40);
    vec3 rim = vec3(0.55, 0.84, 0.94);
    vec3 tint = vertColor.rgb * colDiffuse.rgb;
    if (dot(tint, tint) < 1.0e-6) tint = vec3(1.0);
    vec3 c = mix(deep, rim, fres) * mix(vec3(1.0), tint, 0.35);
    float a = mix(0.16, 0.38, fres) * vertColor.a * colDiffuse.a;
    finalColor = vec4(c, a);
}
)GLSL";

static PlayLook play_look_init(void) {
    PlayLook L;
    memset(&L, 0, sizeof(L));
    L.lit = LoadShaderFromMemory(play_vs, play_fs_lit);
    L.water = LoadShaderFromMemory(play_vs, play_fs_water);
    L.lit_view = GetShaderLocation(L.lit, "viewPos");
    L.water_view = GetShaderLocation(L.water, "viewPos");
    L.lit_color = L.lit.locs[SHADER_LOC_COLOR_DIFFUSE];
    if (L.lit_color < 0) {
        L.lit_color = GetShaderLocation(L.lit, "colDiffuse");
    }
    L.water_color = L.water.locs[SHADER_LOC_COLOR_DIFFUSE];
    if (L.water_color < 0) {
        L.water_color = GetShaderLocation(L.water, "colDiffuse");
    }
    play_look_cur = NULL;
    L.hover = -1;
    L.poke = -1;
    return L;
}

static void play_look_free(PlayLook* L) {
    UnloadShader(L->lit);
    UnloadShader(L->water);
}

static void play_look_tick(PlayLook* L, float dt) {
    if (L->poke_age > 0.0f) {
        L->poke_age -= dt;
        if (L->poke_age <= 0.0f) {
            L->poke = -1;
            L->poke_age = 0.0f;
        }
    }
}

static void play_look_set_view(PlayLook* L, Vector3 view) {
    play_look_cur = L;
    float v[3] = {view.x, view.y, view.z};
    SetShaderValue(L->lit, L->lit_view, v, SHADER_UNIFORM_VEC3);
    SetShaderValue(L->water, L->water_view, v, SHADER_UNIFORM_VEC3);
}

static Color play_palette(int i) {
    static const Color pal[] = {
        {250, 210, 90, 255}, {140, 200, 240, 255}, {240, 120, 100, 255},
        {120, 210, 150, 255}, {210, 160, 240, 255}, {240, 180, 110, 255},
        {110, 200, 200, 255}, {230, 140, 180, 255}
    };
    return pal[i & 7];
}

static Color play_tint_body(const PlayLook* L, int body, Color base) {
    if (body == L->poke && L->poke_age > 0.0f) {
        float t = L->poke_age / 0.28f;
        if (t > 1.0f) t = 1.0f;
        base.r = (unsigned char)(base.r + (255 - base.r) * t);
        base.g = (unsigned char)(base.g + (255 - base.g) * t);
        base.b = (unsigned char)(base.b + (255 - base.b) * t);
    } else if (body == L->hover) {
        base.r = (unsigned char)(base.r < 220 ? base.r + 35 : 255);
        base.g = (unsigned char)(base.g < 220 ? base.g + 35 : 255);
        base.b = (unsigned char)(base.b < 220 ? base.b + 35 : 255);
    }
    return base;
}

static void play_draw_box(B3Vec3 center, B3Quat rot, B3Vec3 half,
        Color fill, Color edge) {
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlMultMatrixf(MatrixToFloat(play_quat_matrix(rot)));
    play_fill(fill);
    DrawCube((Vector3){0.0f, 0.0f, 0.0f},
        half.x * 2.0f, half.y * 2.0f, half.z * 2.0f, fill);
    if (edge.a > 0) {
        DrawCubeWires((Vector3){0.0f, 0.0f, 0.0f},
            half.x * 2.0f, half.y * 2.0f, half.z * 2.0f, edge);
    }
    rlPopMatrix();
}

static void play_draw_sphere(B3Vec3 c, float r, Color fill) {
    play_fill(fill);
    DrawSphereEx(play_v3(c), r, 22, 22, fill);
}

static void play_draw_spin_tick(const B3Body* b, float r) {
    B3Vec3 tip = b3_add(b->center, b3_rotate(b->rotation, b3_v(r, 0.0f, 0.0f)));
    DrawLine3D(play_v3(b->center), play_v3(tip),
        (Color){255, 255, 255, 220});
    DrawSphereEx(play_v3(tip), r * 0.14f, 8, 8, RAYWHITE);
}

static void play_draw_joint_rod(B3Vec3 a, B3Vec3 b, Color col) {
    B3Vec3 d = b3_sub(b, a);
    if (b3_len2(d) < 1.0e-8f) {
        return;
    }
    DrawLine3D(play_v3(a), play_v3(b), col);
    DrawCylinderEx(play_v3(a), play_v3(b), 0.012f, 0.012f, 10, col);
}

static void play_draw_capsule(const B3Body* b, float half, float r, Color fill) {
    B3Vec3 a = b3_add(b->center, b3_rotate(b->rotation, b3_v(0.0f, -half, 0.0f)));
    B3Vec3 c = b3_add(b->center, b3_rotate(b->rotation, b3_v(0.0f, half, 0.0f)));
    play_fill(fill);
    DrawCylinderEx(play_v3(a), play_v3(c), r, r, 16, fill);
    DrawSphereEx(play_v3(a), r, 14, 14, fill);
    DrawSphereEx(play_v3(c), r, 14, 14, fill);
}

static void play_draw_shadow(B3Vec3 c, float r) {
    Vector3 p = {c.x, 0.008f, c.z};
    play_fill((Color){0, 0, 0, 42});
    DrawCylinderEx(p, (Vector3){c.x, 0.010f, c.z}, r, r, 16,
        (Color){0, 0, 0, 42});
}

static void play_draw_trail(const B3Vec3* buf, int n, int cap, Color col) {
    int start = n > cap ? n - cap : 0;
    for (int s = start; s < n - 1; s++) {
        B3Vec3 a = buf[s % cap];
        B3Vec3 b = buf[(s + 1) % cap];
        float t = (float)(s - start) / (float)(cap);
        Color c = col;
        c.a = (unsigned char)(24 + 160 * t);
        DrawLine3D(play_v3(a), play_v3(b), c);
    }
}

static void play_draw_floor(float half) {
    play_fill((Color){36, 42, 52, 255});
    DrawPlane((Vector3){0.0f, 0.0f, 0.0f},
        (Vector2){half * 2.0f, half * 2.0f},
        (Color){36, 42, 52, 255});
}

static void play_draw_water(PlayLook* L, float half, float height) {
    if (!(height > 0.0f)) {
        return;
    }
    BeginShaderMode(L->water);
    rlDisableBackfaceCulling();
    rlDisableDepthMask();
    BeginBlendMode(BLEND_ALPHA);
    play_fill_shader(L->water, L->water_color, (Color){40, 120, 180, 255});
    DrawCube((Vector3){0.0f, height * 0.5f + 0.012f, 0.0f},
        half * 2.0f, height - 0.02f, half * 2.0f, (Color){40, 120, 180, 255});
    play_fill_shader(L->water, L->water_color, (Color){120, 190, 230, 255});
    DrawPlane((Vector3){0.0f, height, 0.0f},
        (Vector2){half * 2.0f, half * 2.0f}, (Color){120, 190, 230, 255});
    EndBlendMode();
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
    EndShaderMode();
    DrawCubeWires((Vector3){0.0f, height * 0.5f, 0.0f},
        half * 2.0f, height, half * 2.0f, (Color){80, 160, 200, 70});
}

static int play_pick_body(const B3World* w, Ray ray) {
    int hit = -1;
    float best = 1.0e9f;
    for (int i = 0; i < w->body_count; i++) {
        const B3Body* b = &w->bodies[i];
        if (b->type != B3_DYNAMIC) {
            continue;
        }
        float rad = 0.08f;
        for (int s = 0; s < w->shape_count; s++) {
            if (w->shapes[s].body == i) {
                const B3Shape* sh = &w->shapes[s];
                float r = sh->type == B3_SPHERE ? sh->radius
                    : (sh->type == B3_CAPSULE ? sh->half.y + sh->radius
                        : b3_len(sh->half));
                if (r > rad) {
                    rad = r;
                }
            }
        }
        RayCollision c = GetRayCollisionSphere(ray, play_v3(b->center), rad);
        if (c.hit && c.distance < best) {
            best = c.distance;
            hit = i;
        }
    }
    return hit;
}

static void play_poke_body(B3World* w, int body, Ray ray, float dv) {
    if (body < 0 || body >= w->body_count) {
        return;
    }
    B3Body* b = &w->bodies[body];
    if (b->type != B3_DYNAMIC || !(b->inv_mass > 0.0f)) {
        return;
    }
    B3Vec3 dir = b3_v(ray.direction.x, ray.direction.y, ray.direction.z);
    float ln = b3_len(dir);
    if (ln < 1.0e-6f) {
        return;
    }
    dir = b3_mul(dir, 1.0f / ln);
    dir = b3_norm(b3_add(dir, b3_v(0.0f, 0.18f, 0.0f)));
    b->lin_vel = b3_add(b->lin_vel, b3_mul(dir, dv));
    B3Vec3 side = b3_cross(dir, b3_v(0.0f, 1.0f, 0.0f));
    if (b3_len2(side) < 1.0e-8f) {
        side = b3_cross(dir, b3_v(1.0f, 0.0f, 0.0f));
    }
    B3Vec3 r = b3_mul(b3_norm(side), 0.08f);
    b->ang_vel = b3_add(b->ang_vel, b3_mul(b3_cross(r, dir), 6.0f * dv));
}

static Color play_underwater_tint(Color c, float depth) {
    float t = depth * 0.22f;
    if (t < 0.0f) t = 0.0f;
    if (t > 0.42f) t = 0.42f;
    c.r = (unsigned char)(c.r * (1.0f - t) + 18.0f * t);
    c.g = (unsigned char)(c.g * (1.0f - t) + 90.0f * t);
    c.b = (unsigned char)(c.b * (1.0f - t) + 150.0f * t);
    return c;
}

#endif
