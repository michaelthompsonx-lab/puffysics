#ifndef FABRIC_RENDER_H
#define FABRIC_RENDER_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "raylib.h"
#define float3 fabric_raymath_float3
#include "raymath.h"
#undef float3
#include "rlgl.h"
#include "puffysics.cuh"

/* Host drawing. CUDA interop writes position/normal VBOs after init. */
struct FabricRenderer {
    Mesh mesh;
    Mesh sphere;
    Material cloth_material;
    Material sphere_material;
    int W, H;
    float spacing, half_x, half_z;
};

static const Color fabric_planet_colors[7] = {
    {112, 196, 250, 255}, {245, 137, 109, 255}, {132, 222, 170, 255},
    {197, 155, 245, 255}, {244, 199, 124, 255}, {105, 219, 213, 255},
    {237, 156, 192, 255}
};

static const char* fabric_vertex_shader = R"GLSL(#version 330
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexTexCoord;
layout(location = 2) in vec3 vertexNormal;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 worldPosition;
out vec3 worldNormal;
out vec2 uv;
void main() {
    worldPosition = vec3(matModel * vec4(vertexPosition, 1.0));
    worldNormal = vec3(matNormal * vec4(vertexNormal, 0.0));
    uv = vertexTexCoord;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)GLSL";

static const char* fabric_cloth_fragment_shader = R"GLSL(#version 330
in vec3 worldPosition;
in vec3 worldNormal;
in vec2 uv;
uniform vec3 viewPos;
uniform vec2 gridCells;
out vec4 finalColor;

// Grid coordinates follow material UVs, so every line bends with the sheet.
// Pixel derivatives keep line widths stable without shimmering at grazing views.
float gridLine(vec2 p, float width) {
    vec2 footprint = max(fwidth(p), vec2(0.00001));
    vec2 distanceToLine = abs(fract(p + 0.5) - 0.5) / footprint;
    vec2 line = 1.0 - smoothstep(vec2(width - 0.45), vec2(width + 0.45), distanceToLine);
    line *= 1.0 - smoothstep(vec2(0.45), vec2(1.25), footprint);
    return max(line.x, line.y);
}

void main() {
    vec3 n = normalize(worldNormal);
    if (!gl_FrontFacing) n = -n;
    vec3 v = normalize(viewPos - worldPosition);
    vec3 light = normalize(vec3(-0.45, 0.85, 0.55));
    float depth = smoothstep(0.0, 0.22, max(0.0, -worldPosition.y));
    vec3 shallow = vec3(0.045, 0.20, 0.275);
    vec3 middle = vec3(0.30, 0.16, 0.42);
    vec3 deep = vec3(0.78, 0.38, 0.14);
    vec3 base = mix(shallow, middle, smoothstep(0.0, 0.55, depth));
    base = mix(base, deep, smoothstep(0.45, 1.0, depth));
    float diffuse = max(dot(n, light), 0.0);
    float fill = max(dot(n, normalize(vec3(0.65, 0.25, -0.7))), 0.0);
    float rim = pow(1.0 - max(dot(n, v), 0.0), 3.0);
    vec3 color = base * (0.47 + 0.64*diffuse + 0.17*fill);
    color += vec3(0.035, 0.075, 0.10) * rim;
    vec2 cells = uv * gridCells;
    float minor = gridLine(cells, 0.48);
    float major = gridLine(cells / 4.0, 0.72);
    color = mix(color, vec3(0.25, 0.48, 0.57), minor * 0.43);
    color = mix(color, vec3(0.56, 0.76, 0.80), major * 0.63);
    vec2 boundaryDistance = min(uv, 1.0 - uv) / max(fwidth(uv), vec2(0.00001));
    float boundary = 1.0 - smoothstep(0.6, 1.8, min(boundaryDistance.x, boundaryDistance.y));
    color = mix(color, vec3(0.53, 0.75, 0.77), boundary * 0.8);
    finalColor = vec4(color, 1.0);
}
)GLSL";

static const char* fabric_sphere_fragment_shader = R"GLSL(#version 330
in vec3 worldPosition;
in vec3 worldNormal;
uniform vec3 viewPos;
uniform vec4 colDiffuse;
out vec4 finalColor;
void main() {
    vec3 n = normalize(worldNormal);
    vec3 v = normalize(viewPos - worldPosition);
    vec3 light = normalize(vec3(-0.45, 0.85, 0.55));
    float diffuse = max(dot(n, light), 0.0);
    float fill = max(dot(n, normalize(vec3(0.65, 0.25, -0.7))), 0.0);
    float specular = pow(max(dot(n, normalize(light + v)), 0.0), 64.0);
    float rim = pow(1.0 - max(dot(n, v), 0.0), 4.0);
    vec3 color = colDiffuse.rgb * (0.20 + 0.76*diffuse + 0.22*fill);
    color += mix(vec3(1.0), colDiffuse.rgb, 0.35) * specular * 0.62;
    color += vec3(0.12, 0.18, 0.23) * rim;
    finalColor = vec4(color, 1.0);
}
)GLSL";

static void fabric_render_fail(const char* message) {
    fprintf(stderr, "fabric renderer: %s\n", message);
    fflush(stderr);
    exit(EXIT_FAILURE);
}

static int fabric_render_uniform(Shader shader, const char* name) {
    int location = GetShaderLocation(shader, name);
    if (location < 0) {
        fprintf(stderr, "fabric renderer: required GLSL uniform '%s' is missing\n", name);
        fabric_render_fail("shader interface mismatch; inspect raylib shader logs above");
    }
    return location;
}

static Shader fabric_render_shader(const char* fragment, int cloth) {
    Shader shader = LoadShaderFromMemory(fabric_vertex_shader, fragment);
    if (!IsShaderValid(shader) || shader.id == rlGetShaderIdDefault())
        fabric_render_fail("GLSL 330 shader failed; OpenGL 3.3 is required (see raylib logs above)");
    shader.locs[SHADER_LOC_VERTEX_POSITION] = GetShaderLocationAttrib(shader, "vertexPosition");
    shader.locs[SHADER_LOC_VERTEX_NORMAL] = GetShaderLocationAttrib(shader, "vertexNormal");
    shader.locs[SHADER_LOC_VERTEX_TEXCOORD01] = GetShaderLocationAttrib(shader, "vertexTexCoord");
    if (shader.locs[SHADER_LOC_VERTEX_POSITION] != 0 ||
        shader.locs[SHADER_LOC_VERTEX_NORMAL] != 2 ||
        (cloth && shader.locs[SHADER_LOC_VERTEX_TEXCOORD01] != 1))
        fabric_render_fail("shader attributes do not match the interop mesh VAO");
    shader.locs[SHADER_LOC_MATRIX_MVP] = fabric_render_uniform(shader, "mvp");
    shader.locs[SHADER_LOC_MATRIX_MODEL] = fabric_render_uniform(shader, "matModel");
    shader.locs[SHADER_LOC_MATRIX_NORMAL] = fabric_render_uniform(shader, "matNormal");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = fabric_render_uniform(shader, "viewPos");
    if (!cloth) shader.locs[SHADER_LOC_COLOR_DIFFUSE] = fabric_render_uniform(shader, "colDiffuse");
    return shader;
}

static void fabric_render_init(FabricRenderer* r, int W, int H, float spacing) {
    if (W < 2 || H < 2 || W > 65535 / H || !isfinite(spacing) || spacing <= 0.0f)
        fabric_render_fail("invalid membrane dimensions or spacing");
    *r = FabricRenderer{};
    r->W = W;
    r->H = H;
    r->spacing = spacing;
    r->half_x = 0.5f * spacing * (W - 1);
    r->half_z = 0.5f * spacing * (H - 1);
    Mesh* m = &r->mesh;
    m->vertexCount = W * H;
    m->triangleCount = (W - 1) * (H - 1) * 2;
    m->vertices = (float*)RL_CALLOC((size_t)m->vertexCount * 3, sizeof(float));
    m->normals = (float*)RL_CALLOC((size_t)m->vertexCount * 3, sizeof(float));
    m->texcoords = (float*)RL_CALLOC((size_t)m->vertexCount * 2, sizeof(float));
    m->indices = (unsigned short*)RL_MALLOC((size_t)m->triangleCount * 3 * sizeof(unsigned short));
    if (!m->vertices || !m->normals || !m->texcoords || !m->indices)
        fabric_render_fail("mesh allocation failed");
    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            int k = j * W + i;
            m->vertices[k * 3] = i * spacing - r->half_x;
            m->vertices[k * 3 + 2] = j * spacing - r->half_z;
            m->normals[k * 3 + 1] = 1.0f;
            m->texcoords[k * 2] = (float)i / (W - 1);
            m->texcoords[k * 2 + 1] = (float)j / (H - 1);
            if (i == W - 1 || j == H - 1) continue;
            int q = (j * (W - 1) + i) * 6;
            m->indices[q] = (unsigned short)k;
            m->indices[q + 1] = (unsigned short)(k + W);
            m->indices[q + 2] = (unsigned short)(k + 1);
            m->indices[q + 3] = (unsigned short)(k + 1);
            m->indices[q + 4] = (unsigned short)(k + W);
            m->indices[q + 5] = (unsigned short)(k + W + 1);
        }
    }
    UploadMesh(m, true);
    if (!m->vaoId || !m->vboId || !m->vboId[0] || !m->vboId[1] || !m->vboId[2])
        fabric_render_fail("dynamic membrane VBO upload failed");
    r->cloth_material = LoadMaterialDefault();
    r->sphere_material = LoadMaterialDefault();
    if (!r->cloth_material.maps || !r->sphere_material.maps)
        fabric_render_fail("material allocation failed");
    r->cloth_material.shader = fabric_render_shader(fabric_cloth_fragment_shader, 1);
    r->sphere_material.shader = fabric_render_shader(fabric_sphere_fragment_shader, 0);
    float cells[2] = {(float)(W - 1), (float)(H - 1)};
    SetShaderValue(r->cloth_material.shader,
        fabric_render_uniform(r->cloth_material.shader, "gridCells"), cells, SHADER_UNIFORM_VEC2);
    r->sphere = GenMeshSphere(1.0f, 32, 48);
    if (!r->sphere.vaoId || !r->sphere.vboId || !r->sphere.vboId[2])
        fabric_render_fail("lit sphere mesh upload failed");
}

static void fabric_render_frame(const FabricRenderer* r) {
    const Color rail = {66, 91, 108, 255};
    const Color pin = {139, 184, 192, 255};
    Vector3 corners[4] = {
        {-r->half_x, 0.0f, -r->half_z}, {r->half_x, 0.0f, -r->half_z},
        {r->half_x, 0.0f, r->half_z}, {-r->half_x, 0.0f, r->half_z}
    };
    for (int k = 0; k < 4; ++k) {
        Vector3 a = corners[k], b = corners[(k + 1) % 4];
        DrawCylinderEx(a, b, 0.012f, 0.012f, 10, rail);
        Vector3 top = {a.x, 0.24f, a.z};
        DrawCylinderEx(a, top, 0.012f, 0.008f, 10, rail);
        DrawSphereEx(top, 0.018f, 8, 12, pin);
        DrawCube(a, 0.046f, 0.026f, 0.046f, pin);
    }
    for (int i = 4; i < r->W - 1; i += 4) {
        float x = i * r->spacing - r->half_x;
        DrawCube(Vector3{x, 0.0f, -r->half_z}, 0.025f, 0.019f, 0.033f, pin);
        DrawCube(Vector3{x, 0.0f, r->half_z}, 0.025f, 0.019f, 0.033f, pin);
    }
    for (int j = 4; j < r->H - 1; j += 4) {
        float z = j * r->spacing - r->half_z;
        DrawCube(Vector3{-r->half_x, 0.0f, z}, 0.033f, 0.019f, 0.025f, pin);
        DrawCube(Vector3{r->half_x, 0.0f, z}, 0.033f, 0.019f, 0.025f, pin);
    }
}

static Color fabric_tint_ball(Color base, int q, int hover, int poke, float poke_age) {
    if (q == poke && poke_age > 0.0f) {
        float t = poke_age / 0.28f;
        if (t > 1.0f) t = 1.0f;
        base.r = (unsigned char)(base.r + (255 - base.r) * t);
        base.g = (unsigned char)(base.g + (255 - base.g) * t);
        base.b = (unsigned char)(base.b + (255 - base.b) * t);
    } else if (q == hover) {
        base.r = (unsigned char)(base.r < 220 ? base.r + 35 : 255);
        base.g = (unsigned char)(base.g < 220 ? base.g + 35 : 255);
        base.b = (unsigned char)(base.b < 220 ? base.b + 35 : 255);
    }
    return base;
}

static void fabric_render_draw(FabricRenderer* r, Camera3D camera,
        const B3Vec3* ball_positions, const float* radii, int nb,
        const B3Vec3* trails, const int* trail_counts, int trail_capacity,
        int hover, int poke, float poke_age) {
    float eye[3] = {camera.position.x, camera.position.y, camera.position.z};
    SetShaderValue(r->cloth_material.shader, r->cloth_material.shader.locs[SHADER_LOC_VECTOR_VIEW],
        eye, SHADER_UNIFORM_VEC3);
    SetShaderValue(r->sphere_material.shader, r->sphere_material.shader.locs[SHADER_LOC_VECTOR_VIEW],
        eye, SHADER_UNIFORM_VEC3);
    rlDrawRenderBatchActive();
    rlDisableBackfaceCulling();
    DrawMesh(r->mesh, r->cloth_material, MatrixIdentity());
    rlEnableBackfaceCulling();
    fabric_render_frame(r);
    rlDrawRenderBatchActive();
    for (int q = 0; q < nb && q < 8; ++q) {
        Color ball = q == 0
            ? Color{255, 201, 85, 255} : fabric_planet_colors[q - 1];
        r->sphere_material.maps[MATERIAL_MAP_DIFFUSE].color =
            fabric_tint_ball(ball, q, hover, poke, poke_age);
        Matrix model = MatrixScale(radii[q], radii[q], radii[q]);
        model.m12 = ball_positions[q].x;
        model.m13 = ball_positions[q].y;
        model.m14 = ball_positions[q].z;
        DrawMesh(r->sphere, r->sphere_material, model);
    }
    if (!trails || !trail_counts || trail_capacity < 2) return;
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (int p = 0; p < nb - 1 && p < 7; ++p) {
        int total = trail_counts[p];
        int n = total < trail_capacity ? total : trail_capacity;
        const B3Vec3* ring = trails + (size_t)p * trail_capacity;
        for (int s = 1; s < n; ++s) {
            const B3Vec3 a = ring[(total - n + s - 1) % trail_capacity];
            const B3Vec3 b = ring[(total - n + s) % trail_capacity];
            float t = (float)s / (n - 1);
            Color color = fabric_planet_colors[p];
            color.a = (unsigned char)(190.0f * t * t * (3.0f - 2.0f * t));
            DrawLine3D(Vector3{a.x, a.y, a.z}, Vector3{b.x, b.y, b.z}, color);
        }
    }
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
}

static void fabric_render_hud(int nb, int paused, float sim_time, float gpu_ms) {
    const Color white = {220, 231, 238, 255};
    const Color muted = {137, 161, 177, 255};
    const Color teal = {111, 211, 193, 255};
    const Color gold = {245, 201, 107, 255};
    int width = GetScreenWidth(), height = GetScreenHeight();
    DrawRectangle(0, 0, width, 88, Color{6, 12, 20, 226});
    DrawRectangle(24, 23, 3, 40, teal);
    DrawText("SPACETIME / FABRIC", 39, 21, 25, white);
    DrawText("A pinned membrane shaped by moving masses", 40, 54, 15, muted);
    char status[128], timing[96];
    snprintf(status, sizeof(status), "CUDA   |   %d / 8 SPHERES   |   %s", nb, paused ? "PAUSED" : "RUNNING");
    if (gpu_ms >= 0.0f && isfinite(gpu_ms))
        snprintf(timing, sizeof(timing), "SIM %7.2f s    GPU %.2f ms", sim_time, gpu_ms);
    else snprintf(timing, sizeof(timing), "SIM %7.2f s    GPU -- ms", sim_time);
    DrawText(status, width - 26 - MeasureText(status, 17), 25, 17, paused ? gold : teal);
    DrawText(timing, width - 26 - MeasureText(timing, 15), 55, 15, muted);
    DrawRectangle(0, height - 64, width, 64, Color{6, 12, 20, 232});
    DrawLine(24, height - 64, width - 24, height - 64, Color{43, 65, 81, 255});
    const char* help = "SPACE spawn   R reset   P pause   N step   Drag orbit   RMB poke   Wheel zoom   Q quit";
    int help_size = width < 1100 ? 14 : 16;
    DrawText(help, 26, height - 49, help_size, white);
    DrawText("HEIGHT", 26, height - 24, 11, muted);
    DrawRectangleGradientH(80, height - 23, 62, 8, Color{33, 109, 132, 255}, Color{108, 83, 161, 255});
    DrawRectangleGradientH(142, height - 23, 62, 8, Color{108, 83, 161, 255}, Color{196, 123, 76, 255});
    DrawText("pinned / deep", 215, height - 25, 12, muted);
    const char* legend = "Gold: central mass   /   Color: planet paths";
    DrawText(legend, width - 26 - MeasureText(legend, 12), height - 25, 12, muted);
}

static void fabric_render_free(FabricRenderer* r) {
    UnloadMesh(r->mesh);
    UnloadMesh(r->sphere);
    UnloadMaterial(r->cloth_material);
    UnloadMaterial(r->sphere_material);
    *r = FabricRenderer{};
}

#endif
