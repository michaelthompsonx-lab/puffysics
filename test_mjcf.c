#define B3_MAX_BODIES 48
#define B3_MAX_SHAPES 64
#define B3_MAX_JOINTS 32
#define B3_MAX_CONTACTS 128
#define B3_JOINT_ITERS 8
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mjcf.h"

#define CHECK(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        exit(1); \
    } \
} while (0)

static const char* k_stl =
    "/tmp/microduck_rl/src/mjlab_microduck/robot/microduck/assets/sole_left.stl";
static const char* k_scene =
    "/tmp/microduck_rl/src/mjlab_microduck/robot/microduck/scene_walk.xml";

static void test_stl_ascii(void) {
    const char* path = "/tmp/tiny.stl";
    FILE* f = fopen(path, "w");
    CHECK(f);
    fputs("solid tiny\n"
          "  facet normal 0 0 1\n"
          "    outer loop\n"
          "      vertex 0 0 0\n"
          "      vertex 1 0 0\n"
          "      vertex 0 1 0\n"
          "    endloop\n"
          "  endfacet\n"
          "endsolid tiny\n", f);
    fclose(f);
    StlMesh m;
    CHECK(stl_load(&m, path));
    CHECK(m.tri_count == 1);
    CHECK(m.positions[0] == 0.0f && m.positions[3] == 1.0f);
    stl_free(&m);
    printf("  ascii stl ok\n");
}

static void test_stl_binary(void) {
    StlMesh m;
    CHECK(stl_load(&m, k_stl));
    CHECK(m.tri_count > 100);
    float dx = m.max[0] - m.min[0];
    float dy = m.max[1] - m.min[1];
    float dz = m.max[2] - m.min[2];
    CHECK(dx > 0.01f && dx < 0.2f);
    CHECK(dy > 0.001f && dy < 0.2f);
    CHECK(dz > 0.01f && dz < 0.2f);
    printf("  binary stl tris=%d aabb=(%.3f,%.3f,%.3f)\n",
        m.tri_count, dx, dy, dz);
    stl_free(&m);
}

static void test_stl_zup(void) {
    StlMesh m;
    CHECK(stl_load(&m, "/tmp/tiny.stl"));
    stl_apply_zup(&m);
    CHECK(m.positions[0] == 0.0f && m.positions[1] == 0.0f && m.positions[2] == 0.0f);
    CHECK(m.positions[3] == 1.0f && m.positions[4] == 0.0f && m.positions[5] == 0.0f);
    CHECK(m.positions[6] == 0.0f && m.positions[7] == 0.0f && m.positions[8] == -1.0f);
    CHECK(m.normals[0] == 0.0f && m.normals[1] == 1.0f && m.normals[2] == 0.0f);
    stl_free(&m);
    printf("  zup remap ok\n");
}

static void pin_static(B3Body* b) {
    b->type = B3_STATIC;
    b->flags &= ~B3_FLAG_DYNAMIC;
    b->inv_mass = 0.0f;
    b->inv_inertia = b3_v(0, 0, 0);
    b->lin_vel = b3_v(0, 0, 0);
    b->ang_vel = b3_v(0, 0, 0);
}

static void test_mjcf_microduck(void) {
    MjcfModel model;
    CHECK(mjcf_load(&model, k_scene, 1));
    CHECK(model.body_count == 15);
    CHECK(model.joint_count == 14);
    CHECK(model.mesh_count >= 30);
    CHECK(model.key_count >= 2);
    CHECK(mjcf_find_key(&model, "STAND") >= 0);
    CHECK(mjcf_find_key(&model, "INIT") >= 0);
    int loaded = 0;
    for (int i = 0; i < model.mesh_count; i++) {
        loaded += model.meshes[i].loaded;
    }
    CHECK(loaded == model.mesh_count);
    int vis = 0, col = 0;
    for (int i = 0; i < model.geom_count; i++) {
        vis += model.geoms[i].visual;
        col += model.geoms[i].collide;
    }
    CHECK(vis > 40);
    CHECK(col >= 2);
    CHECK(strcmp(model.bodies[model.root].name, "trunk_base") == 0);
    int tbi = mjcf_find_mesh(&model, "trunk_base");
    CHECK(tbi >= 0);
    {
        const StlMesh* tb = &model.meshes[tbi].stl;
        float tdx = tb->max[0] - tb->min[0];
        float tdy = tb->max[1] - tb->min[1];
        float tdz = tb->max[2] - tb->min[2];
        /* File is ~57 x 36 x 3 mm Z-up; after remap Y is the thin axis. */
        CHECK(tdx > 0.05f && tdx < 0.07f);
        CHECK(tdy > 0.002f && tdy < 0.006f);
        CHECK(tdz > 0.03f && tdz < 0.04f);
    }
    printf("  xml bodies=%d joints=%d geoms=%d vis=%d col=%d meshes=%d keys=%d\n",
        model.body_count, model.joint_count, model.geom_count,
        vis, col, loaded, model.key_count);

    const char* expect[] = {
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee",
        "left_ankle", "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee",
        "right_ankle"
    };
    for (int i = 0; i < 14; i++) {
        CHECK(strcmp(model.joints[i].name, expect[i]) == 0);
        CHECK(model.joints[i].has_limit);
    }

    B3World w;
    b3_world_init(&w);
    MjcfSpawn spawn;
    CHECK(mjcf_spawn(&model, &w, &spawn, 1));
    CHECK(spawn.n_bodies == 15);
    CHECK(spawn.n_joints == 14);
    CHECK(spawn.n_shapes >= 2);
    float mass = 0.0f;
    for (int i = 0; i < model.body_count; i++) {
        int id = spawn.body_map[i];
        if (w.bodies[id].inv_mass > 0.0f) {
            mass += 1.0f / w.bodies[id].inv_mass;
        }
    }
    CHECK(mass > 0.4f && mass < 1.5f);
    CHECK(mjcf_apply_key(&w, &model, &spawn, "STAND"));
    int ki = mjcf_find_key(&model, "STAND");
    for (int i = 0; i < model.joint_count; i++) {
        float got = b3_joint_angle(&w, spawn.joint_map[i]);
        float want = model.keys[ki].qpos[7 + i];
        CHECK(fabsf(got - want) < 0.05f);
    }
    /* Ankle bodies carry the sole collision OBBs. STAND puts them near the floor. */
    float foot_y = 1.0e9f;
    int ankle_l = spawn.body_map[5];
    int ankle_r = spawn.body_map[14];
    for (int i = 0; i < w.shape_count; i++) {
        const B3Shape* s = &w.shapes[i];
        if (s->body != ankle_l && s->body != ankle_r) {
            continue;
        }
        const B3Body* b = &w.bodies[s->body];
        B3Vec3 wp = b3_xf_point(b->position, b->rotation, s->local_pos);
        if (wp.y < foot_y) {
            foot_y = wp.y;
        }
    }
    CHECK(foot_y < 0.03f);
    CHECK(foot_y > -0.02f);
    printf("  spawn mass=%.3f kg stand_y=%.3f foot_y=%.3f\n",
        mass, w.bodies[spawn.root].position.y, foot_y);

    pin_static(&w.bodies[spawn.root]);
    mjcf_hold_pose(&w, &spawn, model.joint_count, 20.0f, 0.7f, 0.8f);
    for (int i = 0; i < 60; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    CHECK(fabsf(w.bodies[spawn.root].position.y - 0.12f) < 0.001f);
    for (int i = 0; i < model.joint_count; i++) {
        const MjcfJoint* j = &model.joints[i];
        int pa = spawn.body_map[j->parent];
        int pb = spawn.body_map[j->child];
        float d = b3_len(b3_sub(w.bodies[pb].position, w.bodies[pa].position));
        float d0 = b3_len(model.bodies[j->child].rel_pos);
        CHECK(fabsf(d - d0) < 0.03f);
        /* Legs should stay near STAND; the heavy head can sag a bit. */
        if (strstr(j->name, "hip") || strstr(j->name, "knee")
                || strstr(j->name, "ankle")) {
            float got = b3_joint_angle(&w, spawn.joint_map[i]);
            float want = model.keys[ki].qpos[7 + i];
            CHECK(fabsf(got - want) < 0.25f);
        }
    }
    printf("  pinned 1s root_y=%.3f contacts=%d\n",
        w.bodies[spawn.root].position.y, w.contact_count);
    mjcf_free(&model);
}

int main(void) {
    printf("mjcf/stl tests\n");
    test_stl_ascii();
    test_stl_binary();
    test_stl_zup();
    test_mjcf_microduck();
    printf("all passed\n");
    return 0;
}
