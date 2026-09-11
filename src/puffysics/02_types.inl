// ==== puffysics 02_types.inl: INTERNAL: enum constants, math/body/shape/contact/joint/world structs ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
#define B3_STATIC 0
#define B3_KINEMATIC 1
#define B3_DYNAMIC 2

#define B3_SPHERE 0
#define B3_CAPSULE 1
#define B3_BOX 2

#define B3_JOINT_WELD 0
#define B3_JOINT_REVOLUTE 1

#define B3_FLAG_DYNAMIC 0x00001000u
#define B3_LOCK_LIN_X 0x00000001u
#define B3_LOCK_LIN_Y 0x00000002u
#define B3_LOCK_LIN_Z 0x00000004u
#define B3_LOCK_ANG_X 0x00000008u
#define B3_LOCK_ANG_Y 0x00000010u
#define B3_LOCK_ANG_Z 0x00000020u

typedef struct B3Vec3 {
    float x, y, z;
} B3Vec3;

typedef struct B3Vec2 {
    float x, y;
} B3Vec2;

typedef struct B3Quat {
    B3Vec3 v;
    float s;
} B3Quat;

typedef struct B3Mat3 {
    B3Vec3 cx, cy, cz;
} B3Mat3;

typedef struct B3Mat2 {
    B3Vec2 cx, cy;
} B3Mat2;

typedef struct B3AABB {
    B3Vec3 lo, hi;
} B3AABB;

typedef struct B3Soft {
    float bias_rate;
    float mass_scale;
    float impulse_scale;
} B3Soft;

typedef struct B3Body {
    B3Vec3 position;
    B3Quat rotation;
    B3Vec3 center;
    B3Vec3 local_center;
    B3Vec3 lin_vel;
    B3Vec3 ang_vel;
    B3Vec3 force;
    B3Vec3 torque;
    B3Vec3 delta_pos;
    B3Quat delta_rot;
    float inv_mass;
    B3Vec3 inv_inertia;
    B3Mat3 inv_i_world;
    float linear_damping;
    float angular_damping;
    float gravity_scale;
    int type;
    uint32_t flags;
} B3Body;

typedef struct B3Shape {
    int body;
    int type;
    B3Vec3 local_pos;
    B3Quat local_rot;
    float radius;
    B3Vec3 half;
    float friction;
    float restitution;
    float rolling;
    float density;
    uint64_t category;
    uint64_t mask;
} B3Shape;

typedef struct B3Point {
    B3Vec3 r_a;
    B3Vec3 r_b;
    float base_sep;
    float rel_vel;
    float normal_impulse;
    float total_normal;
    float normal_mass;
    float lever;
    uint32_t feature;
} B3Point;

typedef struct B3Contact {
    int shape_a;
    int shape_b;
    int body_a;
    int body_b;
    int point_count;
    int static_contact;
    B3Vec3 normal;
    B3Vec3 tangent1;
    B3Vec3 tangent2;
    B3Point points[B3_MAX_MANIFOLD];
    B3Vec3 center_a;
    B3Vec3 center_b;
    float friction;
    float restitution;
    float rolling;
    float twist_mass;
    float twist_impulse;
    B3Vec2 friction_impulse;
    B3Vec3 rolling_impulse;

    B3Mat2 tangent_mass;
    B3Mat3 rolling_mass;
    float inv_mass_a;
    float inv_mass_b;
    B3Mat3 inv_i_a;
    B3Mat3 inv_i_b;
    B3Soft softness;
} B3Contact;

typedef struct B3Warm {
    int shape_a;
    int shape_b;
    int point_count;
    uint32_t feature[B3_MAX_MANIFOLD];
    float normal_impulse[B3_MAX_MANIFOLD];
    B3Vec2 friction_impulse;
    float twist_impulse;
    B3Vec3 rolling_impulse;
} B3Warm;

typedef struct B3Joint {
#ifndef B3_REVOLUTE_ONLY
    int type;
#endif
    int body_a;
    int body_b;
    int collide_connected;
    int fixed_rotation;
    B3Vec3 local_anchor_a;
    B3Vec3 local_anchor_b;
    B3Quat local_rot_a;
    B3Quat local_rot_b;
    float constraint_hertz;
    float constraint_damping;
    B3Soft softness;
    float inv_mass_a;
    float inv_mass_b;
    B3Mat3 inv_i_a;
    B3Mat3 inv_i_b;
    B3Vec3 frame_p_a;
    B3Vec3 frame_p_b;
    B3Quat frame_q_a;
    B3Quat frame_q_b;
    B3Vec3 delta_center;
    B3Vec3 linear_impulse;
#ifndef B3_REVOLUTE_ONLY
    B3Vec3 angular_impulse;
    B3Mat3 angular_mass;
    float linear_hertz;
    float linear_damping;
    float angular_hertz;
    float angular_damping;
    B3Soft linear_spring;
    B3Soft angular_spring;
#endif
    B3Vec2 perp_impulse;
    float spring_impulse;
#ifndef B3_REVOLUTE_ONLY
    float motor_impulse;
#endif
    float lower_impulse;
    float upper_impulse;
    float hertz;
    float damping_ratio;
    float max_motor_torque;
#ifndef B3_REVOLUTE_ONLY
    float motor_speed;
#endif
    float target_angle;
    float lower_angle;
    float upper_angle;
    int enable_spring;
#ifndef B3_REVOLUTE_ONLY
    int enable_motor;
#endif
    int enable_limit;
    B3Vec3 rotation_axis;
    B3Vec3 perp_x;
    B3Vec3 perp_y;
    float axial_mass;
    B3Soft spring_softness;
#ifdef B3_CACHE_JOINTS
    /* Geometry + inv(K) for one GS pass. K is constant while delta_rot
     * is frozen, so invert once and matvec in the inner loop. */
    B3Vec3 cache_ra;
    B3Vec3 cache_rb;
    B3Mat3 cache_point_invk;
    B3Mat2 cache_ang_invk;
    B3Vec3 cache_ia_ax;
    B3Vec3 cache_ib_ax;
    float cache_twist;
    float cache_rel_x;
    float cache_rel_y;
#endif
} B3Joint;

#ifdef B3_PACKED_GS
/* Inner-loop snapshot. Bodies keep v/w/Δp/Δq + dyn inv(M). Joints keep
 * impulses, cached inv(K), and the hinge rows. Contacts drop unused
 * prepare fields. inv_i lives on the body so 14 hinges do not each
 * carry two 3x3 copies. */
#define B3_GS_FIXED 1
#define B3_GS_SPRING 2
#define B3_GS_LIMIT 4

typedef struct B3GsBody {
    B3Vec3 lin_vel;
    B3Vec3 ang_vel;
    B3Vec3 delta_pos;
    B3Quat delta_rot;
    float inv_mass;
    B3Mat3 inv_i;
    uint32_t flags;
} B3GsBody;

typedef struct B3GsJoint {
    int body_a;
    int body_b;
    int bits;
    float target_angle;
    float lower_angle;
    float upper_angle;
    float axial_mass;
    float max_motor_torque;
    float spring_impulse;
    float lower_impulse;
    float upper_impulse;
    float cache_twist;
    float cache_rel_x;
    float cache_rel_y;
    B3Soft softness;
    B3Soft spring_softness;
    B3Vec3 rotation_axis;
    B3Vec3 perp_x;
    B3Vec3 perp_y;
    B3Vec3 cache_ra;
    B3Vec3 cache_rb;
    B3Vec3 cache_ia_ax;
    B3Vec3 cache_ib_ax;
    B3Vec3 delta_center;
    B3Vec3 linear_impulse;
    B3Vec2 perp_impulse;
    B3Mat3 cache_point_invk;
    B3Mat2 cache_ang_invk;
#if B3_COUPLED_HINGE
    B3Vec3 cache_point_perp_x;
    B3Vec3 cache_point_perp_y;
#endif
} B3GsJoint;

typedef struct B3GsPoint {
    B3Vec3 r_a;
    B3Vec3 r_b;
    float base_sep;
    float normal_impulse;
    float total_normal;
    float normal_mass;
    float lever;
} B3GsPoint;

typedef struct B3GsContact {
    int body_a;
    int body_b;
    int point_count;
    B3Vec3 normal;
    B3Vec3 tangent1;
    B3Vec3 tangent2;
    B3Vec3 center_a;
    B3Vec3 center_b;
    float friction;
    float rolling;
    B3Vec3 rolling_impulse;
    float twist_mass;
    float twist_impulse;
    B3Vec2 friction_impulse;
    B3Mat2 tangent_mass;
    B3Soft softness;
    B3GsPoint points[B3_MAX_MANIFOLD];
} B3GsContact;
#endif


typedef struct B3World {
    B3Vec3 gravity;
    float contact_hertz;
    float contact_damping;
    float contact_speed;
    float restitution_threshold;
    float max_linear_speed;
    int body_count;
    int shape_count;
    int contact_count;
    int joint_count;
    // Sticky capacity diagnostics. Set on overflow, cleared by
    // b3_world_init() / b3_clear_errors(). Creation returns -1 (NULL for
    // b3_add_joint) without writing out of bounds. Contact generation
    // counts dropped pairs in contacts_dropped and sets contact_overflow
    // instead of silently losing them. Part of the struct layout; capacities
    // stay compile-time (B3_MAX_*), so all TUs must use one build config.
    int body_overflow;
    int shape_overflow;
    int joint_overflow;
    int contact_overflow;
    int contacts_dropped;
    B3Body bodies[B3_MAX_BODIES];
    B3Shape shapes[B3_MAX_SHAPES];
    B3Contact contacts[B3_MAX_CONTACTS];
    B3Joint joints[B3_MAX_JOINTS];
} B3World;

// Step/capacity diagnostics snapshot. contacts_dropped counts pairs with a
// non-empty manifold rejected because contact storage was full.
typedef struct B3StepStats {
    int body_count;
    int shape_count;
    int contact_count;
    int joint_count;
    int body_overflow;
    int shape_overflow;
    int joint_overflow;
    int contact_overflow;
    int contacts_dropped;
} B3StepStats;

typedef struct B3BodyDef {
    int type;
    B3Vec3 position;
    B3Quat rotation;
    B3Vec3 lin_vel;
    B3Vec3 ang_vel;
    float linear_damping;
    float angular_damping;
    float gravity_scale;
    uint32_t flags;
} B3BodyDef;

typedef struct B3ShapeDef {
    float density;
    float friction;
    float restitution;
    float rolling;
    uint64_t category;
    uint64_t mask;
} B3ShapeDef;
