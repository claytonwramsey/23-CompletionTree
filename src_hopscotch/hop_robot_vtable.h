#pragma once
// Runtime dispatch to hopcxx_{panda,ur5,pr2}_* -- avoids templating every
// caller over a robot type; the ported domains only need to pick a robot tag
// once per scenario.
#include "hop_bench_cxx.h"

enum class RobotTag { Panda, Ur5, Pr2 };

// The resumable motion-planning handle is typed per-robot on the Rust side
// (hopcxx_panda_Planning* vs hopcxx_ur5_Planning* vs hopcxx_pr2_Planning*),
// but RobotVtable needs one uniform function-pointer shape to dispatch on a
// runtime RobotTag -- so it's type-erased to `void*` here via small
// per-robot trampoline functions (below) rather than in hop-bench-cxx
// itself, keeping the real FFI signatures properly typed for anyone calling
// a specific robot's functions directly.
struct RobotVtable {
    bool (*ik)(CPose, CConfig *);
    CPose (*fk)(CConfig);
    bool (*validate)(CConfig, const Environment *);
    bool (*validate_attached)(CConfig, const Environment *, float, CPose);
    CPose (*sample_rel_pose)();
    CPose (*sample_table_pose)(CTable);
    void *(*motion_plan_start)(CConfig, CConfig, const Environment *, float, bool, CPose);
    int (*motion_plan_resume)(void *, size_t, float *, size_t, size_t *);
    void (*motion_plan_free)(void *);
};

namespace hop_vtable_detail {
#define HOPCXX_VTABLE_TRAMPOLINE(prefix)                                                           \
    inline void *prefix##_motion_plan_start(CConfig q_start, CConfig q_end,                        \
        const Environment *env, float block_r, bool has_held, CPose held_rel_pose) {               \
        return hopcxx_##prefix##_motion_plan_start(                                                \
            q_start, q_end, env, block_r, has_held, held_rel_pose);                                \
    }                                                                                              \
    inline int prefix##_motion_plan_resume(                                                        \
        void *handle, size_t n_samples, float *out_buf, size_t out_cap, size_t *out_len) {         \
        return hopcxx_##prefix##_motion_plan_resume(                                               \
            reinterpret_cast<hopcxx_##prefix##_Planning *>(handle), n_samples, out_buf, out_cap,   \
            out_len);                                                                              \
    }                                                                                              \
    inline void prefix##_motion_plan_free(void *handle) {                                          \
        hopcxx_##prefix##_motion_plan_free(                                                        \
            reinterpret_cast<hopcxx_##prefix##_Planning *>(handle));                               \
    }
HOPCXX_VTABLE_TRAMPOLINE(panda)
HOPCXX_VTABLE_TRAMPOLINE(ur5)
HOPCXX_VTABLE_TRAMPOLINE(pr2)
#undef HOPCXX_VTABLE_TRAMPOLINE
} // namespace hop_vtable_detail

inline const RobotVtable &robot_vtable(RobotTag tag) {
    static const RobotVtable panda {
        hopcxx_panda_ik,
        hopcxx_panda_fk,
        hopcxx_panda_validate,
        hopcxx_panda_validate_attached,
        hopcxx_panda_sample_rel_pose,
        hopcxx_panda_sample_table_pose,
        hop_vtable_detail::panda_motion_plan_start,
        hop_vtable_detail::panda_motion_plan_resume,
        hop_vtable_detail::panda_motion_plan_free,
    };
    static const RobotVtable ur5 {
        hopcxx_ur5_ik,
        hopcxx_ur5_fk,
        hopcxx_ur5_validate,
        hopcxx_ur5_validate_attached,
        hopcxx_ur5_sample_rel_pose,
        hopcxx_ur5_sample_table_pose,
        hop_vtable_detail::ur5_motion_plan_start,
        hop_vtable_detail::ur5_motion_plan_resume,
        hop_vtable_detail::ur5_motion_plan_free,
    };
    static const RobotVtable pr2 {
        hopcxx_pr2_ik,
        hopcxx_pr2_fk,
        hopcxx_pr2_validate,
        hopcxx_pr2_validate_attached,
        hopcxx_pr2_sample_rel_pose,
        hopcxx_pr2_sample_table_pose,
        hop_vtable_detail::pr2_motion_plan_start,
        hop_vtable_detail::pr2_motion_plan_resume,
        hop_vtable_detail::pr2_motion_plan_free,
    };
    switch (tag) {
    case RobotTag::Panda:
        return panda;
    case RobotTag::Ur5:
        return ur5;
    case RobotTag::Pr2:
        return pr2;
    }
    __builtin_unreachable();
}
