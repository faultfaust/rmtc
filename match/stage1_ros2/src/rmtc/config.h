#pragma once
namespace rmtc {
    struct Config {
        int total_ticks = 100;
        float total_value = 200.f;
        int vision_max = 4;
        float risk_weight = 2.0f;
        bool allow_unknown_travel = true;
        int heal_level = 2;
        int fail_limit = 2;
        int strict_finish_ticks = 6;
        float explore_weight = 0.12f;
        float explore_horizon = 12.f;
        int explore_stop_R = 25;
        bool dynamic_stop = true;
        int collect_margin = 4;
        int min_stop = 6;
        int max_stop = 25;
        int explore_full_R = 45;
        int endgame_R = 15;
        bool plan_level_ratio = true;
        float switch_ratio = 1.15f;
        int submit_max = 10;
        bool use_frontier = true;
        int min_region = 3;
        float region_gamma = 0.5f;
        float region_ref = 20.f;
        float corridor_gain = 0.5f;
        int min_gain = 1;
        float res_age_bonus = 0.1f;
        float age_scale = 20.f;
        float high_value = 20.f;
        float high_priority = 1.2f;
        float region_traversal = 1.f;
        bool exact_op = true;
        int exact_max_n = 12;
        int exact_max_cells = 400;

        // ---- [FIX] 语雀实测丢分问题的修复开关（500图配对A/B选定 v8=A/C/D/F/G）----
        // B(fix_lambda) 与 E(fix_riskdist) 在锦标赛中收益不稳（vmaze/pillars 倒退），默认关闭；
        // lam_ 仍随行程统计自适应更新，供 C 的末盘滤除与 G 的探索加价使用。
        bool fix_never_idle = true;   // A: 无可选目标时也朝最优已知资源/最近未知推进，不原地等待
        bool fix_lambda = false;      // B: 全局预算按 λ 收缩（输掉 A/B，关）
        bool fix_endgame = true;      // C: 末盘滤除放宽——safe 可达 或 λ 放大后软可达，任一成立即保留候选
        bool fix_enclosed = true;     // D: 被墙围死（封死连通域）的资源剔除出候选并修正 hidden
        bool fix_riskdist = false;    // E: 收集巡游距离按未知格加价（输掉 A/B，关）
        bool fix_abort = true;        // F: 行程突变中止——实际距离远超预估时弃目标重规划
        bool fix_explore_eff = true;  // G: 探索成本用风险加权距离
        float lam_init = 1.15f;       // lam 初值（行程 EMA，范围 [1,3]）
        float risk_step_cost = 1.0f;  // 每个 unknown 格的风险加价步数（乘 (λ-1) 后生效）
        float abort_factor = 1.6f;    // [FIX-F] 实际距离 > 预估*abort_factor + 3 时中止行程
    };
} // namespace rmtc
