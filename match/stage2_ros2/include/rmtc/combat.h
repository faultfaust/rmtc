#pragma once
#include <vector>
#include "rmtc/protocol.h"
#include "rmtc/world.h"
#include "rmtc/radar.h"
#include "rmtc/planner.h"

// 基础战斗层（第二阶段初稿）：三段模型
//   1) 算本 Tick 承伤 N_n（不防御口径）；
//   2) 致命判定：N_n ≥ HP 时只有 DEFEND 能改变本 Tick 结果；
//   3) 否则：能稳杀就杀、血厚就换血、危险就撤/防。
//
// 规则依据（v1.2）：
//   * 攻击按移动前坐标结算（§5.2 第 4 步），所以 MOVE 不能躲开本 Tick 的攻击；
//   * 普通 20 / 防御受击 10 / 射程直线 1–2；
//   * 保护期不能攻击、免疫普通攻击（§7）。
namespace rmtc {

class Combat {
public:
    struct Params {
        bool enabled = true;        // 关掉后完全交还规划器（便于 A/B 与单测）
        int min_damage = 1;          // 期望伤害 ≥ 该值才出手（保护目标为 0 → 不打）
        int defend_hp = 40;          // 被威胁且 hp ≤ 该值 → 防御/撤离
        int save_hp = 20;            // 保留字段（初稿的濒死线，当前由 N_n 判定替代）
        int attack_fresh_age = 0;    // 攻击目标要求的目击年龄（0 = 必须本 Tick 亲眼看到）
        // 承伤评估只认"本 Tick 亲眼看到"的敌人：
        // 攻击按移动前坐标结算，敌人只要这 Tick 真能打到我，就一定在视野里
        // （射程 1–2 << 视野 4）。用陈旧记忆算承伤会导致"对着空气防御"，
        // 实测在蹲守对手场景里出现连续 334 Tick 的防御。
        int threat_fresh_age = 0;
        int attack_hp_margin = 0;    // 换血时需要比承伤多出的血量余量
        bool retreat = true;         // 危险时优先撤离（横移出轴）而不是原地防御

        // ---- 可选：1 步矩阵博弈（旧价值函数，负结果保留复现用）----
        // 0 = 关闭，用上面的启发式；1 = 对"我的动作 × 对手动作"做期望收益比较。
        // 对手动作只取两种：本 Tick 能打就打（概率 opp_attack_prob），否则朝我靠近。
        // 复杂度：|我的动作| ≤ 7 × 2 ≈ 14 次评估，单次 O(敌人数)，仍是微秒级。
        int lookahead = 0;
        float opp_attack_prob = 0.8f;  // 对手选择攻击的先验概率

        // ---- 可选：多步期望最大搜索（新价值函数，depth = 模拟的 Tick 数）----
        // 0 = 关闭；1–5 = 每层完整模拟一个 Tick（我的最优动作 × 对手战术模型的期望）。
        // 价值函数按实际计分口径：0.03 分/D、击杀 +1.8、死亡成本随当前 D 变化；
        // 不再有裸距离项（旧版失败根源），位置只通过"同轴威胁"起作用。
        // depth≥4 启用置换表 + 节点预算（确定性），深层剪"等待"控分支。
        int depth = 0;

        // ---- 盲点修复开关（Config 同步，A/B 用）----
        bool opp_cost = true;      // 每 Tick 扣采集机会成本 rAlt
        bool ray_block = true;     // 非交战敌车的射线格视为不可进入
        float opp_cost_cap = 1.2f; // rAlt 上限（分/Tick）
        // 排名口径的相对价值（v4.4 §6.2/§18.4）：击杀计入对手 60D 的相对损失，
        // 死亡/击杀按当前 D 的计分边际计价（D≥500 后我方边际为 0）。
        // false = 旧口径（击杀只计我方增益；D 封顶后死亡仍按 1.8 计）。
        bool rank_value = false;
        // 有效伤害 ≥500 后停止“无收益主动攻击”；可击杀/必死换命仍放行。
        bool d_cap_gate = false;

        // ---- v1.2 伤害任务未完成时的接战强化 ----
        // Agent 每 Tick 根据结构化 missions 设置；只在 pending 时生效。
        bool damage_mission = false;
        int damage_defend_hp = 20;        // 替代 defend_hp
        int damage_attack_margin = -5;    // 替代 attack_hp_margin
        bool damage_no_retreat = true;    // true=危险时也不主动撤退

        // ---- v1.2 §8.8 / §8.9 事件伤害修饰 ----
        // thorns_active：本 Tick 洁身自好生效；攻击正在防御的敌人时
        //   目标免伤、自己受 20，expectedDamage 必须按 0 估。
        // self_vulnerable：本 Tick 自己处于易伤；普通受击 40、防御受击 20。
        bool thorns_active = false;
        bool self_vulnerable = false;
        // 本 Tick 至少有一个可见敌人处于易伤；深层搜索没有逐目标易伤，
        // 此时同样退回启发式，由 expectedDamage 按目标结算 40。
        bool enemy_vulnerable = false;
    };

    void configure(const Params& p) { params_ = p; }
    const Params& params() const { return params_; }
    // D 值止盈实际拦下的主动攻击次数（评测器机制计数；生产不读）。
    int dcapBlocks() const { return dcap_blocks_; }

    // rAlt：采集机会成本（分/Tick）= Planner 采集计划的边际得分率，由 Agent
    // 缓存传入（战斗先于规划运行，用的是上一 Tick 的率——采集计划变化慢，可接受）。
    // 0 表示无机会成本（规划器没跑过/已饱和）。
    // 返回 true 时 out 已被填成 MOVE_ATTACK / MOVE_DEFEND / MOVE_MOVE(撤离)；
    // 返回 false 表示"不插手"，由 Agent 继续调用规划器。
    bool decide(const World& w, const Radar& radar, const Perception& p,
                Decision& out, double rAlt = 0.0);

private:
    // 沿方向 dir 扫描直线 1–2 格的最近敌车（障碍不阻挡，最近者挡住后方）。
    const Radar::Sighting* scanDirection(const World& w, const Radar& radar,
                                         const Perception& p, int dir, int& dmgOut) const;

    // 本 Tick 预计承受的攻击伤害之和（不防御口径）。
    int incomingDamage(const Radar& radar, const Perception& p) const;

    // 撤离方向：离最近敌人最远，且尽量脱离对方的行/列（§6.6）。
    int retreatDir(const World& w, const Radar& radar, const Perception& p) const;

    // 1 步矩阵博弈（可选）：返回 true 表示已在 out 里给出动作。
    bool lookaheadDecide(const World& w, const Radar& radar, const Perception& p,
                         Decision& out);

    // 多步期望最大搜索（可选，Params::depth > 0）：新价值函数。
    bool deepDecide(const World& w, const Radar& radar, const Perception& p,
                    Decision& out, double rAlt);

    // deepDecide 的递归主体（dsSettle/dsTerminal 在匿名命名空间）。

    // 实际决策（decide 的包装负责维护连续防御计数）
    bool decideImpl(const World& w, const Radar& radar, const Perception& p,
                    Decision& out, double rAlt);

    // 记录本 Tick 最终动作，维护"连续防御"计数（防止无限防御对峙）。
    void track(const Decision& d) {
        if (d.kind == MOVE_DEFEND) ++defend_streak_;
        else defend_streak_ = 0;
    }

    int expectedDamage(const Radar::Sighting& s, int tick) const;

    Params params_;
    int defend_streak_ = 0;  // 连续防御次数（>=2 时强制换动作）
    int dcap_blocks_ = 0;    // D 值止盈拦下的攻击次数（评测器计数）
    std::vector<unsigned char> ray_danger_;  // 多敌射线格缓存（避免每次分配）
};
}  // namespace rmtc
