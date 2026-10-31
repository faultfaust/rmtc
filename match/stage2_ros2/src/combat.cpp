// rmtc/combat.cpp —— 三段模型（承伤 → 致命判定 → 攻击/防御/撤退）
//
// 数值全部来自规则 v1.2（§4.3）：
//   最大/初始生命 100、普通攻击 20、防御后受击 10、攻击范围 指定方向直线 1–2 格。
//
// 关键机制（决定这一层为什么必须这样写）：
//   * 攻击按**移动前**坐标结算（§5.2 第 4 步在移动之前）——所以本 Tick 的 MOVE
//     不能躲开本 Tick 的攻击，只有 DEFEND 能改变结果；
//   * 双方同 Tick 同时结算 —— "我先打死他"不能撤销他已经打出的伤害，
//     所以"能杀"不等于"能活着拿击杀奖励"。
#include "rmtc/combat.h"

#include <unordered_map>

namespace rmtc {

namespace {
// 与 planner.cpp 的方向编号保持一致：1=right 2=left 3=down 4=up
const int kDx[5] = {0, 1, -1, 0, 0};
const int kDy[5] = {0, 0, 0, 1, -1};

const int kDamageNormal = 20;    // 普通目标受到的伤害
const int kDamageDefended = 10;  // 防御中目标受到的伤害（§6.3）

inline int iabs(int v) { return v < 0 ? -v : v; }
inline int imax(int a, int b) { return a > b ? a : b; }
}  // namespace

int Combat::expectedDamage(const Radar::Sighting& s, int tick) const {
    if (s.protected_) return 0;               // 出生保护：免疫普通攻击（§7）
    // §8.8：洁身自好期间攻击正在防御的敌人，目标不受这次伤害；
    // 攻击者自己受 20 反伤。这里按“本次攻击收益 0”处理，Combat 会改选
    // 防御/撤离/打非防御目标，避免按 10 伤害做亏本交换。
    if (params_.thorns_active && s.defending) return 0;
    int dmg = s.defending ? kDamageDefended : kDamageNormal;
    // §8.9：目标易伤时普通/无双伤害 ×2（防御目标 10→20）。
    if (s.vulnerable_until > tick) dmg *= 2;
    return dmg;
}

const Radar::Sighting* Combat::scanDirection(const World& w, const Radar& radar,
                                             const Perception& p, int dir,
                                             int& dmgOut) const {
    dmgOut = 0;
    int x = p.x;
    int y = p.y;
    for (int step = 1; step <= 2; ++step) {
        x += kDx[dir];
        y += kDy[dir];
        if (!w.inBounds(x, y)) break;  // 出图：这条线上不可能再有目标
        const Radar::Sighting* s =
            radar.at(x, y, p.tick, params_.attack_fresh_age);
        if (s) {
            // 命中最近的一个就停：即使它是保护目标（伤害 0），也挡住后面的目标。
            dmgOut = expectedDamage(*s, p.tick);
            return s;
        }
        // 这一格没有目标：障碍不阻挡普通攻击，继续扫第 2 格。
    }
    return NULL;
}

// 本 Tick 预计承伤：所有"能在本 Tick 攻击到我"的敌车，名义伤害之和。
// 判据与结算一致：同行或同列、距离 1–2、不在出生保护中。
// 注意：这里用"不防御"的口径 N_n；防御后的承伤是 N_n / 2。
int Combat::incomingDamage(const Radar& radar, const Perception& p) const {
    int sum = 0;
    const std::vector<Radar::Sighting>& all = radar.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.protected_) continue;  // 保护期内不能 ATTACK
        if (s.last_tick < 0) continue;
        const int age = p.tick - s.last_tick;
        if (age < 0 || age > params_.threat_fresh_age) continue;
        if (s.x != p.x && s.y != p.y) continue;  // 必须同行或同列
        const int d = iabs(s.x - p.x) + iabs(s.y - p.y);
        if (d >= 1 && d <= 2) {
            // §8.9：自己易伤时普通受击 40（防御时 20，启发式用 N_n 的一半
            // 近似防御收益，倍率保持一致）。
            sum += params_.self_vulnerable ? (2 * kDamageNormal) : kDamageNormal;
        }
    }
    return sum;
}

// 撤退方向：离最近敌人最远，且尽量脱离对方的行/列（§6.6：横移出轴比后退有效）。
int Combat::retreatDir(const World& w, const Radar& radar, const Perception& p) const {
    int bestDir = 0;
    int bestScore = -1000000;
    const std::vector<Radar::Sighting>& all = radar.all();

    for (int dir = 1; dir <= 4; ++dir) {
        const int nx = p.x + kDx[dir];
        const int ny = p.y + kDy[dir];
        if (!w.inBounds(nx, ny)) continue;
        const int ni = w.idx(nx, ny);
        if (w.state()[ni] == World::BLOCKED) continue;  // 撞墙不叫撤退
        if (w.softBlocked(ni)) continue;                // 敌车占位

        int minDist = 999;
        bool offAxis = true;
        for (std::size_t i = 0; i < all.size(); ++i) {
            const Radar::Sighting& s = all[i];
            if (s.last_tick < 0) continue;
            const int age = p.tick - s.last_tick;
            if (age < 0 || age > 3) continue;  // 太旧的目击不参与方向选择
            const int d = imax(iabs(s.x - nx), iabs(s.y - ny));
            if (d < minDist) minDist = d;
            if (s.x == nx || s.y == ny) offAxis = false;
        }
        // 距离优先；同样距离时优先脱离射线
        const int score = minDist * 10 + (offAxis ? 3 : 0);
        if (score > bestScore) {
            bestScore = score;
            bestDir = dir;
        }
    }
    return bestDir;
}

// 包装：维护"连续防御"计数。连续防御 ≥2 时，decideImpl 会强制换动作，
// 这样即使双方都算出"防御更优"，也不会互相僵持到比赛结束。
bool Combat::decide(const World& w, const Radar& radar, const Perception& p,
                    Decision& out, double rAlt) {
    if (!params_.enabled) return false;
    const bool acted = decideImpl(w, radar, p, out, rAlt);
    if (acted) {
        if (out.kind == MOVE_DEFEND) ++defend_streak_;
        else defend_streak_ = 0;
    } else {
        defend_streak_ = 0;  // 交还规划器 = 本 Tick 没有防御
    }
    return acted;
}

bool Combat::decideImpl(const World& w, const Radar& radar, const Perception& p,
                        Decision& out, double rAlt) {
    // 可选：多步期望最大搜索（新价值函数）优先于旧 1 步实现。
    // thorns/vulnerable 会改变单次攻击/受击的实际伤害，而深层搜索内部仍按
    // 20/10 的名义值展开；这两种事件生效时强制走下面的启发式（它已按规则
    // 修正 expectedDamage / incomingDamage），避免深层搜索把亏本交换算成赚。
    const bool effect_risk = params_.thorns_active || params_.self_vulnerable ||
                             params_.enemy_vulnerable;
    if (!effect_risk && params_.depth > 0 &&
        deepDecide(w, radar, p, out, rAlt)) return true;

    // 可选：先跑 1 步矩阵博弈；关闭时完全走下面的启发式（与初稿一致）。
    if (!effect_risk && params_.lookahead > 0 &&
        lookaheadDecide(w, radar, p, out)) return true;

    // 连续防御 ≥2 时不再接受"纯防御"：有目标就打，没目标就交还规划器。
    // 这是对"同 Tick 互杀 → 无限防御"的确定性补丁（不引入随机数，A/B 可复现）。
    // thorns/vulnerable 生效时不强制打破防御：前者攻击防御目标=0 伤害+自己 20，
    // 后者换血按双倍承伤，强制出手会亏。
    const bool mustBreakDefense = defend_streak_ >= 2 && !effect_risk;
    // v1.2 伤害任务未完成：关闭 D 止盈、降低撤退/防御阈值、允许更小换血余量。
    const bool damageUrgent = params_.damage_mission;
    const int defendHp = damageUrgent ? params_.damage_defend_hp : params_.defend_hp;
    const int attackMargin =
        damageUrgent ? params_.damage_attack_margin : params_.attack_hp_margin;
    const bool allowRetreat =
        params_.retreat && !(damageUrgent && params_.damage_no_retreat);

    // ---- 1) 找最优攻击方向 ----
    // 保护期内不能 ATTACK（§7），连评估都不用做。
    int bestDir = 0;
    int bestDmg = 0;
    const Radar::Sighting* bestTarget = NULL;
    if (!p.protected_) {
        for (int dir = 1; dir <= 4; ++dir) {
            int dmg = 0;
            const Radar::Sighting* t = scanDirection(w, radar, p, dir, dmg);
            if (t && dmg > bestDmg) {
                bestDmg = dmg;
                bestDir = dir;
                bestTarget = t;
            }
        }
    }

    // ---- 2) 本 Tick 预计承伤（不防御口径）----
    const int Nn = p.protected_ ? 0 : incomingDamage(radar, p);
    // D 值止盈：500 后主动攻击不再加分，除非能击杀或必死换命。
    const bool dMaxed = params_.d_cap_gate && !damageUrgent &&
                        p.effective_damage >= 500.f;

    // ---- 3) 致命判定：Nn ≥ HP 时只有 DEFEND 能改变结果 ----
    if (Nn >= p.hp && Nn > 0) {
        const bool canKill = bestTarget && bestDmg >= bestTarget->hp;
        // 防御能活（Nn/2 < HP）才值得防御；
        // 能反杀时必须打：否则双方残血会各自"防御等对方先动"，形成无限对峙
        // （初稿实测出现过 14588 次 defend、双方都停在 20 HP）。
        if (Nn / 2 < p.hp) {
            if (canKill || (mustBreakDefense && bestTarget)) {
                out.kind = MOVE_ATTACK;
                out.direction = direction_name(bestDir);
                out.reason = R_ACT_ATTACK;
                out.target_id = bestTarget->id;
                out.damage = static_cast<float>(bestDmg);
                out.targetIdx = w.idx(bestTarget->x, bestTarget->y);
                return true;
            }
            out.kind = MOVE_DEFEND;
            out.reason = R_ACT_DEFEND;
            return true;
        }
        // 防不防都死（Nn/2 ≥ HP）：防御是纯浪费——本 Tick 0 收益且照样阵亡。
        // 按 §7.1 取本 Tick 收益最高的动作：能打就打（伤害/击杀奖励照拿），
        // 否则交还规划器按计划移动（不做无效的原地防御）。
        if (bestTarget) {
            out.kind = MOVE_ATTACK;
            out.direction = direction_name(bestDir);
            out.reason = R_ACT_ATTACK;
            out.target_id = bestTarget->id;
            out.damage = static_cast<float>(bestDmg);
            out.targetIdx = w.idx(bestTarget->x, bestTarget->y);
            return true;
        }
        return false;
    }

    // ---- 4) 能活下来：只有"杀掉且自己活下来"才主动打 ----
    if (bestTarget && bestDmg >= params_.min_damage && !p.protected_) {
        const bool canKill = bestDmg >= bestTarget->hp;
        if (canKill && p.hp > Nn) {
            // 我打死他，而且他的反击打不死我 —— 这是唯一稳赚的交换
            out.kind = MOVE_ATTACK;
            out.direction = direction_name(bestDir);
            out.reason = R_ACT_ATTACK;
            out.target_id = bestTarget->id;
            out.damage = static_cast<float>(bestDmg);
            out.targetIdx = w.idx(bestTarget->x, bestTarget->y);
            return true;
        }
        if (!canKill && p.hp > Nn + attackMargin) {
            // 血够厚，换血不吃亏；D≥500 时这条不再成立（打了也不加分）。
            if (dMaxed) {
                ++dcap_blocks_;
            } else {
                out.kind = MOVE_ATTACK;
                out.direction = direction_name(bestDir);
                out.reason = R_ACT_ATTACK;
                out.target_id = bestTarget->id;
                out.damage = static_cast<float>(bestDmg);
                out.targetIdx = w.idx(bestTarget->x, bestTarget->y);
                return true;
            }
        }
        // 其余情况（残血对残血、或换血会亏）落到下面走保命分支
    }

    // ---- 5) 被威胁且血量危险：先撤，撤不动再防 ----
    // 只有"本 Tick 亲眼看到"的威胁才撤退：看不到攻击者时不知道该往哪躲，
    // 原地防御更稳（也避免被陈旧记忆带偏方向）。
    if (Nn > 0 && !p.protected_ && p.hp <= defendHp) {
        const bool fresh = radar.threatensCell(p.x, p.y, p.tick, 0);
        // 连续防御太久：有目标就换成攻击（打破互相等待），否则才继续撤/防。
        if (mustBreakDefense && bestTarget) {
            if (dMaxed && bestDmg < bestTarget->hp) {
                ++dcap_blocks_;
            } else {
                out.kind = MOVE_ATTACK;
                out.direction = direction_name(bestDir);
                out.reason = R_ACT_ATTACK;
                out.target_id = bestTarget->id;
                out.damage = static_cast<float>(bestDmg);
                out.targetIdx = w.idx(bestTarget->x, bestTarget->y);
                return true;
            }
        }
        if (allowRetreat && fresh) {
            const int dir = retreatDir(w, radar, p);
            if (dir != 0) {
                out.kind = MOVE_MOVE;
                out.direction = direction_name(dir);
                out.reason = R_MOVE_RETREAT;
                return true;
            }
        }
        out.kind = MOVE_DEFEND;
        out.reason = R_ACT_DEFEND;
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// 1 步矩阵博弈（可选，Params::lookahead = 1）
//
// 动机：启发式只看"这一击能不能杀/会不会死"，不看对手的可能反应，也不给
// 位置留价值。这里把本 Tick 的决策写成一个小矩阵：
//     我的动作 ∈ {攻击, 防御, 4 个方向移动, 等待}
//     对手行为 ∈ {本 Tick 能打就打（概率 p）, 朝我靠近}
// 对每个组合按规则算一次即时收益（D 增量、击杀奖励、死亡成本、剩余距离），
// 取期望收益最大的动作。
//
// 复杂度：|A| ≤ 7，对手 2 种，单次评估 O(敌人数 ≤ 3)，总计 ≤ 42 次比较，
// 全是整数运算，微秒级；不需要每 Tick 跑满 3–5 层。
// 关键点：攻击按移动前坐标结算（§5.2），所以"我现在移动"不能躲开本 Tick 的
// 伤害——这一点已体现在 taken 与动作无关（除 DEFEND 减半）。
namespace {
const double kPointPerD = 0.03;      // 1 点 D 的分值（15/500）
const double kKillReward = 1.8;      // 60 D 的击杀奖励
const double kDeathCost = 4.5;       // 死亡：自身 -60 + 对手 +60 + 回场时间
const double kHpRisk = 0.02;         // 每点承伤的额外风险权重
const double kDistWeight = 0.05;     // 每格安全距离的价值（上限 4 格）
}  // namespace

bool Combat::lookaheadDecide(const World& w, const Radar& radar,
                             const Perception& p, Decision& out) {
    // 先找攻击机会（与启发式同一套扫描规则）
    int bestDir = 0;
    int bestDmg = 0;
    const Radar::Sighting* target = NULL;
    if (!p.protected_) {
        for (int dir = 1; dir <= 4; ++dir) {
            int dmg = 0;
            const Radar::Sighting* t = scanDirection(w, radar, p, dir, dmg);
            if (t && dmg > bestDmg) {
                bestDmg = dmg;
                bestDir = dir;
                target = t;
            }
        }
    }

    // 没有"本 Tick 亲眼看到"的敌人时不做战术比较，交还规划器（保证采集节奏）。
    bool anyFresh = false;
    const std::vector<Radar::Sighting>& all = radar.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick < 0) continue;
        const int age = p.tick - s.last_tick;
        if (age < 0 || age > 3) continue;
        const int d = imax(iabs(s.x - p.x), iabs(s.y - p.y));
        if (d <= 3) { anyFresh = true; break; }
    }
    if (!anyFresh) return false;

    const int Nn = p.protected_ ? 0 : incomingDamage(radar, p);
    const double pAttack = params_.opp_attack_prob;

    struct Cand {
        int kind;   // 0=attack 1=defend 2=move 3=wait
        int dir;
        double score;
    };
    Cand best;
    best.kind = -1;
    best.dir = 0;
    best.score = -1e18;

    // 评估一个候选：mode=0 对手攻击，mode=1 对手靠近（本 Tick 不造成伤害）
    for (int ci = 0; ci < 7; ++ci) {
        int kind = 3, dir = 0;
        int moveX = p.x, moveY = p.y;
        if (ci == 0) {
            if (bestDir == 0) continue;
            kind = 0;
            dir = bestDir;
        } else if (ci == 1) {
            // 连续防御 ≥2 后不再把防御列为候选（防止双方无限对峙）
            if (defend_streak_ >= 2) continue;
            kind = 1;
        } else if (ci <= 5) {
            kind = 2;
            dir = ci - 1;  // 1..4
            const int nx = p.x + kDx[dir];
            const int ny = p.y + kDy[dir];
            if (!w.inBounds(nx, ny)) continue;
            const int ni = w.idx(nx, ny);
            if (w.state()[ni] == World::BLOCKED) continue;
            if (w.softBlocked(ni)) continue;
            moveX = nx;
            moveY = ny;
        } else {
            kind = 3;
        }

        double expected = 0.0;
        for (int mode = 0; mode < 2; ++mode) {
            // 本 Tick 承伤：只有 mode=0 会挨打；DEFEND 减半；保护期免疫
            int taken = 0;
            if (mode == 0 && !p.protected_) taken = (kind == 1) ? Nn / 2 : Nn;
            // 本 Tick 输出
            int dealt = 0;
            bool kill = false;
            if (kind == 0) {
                dealt = bestDmg;
                kill = target && bestDmg >= target->hp;
            }
            const bool die = taken >= p.hp && p.hp > 0;

            // 下一 Tick 的最近距离：对手靠近时每 Tick 净接近 1 格
            int minDist = 9;
            for (std::size_t i = 0; i < all.size(); ++i) {
                const Radar::Sighting& s = all[i];
                if (s.last_tick < 0) continue;
                const int age = p.tick - s.last_tick;
                if (age < 0 || age > 3) continue;
                int d = imax(iabs(s.x - moveX), iabs(s.y - moveY));
                if (mode == 1) d = d > 0 ? d - 1 : 0;
                if (d < minDist) minDist = d;
            }
            if (minDist > 4) minDist = 4;

            double sc = kPointPerD * dealt;
            if (kill) sc += kKillReward;
            if (die) sc -= kDeathCost;
            sc -= kHpRisk * taken;
            sc += kDistWeight * minDist;
            expected += (mode == 0 ? pAttack : (1.0 - pAttack)) * sc;
        }

        if (expected > best.score) {
            best.score = expected;
            best.kind = kind;
            best.dir = dir;
        }
    }
    if (best.kind < 0) return false;

    if (best.kind == 0 && target) {
        out.kind = MOVE_ATTACK;
        out.direction = direction_name(best.dir);
        out.reason = R_ACT_ATTACK;
        out.target_id = target->id;
        out.damage = static_cast<float>(bestDmg);
        out.targetIdx = w.idx(target->x, target->y);
        return true;
    }
    if (best.kind == 1) {
        out.kind = MOVE_DEFEND;
        out.reason = R_ACT_DEFEND;
        return true;
    }
    if (best.kind == 2) {
        out.kind = MOVE_MOVE;
        out.direction = direction_name(best.dir);
        out.reason = R_MOVE_RETREAT;
        return true;
    }
    // 等待：交还规划器（让采集/探索继续，而不是空转）
    return false;
}

// ---------------------------------------------------------------------------
// 多步期望最大搜索（Params::depth = 1..3，新价值函数）
//
// 与旧版 1 步矩阵博弈（lookaheadDecide）的三个本质区别：
//   1. 价值函数全部锚定在实际计分上：1 D = 0.03 分、击杀奖励 60 D = 1.8 分、
//      死亡成本 = 1.8 + 0.03×min(60,D) + 0.9 回场时间（D 高时才贵，D=0 时便宜）。
//      **没有任何裸距离项**——旧版把"每格安全距离 ×0.05"乘在每个分支上，
//      双方同速时它必然把双方都推向拉远，于是谁也不打（D 从 500 掉到 73）。
//   2. 位置只用"同轴威胁"表达（规则 §6.2：攻击=同行/列 1–2 格）：
//      对齐 = 双方都能打到；不对称性来自血量差与可击杀性，不来自距离本身。
//   3. 对手不是"最坏情况"而是**战术模型**的期望：同轴时 {打 0.55/防 0.2/横移 0.25}，
//      不同轴时 {逼近 0.6/横移 0.25/停 0.15}。minimax 会高估对手、把自己逼回保守陷阱。
//
// 树结构（depth = 模拟的 Tick 数）：每层 = 我 max(≤7 个动作) × 对手期望(≤4 个动作)，
// 双方动作按规则在各自"移动前坐标"上同时结算，然后各自移动。
// depth=3 时叶子 ≤ (7×4)^3 ≈ 2.2 万，每叶 O(1)，实测 <2 ms；只在交战（敌 ≤4 格）时触发。
//
// 确定性：动作枚举顺序固定、严格 > 才更新；对手权重为常数 → 同输入同输出。
namespace {

// 搜索用的对局状态（只跟踪交战双方）
struct DS {
    int mx, my, mhp;   // 我
    int hx, hy, hhp;   // 交战对手
};

inline int dCheb(int ax, int ay, int bx, int by) {
    const int dx = ax > bx ? ax - bx : bx - ax;
    const int dy = ay > by ? ay - by : by - ay;
    return dx > dy ? dx : dy;
}

// 同轴且距离 1–2：双方都能命中对方（攻击范围对称）
inline bool dsAligned(const DS& s) {
    const int dx = s.hx - s.mx, dy = s.hy - s.my;
    if (dx != 0 && dy != 0) return false;
    const int ad = (dx != 0) ? (dx > 0 ? dx : -dx) : (dy > 0 ? dy : -dy);
    return ad >= 1 && ad <= 2;
}

// 深搜参数（按计分口径标定，见函数头注释）
const double kVD = 0.03;     // 1 点 D / 1 点对敌 HP 的分值
const double kVHp = 0.015;   // 我方 1 点 HP 的影子价格（约为 D 的一半）
const double kKill = 1.8;    // 击杀奖励 60 D
const double kRejoin = 0.9;  // 死亡回场的机会成本
const double kDiscount = 0.95;

// D 的计分价值：15·min(D,500)/500。用于排名口径下按当前 D 计算
// 击杀/死亡的**边际**分值（v4.4 §6.2：死亡成本在 D=560 才归零，
// 不是 D≥500 就免费；击杀在 D≥440 后也不再增加我方对抗分）。
inline double dsScoreOfD(double d) {
    if (d < 0.0) d = 0.0;
    if (d > 500.0) d = 500.0;
    return 15.0 * d / 500.0;
}

// 深搜上下文：节点计数（确定性熔断）+ 置换表（depth≥4 启用）+ 射线危险格。
// 全部保持确定性：节点按固定顺序计数，同输入必同截断；
// 置换表只做"同键同值"的精确复用，不引入任何近似。
struct SearchCtx {
    long long nodes = 0;
    long long budget = 0;
    std::unordered_map<unsigned long long, double>* memo = NULL;
    const unsigned char* ray = NULL;  // 非交战敌车的射线格（576 字节，非 0 = 危险）
};

// 结算一个 Tick 的双方动作，返回立即分值；deadM/deadH 标记死亡。
struct DStep {
    DS s;
    double imm;
    bool deadM, deadH;
};

DStep dsSettle(const DS& s, int myAct, int hisAct, double myDeathCost,
               double myKillReward, double myDmgValue,
               int myHitDmg, int hisHitDmg) {
    // 动作编码：0=attack 1=defend 2..5=move(方向 1..4) 6=wait
    DStep st;
    st.s = s;
    st.imm = 0.0;
    st.deadM = st.deadH = false;

    const bool aligned = dsAligned(s);
    int toHim = 0, toMe = 0;
    if (aligned) {
        if (myAct == 0) toHim = (hisAct == 1) ? 10 : myHitDmg;
        if (hisAct == 0) toMe = (myAct == 1) ? 10 : hisHitDmg;
    }
    st.s.hhp -= toHim;
    st.s.mhp -= toMe;
    st.imm += myDmgValue * toHim - kVHp * toMe;
    if (st.s.hhp <= 0) {
        st.s.hhp = 0;
        st.deadH = true;
        st.imm += myKillReward;
    }
    if (st.s.mhp <= 0) {
        st.s.mhp = 0;
        st.deadM = true;
        st.imm -= myDeathCost;  // 互杀时双方各计各的（规则允许）
    }
    // 死亡车辆本 Tick 不能移动（§5.2）；存活的按各自意图移动（忽略争格近似）
    if (!st.deadM && myAct >= 2 && myAct <= 5) {
        st.s.mx += kDx[myAct];
        st.s.my += kDy[myAct];
    }
    if (!st.deadH && hisAct >= 2 && hisAct <= 5) {
        st.s.hx += kDx[hisAct];
        st.s.hy += kDy[hisAct];
    }
    return st;
}

// 视界上的局面评估：只看同轴威胁与血量差，不看距离
double dsTerminal(const DS& s, int myHitDmg, int hisHitDmg);
double dsSearchInner(const World& w, const DS& s, int depthLeft, int totalDepth,
                     bool attackOk, double myDeathCost, double myKillReward,
                     double myDmgValue, int myHitDmg, int hisHitDmg,
                     int* bestAct, SearchCtx* ctx, double rAlt);

double dsTerminal(const DS& s, int myHitDmg, int hisHitDmg) {
    double v = 0.0;
    if (dsAligned(s)) {
        if (s.hhp <= myHitDmg) v += 1.2;   // 下 Tick 可击杀
        if (s.mhp <= hisHitDmg) v -= 1.2;  // 我可能被击杀
        v += 0.008 * (s.mhp - s.hhp);      // 血量领先 → 对换有利
    } else {
        // 不同轴：想打（血量领先或对方残血）就奖励接近，否则奖励远离。
        // 距离只在这里出现，且上限 4 格（±0.2），不会再主导排序。
        const int d = dCheb(s.mx, s.my, s.hx, s.hy);
        const bool wantFight = (s.mhp >= s.hhp) || (s.hhp <= myHitDmg);
        v += wantFight ? 0.05 * (4 - d) : 0.05 * (d - 4);
    }
    return v;
}

// rAlt = 采集机会成本（分/Tick）：存活的每个交战 Tick 都在放弃规划器的
// 边际采集收入，所以"活着留在交战里"的子树逐层递减 rAlt；死亡分支与
// 地平线不扣（交战已结束/超出视界）。这让"继续打 vs 脱离"在经济上可比。
double dsSearch(const World& w, const DS& s, int depthLeft, int totalDepth,
                bool rootAttackOk, double myDeathCost, double myKillReward,
                double myDmgValue, int myHitDmg, int hisHitDmg,
                int* bestAct, SearchCtx* ctx, double rAlt) {
    if (ctx) {
        ++ctx->nodes;
        // 确定性熔断（§13.9）：超过节点预算就把当前层当地平线处理。
        // 计数顺序完全确定，同输入必同截断。
        if (ctx->nodes > ctx->budget) return dsTerminal(s, myHitDmg, hisHitDmg);
    }
    if (depthLeft <= 0 || s.mhp <= 0 || s.hhp <= 0)
        return dsTerminal(s, myHitDmg, hisHitDmg);
    // 只有"当前 Tick"（depthLeft == totalDepth）受真实阻挡检查约束；
    // 未来 Tick 的对齐是模拟出来的，按干净直线近似。
    const bool attackOk = (depthLeft == totalDepth) ? rootAttackOk : true;

    // 置换表（仅深搜启用）：同一 (局面, 剩余深度) 的子树价值必然相同。
    // 局面键：双方位置(5b×4) + 双方 HP(7b×2) + depthLeft(3b) = 37b。
    // attackOk 只在根层（depthLeft==totalDepth）生效且根只有一个，不影响缓存。
    if (ctx && ctx->memo != NULL && depthLeft < totalDepth) {
        const unsigned long long key =
            (static_cast<unsigned long long>(s.mx) << 0) |
            (static_cast<unsigned long long>(s.my) << 5) |
            (static_cast<unsigned long long>(s.mhp) << 10) |
            (static_cast<unsigned long long>(s.hx) << 17) |
            (static_cast<unsigned long long>(s.hy) << 22) |
            (static_cast<unsigned long long>(s.hhp) << 27) |
            (static_cast<unsigned long long>(depthLeft) << 34);
        std::unordered_map<unsigned long long, double>::const_iterator it =
            ctx->memo->find(key);
        if (it != ctx->memo->end()) return it->second;
        const double v = dsSearchInner(w, s, depthLeft, totalDepth, attackOk,
                                       myDeathCost, myKillReward, myDmgValue,
                                       myHitDmg, hisHitDmg, NULL, ctx, rAlt);
        (*ctx->memo)[key] = v;
        return v;
    }
    return dsSearchInner(w, s, depthLeft, totalDepth, attackOk, myDeathCost,
                         myKillReward, myDmgValue, myHitDmg, hisHitDmg,
                         bestAct, ctx, rAlt);
}

// 搜索主体（dsSearch 的包装负责节点计数/熔断/置换表；这里只做一层展开）。
// deepPly = depthLeft < totalDepth：深层把"等待"从候选里去掉（交战中期等待
// 几乎不优，剪掉可把分支从 7×3 降到 ≤6×3，depth=5 才进得了时间预算）；
// 若剪完无合法动作则保留等待兜底。
double dsSearchInner(const World& w, const DS& s, int depthLeft, int totalDepth,
                     bool attackOk, double myDeathCost, double myKillReward,
                     double myDmgValue, int myHitDmg, int hisHitDmg,
                     int* bestAct, SearchCtx* ctx, double rAlt) {
    const bool deepPly = depthLeft < totalDepth;

    // ---- 对手动作集（战术模型）：0=attack 1=defend 2=approach 3=strafe 4=hold ----
    int hActs[5];
    double hW[5];
    int hn = 0;
    const bool aligned = dsAligned(s);
    if (aligned) {
        hActs[0] = 0; hW[0] = 0.55;
        hActs[1] = 1; hW[1] = 0.20;
        hActs[2] = 3; hW[2] = 0.25;
        hn = 3;
    } else {
        hActs[0] = 2; hW[0] = 0.60;
        hActs[1] = 3; hW[1] = 0.25;
        hActs[2] = 4; hW[2] = 0.15;
        hn = 3;
    }
    // 对手的 approach/strafe 具体方向在这里定死（确定性）：
    //   approach = 缩小主轴差；strafe = 垂直于我所在轴横移。
    // 编码到 5..8（= "move 方向 1..4" + 4 偏移）避免每层重复计算方向。
    int hMoves[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    {
        const int dx = s.mx - s.hx, dy = s.my - s.hy;
        int appDir = 0;
        const int adx = dx > 0 ? dx : -dx, ady = dy > 0 ? dy : -dy;
        if (adx >= ady) appDir = dx > 0 ? 1 : 2;      // 右/左
        else appDir = dy > 0 ? 3 : 4;                 // 下/上
        int strafeDir = 0;
        if (s.hx == s.mx) strafeDir = 4;              // 同列 → 向上横移
        else strafeDir = 1;                           // 否则向右
        // 合法性过滤（边界 + 静态墙）
        struct V { int d; };
        const int cand[2] = {appDir, strafeDir};
        for (int k = 0; k < 2; ++k) {
            const int d = cand[k];
            if (d == 0) { hMoves[k + 7] = 6; continue; }   // 无方向 → wait
            const int nx = s.hx + kDx[d], ny = s.hy + kDy[d];
            if (!w.inBounds(nx, ny) ||
                w.state()[w.idx(nx, ny)] == World::BLOCKED) {
                hMoves[k + 7] = 6;
            } else {
                hMoves[k + 7] = d + 4;  // 5..8 表示移动
            }
        }
    }

    // ---- 我的动作集：attack(需对齐) / defend / 4 个合法移动 / wait ----
    double best = -1e18;
    int bestA = 6;
    bool anyAct = false;
    for (int act = 0; act <= 6; ++act) {
        if (act == 0 && (!aligned || !attackOk)) continue;  // 不对齐/被挡住的攻击是空挥
        if (deepPly && act == 6 && anyAct) continue;        // 深层剪枝：不等待
        if (act >= 2 && act <= 5) {
            const int nx = s.mx + kDx[act], ny = s.my + kDy[act];
            if (!w.inBounds(nx, ny)) continue;
            const int ni = w.idx(nx, ny);
            if (w.state()[ni] == World::BLOCKED) continue;
            if (w.softBlocked(ni)) continue;
            if (nx == s.hx && ny == s.hy) continue;  // 对手占格（近似：不进）
            // 多敌射线（审查 §三 #1）：非交战敌车本 Tick 能打到的格子不可进入。
            // 深视界放大"走向 A 踏进 B 射线"的盲点，这里直接封死。
            if (ctx && ctx->ray != NULL && ctx->ray[ni]) continue;
        }
        anyAct = true;
        double ev = 0.0;
        for (int k = 0; k < hn; ++k) {
            int hisAct;
            if (hActs[k] <= 1) hisAct = hActs[k];
            else if (hActs[k] == 4) hisAct = 6;               // hold
            else hisAct = hMoves[hActs[k] + 6] == 6 ? 6 : (hActs[k] == 2 ? hMoves[7] : hMoves[8]);
            // hActs[k]==2 → approach → hMoves[7]；==3 → strafe → hMoves[8]
            const DStep st = dsSettle(s, act, hisAct, myDeathCost, myKillReward,
                                      myDmgValue, myHitDmg, hisHitDmg);
            double v = st.imm;
            if (!st.deadM && !st.deadH)
                v += kDiscount * dsSearch(w, st.s, depthLeft - 1, totalDepth,
                                          false, myDeathCost, myKillReward,
                                          myDmgValue, myHitDmg, hisHitDmg,
                                          NULL, ctx, rAlt);
            ev += hW[k] * v;
        }
        if (ev > best) {
            best = ev;
            bestA = act;
        }
    }
    // 采集机会成本：每个存活的交战 Tick 扣除规划器的边际采集收入
    // （rAlt=0 时无操作——开关关闭或尚无采集计划）。死亡/地平线分支不扣。
    if (rAlt > 0.0) best -= rAlt;
    if (bestAct) *bestAct = bestA;
    return best;
}

}  // namespace

bool Combat::deepDecide(const World& w, const Radar& radar, const Perception& p,
                        Decision& out, double rAlt) {
    // 出生保护期不参与战斗搜索（不能攻击也免疫，交给规划器移动）
    if (p.protected_) return false;

    // 选择交战对手：只认**本帧目击**（age==0）的敌人。
    // 攻击按本 Tick 移动前坐标结算，目标位置必须精确；陈旧目击的位置不可靠，
    // 拿它做出手/防御判断会"对着已经空了的位置"行动。
    const std::vector<Radar::Sighting>& all = radar.all();
    const Radar::Sighting* foe = NULL;
    int foeD = 99;
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Radar::Sighting& s = all[i];
        if (s.last_tick != p.tick || s.protected_) continue;
        const int d = dCheb(p.x, p.y, s.x, s.y);
        if (d <= 4 && d < foeD) {
            foeD = d;
            foe = &s;
        }
    }
    if (!foe) return false;  // 没在交战：交还规划器，别打扰采集

    // 攻击合法性：沿方向 1–2 格的**最近目击者**必须就是交战对手（§6.2：
    // 最近目标挡住后方，即使它是保护目标）。被挡住时本 Tick 不能出手。
    bool rootAttackOk = false;
    {
        const int dx = foe->x - p.x, dy = foe->y - p.y;
        int dir = 0;
        if (dx == 0 && dy != 0) dir = dy > 0 ? 3 : 4;
        else if (dy == 0 && dx != 0) dir = dx > 0 ? 1 : 2;
        if (dir != 0) {
            int x = p.x, y = p.y;
            for (int step = 1; step <= 2; ++step) {
                x += kDx[dir];
                y += kDy[dir];
                if (!w.inBounds(x, y)) break;
                const Radar::Sighting* first = radar.at(x, y, p.tick, 0);
                if (first) {
                    if (first == foe) rootAttackOk = true;
                    break;  // 最近者挡住后方：无论是不是它
                }
            }
        }
    }

    DS s0;
    s0.mx = p.x;
    s0.my = p.y;
    s0.mhp = p.hp;
    s0.hx = foe->x;
    s0.hy = foe->y;
    s0.hhp = foe->hp;

    // 命中口径：正常 20；对方易伤等增益不在搜索里建模（保守用名义值）
    const int myHitDmg = 20;
    const int hisHitDmg = 20;
    // 战斗价值口径（rank_value）：
    // - 旧口径：击杀只计我方 +1.8，死亡计"对手 +1.8 + 我方 1.8 + 回场 0.9"，
    //   因此一次一换一在价值函数里是 −2.1，系统性偏保守；且 D>500 后
    //   死亡仍按 1.8 计，与规则"D=560 才免费"不符。
    // - 排名口径：击杀 = 我方边际 + 对手 60D 的相对损失；死亡 = 我方边际 +
    //   对手 60D 的相对收益 + 回场。D 的边际用计分曲线，超 500 后为 0。
    double myDeathCost, myKillReward, myDmgValue;
    if (params_.rank_value) {
        const double d0 = p.effective_damage;
        const double dPrev = (d0 > 60.0) ? (d0 - 60.0) : 0.0;
        const double s0 = dsScoreOfD(d0);
        myDeathCost = kKill + (s0 - dsScoreOfD(dPrev)) + kRejoin;
        myKillReward = kKill + (dsScoreOfD(d0 + 60.0) - s0);
        myDmgValue = (d0 >= 500.0) ? 0.0 : kVD;
    } else {
        // 死亡成本随 D 变化：D=0 时只有相对损失 + 回场时间（早期接战更便宜）
        double dVal = p.effective_damage;
        if (dVal > 60.0) dVal = 60.0;
        myDeathCost = kKill + kVD * dVal + kRejoin;
        myKillReward = kKill;
        myDmgValue = kVD;
    }
    // v1.2 伤害任务未完成：D≥500 之后每点伤害仍可能决定任务是否完成，
    // 不能把边际价值降为 0。
    if (params_.damage_mission) myDmgValue = kVD;

    int depth = params_.depth;
    if (depth < 1) depth = 1;
    if (depth > 5) depth = 5;

    // 节点预算对所有深度生效（浅深度远用不到，设了也无妨）；
    // 置换表只在 depth≥4 启用（浅搜索开销小、重复局面少，省掉哈希开销）。
    // 预算按最坏分支（深层剪等待后 ≈18×3/层）估算：80 万节点 ≈ 单次百 ms 量级；
    // 超预算部分直接当地平线截断，同输入必同结果（确定性熔断 §13.9）。
    SearchCtx ctx;
    ctx.budget = 800000;
    std::unordered_map<unsigned long long, double> memo;
    if (depth >= 4) ctx.memo = &memo;

    // 多敌射线格（审查 §三 #1）：除交战对手外，其它本帧目击敌车能打到的
    // 直线 1–2 格标为危险，搜索里禁止移入。交战对手自己的射线由搜索内
    // dsAligned 动态建模，不在此列。
    if (params_.ray_block) {
        const int n = w.w() * w.h();
        if (static_cast<int>(ray_danger_.size()) != n) ray_danger_.assign(n, 0);
        std::fill(ray_danger_.begin(), ray_danger_.end(), 0);
        for (std::size_t i = 0; i < all.size(); ++i) {
            const Radar::Sighting& s = all[i];
            if (&s == foe) continue;
            if (s.last_tick != p.tick) continue;
            for (int dir = 1; dir <= 4; ++dir) {
                for (int step = 1; step <= 2; ++step) {
                    const int rx = s.x + kDx[dir] * step;
                    const int ry = s.y + kDy[dir] * step;
                    if (!w.inBounds(rx, ry)) break;
                    ray_danger_[w.idx(rx, ry)] = 1;
                }
            }
        }
        ctx.ray = ray_danger_.data();
    }

    int bestAct = 6;
    const double bestEv = dsSearch(w, s0, depth, depth, rootAttackOk, myDeathCost,
                                   myKillReward, myDmgValue, myHitDmg, hisHitDmg,
                                   &bestAct, &ctx, rAlt);

    // 采集机会成本（审查 §三 #2）：rAlt>0 时搜索已逐 Tick 扣除采集收入，
    // 这里把"最优战斗选项净期望"与"脱离交战去采集的收入"（Σ0.95^k≈4.2×rAlt）
    // 比较，净亏则交还规划器。
    // 实测（40 局×5 对手）：对避战流 +2.7、蹲守 +0.9、自博弈 +0.8，但对
    // 追击型（战术）−3.2——脱离收入被高估（被追打时根本采不到）。
    // 默认关闭，等 P2 对手模型能区分"逃跑 vs 追击"后再作为 decision_type 启用。
    if (params_.opp_cost && rAlt > 0.0 &&
        bestEv < rAlt * 4.2 - 0.3) {
        return false;
    }

    // 连续防御 ≥2 后不再接受纯防御（与启发式共用防僵持机制）
    if (bestAct == 1 && defend_streak_ >= 2) bestAct = 6;

    if (bestAct == 0) {
        // 攻击方向：对手同轴方向（距离 1 或 2 各试一次，取近的）
        int dx = s0.hx - s0.mx, dy = s0.hy - s0.my;
        int dir = 0;
        if (dx == 0) dir = dy > 0 ? 3 : 4;
        else dir = dx > 0 ? 1 : 2;
        out.kind = MOVE_ATTACK;
        out.direction = direction_name(dir);
        out.reason = R_ACT_ATTACK;
        out.target_id = foe->id;
        out.damage = static_cast<float>(foe->defending ? 10 : 20);
        out.targetIdx = w.idx(foe->x, foe->y);
        return true;
    }
    if (bestAct == 1) {
        out.kind = MOVE_DEFEND;
        out.reason = R_ACT_DEFEND;
        return true;
    }
    if (bestAct >= 2 && bestAct <= 5) {
        out.kind = MOVE_MOVE;
        out.direction = direction_name(bestAct);
        out.reason = R_MOVE_RETREAT;
        return true;
    }
    // wait：交还规划器（按计划采集/探索）
    return false;
}

}  // namespace rmtc
