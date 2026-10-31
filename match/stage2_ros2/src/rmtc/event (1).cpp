// rmtc/event.cpp —— 事件量化评分
#include "rmtc/event.h"

#include <cmath>
#include <cstdio>

namespace rmtc {

namespace {
const double kPtsPerD = 0.03;      // 1 点 D
const double kDeathCost = 4.5;     // 一次死亡的综合损失

inline int iabs(int v) { return v < 0 ? -v : v; }

bool inRect(int x, int y, const EventReport& r) {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

// 离开矩形所需的最少步数（已在外面返回 0）。
int escapeSteps(int x, int y, const EventReport& r) {
    if (!inRect(x, y, r)) return 0;
    const int dl = x - r.x0 + 1;
    const int dr = r.x1 - x + 1;
    const int du = y - r.y0 + 1;
    const int dd = r.y1 - y + 1;
    int m = dl;
    if (dr < m) m = dr;
    if (du < m) m = du;
    if (dd < m) m = dd;
    return m;
}

// 曼哈顿距离（用于判断"值不值得跑过去"）。
int manhattan(int x0, int y0, int x1, int y1) {
    return iabs(x0 - x1) + iabs(y0 - y1);
}
}  // namespace

const char* eventKindName(EventKind k) {
    switch (k) {
        case EVK_EXPLOSION: return "explosion";
        case EVK_ILLUSION: return "illusion";
        case EVK_HEAL: return "heal";
        case EVK_FORTRESS: return "fortress";
        case EVK_SPLIT: return "split";
        case EVK_PEPPER: return "pepper";
        case EVK_THORNS: return "thorns";
        case EVK_VULNERABLE: return "vulnerable";
        default: return "unknown";
    }
}

EventAdvice EventScorer::score(const EventReport& r, const Perception& p,
                               const World& w) const {
    (void)w;  // 目前只需要 self 与时间；保留 World 以便后续做可达性/几何过滤
    EventAdvice a;
    if (r.kind == EVK_UNKNOWN || r.confidence < min_confidence_) return a;
    if (!p.alive) return a;

    const int tick = p.tick;
    const int R = r.activate_tick >= 0 ? r.activate_tick - tick : 0;  // 距生效
    const bool upcoming = r.activate_tick < 0 || tick < r.activate_tick;
    const bool active = r.activate_tick >= 0 && tick >= r.activate_tick &&
                        (r.end_tick < 0 || tick < r.end_tick);
    const bool inside = inRect(p.x, p.y, r);

    switch (r.kind) {
        case EVK_EXPLOSION: {
            // 瞬时事件：activate_tick 那一 Tick 之后爆炸区已经不存在。
            // 只靠 end_tick 判断会把 end=-1 的旧公告当成永久危险区
            // （实测：A 在 t=348 被 t=141 的幽灵爆炸锁在 x=3，reach 无法完成）。
            if (r.activate_tick >= 0 && tick >= r.activate_tick) break;
            // 只有"在矩形内且来不及走"才真正致命；能走也必须撤离（§5.4：
            // 撤离 deadline 前完成），只是代价小于来不及的情形。
            if (inside) {
                const int need = escapeSteps(p.x, p.y, r);
                const bool canEscape = upcoming && R > need;
                const bool lethal = p.hp <= 80 && !p.protected_;
                a.avoid = true;
                if (!canEscape) {
                    a.value = lethal ? -kDeathCost : -1.2;
                    a.reason = lethal ? "inside blast, lethal" : "inside blast";
                } else {
                    a.value = -0.2;
                    a.reason = "inside blast, can escape";
                }
            }
            break;
        }
        case EVK_ILLUSION: {
            // 方向反转可以用 last_action 自行补偿，分值中性。
            a.reason = "illusion (compensated locally)";
            break;
        }
        case EVK_HEAL: {
            if (!upcoming && !active) break;
            if (active && r.end_tick >= 0 && tick >= r.end_tick) break;
            const int gainHp = 100 - p.hp;
            if (gainHp <= 0) break;
            const int d = manhattan(p.x, p.y, r.x0, r.y0);
            // 每次入点约 10 Tick × 10 HP；按"每点 HP 值多少分"折价，再按路程打折。
            double v = static_cast<double>(gainHp > 60 ? 60 : gainHp) * 0.02;
            if (p.hp <= 50) v += 0.8;  // 低血时回血等于降低死亡概率
            v -= d * 0.05;
            if (v > 0.3) {
                a.value = v;
                a.has_target = true;
                a.tx = r.x0;
                a.ty = r.y0;
                a.reason = "heal point";
            }
            break;
        }
        case EVK_FORTRESS: {
            if (!upcoming && !active) break;
            const int cx = (r.x0 + r.x1) / 2;
            const int cy = (r.y0 + r.y1) / 2;
            const int d = manhattan(p.x, p.y, cx, cy);
            // 无双的价值取决于 D 空间：D 越接近 500 越不值；低血时风险太高。
            const double dSpace = 500.0 - static_cast<double>(p.effective_damage);
            if (dSpace <= 0.0 || p.hp <= 40) {
                a.value = -1.0;
                a.reason = "fortress not worth the risk";
                break;
            }
            const double gainD = dSpace < 150.0 ? dSpace : 150.0;
            double v = gainD * kPtsPerD - d * 0.05;
            if (v > 1.0) {
                a.value = v;
                a.has_target = true;
                a.tx = cx;
                a.ty = cy;
                a.reason = "fortress buff";
            }
            break;
        }
        case EVK_SPLIT: {
            // 十字线：x==x0 或 y==y0。线上必死（无视保护/防御）。
            const bool onLine = (p.x == r.x0) || (p.y == r.y0);
            if (onLine && upcoming) {
                const int need = 1;  // 横移一格即可离轴
                const bool canEscape = R > need;
                a.avoid = true;
                a.value = canEscape ? -0.3 : -kDeathCost;
                a.reason = canEscape ? "on split line, can step off" : "on split line, lethal";
            } else if (onLine) {
                a.value = -kDeathCost;
                a.reason = "on split line";
            }
            break;
        }
        case EVK_PEPPER: {
            if (!upcoming && !active) break;
            // 触发者站在交点上免疫；真正的杀伤价值取决于"线上有谁"，
            // 而那属于感知层信息（敌车位置），这里只给踩点机会的基准分：
            // 踩点本身可用于逼位/封路，值 0.8；命中另由上层按敌车 HP 折算。
            double v = 0.8;
            const int d = manhattan(p.x, p.y, r.x0, r.y0);
            v -= d * 0.04;
            if (v > 0.2) {
                a.value = v;
                a.has_target = true;
                a.tx = r.x0;
                a.ty = r.y0;
                a.reason = "pepper trigger point";
            }
            break;
        }
        case EVK_THORNS: {
            // 反伤持续期间：防御的性价比上升（防守方反伤 20）。
            if (p.hp <= 50) {
                a.value = 0.6;
                a.reason = "thorns: defend is cheaper";
            }
            break;
        }
        case EVK_VULNERABLE: {
            // 自己中招 → 避战；对手中招的信息不在事件报告里（由感知判断）。
            // 这里只给"自己易伤"的负分，提醒上层不要主动接战。
            a.value = -1.0;
            a.reason = "vulnerable: avoid trades";
            break;
        }
        default:
            break;
    }
    return a;
}

}  // namespace rmtc
