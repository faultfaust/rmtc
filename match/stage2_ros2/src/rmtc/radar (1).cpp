// rmtc/radar.cpp —— 敌情目击记忆
#include "rmtc/radar.h"

namespace rmtc {

namespace {
inline int iabs(int v) { return v < 0 ? -v : v; }
}  // namespace

void Radar::reset() { enemies_.clear(); }

Radar::Sighting* Radar::find(const std::string& id) {
    for (std::size_t i = 0; i < enemies_.size(); ++i) {
        if (enemies_[i].id == id) return &enemies_[i];
    }
    return NULL;
}

void Radar::update(int tick, const std::vector<RobotView>& seen) {
    update(tick, -1, -1, seen);
}

void Radar::update(int tick, int self_x, int self_y,
                   const std::vector<RobotView>& seen) {
    // 1) 本 Tick 目击到的：刷新位置与状态
    for (std::size_t i = 0; i < seen.size(); ++i) {
        const RobotView& v = seen[i];
        if (v.id.empty()) continue;
        Sighting* s = find(v.id);
        if (!s) {
            Sighting n;
            n.id = v.id;
            enemies_.push_back(n);
            s = &enemies_.back();
        }
        // 上一次目击距离：用于“敌人是否在逼近”。隔了 >2 Tick 才再见到时，
        // 旧距离已经不可信，直接重置为当前距离（不产生假“逼近”信号）。
        if (self_x >= 0 && self_y >= 0 && s->last_tick >= 0) {
            const int oldD = iabs(s->x - self_x) + iabs(s->y - self_y);
            if (tick - s->last_tick <= 2) {
                s->last_dist = oldD;
            } else {
                s->last_dist = iabs(v.x - self_x) + iabs(v.y - self_y);
            }
        }
        s->team = v.team;
        s->x = v.x;
        s->y = v.y;
        s->hp = v.hp;
        s->defending = v.defending;
        s->protected_ = v.protected_;
        s->last_tick = tick;
        s->pos_current = true;
    }
    // 2) 没被目击的：位置失去实时性（可能移动、也可能已阵亡）。
    //    保留最后目击点，供"短时威胁"判断使用。
    for (std::size_t i = 0; i < enemies_.size(); ++i) {
        if (enemies_[i].last_tick != tick) enemies_[i].pos_current = false;
    }
}

void Radar::markAttackers(int tick, int self_x, int self_y) {
    if (self_x < 0 || self_y < 0) return;
    for (std::size_t i = 0; i < enemies_.size(); ++i) {
        Sighting& s = enemies_[i];
        if (s.last_tick < 0 || s.protected_) continue;
        if (tick - s.last_tick > 1) continue;  // 只归因最近一帧的目击
        if (s.x != self_x && s.y != self_y) continue;  // 只有同轴才能打到
        const int d = iabs(s.x - self_x) + iabs(s.y - self_y);
        if (d >= 1 && d <= 2) s.last_attack_tick = tick;
    }
}

bool Radar::recentAttacker(int tick, int maxAge) const {
    for (std::size_t i = 0; i < enemies_.size(); ++i) {
        const Sighting& s = enemies_[i];
        if (s.last_attack_tick < 0) continue;
        if (tick - s.last_attack_tick > maxAge) continue;
        // 开火者已经 2 Tick 以上没再被看到：威胁不再即时，交给掉血时间窗兜底。
        if (s.last_tick < 0 || tick - s.last_tick > 2) continue;
        return true;
    }
    return false;
}

const Radar::Sighting* Radar::at(int x, int y, int tick, int maxAge) const {
    const Sighting* best = NULL;
    for (std::size_t i = 0; i < enemies_.size(); ++i) {
        const Sighting& s = enemies_[i];
        if (s.x != x || s.y != y) continue;
        if (s.last_tick < 0) continue;
        const int age = tick - s.last_tick;
        if (age < 0 || age > maxAge) continue;
        // 同一格有多条记忆时取最新的
        if (!best || s.last_tick > best->last_tick) best = &s;
    }
    return best;
}

bool Radar::threatensCell(int x, int y, int tick, int maxAge) const {
    for (std::size_t i = 0; i < enemies_.size(); ++i) {
        const Sighting& s = enemies_[i];
        if (s.protected_) continue;  // 保护期内不能 ATTACK（规则 §7）
        if (s.last_tick < 0) continue;
        const int age = tick - s.last_tick;
        if (age < 0 || age > maxAge) continue;
        // 普通攻击只沿同行/同列（规则 §6.2），斜角不构成威胁
        if (s.x != x && s.y != y) continue;
        const int d = iabs(s.x - x) + iabs(s.y - y);
        if (d >= 1 && d <= 2) return true;
    }
    return false;
}

}  // namespace rmtc
