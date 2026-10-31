#pragma once
#include <string>
#include <vector>
#include "rmtc/protocol.h"

// 敌情雷达：维护"最后一次目击"的敌车记忆。
//
// 规则边界（§4.1）：感知只给当前视野内的存活敌车；敌人离开视野后不再提供实时状态。
// 所以这里保存的是**目击**而不是真值，并且必须用"目击年龄 = tick - last_tick"
// 表达可信度。调用方要自己决定能容忍多旧的信息：
//   - 判断"这一击能不能打到" → 年龄必须是 0（本 Tick 亲眼看到）；
//   - 判断"会不会被偷袭"     → 可以容忍 1 Tick 的旧记忆。
//
// 这是刻意的：把"记忆可信度"交给查询方，而不是在 Radar 里藏一个隐含假设。
namespace rmtc {

class Radar {
public:
    struct Sighting {
        std::string id;        // robot_id（如 "B"）
        std::string team;
        int x = -1, y = -1;    // 最后已知位置（绝对坐标）
        int hp = 100;
        bool defending = false;
        bool protected_ = false;
        int last_tick = -1;    // 最后目击 Tick；-1 表示从未目击
        bool pos_current = false;  // 本 Tick 是否亲眼确认过位置
        // v1.2 §8.9：最近一次“观察到位置改变”的 Tick。只在连续两帧
        // 都看到同一敌车、且曼哈顿位移恰好 1 时更新；隔帧/复活位移
        // >1 不算成功移动。用于判断敌方是否在潘潘触发窗口内移动。
        int last_moved_tick = -1;
        // 敌方易伤的绝对到期 Tick；-1=无。Combat 用该字段判断伤害 ×2。
        int vulnerable_until = -1;
        // 接战门用的意图记忆（协议不提供敌方动作，攻击性由我方掉血反推）
        int last_attack_tick = -1; // 最近一次推断对方开火的 Tick
        int last_dist = -1;        // 上一次目击时的曼哈顿距离（判断逼近）
    };

    // 清空记忆（换场次时调用）。
    void reset();

    // 每 Tick 调用一次：seen 是本 Tick 视野内的敌车。
    // 未见到的条目保留旧值，只是 pos_current 置 false。
    // 注意：这里不记录"敌人已死亡"——死亡只会让它从视野消失，
    // 与"走出视野"无法区分，硬记死亡会误判复活后的敌人。
    void update(int tick, const std::vector<RobotView>& seen);
    // 带自车坐标的版本：额外维护 last_dist（上一次目击距离，用于“逼近”判断）。
    void update(int tick, int self_x, int self_y,
                const std::vector<RobotView>& seen);

    // 我方本 Tick 掉血时，把“上一帧与我同轴 1–2 格”的目击者标为开火者。
    // 协议不提供敌方动作，这是唯一合规的近似；多敌同轴时保守地都标。
    void markAttackers(int tick, int self_x, int self_y);

    // 潘潘的神：把“在触发窗口 [activate, activate+20) 内观察到成功移动”
    // 的敌车标记为易伤，直到 end_tick（缺省 activate+30）。
    void markVulnerableMoves(int activate, int end_tick);

    // 最近 maxAge Tick 内是否有"确实开过火"的敌车仍在视野里（目击年龄 ≤2）。
    // 用于经济门承伤豁免/威胁分级：比"我方掉血"更精确，掉血也可能来自环境。
    bool recentAttacker(int tick, int maxAge) const;

    // (x,y) 上是否有年龄 ≤ maxAge 的目击；返回最新的一条，没有则 NULL。
    const Sighting* at(int x, int y, int tick, int maxAge) const;

    // (x,y) 是否被"能在本 Tick 攻击到它的敌车"威胁。
    // 判据：目标与本车同行或同列、距离 1–2（规则 §6.2 的攻击范围），
    // 且该敌车不处于出生保护（保护期不能 ATTACK）、目击年龄 ≤ maxAge。
    bool threatensCell(int x, int y, int tick, int maxAge) const;

    const std::vector<Sighting>& all() const { return enemies_; }

private:
    Sighting* find(const std::string& id);

    std::vector<Sighting> enemies_;
};
}  // namespace rmtc
