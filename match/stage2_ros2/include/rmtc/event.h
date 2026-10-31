#pragma once
#include <string>
#include "rmtc/protocol.h"
#include "rmtc/world.h"

// 事件量化（问题 3 的"把自然语言事件折算成分数"部分）。
//
// 分工（合规要求，规则 §8.1）：
//   * 自然语言 → 结构化 EventReport 必须由大模型完成（禁止正则/关键词/模板）；
//   * 本文件只负责"结构化结果 → 对本车的期望分值 + 行为建议"。
//   所以这里不解析文本，只吃 LLM 输出，便于单测与离线标定。
//
// 分值口径与决策文本 §2.1 一致（60 分制）：
//   1 点 D = 0.03 分；1 点敌方 HP = 0.03 分；1 资源价值 = 0.25 分；
//   1 个任务 = 5 分；1 次死亡 ≈ 4.5 分（自身 -60 + 对手 +60 + 回场时间）。
namespace rmtc {

enum EventKind {
    EVK_UNKNOWN = 0,
    EVK_EXPLOSION,   // 爆炸：矩形，80 伤害，移动前判定
    EVK_ILLUSION,    // 幻术：矩形，MOVE 方向反转 50 Tick
    EVK_HEAL,        // 天使恩泽：点位，进入后 10 Tick × +10 HP
    EVK_FORTRESS,    // 天下无双：公告给出的 3×3 堡垒（v1.2 位置可特殊），驻留 10 Tick
    EVK_SPLIT,       // 世界分裂：十字墙 30 Tick，线上立即死亡
    EVK_PEPPER,      // 火爆辣椒：点位，踩点触发整行整列 50 伤害
    EVK_THORNS,      // 洁身自好：20 Tick，攻击防御者会被反伤
    EVK_VULNERABLE   // 潘潘的神（易伤）：30 Tick，普通/无双伤害 ×2
};

const char* eventKindName(EventKind k);

// LLM 应当产出的最小结构（字段缺失按默认值处理，类型错则由 LLM 层拒绝）。
struct EventReport {
    EventKind kind = EVK_UNKNOWN;
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // 矩形包含边界；点位时 x0==x1,y0==y1
    int activate_tick = -1;              // 生效 Tick（含）
    int end_tick = -1;                   // 结束 Tick（不含），-1 = 未知/一次性
    float confidence = 0.f;              // 模型置信度 0–1，低于阈值不采纳
};

// 对当前局面的量化结果。
struct EventAdvice {
    double value = 0.0;        // 净分值（正=值得去/利用，负=需要规避）
    bool avoid = false;        // 需要离开/避开矩形
    bool has_target = false;   // 有推荐前往的格子（点位/堡垒/回血点）
    int tx = 0, ty = 0;
    std::string reason;        // 便于日志与复盘
};

class EventScorer {
public:
    // min_confidence：低于该置信度直接返回 0 分（不把不确定结论用于决策）。
    explicit EventScorer(float min_confidence = 0.5f)
        : min_confidence_(min_confidence) {}

    // p 为本帧感知（只需 self 与 score），w 只读（可达性/地形），tick 为当前 Tick。
    EventAdvice score(const EventReport& r, const Perception& p, const World& w) const;

private:
    float min_confidence_;
};
}  // namespace rmtc
