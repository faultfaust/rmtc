#pragma once
#include <vector>
#include "rmtc/config.h"
#include "rmtc/protocol.h"
#include "rmtc/radar.h"
#include "rmtc/world.h"

// 采集/探索规划器（第一阶段逻辑，第二阶段复用）。
//
// 职责边界（重要）：
//   * 只产出 MOVE / WAIT，以及"为什么这么做"的诊断信息；
//   * 不处理攻击/防御（combat.cpp）；
//   * 不解析 JSON、不碰 ROS（agent.cpp / main.cpp）；
//   * 不写地图记忆（只读 World；唯一例外是回执自愈，见 absorbReceipt）。
namespace rmtc {

// 动作种类。命名沿用第一阶段的 MOVE_* 前缀（历史原因），
// 现在语义是"本 Tick 的动作类型"。
enum { MOVE_WAIT, MOVE_MOVE, MOVE_ATTACK, MOVE_DEFEND };

// 决策原因：只用于日志/调参，不参与逻辑判断。
enum Reason {
  R_WAIT_DEAD,      // 阵亡，只能等
  R_WAIT_TIME,      // 剩余时间耗尽
  R_WAIT_ON_RES,    // 站在资源格上等待（避免抖动）
  R_WAIT_NO_TARGET, // 没有可达目标
  R_WAIT_ALREADY,   // 已到达目标格
  R_WAIT_NO_SAFE,   // 保守模式下无安全第一步
  R_MOVE_RESOURCE,  // 走向资源
  R_MOVE_EXPLORE,   // 走未知区（第一步是已知空地）
  R_MOVE_SAFE,      // 走未知区（第一步是未知格）
  R_MOVE_RETREAT,   // 撤离（combat 层产出，躲离威胁）
  R_MOVE_EVENT,     // 事件驱动（避险撤离 / 前往回血点·堡垒，LLM 量化结果）
  R_ACT_ATTACK,     // 攻击（combat 层产出）
  R_ACT_DEFEND      // 防御（combat 层产出）
};
const char* reason_name(int r);

// 方向编号 → 协议字符串（1=right 2=left 3=down 4=up），0/越界返回 NULL。
const char* direction_name(int d);

// 一次决策的完整输出。字段除了给 Agent 用，也是离线调参的分析口径。
struct Decision {
  int kind = MOVE_WAIT;
  const char* direction = NULL;  // 仅 MOVE_MOVE / MOVE_ATTACK 有效
  int targetIdx = -1;            // 目标格（资源/探索点/攻击目标所在格）
  int reason = R_WAIT_NO_TARGET;
  int commitUntil = -1;          // 目标锁定到哪个 Tick（诊断用）

  // ---- 诊断量：解释这次选择 ----
  float hidden = 0.f;       // 估计的"还没看见的资源价值"
  float rate = 0.f;         // 探索强度（受剩余时间/动态阈值调节）
  float stopR = 0.f;        // 当前动态放弃阈值（剩余时间小于它就停止探索）
  float ratioRes = 0.f;     // 采集侧密度（计划价值/步数）
  float ratioExp = 0.f;     // 探索侧密度
  float exploreRatio = 0.f; // 同上（保留字段）
  float planValue = 0.f;    // 收集计划总价值
  float planCost = 0.f;     // 收集计划总步数

  // ---- 战斗输出（MOVE_ATTACK 时有效）----
  std::string target_id;  // 攻击目标的 robot_id
  float damage = 0.f;     // 期望伤害（20 / 10 / 0）
};

// 收集计划：一次巡游访问的资源点顺序。
struct CollectPlan {
  bool valid = false;
  float value = 0.f;   // 计划覆盖的资源价值（含高价值加成/目击加成）
  float cost = 0.f;    // 总步数（最后一点到终点，不要求回到起点）
  int first = -1;      // 首站格下标（Agent 用它当本 Tick 的目标）
  std::vector<int> order;  // 资源点在 cands_ 里的下标序列
};

// 探索候选：去哪个未知区域/格子能换来最多信息。
struct ExplorePlan {
  bool valid = false;
  float value = 0.f;
  float cost = 0.f;
  float ratio = 0.f;   // value / cost
  int target = -1;     // 目标格下标
  int region = -1;     // 目标所属连通块（frontier 模式用）
};

class Planner {
public:
  explicit Planner(const Config& cfg);

  // 换场次时清空内部状态（目标锁定、失败计数等），配置不变。
  void reset();

  // 登记本 Tick 实际发出的移动，便于下一 Tick 解释回执。
  // direction 为 NULL 表示本 Tick 不是移动（清空登记）。
  void noteAction(int tick, int x, int y, const char* direction);

  // 消化上一 Tick 的回执。
  // hardTerrain=true：把失败格写回为障碍并进入保守模式（第一阶段行为）。
  // false：失败由占位/争格/状态引起（第二阶段敌车环境），只放弃当前目标。
  // 归因由 Agent 完成——规划器不解析错误码，避免和协议细节耦合。
  void absorbReceipt(const Perception& p, World& world, bool hardTerrain = true);

  // 主决策：给定世界模型与本帧感知，返回本 Tick 动作。
  // radar 可选：非 NULL 且 cfg.race_gate 开启时，用于高价值资源的时间竞争判定。
  Decision decide(World& w, const Perception& p, const Radar* radar = NULL);

  // race 门控累计剔除的高价值候选数（诊断/selfcheck；reset() 清零）。
  int raceDrops() const { return race_drops_; }

private:
  // 资源候选：格子下标 + 坐标 + 步数 + 已加权价值 + 是否高价值原始资源
  struct ResCand { int idx, x, y; float dist, value; bool high; };

  Config cfg_;

  // ---- 目标锁定 ----
  int target_;        // 当前锁定目标格（-1 无）
  int target_kind_;   // 0=资源 1=探索
  int last_dir_;      // 上一 Tick 实际输出的方向（抑制 A→B→A 回摆）
  int last_target_;   // 上一 Tick 的目标格
  float self_r2_;     // 本 Tick 自己已采集资源价值（P7 让资源用）
  std::vector<int> path_;  // 已锁定的后续路径（从下一格到目标）
  int path_target_;   // path_ 对应的目标格（-1 无）
  float target_ratio_; // 选中它时的评分（迟滞比较用）
  int commit_until_;  // 锁定到哪个 Tick

  // ---- 回执自愈 ----
  int attempt_tick_, attempt_x_, attempt_y_;
  bool has_attempt_;
  int cur_vision_;         // 最近一次感知的视野半径
  int fail_streak_;        // 连续失败次数
  int total_failures_;     // 累计失败（诊断）
  int conservative_until_; // 保守模式持续到哪个 Tick
  int plan_exact_calls_;   // 精确解调用次数（诊断）
  int plan_greedy_calls_;  // 贪心解调用次数（诊断）

  // ---- 复用缓冲（热路径零分配）----
  std::vector<int> parent_, parent_safe_;
  std::vector<float> dist_, risk_, dist_safe_, risk_safe_;
  std::vector<int> label_;
  std::vector<int> region_size_, region_entry_;
  std::vector<float> region_dist_;
  std::vector<float> cell_weight_, gainf_;

  std::vector<ResCand> cands_;
  std::vector<World::ResCell> res_all_;
  std::vector<float> d0_, dij_, dp_, bdist_, brisk_;
  std::vector<int> bparent_;
  // race 门控的敌车 BFS 复用缓冲（与 dist_ 同纪律：成员复用，零分配）
  std::vector<float> race_steps_;
  std::vector<int> race_parent_;
  int race_drops_ = 0;  // 累计剔除的高价值候选数（诊断/selfcheck）

  CollectPlan cp_;
  std::vector<int> rev_, best_order_, order_;
  std::vector<char> used_;

  float ageBonus(int seenTick, int tick) const;

  // 从世界记忆里挑出剩余时间 R 内可达的资源，按距离排序。
  // 价值 = 原始价值 × 高价值加成 × 目击年龄加成。
  // race 门控：对高价值候选，用雷达目击估计敌车到达时间，明显先到的剔除。
  void buildResourceCandidates(const World& w, int R, int tick, const Radar* radar,
                               bool wantNormal = false, bool wantHigh = false);

  // 精确巡游（Held-Karp）：n ≤ exact_max_n 时用，返回价值最大的访问集合与顺序。
  void planCollectExact(World& w, const std::vector<float>& d0,
                        const std::vector<float>& dij, int n, int R,
                        CollectPlan& out);

  // 多起点贪心链：对每个可能的首站贪心续接，取总价值最大者（大图/点多时用）。
  void planCollectGreedy(const std::vector<float>& d0,
                         const std::vector<float>& dij, int n, int R,
                         CollectPlan& out);

  // 收集计划入口：整理两两距离，按规模选择精确解或贪心解。
  void planCollect(World& w, int start, int R, CollectPlan& out);

  // 前沿（连通块）探索：对每个未知块的入口估"期望价值/代价"。
  void planExploreFrontier(World& w, int R, int tick, int visionR,
                           float hidden, int unseen, float rate,
                           float stopR, ExplorePlan& out);

  // 逐格覆盖增益探索：对每个可达格估"站上去能看见多少未知格"。
  void planExploreGain(World& w, int R, int tick, int visionR,
                       float hidden, int unseen, float rate,
                       float stopR, bool hasRes, ExplorePlan& out);
};
}  // namespace rmtc
