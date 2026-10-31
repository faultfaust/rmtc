#pragma once
#include <vector>
#include "rmtc/config.h"
#include "rmtc/protocol.h"
#include "rmtc/world.h"

namespace rmtc {
enum { MOVE_WAIT, MOVE_MOVE };
enum Reason {
  R_WAIT_DEAD, R_WAIT_TIME, R_WAIT_ON_RES, R_WAIT_NO_TARGET, R_WAIT_ALREADY, R_WAIT_NO_SAFE,
  R_MOVE_RESOURCE, R_MOVE_EXPLORE, R_MOVE_SAFE
};
const char* reason_name(int r);

struct Decision {
  int kind = MOVE_WAIT;
  const char* direction = NULL;
  int targetIdx = -1;
  int reason = R_WAIT_NO_TARGET;
  int commitUntil = -1;
  float hidden = 0.f;
  float rate = 0.f;
  float stopR = 0.f;
  float ratioRes = 0.f;
  float ratioExp = 0.f;
  float exploreRatio = 0.f;
  float planValue = 0.f;
  float planCost = 0.f;
};

struct CollectPlan {
  bool valid = false;
  float value = 0.f;
  float cost = 0.f;
  int first = -1;
  std::vector<int> order;
};

struct ExplorePlan {
  bool valid = false;
  float value = 0.f;
  float cost = 0.f;
  float ratio = 0.f;
  int target = -1;
  int region = -1;
};

class Planner {
public:
  Planner(const Config& cfg);
  void reset();
  void noteAction(int tick, int x, int y, const char* direction);
  void absorbReceipt(const Perception& p, World& world);
  Decision decide(World& w, const Perception& p);

private:
  struct ResCand { int idx, x, y; float dist, value; };

  Config cfg_;
  int target_, target_kind_;
  float target_ratio_;
  int commit_until_;
  int attempt_tick_, attempt_x_, attempt_y_;
  bool has_attempt_;
  int cur_vision_;
  int fail_streak_, total_failures_, conservative_until_;
  int plan_exact_calls_, plan_greedy_calls_;

  // ---- [FIX] A/B 实验新增状态 ----
  float lam_;                    // 行程可信度：实测tick/预估距离 的 EMA（1=预测可信）
  int j_tgt_;                    // [FIX-B] 当前统计中的行程目标
  float j_est_;                  // [FIX-B] 出发时的预估距离
  int j_tick0_;                  // [FIX-B] 出发 tick
  std::vector<char> excl_mask_;  // 确认不可达（围死）资源标记
  float excl_value_;             // 被剔除资源总价值（hidden 修正用）
  std::vector<int> seal_stack_;  // 围死判定 BFS 缓冲
  std::vector<char> seal_vis_;
  std::vector<float> eff_;       // [FIX-E] 风险加权距离场（收集规划用）

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

  CollectPlan cp_;
  std::vector<int> rev_, best_order_, order_;
  std::vector<char> used_;

  float ageBonus(int seenTick, int tick) const;
  void buildResourceCandidates(const World& w, int R, int tick);
  void planCollectExact(World& w, const std::vector<float>& d0, const std::vector<float>& dij, int n, int R, CollectPlan& out);
  void planCollectGreedy(const std::vector<float>& d0, const std::vector<float>& dij, int n, int R, CollectPlan& out);
  void planCollect(World& w, int start, int R, CollectPlan& out);
  void planExploreFrontier(World& w, int R, int tick, int visionR, float hidden, int unseen, float rate, float stopR, ExplorePlan& out);
  void planExploreGain(World& w, int R, int tick, int visionR, float hidden, int unseen, float rate, float stopR, bool hasRes, ExplorePlan& out);
};
} // namespace rmtc
