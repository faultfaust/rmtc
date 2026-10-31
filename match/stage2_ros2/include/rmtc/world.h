#pragma once
#include <vector>
#include <utility>
#include "rmtc/protocol.h"

// 世界模型：地图记忆 + 最短路 + 覆盖增益 + 未知区连通块。
//
// 设计要点：
//   1. 地形只有三态：UNKNOWN（没看过）/ FREE（看过且空）/ BLOCKED（障碍）。
//      "看过但空"这个信息很关键——它是判断"未知格值得去看"的基础。
//   2. 记忆是永久的（除换场重置）。资源会被"看见即刷新"：每帧视野内的资源格
//      先被清空，再按本帧实体重新写入；视野里没有的资源视为已被捡走。
//   3. soft_block_ 是"会移动的占位"（敌车、刚失败过的格子），只影响寻路，
//      不写入地形状态，也不影响覆盖/区域计算。每 Tick 由 Agent 重新设置。
//   4. 所有临时缓冲（dist/risk/parent/heap/前缀和）都是成员，复用以避免
//      每 Tick 的堆分配（第一阶段实测 0 分配/Tick，别在热路径里加 vector 局部变量）。
namespace rmtc {

class World {
public:
  enum State { UNKNOWN = 0, FREE = 1, BLOCKED = 2 };

  // 资源记忆项：idx = y*w+x；value 为价值；seenTick 用于"越久没捡价值越高"的加成。
  struct ResCell { int idx; float value; int seenTick; };

  World();
  World(int w, int h);
  // 清空所有记忆，按新尺寸重建缓冲。
  void reset(int w, int h);
  // 尺寸不同才重置；返回是否真的重置了（调用方借此判断要不要清其它状态）。
  bool ensureSize(int w, int h);

  int w() const { return w_; }
  int h() const { return h_; }
  int cells() const { return w_ * h_; }
  int idx(int x, int y) const { return y * w_ + x; }
  bool inBounds(int x, int y) const { return x >= 0 && x < w_ && y >= 0 && y < h_; }

  // 用一帧感知更新地形与资源记忆（不改软阻挡，软阻挡由 Agent 另行设置）。
  void updateFromPerception(const Perception& p);
  // 记录一个永久障碍（撞墙自愈用）。
  void markBlocked(int x, int y);
  // 只在 UNKNOWN 时标记为 FREE（不会覆盖 BLOCKED）。
  void markFree(int x, int y);
  // 清掉某格资源记忆，返回被清掉的价值（第一阶段收集判断用）。
  float dropResource(int x, int y);

  // ---- 软阻挡（第二阶段）----
  // 敌车等"会移动的占位"：只影响寻路（dijkstra 绕开），不写入地形记忆，
  // 也改变不了 UNKNOWN/FREE 状态，每 Tick 由 Agent 刷新。
  void clearSoft();
  void setSoft(int idx);
  bool softBlocked(int idx) const {
    return idx >= 0 && idx < static_cast<int>(soft_block_.size()) && soft_block_[idx] != 0;
  }
  // 黄灯软代价：不禁止进入，只把进入该格的寻路代价抬高（接战门 ALERT 用）。
  // 与 soft_block_ 一样每 Tick 清空，不写地形记忆。
  void setSoftCost(int idx, float cost);
  float softCost(int idx) const {
    return (idx >= 0 && idx < static_cast<int>(soft_cost_.size())) ? soft_cost_[idx] : 0.f;
  }

  float resValue(int i) const { return res_val_[i]; }
  float resValueSum() const { return res_value_sum_; }
  const std::vector<unsigned char>& state() const { return state_; }

  void counts(int& unknown, int& freeCells, int& blocked) const;
  void resourceList(std::vector<ResCell>& out) const;

  // 单源最短路（4 邻接，单位步长）。
  //   steps:   步数（不可达为 inf）——保持真实步数，供"剩余时间可达"判断
  //   risk:    途经 UNKNOWN 格的累计风险（每格 +riskWeight）
  //   parent:  路径前驱（-1 表示无）
  // allowUnknown=false 时完全不进入未知格。
  // 比较顺序是"先步数、再风险"，所以等价步数下会挑更保守的路。
  // chokeRisk>0 时，进入咽喉格（isChoke）的次级风险 +chokeRisk：
  // 不扭曲步数（可行性判断安全），只在等长路径间绕开咽喉。
  void dijkstra(int start, float riskWeight, bool allowUnknown,
                std::vector<float>& steps, std::vector<float>& risk,
                std::vector<int>& parent, float chokeRisk = 0.f) const;

  void dijkstraSteps(int start, bool allowUnknown,
                     std::vector<float>& steps, std::vector<int>& parent,
                     float chokeRisk = 0.f) const;

  // 沿 parent 链回退，返回 start 出发的第一步（目标不可达/就是起点返回 -1）。
  int firstStep(const std::vector<int>& parent, int target, int start) const;

  // 每个格子"站上去能看见多少个未知格"（半径 r 的方形窗口，
  // 用二维前缀和 O(1) 求窗口和，整体 O(w*h)）。
  void coverageGain(int r, std::vector<int>& gain) const;
  // 同上，但未知格可以带权（weights 为 NULL 表示每格权重 1）。
  void coverageGainWeighted(int r, const float* weights, std::vector<float>& gain) const;

  // 给未知格做 4 邻接连通块标记。
  //   label: 每格所属块编号（-1 非未知，-2 是过小的块）
  //   size:  每块大小（被丢弃的小块记为 0）
  // 返回块总数。
  int labelRegions(std::vector<int>& label, std::vector<int>& size, int minSize) const;

  // ---- 地形拓扑（割点/咽喉，建议书《地形问题改进》）----
  // 图：FREE+UNKNOWN 格 4 邻接（BLOCKED 不建边）。迭代式 Tarjan 求割点，
  // 对每个割点 v 计算"删掉 v 后各连通块的最小一侧规模" choke_value。
  // 只写缓存数组，绝不写地形记忆（UNKNOWN 揭开后重算覆盖）。
  void analyzeTopology(int tick);
  bool topologyValid() const { return topology_valid_; }
  // 咽喉判定阈值（Config::choke_min_value → World 同步；改后下次重算生效）
  void setChokeMinValue(int k) { choke_min_value_ = k; }
  bool isChoke(int idx) const {
    return topology_valid_ && idx >= 0 && idx < static_cast<int>(is_choke_.size()) &&
           is_choke_[idx] != 0;
  }
  int chokeValue(int idx) const {
    return (idx >= 0 && idx < static_cast<int>(choke_value_.size())) ? choke_value_[idx] : 0;
  }
  int chokeCount() const;  // 当前咽喉数（诊断用）

private:
  int w_, h_;
  std::vector<unsigned char> state_;       // UNKNOWN / FREE / BLOCKED
  std::vector<unsigned char> hard_block_;  // 永久障碍标记（防止被 FREE 覆盖）
  std::vector<unsigned char> miss_count_;  // 视野内连续未报障碍的帧数（2 帧迟滞）
  std::vector<unsigned char> soft_block_;  // 每 Tick 可变的占位
  std::vector<float> soft_cost_;           // 每 Tick 可变的黄灯软代价（不禁止通行）
  std::vector<float> res_val_;             // 资源价值（0 = 无）
  std::vector<int> res_seen_;              // 资源最后目击 Tick
  float res_value_sum_;                    // 已知资源价值之和（增量维护）
  int obstacles_version_;
  int resources_version_;

  // 复用的临时缓冲（热路径零分配的关键）
  mutable std::vector<float> dist_buf_, parent_buf_;
  mutable std::vector<float> cost_buf_;
  mutable std::vector<int> pref_buf_;
  mutable std::vector<int> queue_buf_;

  // dijkstra 用的手写二叉堆：先比“步数+黄灯代价”，再比风险，最后比下标。
  // 黄灯代价为 0 时与原“先比步数”完全等价；加代价后允许 1–2 格绕行。
  struct HeapNode { int steps; float cost; float risk; int idx; };
  struct HeapGreater { bool operator()(const HeapNode& a, const HeapNode& b) const {
      if (a.cost != b.cost) return a.cost > b.cost;
      if (a.risk != b.risk) return a.risk > b.risk;
      return a.idx > b.idx;
  } };
  mutable std::vector<HeapNode> heap_;
  mutable std::vector<int> region_stack_, region_members_;
  mutable std::vector<float> pref_f_;

  // ---- 拓扑缓存与 Tarjan 工作缓冲（全部成员复用，零分配）----
  std::vector<unsigned char> is_choke_;   // 咽喉标记（随重算覆盖）
  std::vector<int> choke_value_;          // 删除该格后最小一侧的规模
  int choke_min_value_ = 8;               // 咽喉判定阈值 K（两侧最小规模 ≥K）
  bool topology_valid_;
  int topo_free_mark_;                    // 上次计算时的已探明格数（脏检查）
  int topo_tick_mark_;                    // 上次计算时的 Tick
  mutable std::vector<int> t_dfn_, t_low_, t_parent_, t_order_;
  mutable std::vector<int> t_stack_, t_comp_, t_ap_;  // Tarjan 栈 / 连通块 / 候选割点
};
}  // namespace rmtc
