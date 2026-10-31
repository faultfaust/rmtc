#pragma once
#include <vector>
#include <utility>
#include "rmtc/protocol.h"

namespace rmtc {
class World {
public:
  enum State { UNKNOWN = 0, FREE = 1, BLOCKED = 2 };
  
  struct ResCell { int idx; float value; int seenTick; };

  World();
  World(int w, int h);
  void reset(int w, int h);
  bool ensureSize(int w, int h);

  int w() const { return w_; }
  int h() const { return h_; }
  int cells() const { return w_ * h_; }
  int idx(int x, int y) const { return y * w_ + x; }
  bool inBounds(int x, int y) const { return x >= 0 && x < w_ && y >= 0 && y < h_; }

  void updateFromPerception(const Perception& p);
  void markBlocked(int x, int y);
  void markFree(int x, int y);
  float dropResource(int x, int y);
  
  float resValue(int i) const { return res_val_[i]; }
  float resValueSum() const { return res_value_sum_; }
  const std::vector<unsigned char>& state() const { return state_; }

  void counts(int& unknown, int& freeCells, int& blocked) const;
  void resourceList(std::vector<ResCell>& out) const;

  void dijkstra(int start, float riskWeight, bool allowUnknown,
                std::vector<float>& steps, std::vector<float>& risk,
                std::vector<int>& parent) const;
                
  void dijkstraSteps(int start, bool allowUnknown,
                     std::vector<float>& steps, std::vector<int>& parent) const;

  int firstStep(const std::vector<int>& parent, int target, int start) const;

  void coverageGain(int r, std::vector<int>& gain) const;
  void coverageGainWeighted(int r, const float* weights, std::vector<float>& gain) const;
  
  int labelRegions(std::vector<int>& label, std::vector<int>& size, int minSize) const;

private:
  int w_, h_;
  std::vector<unsigned char> state_;
  std::vector<unsigned char> hard_block_;
  std::vector<float> res_val_;
  std::vector<int> res_seen_;
  float res_value_sum_;
  int obstacles_version_;
  int resources_version_;

  mutable std::vector<float> dist_buf_, parent_buf_;
  mutable std::vector<int> pref_buf_;
  mutable std::vector<int> queue_buf_;
  
  struct HeapNode { int steps; float risk; int idx; };
  struct HeapGreater { bool operator()(const HeapNode& a, const HeapNode& b) const {
      if (a.steps != b.steps) return a.steps > b.steps;
      if (a.risk != b.risk) return a.risk > b.risk;
      return a.idx > b.idx;
  } };
  mutable std::vector<HeapNode> heap_;
  mutable std::vector<int> region_stack_, region_members_;
  mutable std::vector<float> pref_f_;
};
} // namespace rmtc