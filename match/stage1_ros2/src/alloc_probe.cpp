// tools/alloc_probe.cpp —— 测量单次决策的堆分配次数与耗时（离线性性能探针）
//
// 用法：alloc_probe [场景]
//   0 = 20x20 常规（8 个已知资源，精确解可用）
//   1 = 20x20 全图已知（16 个资源，走贪心链）
//   2 = 50x50（精确解最坏情况）
//   3 = 200x200（大图，强制贪心）
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>


#include "rmtc/config.h"
#include "rmtc/planner.h"
#include "rmtc/protocol.h"
#include "rmtc/world.h"

extern unsigned long alloc_count();
extern void alloc_reset();

namespace {

rmtc::Entity resEnt(int x, int y, float v) {
  rmtc::Entity e;
  e.kind = rmtc::ENT_RESOURCE;
  e.x = x;
  e.y = y;
  e.value = v;
  e.high = v >= 20.f;
  return e;
}

struct Scene {
  int w, h;
  int nRes;
  int px, py;
};

std::vector<rmtc::Entity> buildEntities(const Scene& s) {
  std::vector<rmtc::Entity> ents;
  for (int k = 0; k < s.nRes; ++k) {
    const int x = 2 + (k * 7) % (s.w - 4);
    const int y = 2 + (k * 11) % (s.h - 4);
    if (x == s.px && y == s.py) continue;  // 别放机器人脚下，否则会一直 collect_on_cell
    ents.push_back(resEnt(x, y, (k % 3 == 0) ? 20.f : 10.f));
  }
  return ents;
}

}  // namespace

int main(int argc, char** argv) {
  int which = argc > 1 ? std::atoi(argv[1]) : 0;
  Scene s;
  const char* name = "";
  switch (which) {
    case 0: s.w = 20; s.h = 20; s.nRes = 8; s.px = 10; s.py = 10; name = "20x20 常规"; break;
    case 1: s.w = 20; s.h = 20; s.nRes = 16; s.px = 10; s.py = 10; name = "20x20 全图已知"; break;
    case 2: s.w = 50; s.h = 50; s.nRes = 12; s.px = 25; s.py = 25; name = "50x50 精确解最坏"; break;
    default: s.w = 200; s.h = 200; s.nRes = 40; s.px = 100; s.py = 100; name = "200x200 大图"; break;
  }

  rmtc::Perception p;
  p.session_id = "probe";
  p.vision_radius = 4;
  p.map_w = s.w;
  p.map_h = s.h;
  p.x = s.px;
  p.y = s.py;
  p.alive = true;
  p.entities = buildEntities(s);
  p.tick = 50;

  rmtc::World w(s.w, s.h);
  for (int i = 0; i < w.cells(); ++i) w.markFree(i % s.w, i / s.w);
  w.updateFromPerception(p);

  rmtc::Config cfg;
  rmtc::Planner planner(cfg);

  // 预热：让所有内部缓冲完成一次扩容
  for (int k = 0; k < 90; ++k) {
    p.tick = k;
    planner.decide(w, p);
  }

  const int kIter = 500;
  alloc_reset();


  //
  auto start_time = std::chrono::high_resolution_clock::now(); //开始计时
  //

  for (int k = 0; k < kIter; ++k) {
    p.tick = k % 90;  // 单调推进，模拟真实 Tick 序列
    planner.decide(w, p);
  }


  ////结束及时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  double ms_per_decide = (double)total_ms / kIter;
  ////

  const unsigned long allocs = alloc_count();

  std::printf("%-18s cells=%5d nRes=%2d  allocations/decide = %.2f  (total %lu / %d)\n",
              name, w.cells(), s.nRes,
              static_cast<double>(allocs) / kIter, allocs, kIter);

  ////打印时间
  std::printf(">> 总耗时: %lld ms | 平均单次决策耗时: %.3f ms\n", total_ms, ms_per_decide);


  return 0;
}

