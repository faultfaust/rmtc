#pragma once
#include <cstdlib>

// 规则 v1.2 §7 的复活选点（纯函数，便于单测与模拟器/裁判共用）。
//
// 规则原文：
//   * 以该车初始出生点为基准，按曼哈顿距离由近到远搜索；
//   * 同距离按“上→右→下→左”的优先顺序，取第一个合法格；
//   * 合法格 = 图内、无静态障碍、无生效中的分裂墙、无其他存活车占用；
//     资源格允许复活；障碍只排除落点，不阻断搜索；
//   * 多车同时复活按席位 A→B→C→D 依次处理（调用方按顺序调用本函数即可）。
//
// blockedAt(x,y,tick) 返回“该格在 tick 是否为墙”（静态障碍或生效分裂墙）。
// occupiedAt(x,y)     返回“该格是否被其他存活车占用”。
// 这两个回调把地图/车辆细节留在调用方，规则本身只在这里实现一次。
namespace rmtc {

template <class BlockedFn, class OccupiedFn>
bool chooseRespawnCell(int sx, int sy, int w, int h, int tick,
                       BlockedFn blockedAt, OccupiedFn occupiedAt,
                       int& ox, int& oy) {
    if (w <= 0 || h <= 0) return false;

    // 同距离内的扫描顺序：上边（左→右）、右边（上→下）、
    // 下边（左→右）、左边（上→下）。只取正好落在该曼哈顿距离上的格，
    // 因此等价于“上→右→下→左”的四条边优先，而不是按 dx/dy 数值排序。
    for (int r = 0; r <= w + h; ++r) {
        for (int pri = 0; pri < 4; ++pri) {
            for (int t = -r; t <= r; ++t) {
                int dx = 0, dy = 0;
                if (pri == 0) { dx = t; dy = -r; }
                else if (pri == 1) { dx = r; dy = t; }
                else if (pri == 2) { dx = t; dy = r; }
                else { dx = -r; dy = t; }
                if (std::abs(dx) + std::abs(dy) != r) continue;
                const int x = sx + dx, y = sy + dy;
                if (x < 0 || x >= w || y < 0 || y >= h) continue;
                if (blockedAt(x, y, tick)) continue;
                if (occupiedAt(x, y)) continue;
                ox = x;
                oy = y;
                return true;
            }
        }
    }
    ox = sx;
    oy = sy;
    return false;
}

}  // namespace rmtc
