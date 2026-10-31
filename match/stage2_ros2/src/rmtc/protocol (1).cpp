// rmtc/protocol.cpp —— 协议级解析辅助
//
// 这里只放"与 ROS/JSON 无关"的纯数据解析，方便离线单测直接调用。
#include "rmtc/protocol.h"

namespace rmtc {

bool parseRelative9x9(const std::string& s, int& dx, int& dy) {
  dx = 0;
  dy = 0;
  if (s.size() != 89) return false;  // 81 格 + 8 换行；无末尾换行

  int twos = 0;
  for (int r = 0; r < 9; ++r) {
    for (int c = 0; c < 9; ++c) {
      // 每行 9 字符 + 1 个换行，所以第 r 行第 c 列的下标是 r*10 + c。
      const char ch = s[static_cast<std::size_t>(r) * 10 + c];
      if (ch == '2') {
        // 相对位移 = 与中心 (4,4) 的差；屏幕坐标 y 向下，正好等于行差。
        dx = c - 4;
        dy = r - 4;
        ++twos;
      }
    }
    // 行的分隔符必须是 '\n'：防止把 JSON 里的 "\\n" 两个字符误当成换行。
    if (r < 8 && s[static_cast<std::size_t>(r) * 10 + 9] != '\n') return false;
  }

  if (twos > 1) return false;
  // twos == 0：实体与自身重叠，图上只有 '1'，按约定表示 [0,0]（保持初值）。
  return true;
}

}  // namespace rmtc
