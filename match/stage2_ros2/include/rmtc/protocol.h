#pragma once
#include <string>
#include <vector>

// 协议数据结构：把 /team_X/perception 的 JSON 映射成纯 C++ 值类型。
//
// 约定（重要，改动前先读）：
//   1. 所有字段都带默认值。解析端遇到"字段缺失 / 类型不对"时保持默认值，
//      绝不抛异常，也绝不因为一个坏字段丢掉整帧（协议 §3 感知解析要求）。
//   2. self.position 是绝对坐标；visible_entities[].position 是 9×9 相对位置
//      字符串。两者类型完全不同，不能混用（本文件用 int x/y 和字符串分别承载）。
//   3. 本文件不依赖 JSON 库，解析在 agent.cpp 里做；这样世界模型/规划器可以被
//      离线测试直接构造，不需要真实感知消息。
namespace rmtc {

enum EntityKind {
    ENT_OBSTACLE,  // 障碍：不可进入，不阻挡普通攻击（规则 §6.2）
    ENT_RESOURCE,  // 资源：站上去自动收集（规则 §4.2）
    ENT_ROBOT      // 保留值。敌车实际走 RobotView（因为要带 hp/defending 等字段）
};

// 地形/资源实体。坐标已由 Agent 从 9×9 相对图换算成绝对坐标。
struct Entity {
    EntityKind kind;
    int x, y;
    float value = 0.f;   // 资源价值（普通 10 / 高价值 20）；障碍恒为 0
    bool high = false;   // value >= 20 视为高价值（协议口径）
};

// 视野内的敌方车辆（type=robot）。只包含"当前视野内的存活敌车"，
// 离开视野后由 Radar 用自己的目击记忆补位，不从这里读。
struct RobotView {
    std::string id;        // robot_id（如 "B"），缺失时退回 id 字段
    std::string team;
    int x = 0, y = 0;      // 绝对坐标（由相对图换算）
    int hp = 100;
    bool defending = false;   // 最近状态，不保证本 Tick 仍在防御（协议 §3）
    bool protected_ = false;  // 出生保护中：免疫普通攻击、不能攻击（规则 §7）
};

// 上一 Tick 的动作回执（self.last_action）。开局为 null → present=false。
// 注意：这里没有起点/终点，只有"是否移动成功"和错误码，
// 所以定位一律用 self.position，不能用回执累加（规则 §4.1）。
struct LastAction {
    bool present = false;
    int tick = -1;
    std::string action;              // move / attack / defend / wait
    std::string error;               // E_* 错误码，成功时为空
    bool moved = false;              // move 是否实际生效
    std::string effective_direction; // 实际生效方向（幻术反转已体现在这里）
};

// 结构化任务（perception.missions[]）。任务判定由裁判做，这里只作为决策输入。
struct Mission {
    std::string id;
    std::string type;   // normal_resources / high_resources / resource_value /
                        // effective_damage / reach
    int target = 0;
    bool has_pos = false;  // 仅 reach 任务带坐标
    int px = 0, py = 0;
};

// v1.2 完整感知。
// 字段顺序与协议文档一致，便于对照检查；新增字段请同步 parsePerception。
struct Perception {
    std::string session_id;   // 动作必须原样回填（协议 §4）
    int tick = 0;             // 动作必须原样回填

    // ---- self ----
    std::string robot_id;
    int x = 0, y = 0;              // 绝对坐标，每 Tick 都提供（含阵亡期间）
    bool alive = true;
    int hp = 100;                  // 100 上限（规则 §4.3）
    bool defending = false;        // 本车最近是否在防御
    bool protected_ = false;       // 出生保护（复活后 2 Tick；开局无保护）
    int respawn_in = 0;            // 距复活还有多少 Tick，存活时为 0
    bool has_position_reason = false;  // 仅开局/重生首帧有
    std::string position_reason;       // "initial" / "respawn"

    int vision_radius = 4;         // 0–4，实体按横纵差筛选
    int map_w = 20, map_h = 20;    // 第二阶段标准 24×24，以本帧为准

    // ---- score（本队累计） ----
    float resource_value = 0.f;        // 已收集资源总价值
    float effective_damage = 0.f;      // 有效伤害：实际伤害 + 击杀奖励 - 每次死亡 60，最低 0
    int mission_completed = 0;
    std::vector<std::string> mission_completed_ids;
    int invalid_actions = 0;

    // ---- 实体 ----
    // entities 只放地形/资源（给 World）；
    // robots 放存活敌车（给 Radar / Combat）。分开是因为两者语义与生命周期不同。
    std::vector<Entity> entities;
    std::vector<RobotView> robots;

    // ---- 任务与事件 ----
    std::vector<Mission> missions;
    std::string mission_text;  // 自然语言任务描述（与 missions 同批）
    std::string event_text;    // 自然语言随机事件公告，多条以换行分隔；无公告为空串

    LastAction last_action;
};

// 解析 visible_entities[].position 的 9×9 相对位置图。
//
// 格式（协议 §3 / 规则 §4.1）：
//   - 89 个字符 = 9 行 × 9 字符 + 8 个换行，无末尾换行；
//   - 中心 (第 5 行第 5 列) 固定是 '1'（自身）；
//   - '2' 是该实体；整图没有 '2' 表示实体与自身同格，相对位移 [0,0]；
//   - 图不随朝向旋转，x 向右、y 向下，与地图轴一致。
//
// 输出 dx/dy = [2 的列号 - 5, 2 的行号 - 5]。
// 结构非法（长度不对、换行位置不对、出现多个 '2'）返回 false，调用方丢弃该实体。
bool parseRelative9x9(const std::string& s, int& dx, int& dy);

}  // namespace rmtc
