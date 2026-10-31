// rmtc/event_prompt.cpp —— Prompt 组装 + LLM 输出解析与硬校验
//
// 设计要点（对照《网关参考建议》阶段二）：
//   * System prompt 给出字段 schema 并禁止 Markdown——但模型仍常带 ```json
//     围栏，所以 parseLlmEvent 里的围栏剥离是必选项不是可选项；
//   * 公告原文原样进 user 消息：任何"预处理"都可能踩 §8.1 的合规红线；
//   * 所有数字字段用 isNum() 强校验——模型会输出 "x0":"4" 这种字符串数字。
#include "rmtc/event_prompt.h"

#include "rmtc/json.h"

#include <cstring>
#include <cstdlib>
#include <cctype>

namespace rmtc {

namespace {

// kind 字符串 → 枚举（与 EventScorer/eventKindName 的 8 类一致）
EventKind kindFromString(const std::string& s) {
    std::string k;
    k.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i)
        k.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(s[i]))));
    if (k == "explosion") return EVK_EXPLOSION;
    if (k == "illusion") return EVK_ILLUSION;
    if (k == "heal") return EVK_HEAL;
    if (k == "fortress") return EVK_FORTRESS;
    if (k == "split") return EVK_SPLIT;
    if (k == "pepper") return EVK_PEPPER;
    if (k == "thorns") return EVK_THORNS;
    if (k == "vulnerable") return EVK_VULNERABLE;
    return EVK_UNKNOWN;
}

// 模型经常把数字写成字符串（"x0":"4"），这里统一接受两种形式；
// 仍然做范围校验，不改变“防幻觉”的语义。
bool numericValue(const JsonValue* v, double& out) {
    if (!v) return false;
    if (v->isNum()) {
        out = v->num;
        return true;
    }
    if (v->isStr() && !v->str.empty()) {
        char* end = NULL;
        const double x = std::strtod(v->str.c_str(), &end);
        if (end && *end == '\0') {
            out = x;
            return true;
        }
    }
    return false;
}

int clampCoord(const JsonValue* v, int hi, bool& ok) {
    (void)hi;  // 上界校验在调用方统一做（配合 map_w/map_h）
    double n = 0.0;
    if (!numericValue(v, n)) {
        ok = false;
        return 0;
    }
    if (n < 0 || n > 1e6) {
        ok = false;  // 离谱数值（幻觉）
        return 0;
    }
    return static_cast<int>(n);
}

// 剥离 Markdown 围栏 / 前后缀废话：取第一个 '{' 到最后一个 '}'。
// 没有大括号 → 返回 false（模型没输出 JSON）。
bool stripToJsonObject(const std::string& in, std::string& out) {
    const std::size_t b = in.find('{');
    if (b == std::string::npos) return false;
    const std::size_t e = in.rfind('}');
    if (e == std::string::npos || e < b) return false;
    out = in.substr(b, e - b + 1);
    return true;
}

}  // namespace

namespace {

const char* kDefaultModel = "deepseek-flash";

// 解析器的唯一职责是“公告原文 → EventReport”。不要让模型做战术决策：
//   * 决策由本地 Agent/Combat/Planner 完成，LLM 输出动作这里不会消费；
//   * JSON mode 下过多自由文本会挤占 token，且容易漏字段。
const char* kSystemPrompt =
    R"JSON(你是 RMTC 机甲大师图灵对决的“事件公告解析器”。你的唯一任务是把裁判的
自然语言公告抽取成一个 json 对象；不要做战术决策、不要输出动作、不要解释、
不要 Markdown。

只输出一个 json 对象，字段固定为：
{
  "kind": "explosion",
  "x0": 0, "y0": 0, "x1": 0, "y1": 0,
  "activate_tick": 0,
  "end_tick": -1,
  "confidence": 0.0
}
kind 只能取：explosion / illusion / heal / fortress / split / pepper / thorns /
vulnerable / unknown。

规则：
1. 坐标一律用绝对坐标（0..地图尺寸-1），不是相对坐标，也不是 9×9 相对图。
2. 区域用左上 (x0,y0) 和右下 (x1,y1)；单点 x1=x0,y1=y0；整行/整列/十字用
   交点坐标，x0=x1 或 y0=y1。
3. activate_tick 按公告原文换算，不要自己猜：
   * “N Tick 后 / 再过 N Tick” = 当前Tick + N；
   * “立即 / 本 Tick / 现在” = 当前Tick；
   * 绝对时刻（“Tick 75”“第 75 Tick”“t=75”）直接填 75，不要再加当前Tick；
   * 公告同时给出持续时长时，end_tick = activate_tick + 持续Tick数。
4. end_tick：持续型事件必须给，= activate_tick + 公告给出的持续 Tick 数；
   explosion 给 -1；split 必须给“墙”的结束 Tick（规则 §8.6 默认 30 Tick，
   公告给出时长则以公告为准），墙到期后不再阻挡。公告没给持续时间且无法
   确定时，把 confidence 降到 0.3 以下。
5. kind 语义：explosion=矩形爆炸（瞬时）；illusion=幻术区域（持续）；
   heal=回血（持续）；fortress=堡垒/无双区域（持续）；split=世界分裂十字
   （瞬时击杀 + 30 Tick 临时墙）；pepper=火爆辣椒点位（持续/触发）；
   thorns=反伤（持续）；vulnerable=易伤/潘潘的神（持续）。
6. 只依据公告原文，禁止编造公告没有的坐标、时间、持续时间；无法确定 kind
   或坐标时输出 kind="unknown"、坐标 0、confidence≤0.3。
7. 输出必须能被 JSON.parse 直接解析：不要代码块、不要前后缀、不要注释。
8. 如果公告是“已结束/已解除/效果消失/已过期”这类结束通知，而不是新事件，
   输出 kind="unknown"、坐标 0、confidence=0.0；不要编造坐标，也不要把它
   当成新的持续事件。
)JSON";

std::string effectiveModel(const std::string& model) {
    return model.empty() ? std::string(kDefaultModel) : model;
}

}  // namespace

std::string buildEventRequestBody(int tick, const std::string& event_text,
                                  const std::string& model) {
    // system 与 user 内容都走 jsonEscape，公告原文里的引号/换行不会破坏结构
    const std::string useModel = effectiveModel(model);
    const std::string user = "当前 Tick: " + std::to_string(tick) +
                             "\n公告内容（原样）: " + event_text +
                             "\n请只输出符合 schema 的 json。";

    std::string s = "{";
    s += "\"model\":\"" + jsonEscape(useModel) + "\",";
    // 正式网关不主动关闭思考：deepseek-flash 默认开，必须显式 disabled。
    s += "\"thinking\":{\"type\":\"disabled\"},";
    // 官方 JSON mode：上游保证输出可被 JSON.parse；prompt 里已含 "json"。
    s += "\"response_format\":{\"type\":\"json_object\"},";
    s += "\"stream\":false,";
    s += "\"temperature\":0,";
    s += "\"max_tokens\":512,";
    s += "\"messages\":[{\"role\":\"system\",\"content\":\"" +
         jsonEscape(kSystemPrompt) + "\"},";
    s += "{\"role\":\"user\",\"content\":\"" + jsonEscape(user) + "\"}]}";
    return s;
}

std::string buildHandshakeRequestBody(int tick, const std::string& model) {
    // 带 tick 的唯一文本：每次重试都是不同的请求，避免被客户端/网关的
    // “同文本去重”再次吞掉；也便于赛后按 tick 对账。
    const std::string useModel = effectiveModel(model);
    const std::string user =
        "请回复 OK。（本场握手 t" + std::to_string(tick) + "）";
    std::string s = "{";
    s += "\"model\":\"" + jsonEscape(useModel) + "\",";
    s += "\"thinking\":{\"type\":\"disabled\"},";
    s += "\"stream\":false,";
    s += "\"temperature\":0,";
    s += "\"max_tokens\":16,";
    s += "\"messages\":[{\"role\":\"user\",\"content\":\"" +
         jsonEscape(user) + "\"}]}";
    return s;
}

bool parseLlmEvent(const std::string& content, int current_tick,
                   int map_w, int map_h, float min_confidence,
                   EventReport& out) {
    std::string json_text;
    if (!stripToJsonObject(content, json_text)) return false;

    JsonValue root;
    std::string err;
    if (!jsonParse(json_text, root, err) || !root.isObj()) return false;

    out = EventReport();

    // ---- kind ----
    const JsonValue* jkind = root.get("kind");
    if (!jkind || !jkind->isStr()) return false;
    out.kind = kindFromString(jkind->str);
    if (out.kind == EVK_UNKNOWN) return false;

    // ---- 置信度（低于阈值视为未解析）----
    const JsonValue* jconf = root.get("confidence");
    double conf = 0.0;
    if (!numericValue(jconf, conf)) return false;
    out.confidence = static_cast<float>(conf);
    if (out.confidence < min_confidence) return false;

    // ---- 坐标（防幻觉：越界拒绝）----
    bool ok = true;
    out.x0 = clampCoord(root.get("x0"), map_w - 1, ok);
    out.y0 = clampCoord(root.get("y0"), map_h - 1, ok);
    out.x1 = clampCoord(root.get("x1"), map_w - 1, ok);
    out.y1 = clampCoord(root.get("y1"), map_h - 1, ok);
    if (!ok) return false;
    if (out.x0 > out.x1) { const int t = out.x0; out.x0 = out.x1; out.x1 = t; }
    if (out.y0 > out.y1) { const int t = out.y0; out.y0 = out.y1; out.y1 = t; }
    if (out.x0 < 0 || out.x0 >= map_w || out.x1 < 0 || out.x1 >= map_w ||
        out.y0 < 0 || out.y0 >= map_h || out.y1 < 0 || out.y1 >= map_h)
        return false;

    // ---- 时间 ----
    // 规则 §8.1：生效/结束时间以本场自然语言公告为准，不得假设不同场次
    // 使用相同日程。公告从裁判发出到 LLM 返回可能有延迟，因此：
    //   * 解析时 activate_tick 可以已经过去，但必须有模型给出的 end_tick，
    //     且 end_tick 仍在当前 Tick 之后——这证明公告所述效果仍然有效；
    //   * 瞬时事件（爆炸/分裂）过去即拒绝；
    //   * end_tick 缺失时不再猜“最近 8 Tick”之类的经验值：没有结束时间就
    //     无法证明事件仍有效，直接拒绝，避免把已结束事件当成有效。
    const JsonValue* jact = root.get("activate_tick");
    double act = 0.0;
    if (!numericValue(jact, act)) return false;
    out.activate_tick = static_cast<int>(act);
    const JsonValue* jend = root.get("end_tick");
    double endv = 0.0;
    if (numericValue(jend, endv) && endv >= 0) {
        out.end_tick = static_cast<int>(endv);
        if (out.end_tick <= out.activate_tick) return false;
    } else {
        out.end_tick = -1;
    }
    // 爆炸是一次性事件（规则 §8.2：激活 Tick 移动前一次性结算）：把 end
    // 写死成 activate+1，下游不用再猜“end<0 是不是永久有效”。事件池在爆炸
    // Tick 之后立即回收，避免 end=-1 的旧爆炸长期作为危险区 Veto reach。
    if (out.kind == EVK_EXPLOSION && out.activate_tick >= 0 &&
        out.end_tick < 0) {
        out.end_tick = out.activate_tick + 1;
    }
    if (out.activate_tick < current_tick) {
        const bool durationEvent =
            out.kind != EVK_EXPLOSION && out.kind != EVK_SPLIT;
        // 没有 end_tick 就没有规则依据证明“仍生效”；拒绝而不是猜。
        if (!durationEvent || out.end_tick < 0 || out.end_tick <= current_tick)
            return false;
    }
    return true;
}

}  // namespace rmtc
