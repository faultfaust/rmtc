#pragma once
#include <cstddef>
#include <string>
#include "rmtc/event.h"

// 事件 Prompt 组装 + LLM 输出解析（网关清单 §2）。
//
// 合规边界（规则 §8.1）：自然语言 → 结构化必须由大模型完成；
// 本文件只负责 (a) 把公告原文原样装进 OpenAI 请求体、(b) 把模型输出
// 解析回 EventReport 并做硬校验。绝不做正则/关键词式的事件理解。
namespace rmtc {

// 组装 /v1/chat/completions 的完整请求体（LlmClient 直接 POST 它）。
//   system：角色约束 + 输出 JSON schema + 禁止 Markdown；
//   user  ：当前 Tick + 公告原文（原样，不改写）。
// model 为空串时用默认 "deepseek-flash"（网关统一模型）。
//
// 2026-10-28：正式网关不会主动关闭思考，请求顶层必须带
//   "thinking":{"type":"disabled"}
// 否则 deepseek-flash 默认开思考，延迟/输出形态都不适合 500ms 的 Tick。
// 同时带 "response_format":{"type":"json_object"}，让上游直接按 JSON 约束输出。
std::string buildEventRequestBody(int tick, const std::string& event_text,
                                  const std::string& model);

// 资格握手请求体（保住本场 AI 资格，规则 §10.2）——最短、无害。
// 带 tick 让每次重试的文本唯一：既避免客户端/网关把重复文本当成同一次调用，
// 也方便日志对账。模型仍只需回 OK；同样关闭思考、max_tokens 压到 16。
std::string buildHandshakeRequestBody(int tick,
                                      const std::string& model = "deepseek-flash");

// 解析模型输出 → EventReport，带硬校验（网关清单 §2.4/§3）：
//   1. 剥离 Markdown 围栏/前后缀废话（取第一个 '{' 到最后一个 '}'）；
//   2. jsonParse + 字段类型强校验（isNum/isStr，不盲目转换）；
//   3. kind ∈ 8 类枚举；
//   4. 坐标防幻觉：0 ≤ x0 ≤ x1 < map_w、0 ≤ y0 ≤ y1 < map_h；
//   5. 时间防算错：activate_tick ≥ current_tick；过去的瞬时报 STALE，
//      持续事件若 end_tick 仍在未来则接受（迟到但仍有效）。
//   6. confidence ≥ min_confidence（低于阈值视为未解析）。
// 任一不过 → 返回 false（调用方重试/按未知危险保守处理）。
// 解析结果分类：调用方据此区分“真错误”和“正常的无事件/迟到”。
//   * OK       ：可用事件；
//   * NO_EVENT ：模型明确说 unknown，或置信度低于阈值（“已结束/已解除”等）；
//   * STALE    ：结构合法，但 activate_tick 已经过去（重复公告/迟到的爆炸）；
//   * INVALID  ：JSON/字段/坐标/时间窗口非法（真正需要告警的模型错误）。
enum EventParseStatus {
    EVENT_PARSE_OK = 0,
    EVENT_PARSE_NO_EVENT,
    EVENT_PARSE_STALE,
    EVENT_PARSE_INVALID
};

bool parseLlmEvent(const std::string& content, int current_tick,
                   int map_w, int map_h, float min_confidence,
                   EventReport& out, EventParseStatus* status = NULL);

}  // namespace rmtc
