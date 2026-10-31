#pragma once
#include <condition_variable>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

// 异步 LLM 客户端（问题 3 的"可行调用"部分）。
//
// 为什么必须异步：一个 Tick 只有 500 ms，而网关的上游超时是 30 s、读超时 5 s；
// 同步调用会直接错过动作窗口。这里把请求丢给后台线程，主线程每 Tick 只做
// 一次非阻塞 poll()，结果按 (session, seq, tick) 校验后使用。
//
// 三条硬约束（对应决策文本 §13.9 / §13.10）：
//   1. 去重：同一段公告文本只请求一次（配额 100 次要省）；
//      **资格握手是控制请求，显式绕过去重**，否则重试会被吞成 1 次；
//   2. 隔离：结果带 session 与 seq，换场/旧序号一律丢弃；
//   3. 时效：事件结果年龄超过 max_age_ticks 直接丢弃；握手结果不丢
//      （只要 2xx，它就已经是“本场成功调用记录”）。
//
// 传输层可注入：正式用 HTTP 实现（协议 §5），测试用假实现（可模拟延迟/超时/
// 429），因此本模块可以完全离线验证。
namespace rmtc {

struct LlmConfig {
    // 单次事件请求总超时。协议 §5：网关读取超时 5 s、上游超时默认 30 s；
    // 1.8 s（4 Tick）对 deepseek-flash 的 JSON 模式应答偏紧：网关冷启动或
    // 上游排队时会稳定超时，公告行反复重试直到事件过期。2026-10-30 改为 3 s
    // （6 Tick，异步执行，不占动作窗口），给真实网关留出余量。
    int timeout_ms = 3000;
    int max_queue = 8;        // 本地待发队列上限（超过就拒绝新的）
    // 注：当前实现是单 worker 串行发送，并发度 1；协议允许 2，串行更省配额也更安全。
    int max_age_ticks = 100;  // 结果年龄超过该值不再使用；网关上游 30s ≈ 60 Tick，
                              // 加上排队/读超时，40 Tick 会误丢迟到的持续型事件
    // 0 = 不自己设上限，完全由网关额度（429 quota_exceeded）兜底。
    // 协议：额度按席位令牌全生命周期累计、每场上限赛前公告、0 表示不限；
    // 旧实现硬编码 100，遇到“0=不限”或上限 >100 时会把还能用的额度提前掐掉。
    int quota = 0;
};

// 请求类型：事件公告解析 vs 资格握手。
// 握手是控制请求，必须绕过“同文本只发一次”的事件去重；否则重试会被静默吞掉。
enum LlmRequestKind {
    LLM_EVENT = 0,
    LLM_HANDSHAKE = 1
};

struct LlmResult {
    bool ok = false;
    std::string content;      // 模型输出（OpenAI 兼容 choices[0].message.content）
    int status = 0;           // HTTP 状态码；0 = 传输失败
    std::string error;        // 失败原因（本地错误码或响应片段）
    std::string session;      // 提交时的场次
    int tick = -1;            // 提交时的 Tick
    long seq = -1;            // 请求序号
    int latency_ms = 0;       // 实测耗时
    int kind = LLM_EVENT;     // 该结果属于哪个请求（握手/事件）
    // 该失败是否值得退避重试（429 rate_limit/concurrency_limit、传输失败、
    // 502/504）。quota_exceeded/401/404/400/413/501/503 不重试。
    bool retryable = false;
};

class LlmTransport {
public:
    virtual ~LlmTransport() {}
    // 返回 false 表示连接失败；status/err 由实现填写。
    virtual bool post(const std::string& url, const std::string& body,
                      const std::string& token, int timeout_ms, int& status,
                      std::string& response, std::string& err) = 0;
};

class LlmClient {
public:
    LlmClient();
    ~LlmClient();

    void configure(const LlmConfig& c);
    void setEndpoint(const std::string& url, const std::string& token);
    void setTransport(LlmTransport* t);  // 不接管所有权

    void start();
    void stop();

    // 换场时清空去重表（同一段文本在新场次要重新解析）。
    void resetSession();

    // 非阻塞提交。返回请求序号；0 = 被拒绝（队列满/额度用尽/重复）。
    // dup 为 true 表示命中文本去重（未真正提交）。
    // kind=LLM_HANDSHAKE 时不做文本去重（资格握手必须能重试），并可使用独立超时；
    // timeout_override_ms > 0 时覆盖 LlmConfig.timeout_ms（仅本次请求）。
    long submit(const std::string& session, int tick, const std::string& text,
                bool* dup = 0, int kind = LLM_EVENT, int timeout_override_ms = -1);

    // 非阻塞取结果。now_tick 用于判定结果是否过期；过期结果被丢弃并计数。
    bool poll(int now_tick, LlmResult& out);

    int inflight() const;
    int quotaLeft() const;
    int deduped() const;
    int droppedStale() const;
    int droppedQueue() const;
    int failed() const;

private:
    void workerLoop();
    void pushResult(const LlmResult& r);

    struct Job {
        long seq;
        std::string session;
        std::string text;
        int tick;
        int kind;
        int timeout_override_ms;
    };

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Job> queue_;
    std::deque<LlmResult> done_;
    std::set<std::size_t> seen_;   // 文本 hash（去重）
    // 2 个 worker：协议允许每个席位令牌同时存在 2 个在途调用。旧实现单
    // worker 串行，一个 3s 的事件请求会把到期的资格握手堵 3s，连续多条
    // 公告也要逐条等待；两个 worker 让“握手 + 事件”与“公告 A + 公告 B”
    // 能真正并发（入队仍按握手优先）。
    std::vector<std::thread> workers_;
    bool running_;

    LlmConfig cfg_;
    std::string url_;
    std::string token_;
    LlmTransport* transport_;

    long next_seq_;
    int inflight_;
    int quota_used_;
    int deduped_;
    int dropped_stale_;
    int dropped_queue_;
    int failed_;
};
}  // namespace rmtc
