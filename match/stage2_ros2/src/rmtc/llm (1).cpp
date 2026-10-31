// rmtc/llm.cpp —— 异步 LLM 客户端实现
//
// 线程模型：一个后台 worker + 两个队列（待发 / 已完成），一把互斥锁。
// 主线程路径（submit / poll）都是 O(1) 且不会阻塞在网络 I/O 上。
#include "rmtc/llm.h"

#include <chrono>
#include <cstddef>
#include <cstdio>

#include "rmtc/json.h"

namespace rmtc {

namespace {
// 简单的 FNV-1a：用于文本去重（不需要密码学强度）。
std::size_t hashText(const std::string& s) {
    std::size_t h = 1469598103934665603ULL;
    for (std::size_t i = 0; i < s.size(); ++i) {
        h ^= static_cast<unsigned char>(s[i]);
        h *= 1099511628211ULL;
    }
    return h;
}
}  // namespace

LlmClient::LlmClient()
    : running_(false), transport_(NULL), next_seq_(1), inflight_(0),
      quota_used_(0), deduped_(0), dropped_stale_(0), dropped_queue_(0),
      failed_(0) {}

LlmClient::~LlmClient() { stop(); }

void LlmClient::configure(const LlmConfig& c) { cfg_ = c; }

void LlmClient::setEndpoint(const std::string& url, const std::string& token) {
    url_ = url;
    token_ = token;
}

void LlmClient::setTransport(LlmTransport* t) { transport_ = t; }

void LlmClient::start() {
    std::lock_guard<std::mutex> lk(mu_);
    if (running_) return;
    running_ = true;
    // 协议：每个席位令牌最多同时存在 2 个在途调用。开两个 worker 让握手与
    // 事件互不阻塞；workerLoop 的入队仍保持“握手优先”。
    const int kWorkers = 2;
    for (int i = 0; i < kWorkers; ++i)
        workers_.push_back(std::thread(&LlmClient::workerLoop, this));
}

void LlmClient::stop() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (!running_) return;
        running_ = false;
    }
    cv_.notify_all();
    for (std::size_t i = 0; i < workers_.size(); ++i) {
        if (workers_[i].joinable()) workers_[i].join();
    }
    workers_.clear();
}

void LlmClient::resetSession() {
    std::lock_guard<std::mutex> lk(mu_);
    // 新 session = 新一场比赛：文本去重、待发/已完成队列、配额都必须按场清零。
    // 旧实现只清 seen_，同一进程连打多场时第一场用满 quota 后，
    // 后续场次会拒绝所有 AI 调用 → 直接 M=0。
    seen_.clear();
    queue_.clear();
    done_.clear();
    quota_used_ = 0;
    // 诊断计数（deduped/failed/dropped_*）保留为整个进程的累计值，便于赛后看总账。
}

long LlmClient::submit(const std::string& session, int tick, const std::string& text,
                       bool* dup, int kind, int timeout_override_ms) {
    if (dup) *dup = false;
    std::lock_guard<std::mutex> lk(mu_);
    if (!running_) return 0;

    // 只对事件公告做文本去重。资格握手是控制请求，重试必须真的发出去；
    // 早期版本没区分，导致 6 次“重试”里只有第 1 次进入网络（P0 根因）。
    std::size_t h = 0;
    const bool isEvent = (kind == LLM_EVENT);
    if (isEvent) {
        h = hashText(text);
        if (seen_.find(h) != seen_.end()) {
            ++deduped_;
            if (dup) *dup = true;
            return 0;
        }
    }

    // quota<=0 表示不限（与 LlmConfig 注释一致）；旧实现 0 会拒绝所有调用。
    if (cfg_.quota > 0 && quota_used_ >= cfg_.quota) return 0;
    // 握手允许比普通事件多排 2 个，避免队列被事件占满时丢掉资格保险丝。
    const int queue_limit = cfg_.max_queue + (kind == LLM_HANDSHAKE ? 2 : 0);
    if (static_cast<int>(queue_.size()) >= queue_limit) {
        ++dropped_queue_;
        return 0;
    }
    if (isEvent) seen_.insert(h);
    Job j;
    j.seq = next_seq_++;
    j.session = session;
    j.text = text;
    j.tick = tick;
    j.kind = kind;
    j.timeout_override_ms = timeout_override_ms;
    queue_.push_back(j);
    cv_.notify_one();
    return j.seq;
}

bool LlmClient::poll(int now_tick, LlmResult& out) {
    std::lock_guard<std::mutex> lk(mu_);
    while (!done_.empty()) {
        LlmResult r = done_.front();
        done_.pop_front();
        // 事件结果过期就不能用于决策；握手结果不丢：只要它是 2xx，
        // 它已经是“本场成功调用记录”，晚到也必须让 Agent 落资格。
        if (r.kind != LLM_HANDSHAKE && cfg_.max_age_ticks > 0 &&
            now_tick - r.tick > cfg_.max_age_ticks) {
            ++dropped_stale_;
            continue;  // 过期结果直接丢弃，继续找下一个
        }
        out = r;
        return true;
    }
    return false;
}

int LlmClient::inflight() const {
    std::lock_guard<std::mutex> lk(mu_);
    return inflight_;
}
int LlmClient::quotaLeft() const {
    std::lock_guard<std::mutex> lk(mu_);
    if (cfg_.quota <= 0) return -1;  // 不限
    return cfg_.quota - quota_used_;
}
int LlmClient::deduped() const {
    std::lock_guard<std::mutex> lk(mu_);
    return deduped_;
}
int LlmClient::droppedStale() const {
    std::lock_guard<std::mutex> lk(mu_);
    return dropped_stale_;
}
int LlmClient::droppedQueue() const {
    std::lock_guard<std::mutex> lk(mu_);
    return dropped_queue_;
}
int LlmClient::failed() const {
    std::lock_guard<std::mutex> lk(mu_);
    return failed_;
}

void LlmClient::pushResult(const LlmResult& r) {
    std::lock_guard<std::mutex> lk(mu_);
    done_.push_back(r);
}

void LlmClient::workerLoop() {
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [this]() { return !running_ || !queue_.empty(); });
            if (!running_) return;
            // 资格握手是控制请求：队列里只要有一笔，就先发它。
            // 单 worker 无法抢占"已在网络中的事件"，但可以保证排队顺序——
            // 否则连续事件会把重试的握手压到很后面，开场 120 Tick 内拿不到资格。
            std::size_t pick = 0;
            for (std::size_t i = 0; i < queue_.size(); ++i) {
                if (queue_[i].kind == LLM_HANDSHAKE) {
                    pick = i;
                    break;
                }
            }
            job = queue_[pick];
            queue_.erase(queue_.begin() + static_cast<std::ptrdiff_t>(pick));
            ++inflight_;
        }

        LlmResult r;
        r.session = job.session;
        r.tick = job.tick;
        r.seq = job.seq;
        r.kind = job.kind;

        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        bool http_ok = false;
        bool conn = false;  // 传输层是否拿到完整响应（false = 超时/断连/截断）
        if (transport_) {
            int status = 0;
            std::string resp, err;
            const int timeout_ms =
                job.timeout_override_ms > 0 ? job.timeout_override_ms : cfg_.timeout_ms;
            conn = transport_->post(url_, job.text, token_, timeout_ms,
                                    status, resp, err);
            r.status = status;
            r.error = err;
            http_ok = conn && status >= 200 && status < 300;
            if (http_ok) {
                // OpenAI 兼容响应：choices[0].message.content。
                // 解析失败不能当成成功（协议 §5：HTTP 成功 ≠ 策略正确）。
                JsonValue j;
                std::string jerr;
                if (!jsonParse(resp, j, jerr) || !j.isObj()) {
                    r.error = "bad response json";
                } else {
                    const JsonValue* ch = j.get("choices");
                    if (!ch || !ch->isArr() || ch->arr.empty()) {
                        r.error = "missing choices";
                    } else {
                        const JsonValue* msg = ch->arr[0].get("message");
                        const JsonValue* content = msg ? msg->get("content") : NULL;
                        if (!content || !content->isStr()) {
                            r.error = "missing content";
                        } else {
                            r.ok = true;
                            r.content = content->str;
                        }
                    }
                }
            } else if (conn) {
                // 非 2xx：把响应体带给上层——429 的 quota_exceeded/rate_limit
                // 区分就靠它（协议 §5）
                r.error = resp;
            } else if (r.error.empty()) {
                r.error = "connection failed";
            }
        } else {
            r.error = "no transport";
        }
        // 可退避重试的失败：传输层超时/断连、429 rate_limit/concurrency_limit、
        // 502 upstream、504 timeout。quota_exceeded/401/404/400/413/501/503 不重试。
        if (!http_ok) {
            if (!conn) {
                // 传输层失败优先于状态行：body 被截断时可能已经解析出 200，
                // 但那不是一次可用响应（HttpTransport 已把 status 归 0）。
                r.retryable = true;
            } else if (r.status == 429 &&
                       (r.error.find("rate_limit") != std::string::npos ||
                        r.error.find("concurrency_limit") != std::string::npos)) {
                r.retryable = true;
            } else if (r.status == 502 || r.status == 504) {
                r.retryable = true;
            }
            // 事件失败后必须把文本 hash 从去重表里移除，否则下一次 submit 会被
            // 当成“重复文本”直接吞掉（旧实现只打日志，实际不会重试）。
            if (r.retryable && job.kind == LLM_EVENT) {
                std::lock_guard<std::mutex> lk(mu_);
                seen_.erase(hashText(job.text));
            }
        } else if (!r.ok && job.kind == LLM_EVENT) {
            // 2xx 但 content 不可用（空 content / 响应信封异常）：Agent 会重试
            // 同一公告行，这里必须放行去重表，否则重试会被静默吞掉。
            std::lock_guard<std::mutex> lk(mu_);
            seen_.erase(hashText(job.text));
        }
        r.latency_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count());

        {
            std::lock_guard<std::mutex> lk(mu_);
            if (--inflight_ < 0) inflight_ = 0;
            if (http_ok) ++quota_used_;  // 协议：2xx 即消耗额度（无论内容是否可用）
            if (!r.ok) ++failed_;
        }
        pushResult(r);
    }
}

}  // namespace rmtc
