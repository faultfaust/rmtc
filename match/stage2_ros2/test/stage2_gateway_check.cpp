// stage2_gateway_check.cpp —— 真网关连通性/资格/事件解析自检（不依赖 ROS）
//
// 用途：赛前用真网关的 URL + 本席位令牌，验证四件事：
//   1) 网络可达、令牌有效（401/403 会直接暴露）；
//   2) 握手请求能拿到合法 OpenAI 响应（M=0 的保险丝）；
//   3) 连续握手不会被网关当成重复请求拒掉；
//   4) 事件解析 prompt 能返回可被 parseLlmEvent 接受的 JSON。
//
// 用法（Git Bash / Linux）：
//   export RMTC_AI_URL='http://<裁判机IP>:8080/v1/chat/completions'  # 可省略，默认本机 8080
//   export RMTC_TEAM_TOKEN='practice-A'  # 或 RMTC_AI_TOKEN
//   # 可选：export RMTC_AI_MODEL='<赛前公布的模型>'
//   ../build/gateway_check                 # 默认 2 次握手 + 1 次事件
//   ../build/gateway_check --repeat=3 --timeout=5000
//   ../build/gateway_check --event="10 Tick 后，(8,8)-(12,12) 发生爆炸"
//
// 退出码：0=握手+事件都通过；2=配置/用法错误；3=传输/HTTP 失败；4=握手通过但事件解析失败。
// 注意：令牌绝不打印；URL 的 query 部分会被截掉再显示。
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "rmtc/event.h"
#include "rmtc/event_prompt.h"
#include "rmtc/http_transport.h"
#include "rmtc/json.h"

namespace {

std::string envOr(const char* name, const std::string& fallback = "") {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
}

std::string oneLine(const std::string& s, std::size_t maxLen = 240) {
    std::string out;
    for (std::size_t i = 0; i < s.size() && out.size() < maxLen; ++i) {
        const char c = s[i];
        if (c == '\n') out += "\\n";
        else if (c == '\r') continue;
        else if (c == '\t') out += ' ';
        else out += c;
    }
    if (s.size() > maxLen) out += "...";
    return out;
}

std::string safeUrl(const std::string& url) {
    const std::size_t q = url.find('?');
    return q == std::string::npos ? url : url.substr(0, q);
}

bool extractContent(const std::string& resp, std::string& content, std::string& err) {
    rmtc::JsonValue root;
    if (!rmtc::jsonParse(resp, root, err) || !root.isObj()) {
        if (err.empty()) err = "response is not a JSON object";
        return false;
    }
    const rmtc::JsonValue* ch = root.get("choices");
    if (!ch || !ch->isArr() || ch->arr.empty()) {
        err = "missing choices[0]";
        return false;
    }
    const rmtc::JsonValue* msg = ch->arr[0].get("message");
    const rmtc::JsonValue* c = msg ? msg->get("content") : NULL;
    if (!c || !c->isStr()) {
        err = "missing choices[0].message.content";
        return false;
    }
    content = c->str;
    return true;
}

struct PostResult {
    bool connected = false;
    int status = 0;
    int latency_ms = 0;
    std::string body;
    std::string error;
};

PostResult post(const std::string& url, const std::string& token,
                const std::string& body, int timeout_ms) {
    PostResult r;
    rmtc::HttpTransport tr;
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const bool ok = tr.post(url, body, token, timeout_ms, r.status, r.body, r.error);
    r.connected = ok;
    r.latency_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0)
            .count());
    return r;
}

bool reportHttp(const char* label, const PostResult& r) {
    std::printf("[%s] connect=%s http=%d latency=%dms\n", label,
                r.connected ? "yes" : "NO", r.status, r.latency_ms);
    if (!r.connected) {
        std::printf("  transport error: %s\n", r.error.c_str());
        return false;
    }
    if (r.status < 200 || r.status >= 300) {
        std::printf("  HTTP error: %s\n", oneLine(r.body).c_str());
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::string url = envOr("RMTC_AI_URL",
                            "http://127.0.0.1:8080/v1/chat/completions");
    std::string token = envOr("RMTC_TEAM_TOKEN", envOr("RMTC_AI_TOKEN"));
    std::string model = envOr("RMTC_AI_MODEL", "deepseek-flash");
    std::string eventText =
        "10 Tick 后，坐标 (8,8) 到 (12,12) 的矩形区域发生爆炸。";
    int repeat = 2;
    int timeoutMs = 5000;
    int tick = 50;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            std::printf("usage: gateway_check [--repeat=N] [--timeout=MS] [--tick=N] "
                        "[--event=TEXT] [--model=NAME]\n"
                        "env: RMTC_AI_URL (default 127.0.0.1:8080), "
                        "RMTC_AI_TOKEN or RMTC_TEAM_TOKEN, RMTC_AI_MODEL(optional, "
                        "default deepseek-flash)\n");
            return 0;
        } else if (a.compare(0, 9, "--repeat=") == 0) {
            repeat = std::atoi(a.c_str() + 9);
        } else if (a.compare(0, 10, "--timeout=") == 0) {
            timeoutMs = std::atoi(a.c_str() + 10);
        } else if (a.compare(0, 7, "--tick=") == 0) {
            tick = std::atoi(a.c_str() + 7);
        } else if (a.compare(0, 8, "--event=") == 0) {
            eventText = a.substr(8);
        } else if (a.compare(0, 8, "--model=") == 0) {
            model = a.substr(8);
        } else {
            std::printf("unknown arg: %s (use --help)\n", a.c_str());
            return 2;
        }
    }

    if (url.empty() || token.empty()) {
        std::printf("ERROR: RMTC_AI_URL / RMTC_AI_TOKEN not set.\n");
        return 2;
    }
    if (url.rfind("http://", 0) != 0) {
        std::printf("ERROR: this transport only supports http:// (got %.16s...).\n",
                    url.c_str());
        return 2;
    }
    if (repeat < 1) repeat = 1;
    if (repeat > 8) repeat = 8;
    if (timeoutMs < 200) timeoutMs = 200;

    std::printf("endpoint : %s\n", safeUrl(url).c_str());
    std::printf("token    : set (not printed)\n");
    std::printf("model    : %s\n", model.c_str());
    std::printf("timeout  : %d ms, handshake repeats=%d, tick=%d\n\n", timeoutMs,
                repeat, tick);

    bool handshakeOk = true;
    for (int i = 0; i < repeat; ++i) {
        const std::string body = rmtc::buildHandshakeRequestBody(tick + i, model);
        const PostResult r = post(url, token, body, timeoutMs);
        char label[32];
        std::snprintf(label, sizeof(label), "handshake %d/%d", i + 1, repeat);
        if (!reportHttp(label, r)) { handshakeOk = false; continue; }
        std::string content, err;
        if (!extractContent(r.body, content, err)) {
            std::printf("  invalid OpenAI body: %s\n", err.c_str());
            std::printf("  body: %s\n", oneLine(r.body).c_str());
            handshakeOk = false;
            continue;
        }
        std::printf("  content: %s\n", oneLine(content).c_str());
    }
    if (!handshakeOk) {
        std::printf("\nRESULT: HANDSHAKE FAILED — 检查 token/URL/网关；M=0 风险。\n");
        return 3;
    }

    const PostResult ev = post(
        url, token, rmtc::buildEventRequestBody(tick, eventText, model), timeoutMs);
    if (!reportHttp("event", ev)) {
        std::printf("\nRESULT: HANDSHAKE OK, EVENT HTTP FAILED — 检查额度/模型/请求体。\n");
        return 3;
    }
    std::string content, err;
    if (!extractContent(ev.body, content, err)) {
        std::printf("  invalid OpenAI body: %s\n", err.c_str());
        std::printf("  body: %s\n", oneLine(ev.body).c_str());
        std::printf("\nRESULT: HANDSHAKE OK, EVENT BODY INVALID — 资格可用，事件解析不可用。\n");
        return 4;
    }
    std::printf("  content: %s\n", oneLine(content).c_str());

    rmtc::EventReport rep;
    if (!rmtc::parseLlmEvent(content, tick, 24, 24, 0.5f, rep)) {
        std::printf("\nRESULT: HANDSHAKE OK, EVENT PARSE REJECTED — 模型输出不符合 schema。\n");
        return 4;
    }
    std::printf("  parsed: kind=%s rect(%d,%d)-(%d,%d) activate=%d conf=%.2f\n",
                rmtc::eventKindName(rep.kind), rep.x0, rep.y0, rep.x1, rep.y1,
                rep.activate_tick, rep.confidence);
    std::printf("\nRESULT: GATEWAY VERIFIED (handshake 2xx + event parsed)\n");
    return 0;
}
