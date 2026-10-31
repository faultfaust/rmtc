// rmtc/http_transport.cpp —— 最小 HTTP/1.1 POST（socket，Win/POSIX 双平台）
//
// 只实现网关需要的能力：http:// POST + Bearer 头 + Content-Length +
// **整请求总截止** + 读完整响应。不支持 HTTPS/代理/重定向（协议场景不需要）。
//
// 2026-10-28 修复（稳定性清单 #5）：
//   * 旧实现只有 SO_RCVTIMEO/SO_SNDTIMEO：每次 recv 各自超时，慢速滴流的响应
//     可以把一次请求拖到 N × timeout；connect 也不受 SO_SNDTIMEO 约束。
//   * 现在从调用开始记 steady_clock 总截止：DNS 之后、connect（非阻塞+poll）、
//     send、每次 recv 都按"剩余时间"设置超时；到点立即失败。
//   * 响应头一到就解析 Content-Length，body 收满立即返回，不再等对端 close。
//   * host 是数字 IPv4 时跳过 getaddrinfo（DNS 没有可移植的超时接口，见 §5）。
//
// 响应读取：优先按 Content-Length；无该头则读到连接关闭或总截止。
//
// MinGW 注意：ws2tcpip.h 只在 _WIN32_WINNT ≥ 0x0600 时声明 inet_ntop/
// inet_pton；不定义会编译失败。同时 MinGW 忽略 MSVC 的 #pragma comment(lib)，
// 链接时必须显式加 -lws2_32（见 README/修改报告第 7 节）。
#if defined(_WIN32)
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif
#endif

#include "rmtc/http_transport.h"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <ws2ipdef.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
typedef long ssize_t_http;
#define RMTC_SOCK_CLOSE closesocket
#define RMTC_SOCK_ERR WSAGetLastError()
#define RMTC_WOULDBLOCK(e) ((e) == WSAEWOULDBLOCK || (e) == WSAEINPROGRESS)
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
typedef ssize_t ssize_t_http;
#define RMTC_SOCK_CLOSE ::close
#define RMTC_SOCK_ERR errno
#define RMTC_WOULDBLOCK(e) ((e) == EWOULDBLOCK || (e) == EAGAIN || (e) == EINPROGRESS)
#endif

namespace rmtc {

namespace {

typedef std::chrono::steady_clock Clock;

int remainingMs(Clock::time_point deadline) {
    const long long ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now())
            .count();
    if (ms <= 0) return 0;
    if (ms > 3600000) return 3600000;
    return static_cast<int>(ms);
}

// URL 解析：http://host[:port]/path。只支持 http。
bool parseUrl(const std::string& url, std::string& host, std::string& port,
              std::string& path) {
    const std::string scheme = "http://";
    if (url.compare(0, scheme.size(), scheme) != 0) return false;
    const std::size_t hostBegin = scheme.size();
    const std::size_t slash = url.find('/', hostBegin);
    std::string authority = (slash == std::string::npos) ? url.substr(hostBegin)
                                                         : url.substr(hostBegin, slash - hostBegin);
    path = (slash == std::string::npos) ? "/" : url.substr(slash);
    const std::size_t colon = authority.rfind(':');
    if (colon != std::string::npos) {
        host = authority.substr(0, colon);
        port = authority.substr(colon + 1);
    } else {
        host = authority;
        port = "80";
    }
    return !host.empty() && !port.empty();
}

// 平台套接字初始化：进程级一次（C++11 魔法静态保证线程安全），
// 绝不在调用中途 WSACleanup——那会把其它线程还开着的 socket 一起拆掉
// （实测 flake 根因：每次 POST 的栈式 init/cleanup 与并发 socket 冲突）。
inline bool ensureSockets() {
#ifdef _WIN32
    static const bool ok = []() {
        WSADATA w;
        return WSAStartup(MAKEWORD(2, 2), &w) == 0;
    }();
    return ok;
#else
    return true;
#endif
}

// 解析 host[:port] → IPv4 地址字符串。
//
// getaddrinfo 没有可移植的超时参数：DNS 卡住时整个 LlmClient worker 会被
// 拖死，后续公告全部排队到过期（表现为“LLM 调用失败/事件没解析出来”）。
// 这里把 DNS 放进后台线程，调用方最多等 kDnsTimeoutMs（且不超过本次请求
// 的总截止），超时就按失败返回，并给该 host 30 秒冷却，避免每个请求都
// 新建一个卡死的解析线程。host 已是数字 IPv4 时走快速路径，完全不触发 DNS。
namespace {
const int kDnsTimeoutMs = 1500;

struct DnsCache {
    std::mutex mu;
    std::string host;
    std::string ip;
    bool valid = false;
    Clock::time_point fail_until = Clock::time_point::min();
};

DnsCache& dnsCache() {
    static DnsCache c;
    return c;
}

struct DnsJob {
    std::mutex m;
    std::condition_variable cv;
    bool done = false;
    bool ok = false;
    std::string ip;
};

bool resolveHostBlocking(const std::string& host, std::string& ip) {
    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo* res = NULL;
    if (getaddrinfo(host.c_str(), NULL, &hints, &res) != 0 || !res) return false;
    char buf[INET_ADDRSTRLEN] = {0};
    if (!inet_ntop(AF_INET, &((struct sockaddr_in*)res->ai_addr)->sin_addr,
                   buf, sizeof(buf))) {
        freeaddrinfo(res);
        return false;
    }
    ip = buf;
    freeaddrinfo(res);
    return true;
}

bool resolveHost(const std::string& host, std::string& ip,
                 Clock::time_point deadline) {
    struct in_addr numeric;
    if (inet_pton(AF_INET, host.c_str(), &numeric) == 1) {
        ip = host;
        return true;
    }
    {
        std::lock_guard<std::mutex> lk(dnsCache().mu);
        if (dnsCache().valid && dnsCache().host == host) {
            ip = dnsCache().ip;
            return true;
        }
        if (Clock::now() < dnsCache().fail_until) return false;  // 冷却中，快速失败
    }

    std::shared_ptr<DnsJob> job(new DnsJob());
    std::thread([job, host]() {
        const bool ok = resolveHostBlocking(host, job->ip);
        std::lock_guard<std::mutex> lk(job->m);
        job->ok = ok;
        job->done = true;
        job->cv.notify_all();
    }).detach();

    int waitMs = remainingMs(deadline);
    if (waitMs > kDnsTimeoutMs) waitMs = kDnsTimeoutMs;
    if (waitMs <= 0) waitMs = 1;
    {
        std::unique_lock<std::mutex> lk(job->m);
        if (!job->cv.wait_for(lk, std::chrono::milliseconds(waitMs),
                              [&job]() { return job->done; })) {
            std::lock_guard<std::mutex> g(dnsCache().mu);
            dnsCache().fail_until = Clock::now() + std::chrono::seconds(30);
            return false;  // 解析线程会自行结束，不 join（不能阻塞动作窗口）
        }
    }
    if (!job->ok) {
        std::lock_guard<std::mutex> g(dnsCache().mu);
        dnsCache().fail_until = Clock::now() + std::chrono::seconds(30);
        return false;
    }
    ip = job->ip;
    {
        std::lock_guard<std::mutex> g(dnsCache().mu);
        dnsCache().host = host;
        dnsCache().ip = ip;
        dnsCache().valid = true;
    }
    return true;
}
}  // namespace

// 把阻塞超时设成"剩余时间"；每次 send/recv 前都要重设，
// 否则前一次操作吃掉的预算不会计入后续操作。
//
// Windows 的 SO_RCVTIMEO/SO_SNDTIMEO 要的是 DWORD 毫秒（不是 BSD timeval）；
// 旧实现传 timeval，Winsock 只读前 4 字节 = tv_sec：
//   1800ms → 1ms（真实网关必超时）、600ms → 0（无限等待）。
// 这是"本地 mock 能通、真实网关调不通"的一个直接原因。
bool setBlockingTimeout(int fd, int ms) {
    if (ms <= 0) return false;
#if defined(_WIN32)
    const DWORD tv = static_cast<DWORD>(ms);
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));
#else
    struct timeval tv;
    tv.tv_sec = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));
#endif
    return true;
}

bool setNonBlocking(int fd, bool on) {
#if defined(_WIN32)
    u_long mode = on ? 1 : 0;
    return ioctlsocket(fd, FIONBIO, &mode) == 0;
#else
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    const int next = on ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    return fcntl(fd, F_SETFL, next) == 0;
#endif
}

// 非阻塞 connect + select 等待可写，受总截止约束。
bool connectWithDeadline(int fd, const struct sockaddr_in& addr,
                         Clock::time_point deadline) {
    if (!setNonBlocking(fd, true)) return false;
    const int rc = connect(fd, (const struct sockaddr*)&addr, sizeof(addr));
    if (rc != 0) {
        const int e = RMTC_SOCK_ERR;
        if (!RMTC_WOULDBLOCK(e)) return false;
        for (;;) {
            const int left = remainingMs(deadline);
            if (left <= 0) return false;
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            struct timeval tv;
            tv.tv_sec = left / 1000;
            tv.tv_usec = (left % 1000) * 1000;
            const int sel = select(fd + 1, NULL, &wfds, NULL, &tv);
            if (sel == 0) continue;  // 超时：回到循环用剩余时间判断
            if (sel < 0) return false;
            if (!FD_ISSET(fd, &wfds)) continue;
            int soErr = 0;
            socklen_t len = sizeof(soErr);
            if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (char*)&soErr, &len) != 0)
                return false;
            if (soErr != 0) return false;
            break;
        }
    }
    return setNonBlocking(fd, false);
}

bool sendAll(int fd, const char* data, std::size_t len,
             Clock::time_point deadline) {
    std::size_t sent = 0;
    while (sent < len) {
        const int left = remainingMs(deadline);
        if (left <= 0 || !setBlockingTimeout(fd, left)) return false;
        const ssize_t_http n = send(fd, data + sent, len - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<std::size_t>(n);
    }
    return true;
}

// 从响应头里取 Content-Length；没有返回 -1（读到连接关闭为止）
long contentLengthOf(const std::string& headerBlock) {
    // 大小写不敏感查找
    std::string lower;
    lower.reserve(headerBlock.size());
    for (std::size_t i = 0; i < headerBlock.size(); ++i) {
        char c = headerBlock[i];
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        lower += c;
    }
    const std::size_t p = lower.find("content-length:");
    if (p == std::string::npos) return -1;
    const std::size_t digitStart = lower.find_first_not_of(" \t", p + 15);
    if (digitStart == std::string::npos) return -1;
    std::size_t digitEnd = lower.find_first_not_of("0123456789", digitStart);
    if (digitEnd == std::string::npos) digitEnd = lower.size();
    return std::atol(lower.substr(digitStart, digitEnd - digitStart).c_str());
}

}  // namespace

bool HttpTransport::post(const std::string& url, const std::string& body,
                         const std::string& token, int timeout_ms, int& status,
                         std::string& response, std::string& err) {
    status = 0;
    response.clear();
    err.clear();

    const int budget = timeout_ms > 0 ? timeout_ms : 3000;
    const Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(budget);

    std::string host, port, path;
    if (!parseUrl(url, host, port, path)) {
        err = "url must be http://host[:port]/path";
        return false;
    }
    // 现场可能只公布 <GATEWAY> 根地址（协议 §5 的写法），也可能给完整
    // /v1/chat/completions。只给根地址时补全，避免 404 not_found。
    if (path.empty() || path == "/") path = "/v1/chat/completions";

    if (!ensureSockets()) {
        err = "socket init failed";
        return false;
    }

    std::string ip;
    if (!resolveHost(host, ip, deadline)) {
        err = "dns failed";
        return false;
    }
    if (remainingMs(deadline) <= 0) {
        err = "deadline before connect";
        return false;
    }

    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        err = "socket failed";
        return false;
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<unsigned short>(std::atoi(port.c_str())));
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        RMTC_SOCK_CLOSE(fd);
        err = "bad address";
        return false;
    }
    if (!connectWithDeadline(fd, addr, deadline)) {
        RMTC_SOCK_CLOSE(fd);
        err = remainingMs(deadline) <= 0 ? "connect timeout" : "connect failed";
        return false;
    }

    // 请求：Content-Length 必带（网关对 Transfer-Encoding 返回 501）
    std::string req = "POST " + path + " HTTP/1.1\r\n";
    req += "Host: " + host;
    if (port != "80") req += ":" + port;
    req += "\r\n";
    req += "Content-Type: application/json\r\n";
    if (!token.empty()) req += "Authorization: Bearer " + token + "\r\n";
    req += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    req += "Connection: close\r\n";
    req += "\r\n";

    if (!sendAll(fd, req.c_str(), req.size(), deadline) ||
        !sendAll(fd, body.c_str(), body.size(), deadline)) {
        err = remainingMs(deadline) <= 0 ? "send timeout" : "send failed";
        RMTC_SOCK_CLOSE(fd);
        return false;
    }

    // 读响应：边收边判断"头结束 + body 收满"，收满立即返回（不等对端 close）。
    std::string raw;
    char buf[4096];
    bool readErr = false;
    bool timedOut = false;
    bool headerDone = false;
    long contentLen = -1;
    std::size_t headerEnd = 0;
    for (;;) {
        if (headerDone && contentLen >= 0 &&
            raw.size() >= headerEnd + 4 + static_cast<std::size_t>(contentLen))
            break;  // body 已完整
        const int left = remainingMs(deadline);
        if (left <= 0) {
            timedOut = true;
            readErr = true;
            break;
        }
        if (!setBlockingTimeout(fd, left)) {
            readErr = true;
            break;
        }
        const ssize_t_http n = recv(fd, buf, sizeof(buf), 0);
        if (n > 0) {
            raw.append(buf, static_cast<std::size_t>(n));
            if (!headerDone) {
                const std::size_t he = raw.find("\r\n\r\n");
                if (he != std::string::npos) {
                    headerDone = true;
                    headerEnd = he;
                    contentLen = contentLengthOf(raw.substr(0, he));
                }
            }
            // 防御：响应超过 8 MiB 直接视为异常（正常响应几 KB）
            if (raw.size() > 8u * 1024 * 1024) {
                readErr = true;
                break;
            }
        } else if (n == 0) {
            break;  // 对端关闭
        } else {
            // 超时/错误：保留已收到的部分用于诊断
            if (remainingMs(deadline) <= 0) timedOut = true;
            readErr = true;
            break;
        }
    }
    RMTC_SOCK_CLOSE(fd);

    // 解析状态行
    const std::size_t sp1 = raw.find(' ');
    if (sp1 == std::string::npos) {
        err = timedOut ? "recv timeout" : (readErr ? "recv failed" : "bad response");
        return false;
    }
    status = std::atoi(raw.c_str() + sp1 + 1);

    if (!headerDone) {
        err = timedOut ? "recv timeout" : "bad response (no header end)";
        return false;
    }
    const std::string headerBlock = raw.substr(0, headerEnd);
    std::string bodyOut = raw.substr(headerEnd + 4);

    // 无 Content-Length 且 Connection: close：已读到关闭即为完整 body；
    // 有 Content-Length：截断多余部分（正常情况）
    if (contentLen >= 0 && static_cast<long>(bodyOut.size()) > contentLen) {
        bodyOut.resize(static_cast<std::size_t>(contentLen));
    }

    // 读到一半失败（超时/断连）时，响应头里的 200 不代表 body 完整：
    // 旧实现把“已解析出状态行但 body 被截断”当成成功返回，上层会
    //   * 误记一次 2xx（消耗配额、落下“AI 资格”），
    //   * 把这条公告当成已处理而不再重试。
    // 这里一律按传输失败返回（status 归 0），让 LlmClient 走可重试路径。
    if (readErr) {
        err = timedOut ? "recv timeout" : "recv failed";
        status = 0;
        return false;
    }
    response = bodyOut;
    return true;
}

}  // namespace rmtc
