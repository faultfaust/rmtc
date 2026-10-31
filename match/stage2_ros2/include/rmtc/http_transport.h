#pragma once
#include <string>
#include "rmtc/llm.h"

// 零依赖 HTTP 传输（网关清单 §1）。
//
// 为什么不用 libcurl：网关是纯 HTTP（协议示例 http://<IP>:8000，无 TLS），
// 一个最小 HTTP/1.1 POST 用 socket 就能实现，保持本仓库"零第三方依赖"
// 的离线可测传统——本机即可起一个本地服务器做端到端验证。
// 若现场需要 HTTPS/代理，再在 CMake 里切到 libcurl 实现（接口不变）。
//
// 行为约定（与 LlmClient 的分工）：
//   * 本类只做传输：成功建立 HTTP 会话返回 true（status 可能是 4xx/5xx），
//     连接失败/超时返回 false（status=0）；业务分支（429/502/...）由上层处理；
//   * 必须带 Content-Length（网关对 Transfer-Encoding 返回 501）；
//   * timeout_ms 同时约束连接与读写（SO_SNDTIMEO/SO_RCVTIMEO）。
namespace rmtc {

class HttpTransport : public LlmTransport {
public:
    HttpTransport() = default;
    ~HttpTransport() override = default;

    bool post(const std::string& url, const std::string& body,
              const std::string& token, int timeout_ms, int& status,
              std::string& response, std::string& err) override;
};

}  // namespace rmtc
