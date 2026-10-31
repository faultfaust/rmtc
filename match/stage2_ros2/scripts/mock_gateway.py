#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stage2_tm/mock_gateway.py —— 与 rmtc::parseLlmEvent 兼容的最小 OpenAI 网关

用途：在 Linux/Windows 本地联调 stage2_tm 的 LlmClient / stage2_gateway_check，
不依赖主办方网关。它返回**顶层** EventReport JSON：
    {"kind":"explosion","x0":8,...,"activate_tick":...,"end_tick":...,"confidence":0.99}
而不是 stage2_lab 那套 {"events":[...]}，因此可直接被 stage2_tm 的
rmtc/event_prompt.cpp::parseLlmEvent 接受。

    python3 mock_gateway.py --port 8000
    python3 mock_gateway.py --port 8000 --fail quota
    python3 mock_gateway.py --port 8000 --delay-ms 3000

真实网关联调则不要启动它，直接设置：
    RMTC_AI_URL=http://<gateway>/v1/chat/completions
    RMTC_AI_TOKEN=<本场席位令牌>
"""
import argparse
import json
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ARGS = None
LOCK = threading.Lock()
STATE = {"calls": 0, "success": 0}


def _tick_from_prompt(prompt):
    m = re.search(r"当前\s*Tick\s*[：:]\s*(\d+)", prompt, re.I)
    return int(m.group(1)) if m else 0


def content_for(prompt):
    # 资格握手：不需要 JSON，返回 OK 即可（调用方只认 2xx）。
    if "请回复 OK" in prompt or "握手" in prompt:
        return "OK"

    tick = _tick_from_prompt(prompt)
    a = tick + 10
    kind = "explosion"
    x0, y0, x1, y1 = 8, 8, 10, 12
    end = a + 1
    if "分裂" in prompt or "split" in prompt.lower():
        kind, x0, y0, x1, y1, end = "split", 12, 12, 12, 12, a + 30
    elif "辣椒" in prompt or "pepper" in prompt.lower():
        kind, x0, y0, x1, y1, end = "pepper", 7, 15, 7, 15, a + 30
    elif "无双" in prompt or "fortress" in prompt.lower():
        kind, x0, y0, x1, y1, end = "fortress", 11, 11, 13, 13, a + 50
    elif "恩泽" in prompt or "回血" in prompt or "heal" in prompt.lower():
        kind, x0, y0, x1, y1, end = "heal", 6, 6, 6, 6, a + 20
    elif "幻术" in prompt or "illusion" in prompt.lower():
        kind, x0, y0, x1, y1, end = "illusion", 6, 6, 17, 17, a + 50
    elif "洁身" in prompt or "反伤" in prompt or "thorns" in prompt.lower():
        kind, x0, y0, x1, y1, end = "thorns", 0, 0, 23, 23, a + 20
    elif "易伤" in prompt or "潘潘" in prompt or "vulnerable" in prompt.lower():
        kind, x0, y0, x1, y1, end = "vulnerable", 0, 0, 23, 23, a + 30
    return json.dumps({
        "kind": kind, "x0": x0, "y0": y0, "x1": x1, "y1": y1,
        "activate_tick": a, "end_tick": end, "confidence": 0.99,
    }, ensure_ascii=False)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        sys.stderr.write("[gw] " + (fmt % args) + "\n")

    def _send(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/healthz":
            self._send(200, {"status": "ok"})
        else:
            self._send(404, {"error": {"message": "not found", "code": "not_found"}})

    def do_POST(self):
        if self.path != "/v1/chat/completions":
            self._send(404, {"error": {"message": "not found", "code": "not_found"}})
            return
        auth = self.headers.get("Authorization", "")
        if not auth.startswith("Bearer ") or len(auth) <= 7:
            self._send(401, {"error": {"message": "missing token", "code": "missing_token"}})
            return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(n)
            req = json.loads(raw.decode("utf-8"))
        except Exception:
            self._send(400, {"error": {"message": "bad json", "code": "bad_request"}})
            return

        with LOCK:
            STATE["calls"] += 1
            call_no = STATE["calls"]
        if ARGS.fail == "quota" and call_no > 1:
            self._send(429, {"error": {"message": "quota exceeded", "code": "quota_exceeded"}})
            return
        if ARGS.fail in ("504", "502", "503"):
            self._send(int(ARGS.fail), {"error": {"message": "mock upstream", "code": "upstream"}})
            return
        if ARGS.delay_ms > 0:
            time.sleep(ARGS.delay_ms / 1000.0)

        msgs = req.get("messages") or []
        prompt = "\n".join(str(m.get("content", "")) for m in msgs
                           if isinstance(m, dict) and m.get("role") == "user")
        content = content_for(prompt)
        with LOCK:
            STATE["success"] += 1
        self._send(200, {
            "id": "mock-%d" % call_no,
            "object": "chat.completion",
            "choices": [{
                "index": 0,
                "message": {"role": "assistant", "content": content},
                "finish_reason": "stop",
            }],
            "usage": {"prompt_tokens": len(prompt), "completion_tokens": 32},
        })


def main():
    global ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--fail", default="", choices=["", "quota", "504", "502", "503"])
    ap.add_argument("--delay-ms", type=int, default=0)
    ARGS = ap.parse_args()
    srv = ThreadingHTTPServer((ARGS.host, ARGS.port), Handler)
    sys.stderr.write("[gw] listening on http://%s:%d fail=%r delay=%dms\n" %
                     (ARGS.host, ARGS.port, ARGS.fail or "none", ARGS.delay_ms))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
