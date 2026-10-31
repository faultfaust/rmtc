// rmtc/json.cpp —— 极简递归下降 JSON 解析
//
// 结构：Cursor 保存"当前读到哪里"，每个 parseXxx 成功返回 true 并推进指针，
// 失败返回 false 并把原因写进 Cursor::err（只用于日志）。
// 关键取舍：任何语法错误都立刻失败，不做"尽量恢复"——感知失败时宁可本轮
// 不提交动作（裁判按 WAIT 结算），也不要基于半截数据做决策。
#include "rmtc/json.h"

#include <cstdio>
#include <cstdlib>

namespace rmtc {

const JsonValue* JsonValue::get(const char* key) const {
    if (type != J_OBJ) return NULL;
    for (std::size_t i = 0; i < obj.size(); ++i) {
        if (obj[i].first == key) return &obj[i].second;
    }
    return NULL;
}

namespace {

// 嵌套深度上限：防止恶意/损坏的深嵌套把栈打爆。
const int kMaxDepth = 64;

struct Cursor {
    const char* p;
    const char* end;
    std::string* err;
};

// 只记录第一条错误原因（后续错误通常是它的连锁反应）。
bool fail(Cursor& c, const char* msg) {
    if (c.err && c.err->empty()) *c.err = msg;
    return false;
}

void skipWs(Cursor& c) {
    while (c.p < c.end &&
           (*c.p == ' ' || *c.p == '\t' || *c.p == '\n' || *c.p == '\r'))
        ++c.p;
}

// 把一个 Unicode 码点按 UTF-8 追加到 out（\uXXXX 转义解码用）。
void appendUtf8(std::string& out, unsigned cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

// 读 \u 后面的 4 位十六进制。
bool readHex4(Cursor& c, unsigned& out) {
    if (c.end - c.p < 4) return fail(c, "bad \\u escape");
    unsigned v = 0;
    for (int i = 0; i < 4; ++i) {
        const char ch = *c.p++;
        v <<= 4;
        if (ch >= '0' && ch <= '9') v |= static_cast<unsigned>(ch - '0');
        else if (ch >= 'a' && ch <= 'f') v |= static_cast<unsigned>(ch - 'a' + 10);
        else if (ch >= 'A' && ch <= 'F') v |= static_cast<unsigned>(ch - 'A' + 10);
        else return fail(c, "bad \\u digit");
    }
    out = v;
    return true;
}

// 字符串：开头的引号已被调用方跳过前先由这里消费；支持常见转义与 \u。
// 未转义的 < 0x20 控制字符视为非法（防止把二进制垃圾当文本收下）。
bool parseString(Cursor& c, std::string& out) {
    ++c.p;  // 跳过开头引号
    out.clear();
    while (c.p < c.end) {
        const unsigned char ch = static_cast<unsigned char>(*c.p);
        if (ch == '"') {
            ++c.p;
            return true;
        }
        if (ch == '\\') {
            ++c.p;
            if (c.p >= c.end) return fail(c, "bad escape");
            const char e = *c.p++;
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned cp = 0;
                    if (!readHex4(c, cp)) return false;
                    if (cp >= 0xD800 && cp <= 0xDBFF) {
                        // 高代理：尝试配对低代理；配不上则按原码点输出后继续
                        if (c.end - c.p >= 6 && c.p[0] == '\\' && c.p[1] == 'u') {
                            c.p += 2;
                            unsigned lo = 0;
                            if (!readHex4(c, lo)) return false;
                            if (lo >= 0xDC00 && lo <= 0xDFFF) {
                                cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                            } else {
                                appendUtf8(out, cp);
                                cp = lo;
                            }
                        }
                    }
                    appendUtf8(out, cp);
                    break;
                }
                default: return fail(c, "bad escape char");
            }
            continue;
        }
        if (ch < 0x20) return fail(c, "raw control char in string");
        out += static_cast<char>(ch);
        ++c.p;
    }
    return fail(c, "unterminated string");
}

// 数字交给 strtod，但先限定首字符——strtod 会接受 "nan"/"inf"，
// 而 JSON 不允许，且这类值会污染后续的距离/评分计算。
bool parseNumber(Cursor& c, double& out) {
    if (c.p >= c.end) return fail(c, "expected number");
    const char ch = *c.p;
    if (ch != '-' && (ch < '0' || ch > '9')) return fail(c, "expected number");
    char* endp = NULL;
    const double v = std::strtod(c.p, &endp);
    if (endp == c.p) return fail(c, "bad number");
    out = v;
    c.p = endp;
    return true;
}

bool parseValue(Cursor& c, JsonValue& out, int depth);

// 对象：顺序读取 "key": value，逗号分隔，遇 '}' 结束。
// 重复键不报错，按顺序全部保留（get() 取第一个）。
bool parseObject(Cursor& c, JsonValue& out, int depth) {
    ++c.p;  // '{'
    out.type = JsonValue::J_OBJ;
    skipWs(c);
    if (c.p < c.end && *c.p == '}') {
        ++c.p;
        return true;
    }
    for (;;) {
        skipWs(c);
        if (c.p >= c.end || *c.p != '"') return fail(c, "expected key");
        std::string key;
        if (!parseString(c, key)) return false;
        skipWs(c);
        if (c.p >= c.end || *c.p != ':') return fail(c, "expected ':'");
        ++c.p;
        JsonValue val;
        if (!parseValue(c, val, depth + 1)) return false;
        out.obj.push_back(std::make_pair(key, val));
        skipWs(c);
        if (c.p < c.end && *c.p == ',') {
            ++c.p;
            continue;
        }
        if (c.p < c.end && *c.p == '}') {
            ++c.p;
            return true;
        }
        return fail(c, "expected ',' or '}'");
    }
}

// 数组：逗号分隔，遇 ']' 结束。
bool parseArray(Cursor& c, JsonValue& out, int depth) {
    ++c.p;  // '['
    out.type = JsonValue::J_ARR;
    skipWs(c);
    if (c.p < c.end && *c.p == ']') {
        ++c.p;
        return true;
    }
    for (;;) {
        JsonValue val;
        if (!parseValue(c, val, depth + 1)) return false;
        out.arr.push_back(val);
        skipWs(c);
        if (c.p < c.end && *c.p == ',') {
            ++c.p;
            continue;
        }
        if (c.p < c.end && *c.p == ']') {
            ++c.p;
            return true;
        }
        return fail(c, "expected ',' or ']'");
    }
}

// 分派：按首字符决定类型。true/false/null 必须整词匹配。
bool parseValue(Cursor& c, JsonValue& out, int depth) {
    if (depth > kMaxDepth) return fail(c, "too deep");
    skipWs(c);
    if (c.p >= c.end) return fail(c, "unexpected end");
    const char ch = *c.p;
    switch (ch) {
        case '{': return parseObject(c, out, depth);
        case '[': return parseArray(c, out, depth);
        case '"':
            out.type = JsonValue::J_STR;
            return parseString(c, out.str);
        case 't':
            if (c.end - c.p >= 4 && c.p[1] == 'r' && c.p[2] == 'u' && c.p[3] == 'e') {
                c.p += 4;
                out.type = JsonValue::J_BOOL;
                out.b = true;
                return true;
            }
            return fail(c, "bad literal");
        case 'f':
            if (c.end - c.p >= 5 && c.p[1] == 'a' && c.p[2] == 'l' && c.p[3] == 's' &&
                c.p[4] == 'e') {
                c.p += 5;
                out.type = JsonValue::J_BOOL;
                out.b = false;
                return true;
            }
            return fail(c, "bad literal");
        case 'n':
            if (c.end - c.p >= 4 && c.p[1] == 'u' && c.p[2] == 'l' && c.p[3] == 'l') {
                c.p += 4;
                out.type = JsonValue::J_NULL;
                return true;
            }
            return fail(c, "bad literal");
        default:
            out.type = JsonValue::J_NUM;
            return parseNumber(c, out.num);
    }
}

}  // namespace

bool jsonParse(const std::string& data, JsonValue& out, std::string& err) {
    err.clear();
    Cursor c;
    c.p = data.c_str();
    c.end = c.p + data.size();
    c.err = &err;
    out = JsonValue();
    if (!parseValue(c, out, 0)) {
        if (err.empty()) err = "parse error";
        return false;
    }
    skipWs(c);
    // 文档末尾还有内容 → 视为损坏（消息里混入第二条 JSON 或截断的残片）。
    if (c.p != c.end) {
        err = "trailing garbage";
        return false;
    }
    return true;
}

std::string jsonEscape(const std::string& s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (std::size_t i = 0; i < s.size(); ++i) {
        const unsigned char ch = static_cast<unsigned char>(s[i]);
        switch (ch) {
            case '"': o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\r': o += "\\r"; break;
            case '\t': o += "\\t"; break;
            default:
                if (ch < 0x20) {
                    // 其余控制字符用 \u00XX
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(ch));
                    o += buf;
                } else {
                    o += static_cast<char>(ch);
                }
        }
    }
    return o;
}

}  // namespace rmtc
