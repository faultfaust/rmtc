#pragma once
#include <string>
#include <utility>
#include <vector>

// 极简 JSON（C++11，无第三方依赖）。
//
// 为什么不用 nlohmann/json：第一阶段原本依赖它，导致离线测试和交叉编译都要装库。
// 感知/动作只需要"解析一个只读 JSON 树"和"转义字符串"，自己实现约 300 行即可，
// 换来零依赖、可控的容错行为（坏输入只返回 false，不抛异常）。
//
// 支持：对象、数组、字符串（含 \uXXXX 与代理对）、数字、true/false/null。
// 不支持：注释、单引号字符串、尾随逗号、数字的十六进制/NaN 形态。
// 限制：嵌套深度 64 层；重复键按出现顺序保留，get() 取第一个匹配。
namespace rmtc {

class JsonValue {
public:
    enum Type { J_NULL, J_BOOL, J_NUM, J_STR, J_ARR, J_OBJ };

    JsonValue() : type(J_NULL), b(false), num(0.0) {}

    Type type;
    bool b;                     // J_BOOL
    double num;                 // J_NUM
    std::string str;            // J_STR
    std::vector<JsonValue> arr; // J_ARR
    std::vector<std::pair<std::string, JsonValue> > obj;  // J_OBJ

    bool isNull() const { return type == J_NULL; }
    bool isBool() const { return type == J_BOOL; }
    bool isNum() const { return type == J_NUM; }
    bool isStr() const { return type == J_STR; }
    bool isArr() const { return type == J_ARR; }
    bool isObj() const { return type == J_OBJ; }

    // 按键取成员；非对象或键不存在返回 NULL（调用方必须判空再取值）。
    const JsonValue* get(const char* key) const;
};

// 解析完整 JSON 文档。
// 成功返回 true；失败返回 false，err 里是便于日志定位的原因（不是给程序判断用的）。
// 失败时 out 的内容未定义，调用方应直接丢弃本轮数据。
bool jsonParse(const std::string& data, JsonValue& out, std::string& err);

// 把字符串转义成可安全放进 JSON 双引号内的形式
// （" \ 与所有 < 0x20 的控制字符；UTF-8 多字节原样通过）。
std::string jsonEscape(const std::string& s);

}  // namespace rmtc
