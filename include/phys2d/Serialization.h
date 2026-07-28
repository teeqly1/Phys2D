// phys2d — минимальный JSON (без внешних зависимостей) для сериализации мира.
#pragma once

#include <string>
#include <vector>
#include <utility>

namespace phys2d {

struct JsonValue {
    enum class Type { Null, Bool, Number, String, Array, Object };

    Type        type = Type::Null;
    bool        b    = false;
    double      num  = 0.0;
    std::string str;
    std::vector<JsonValue>                         arr;
    std::vector<std::pair<std::string, JsonValue>> obj;

    JsonValue() = default;
    static JsonValue makeObject() { JsonValue v; v.type = Type::Object; return v; }
    static JsonValue makeArray()  { JsonValue v; v.type = Type::Array;  return v; }
    static JsonValue number(double d) { JsonValue v; v.type = Type::Number; v.num = d; return v; }
    static JsonValue boolean(bool x)  { JsonValue v; v.type = Type::Bool;   v.b   = x; return v; }
    static JsonValue string(const std::string& s) { JsonValue v; v.type = Type::String; v.str = s; return v; }

    void set(const std::string& key, JsonValue v) { obj.emplace_back(key, std::move(v)); type = Type::Object; }
    void set(const std::string& key, double d)    { set(key, number(d)); }
    void set(const std::string& key, int i)       { set(key, number((double)i)); }
    void set(const std::string& key, bool x)      { set(key, boolean(x)); }
    void set(const std::string& key, const std::string& s) { set(key, string(s)); }
    void push(JsonValue v) { arr.push_back(std::move(v)); type = Type::Array; }

    const JsonValue* find(const std::string& key) const;
    double      numberOr(const std::string& key, double def) const;
    bool        boolOr  (const std::string& key, bool def) const;
    std::string stringOr(const std::string& key, const std::string& def) const;

    std::string dump(bool pretty = false, int indent = 0) const;
    static bool parse(const std::string& text, JsonValue& out, std::string* error = nullptr);
};

} // namespace phys2d
