#include "phys2d/Serialization.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace phys2d {

const JsonValue* JsonValue::find(const std::string& key) const {
    for (const auto& kv : obj)
        if (kv.first == key) return &kv.second;
    return nullptr;
}

double JsonValue::numberOr(const std::string& key, double def) const {
    const JsonValue* v = find(key);
    return (v && v->type == Type::Number) ? v->num : def;
}
bool JsonValue::boolOr(const std::string& key, bool def) const {
    const JsonValue* v = find(key);
    if (!v) return def;
    if (v->type == Type::Bool)   return v->b;
    if (v->type == Type::Number) return v->num != 0.0;
    return def;
}
std::string JsonValue::stringOr(const std::string& key, const std::string& def) const {
    const JsonValue* v = find(key);
    return (v && v->type == Type::String) ? v->str : def;
}

static void escapeTo(std::ostringstream& out, const std::string& s) {
    out << '"';
    for (char c : s) {
        switch (c) {
            case '"':  out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\n': out << "\\n";  break;
            case '\r': out << "\\r";  break;
            case '\t': out << "\\t";  break;
            default:   out << c;      break;
        }
    }
    out << '"';
}

static void dumpTo(const JsonValue& v, std::ostringstream& out, bool pretty, int indent) {
    const std::string pad  = pretty ? std::string((size_t)indent * 2, ' ') : std::string();
    const std::string pad2 = pretty ? std::string((size_t)(indent + 1) * 2, ' ') : std::string();
    const char* nl = pretty ? "\n" : "";

    switch (v.type) {
        case JsonValue::Type::Null:   out << "null"; break;
        case JsonValue::Type::Bool:   out << (v.b ? "true" : "false"); break;
        case JsonValue::Type::Number: {
            if (!std::isfinite(v.num)) { out << "0"; break; }
            char buf[40];
            std::snprintf(buf, sizeof(buf), "%.17g", v.num);
            out << buf;
            break;
        }
        case JsonValue::Type::String: escapeTo(out, v.str); break;
        case JsonValue::Type::Array: {
            if (v.arr.empty()) { out << "[]"; break; }
            out << '[' << nl;
            for (size_t i = 0; i < v.arr.size(); ++i) {
                out << pad2;
                dumpTo(v.arr[i], out, pretty, indent + 1);
                if (i + 1 < v.arr.size()) out << ',';
                out << nl;
            }
            out << pad << ']';
            break;
        }
        case JsonValue::Type::Object: {
            if (v.obj.empty()) { out << "{}"; break; }
            out << '{' << nl;
            for (size_t i = 0; i < v.obj.size(); ++i) {
                out << pad2;
                escapeTo(out, v.obj[i].first);
                out << (pretty ? ": " : ":");
                dumpTo(v.obj[i].second, out, pretty, indent + 1);
                if (i + 1 < v.obj.size()) out << ',';
                out << nl;
            }
            out << pad << '}';
            break;
        }
    }
}

std::string JsonValue::dump(bool pretty, int indent) const {
    std::ostringstream out;
    dumpTo(*this, out, pretty, indent);
    return out.str();
}

// ---------------------------------------------------------------- парсер
namespace {

struct Parser {
    const std::string& s;
    size_t i = 0;
    std::string err;

    explicit Parser(const std::string& text) : s(text) {}

    void skip() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i;
    }
    bool fail(const char* msg) { if (err.empty()) err = msg; return false; }

    bool parseValue(JsonValue& out) {
        skip();
        if (i >= s.size()) return fail("unexpected end");
        const char c = s[i];
        if (c == '{') return parseObject(out);
        if (c == '[') return parseArray(out);
        if (c == '"') { out.type = JsonValue::Type::String; return parseString(out.str); }
        if (s.compare(i, 4, "true") == 0)  { out = JsonValue::boolean(true);  i += 4; return true; }
        if (s.compare(i, 5, "false") == 0) { out = JsonValue::boolean(false); i += 5; return true; }
        if (s.compare(i, 4, "null") == 0)  { out = JsonValue(); i += 4; return true; }
        return parseNumber(out);
    }

    bool parseNumber(JsonValue& out) {
        const size_t start = i;
        if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
        while (i < s.size() && (std::isdigit((unsigned char)s[i]) || s[i] == '.' ||
                                s[i] == 'e' || s[i] == 'E' || s[i] == '-' || s[i] == '+')) ++i;
        if (i == start) return fail("bad number");
        out = JsonValue::number(std::strtod(s.substr(start, i - start).c_str(), nullptr));
        return true;
    }

    bool parseString(std::string& out) {
        if (s[i] != '"') return fail("expected string");
        ++i;
        out.clear();
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\' && i + 1 < s.size()) {
                ++i;
                switch (s[i]) {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'r': out += '\r'; break;
                    default:  out += s[i]; break;
                }
            } else {
                out += s[i];
            }
            ++i;
        }
        if (i >= s.size()) return fail("unterminated string");
        ++i;
        return true;
    }

    bool parseArray(JsonValue& out) {
        out = JsonValue::makeArray();
        ++i;  // '['
        skip();
        if (i < s.size() && s[i] == ']') { ++i; return true; }
        for (;;) {
            JsonValue item;
            if (!parseValue(item)) return false;
            out.arr.push_back(std::move(item));
            skip();
            if (i < s.size() && s[i] == ',') { ++i; continue; }
            if (i < s.size() && s[i] == ']') { ++i; return true; }
            return fail("expected , or ]");
        }
    }

    bool parseObject(JsonValue& out) {
        out = JsonValue::makeObject();
        ++i;  // '{'
        skip();
        if (i < s.size() && s[i] == '}') { ++i; return true; }
        for (;;) {
            skip();
            std::string key;
            if (!parseString(key)) return false;
            skip();
            if (i >= s.size() || s[i] != ':') return fail("expected :");
            ++i;
            JsonValue val;
            if (!parseValue(val)) return false;
            out.obj.emplace_back(std::move(key), std::move(val));
            skip();
            if (i < s.size() && s[i] == ',') { ++i; continue; }
            if (i < s.size() && s[i] == '}') { ++i; return true; }
            return fail("expected , or }");
        }
    }
};

} // namespace

bool JsonValue::parse(const std::string& text, JsonValue& out, std::string* error) {
    Parser p(text);
    const bool ok = p.parseValue(out);
    if (!ok && error) *error = p.err;
    return ok;
}

} // namespace phys2d
