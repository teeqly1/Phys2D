#include "phys2d/Nova.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace phys2d {
namespace nova {

double LuaValue::toNumber() const
{
    if(type == Number) return num;
    if(type == Bool) return b ? 1.0 : 0.0;
    if(type == Str) return std::atof(str.c_str());
    return 0.0;
}
std::string LuaValue::toString() const
{
    char buf[64];
    switch(type){
        case Nil: return "nil";
        case Bool: return b ? "true" : "false";
        case Str: return str;
        default: break;
    }
    if(num == (double)(long long)num){
        std::snprintf(buf, sizeof(buf), "%lld", (long long)num);
        return std::string(buf);
    }
    std::snprintf(buf, sizeof(buf), "%.14g", num);
    return std::string(buf);
}

struct LuaError { std::string msg; };

// ------------------------------------------------------------------ lexer
enum TokType { T_NAME, T_NUM, T_STR, T_OP, T_EOF };
struct Token {
    TokType type;
    std::string text;
    double num;
    int line;
    Token() : type(T_EOF), num(0), line(0) {}
};
static bool isNameStart(char c){ return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; }
static bool isNameChar(char c){ return isNameStart(c) || (c >= '0' && c <= '9') || c == '.'; }

static std::vector<Token> lex(const std::string& s)
{
    std::vector<Token> out;
    size_t i = 0;
    int line = 1;
    while(i < s.size()){
        char c = s[i];
        if(c == '\n'){ ++line; ++i; continue; }
        if(c == ' ' || c == '\t' || c == '\r'){ ++i; continue; }
        if(c == '-' && i + 1 < s.size() && s[i+1] == '-'){
            while(i < s.size() && s[i] != '\n') ++i;
            continue;
        }
        Token t;
        t.line = line;
        if(isNameStart(c)){
            size_t j = i;
            while(j < s.size() && isNameChar(s[j])) ++j;
            t.type = T_NAME;
            t.text = s.substr(i, j - i);
            i = j;
        } else if((c >= '0' && c <= '9') || (c == '.' && i + 1 < s.size() && s[i+1] >= '0' && s[i+1] <= '9')){
            size_t j = i;
            while(j < s.size() && ((s[j] >= '0' && s[j] <= '9') || s[j] == '.')) ++j;
            if(j < s.size() && (s[j] == 'e' || s[j] == 'E')){
                ++j;
                if(j < s.size() && (s[j] == '+' || s[j] == '-')) ++j;
                while(j < s.size() && s[j] >= '0' && s[j] <= '9') ++j;
            }
            t.type = T_NUM;
            t.text = s.substr(i, j - i);
            t.num = std::atof(t.text.c_str());
            i = j;
        } else if(c == '"' || c == '\''){
            char quote = c;
            ++i;
            std::string v;
            while(i < s.size() && s[i] != quote){
                if(s[i] == '\\' && i + 1 < s.size()){
                    char n = s[i+1];
                    if(n == 'n') v += '\n';
                    else if(n == 't') v += '\t';
                    else v += n;
                    i += 2;
                    continue;
                }
                v += s[i++];
            }
            ++i;
            t.type = T_STR;
            t.text = v;
        } else {
            std::string two = s.substr(i, 2);
            if(two == "==" || two == "~=" || two == "<=" || two == ">=" || two == ".." || two == "::"){
                t.type = T_OP; t.text = two; i += 2;
            } else {
                t.type = T_OP; t.text = std::string(1, c); ++i;
            }
        }
        out.push_back(t);
    }
    Token e;
    e.type = T_EOF;
    e.line = line;
    out.push_back(e);
    return out;
}

static bool isKeyword(const std::string& s)
{
    static const char* kw[] = {"local","if","then","elseif","else","end","while","do","for","function",
                               "return","break","nil","true","false","and","or","not","repeat","until"};
    for(size_t i = 0; i < sizeof(kw) / sizeof(kw[0]); ++i) if(s == kw[i]) return true;
    return false;
}

// ------------------------------------------------------------------- ast
struct Node {
    enum K { Num, Str, True, False, Nil, Name, Bin, Un, Call, Block, Local, Assign, If, While, NumFor, Return, Break, ExprStat, FuncDef };
    K k;
    double num;
    std::string str;
    std::vector<Node*> kids;
    std::vector<std::string> params;
    int line;
    Node(K kk) : k(kk), num(0), line(0) {}
};

struct Parser {
    std::vector<Token> toks;
    size_t p;
    std::vector<Node*>* pool;

    Node* mk(Node::K k){ Node* n = new Node(k); n->line = toks[p].line; pool->push_back(n); return n; }
    const Token& cur() const { return toks[p]; }
    bool isOp(const char* o) const { return toks[p].type == T_OP && toks[p].text == o; }
    bool isName(const char* o) const { return toks[p].type == T_NAME && toks[p].text == o; }
    void expectOp(const char* o){
        if(!isOp(o)) fail(std::string("expected '") + o + "'");
        ++p;
    }
    void expectName(const char* o){
        if(!isName(o)) fail(std::string("expected '") + o + "'");
        ++p;
    }
    void fail(const std::string& msg){
        char buf[128];
        std::snprintf(buf, sizeof(buf), "line %d: ", toks[p].line);
        throw LuaError{std::string(buf) + msg};
    }

    Node* parseChunk(){
        Node* b = parseBlock();
        if(toks[p].type != T_EOF) fail("unexpected token '" + toks[p].text + "'");
        return b;
    }
    bool blockEnd() const {
        if(toks[p].type == T_EOF) return true;
        if(toks[p].type != T_NAME) return false;
        const std::string& t = toks[p].text;
        return t == "end" || t == "else" || t == "elseif" || t == "until";
    }
    Node* parseBlock(){
        Node* b = mk(Node::Block);
        while(!blockEnd()) b->kids.push_back(parseStatement());
        return b;
    }
    Node* parseStatement(){
        if(toks[p].type == T_NAME){
            const std::string t = toks[p].text;
            if(t == "local"){
                ++p;
                if(toks[p].type != T_NAME || isKeyword(toks[p].text)) fail("expected name after local");
                Node* n = mk(Node::Local);
                n->str = toks[p].text;
                ++p;
                if(isOp("=")){ ++p; n->kids.push_back(parseOr()); }
                else n->kids.push_back(mk(Node::Nil));
                return n;
            }
            if(t == "if") return parseIf();
            if(t == "while"){
                ++p;
                Node* n = mk(Node::While);
                n->kids.push_back(parseOr());
                expectName("do");
                n->kids.push_back(parseBlock());
                expectName("end");
                return n;
            }
            if(t == "repeat"){
                ++p;
                Node* body = parseBlock();
                expectName("until");
                Node* cond = parseOr();
                Node* notCond = mk(Node::Un);
                notCond->str = "not";
                notCond->kids.push_back(cond);
                Node* n = mk(Node::While);
                Node* alwaysTrue = mk(Node::True);
                n->kids.push_back(alwaysTrue);
                Node* wrapper = mk(Node::Block);
                for(size_t i = 0; i < body->kids.size(); ++i) wrapper->kids.push_back(body->kids[i]);
                Node* brk = mk(Node::Break);
                Node* ifNode = mk(Node::If);
                ifNode->kids.push_back(cond);
                Node* brkBlock = mk(Node::Block);
                brkBlock->kids.push_back(brk);
                ifNode->kids.push_back(brkBlock);
                wrapper->kids.push_back(ifNode);
                n->kids.push_back(wrapper);
                (void)notCond;
                return n;
            }
            if(t == "for"){
                ++p;
                if(toks[p].type != T_NAME) fail("expected loop variable");
                Node* n = mk(Node::NumFor);
                n->str = toks[p].text;
                ++p;
                expectOp("=");
                n->kids.push_back(parseOr());
                expectOp(",");
                n->kids.push_back(parseOr());
                if(isOp(",")){ ++p; n->kids.push_back(parseOr()); }
                else { Node* one = mk(Node::Num); one->num = 1; n->kids.push_back(one); }
                expectName("do");
                n->kids.push_back(parseBlock());
                expectName("end");
                return n;
            }
            if(t == "function"){
                ++p;
                if(toks[p].type != T_NAME) fail("expected function name");
                Node* n = mk(Node::FuncDef);
                n->str = toks[p].text;
                ++p;
                expectOp("(");
                while(!isOp(")")){
                    if(toks[p].type != T_NAME) fail("bad parameter");
                    n->params.push_back(toks[p].text);
                    ++p;
                    if(isOp(",")) ++p;
                }
                expectOp(")");
                n->kids.push_back(parseBlock());
                expectName("end");
                return n;
            }
            if(t == "return"){
                ++p;
                Node* n = mk(Node::Return);
                if(!blockEnd() && !isOp(";")) n->kids.push_back(parseOr());
                return n;
            }
            if(t == "break"){ ++p; return mk(Node::Break); }
            if(t == "do"){
                ++p;
                Node* b = parseBlock();
                expectName("end");
                return b;
            }
            // assignment or call
            if(toks[p + 1].type == T_OP && toks[p + 1].text == "="){
                Node* n = mk(Node::Assign);
                n->str = toks[p].text;
                p += 2;
                n->kids.push_back(parseOr());
                return n;
            }
        }
        Node* e = mk(Node::ExprStat);
        e->kids.push_back(parseOr());
        if(isOp(";")) ++p;
        return e;
    }
    Node* parseIf(){
        expectName("if");
        Node* n = mk(Node::If);
        n->kids.push_back(parseOr());
        expectName("then");
        n->kids.push_back(parseBlock());
        while(isName("elseif")){
            ++p;
            n->kids.push_back(parseOr());
            expectName("then");
            n->kids.push_back(parseBlock());
        }
        if(isName("else")){
            ++p;
            n->kids.push_back(parseBlock());
        }
        expectName("end");
        return n;
    }
    Node* bin(const char* op, Node* a, Node* b){
        Node* n = new Node(Node::Bin);
        n->str = op;
        n->kids.push_back(a);
        n->kids.push_back(b);
        pool->push_back(n);
        return n;
    }
    Node* parseOr(){
        Node* a = parseAnd();
        while(isName("or")){ ++p; a = bin("or", a, parseAnd()); }
        return a;
    }
    Node* parseAnd(){
        Node* a = parseCmp();
        while(isName("and")){ ++p; a = bin("and", a, parseCmp()); }
        return a;
    }
    Node* parseCmp(){
        Node* a = parseConcat();
        while(toks[p].type == T_OP && (toks[p].text == "==" || toks[p].text == "~=" || toks[p].text == "<" ||
                                       toks[p].text == ">" || toks[p].text == "<=" || toks[p].text == ">=")){
            std::string op = toks[p].text;
            ++p;
            a = bin(op.c_str(), a, parseConcat());
        }
        return a;
    }
    Node* parseConcat(){
        Node* a = parseAdd();
        while(isOp("..")){ ++p; a = bin("..", a, parseAdd()); }
        return a;
    }
    Node* parseAdd(){
        Node* a = parseMul();
        while(toks[p].type == T_OP && (toks[p].text == "+" || toks[p].text == "-")){
            std::string op = toks[p].text;
            ++p;
            a = bin(op.c_str(), a, parseMul());
        }
        return a;
    }
    Node* parseMul(){
        Node* a = parseUnary();
        while(toks[p].type == T_OP && (toks[p].text == "*" || toks[p].text == "/" || toks[p].text == "%")){
            std::string op = toks[p].text;
            ++p;
            a = bin(op.c_str(), a, parseUnary());
        }
        return a;
    }
    Node* parseUnary(){
        if(isOp("-") || isName("not")){
            std::string op = isOp("-") ? "-" : "not";
            ++p;
            Node* n = mk(Node::Un);
            n->str = op;
            n->kids.push_back(parseUnary());
            return n;
        }
        return parsePow();
    }
    Node* parsePow(){
        Node* a = parsePrimary();
        if(isOp("^")){ ++p; return bin("^", a, parseUnary()); }
        return a;
    }
    Node* parsePrimary(){
        const Token& t = toks[p];
        if(t.type == T_NUM){ Node* n = mk(Node::Num); n->num = t.num; ++p; return n; }
        if(t.type == T_STR){ Node* n = mk(Node::Str); n->str = t.text; ++p; return n; }
        if(t.type == T_OP && t.text == "("){
            ++p;
            Node* e = parseOr();
            expectOp(")");
            return e;
        }
        if(t.type == T_NAME){
            if(t.text == "true"){ ++p; return mk(Node::True); }
            if(t.text == "false"){ ++p; return mk(Node::False); }
            if(t.text == "nil"){ ++p; return mk(Node::Nil); }
            if(isKeyword(t.text)) fail("unexpected '" + t.text + "'");
            std::string name = t.text;
            ++p;
            if(isOp("(")){
                ++p;
                Node* n = mk(Node::Call);
                n->str = name;
                while(!isOp(")")){
                    n->kids.push_back(parseOr());
                    if(isOp(",")) ++p;
                    else break;
                }
                expectOp(")");
                return n;
            }
            Node* n = mk(Node::Name);
            n->str = name;
            return n;
        }
        fail("unexpected expression");
        return nullptr;
    }
};

// ---------------------------------------------------------------- runtime
struct FuncDecl {
    std::vector<std::string> params;
    Node* body;
    FuncDecl() : body(nullptr) {}
};

struct LuaVM::Impl {
    World* world;
    std::map<std::string, LuaNative> natives;
    std::map<std::string, LuaValue> globals;
    std::map<std::string, FuncDecl> funcs;
    std::vector<std::map<std::string, LuaValue> > scopes;
    std::vector<Node*> pool;
    std::string out;
    uint32_t rng;
    int depth;
    long long loopGuard;

    enum Flow { Normal, Ret, Brk };

    Impl() : world(nullptr), rng(22695477u), depth(0), loopGuard(0) {}
    ~Impl(){ for(size_t i = 0; i < pool.size(); ++i) delete pool[i]; }

    LuaValue* findVar(const std::string& n){
        for(size_t i = scopes.size(); i > 0; --i){
            std::map<std::string, LuaValue>::iterator it = scopes[i - 1].find(n);
            if(it != scopes[i - 1].end()) return &it->second;
        }
        std::map<std::string, LuaValue>::iterator g = globals.find(n);
        if(g != globals.end()) return &g->second;
        return nullptr;
    }
    void setVar(const std::string& n, const LuaValue& v){
        for(size_t i = scopes.size(); i > 0; --i){
            std::map<std::string, LuaValue>::iterator it = scopes[i - 1].find(n);
            if(it != scopes[i - 1].end()){ it->second = v; return; }
        }
        globals[n] = v;
    }
    LuaValue call(const std::string& name, const std::vector<LuaValue>& args){
        std::map<std::string, LuaNative>::iterator nit = natives.find(name);
        if(nit != natives.end()) return nit->second(args);
        std::map<std::string, FuncDecl>::iterator fit = funcs.find(name);
        if(fit == funcs.end()) throw LuaError{"unknown function '" + name + "'"};
        if(++depth > 128){ --depth; throw LuaError{"stack overflow"}; }
        std::map<std::string, LuaValue> frame;
        for(size_t i = 0; i < fit->second.params.size(); ++i)
            frame[fit->second.params[i]] = i < args.size() ? args[i] : LuaValue();
        scopes.push_back(frame);
        LuaValue ret;
        Flow f = Normal;
        execBlock(fit->second.body, f, ret);
        scopes.pop_back();
        --depth;
        return ret;
    }
    LuaValue eval(Node* n){
        switch(n->k){
            case Node::Num: return LuaValue(n->num);
            case Node::Str: return LuaValue(n->str);
            case Node::True: return LuaValue(true);
            case Node::False: return LuaValue(false);
            case Node::Nil: return LuaValue();
            case Node::Name: {
                LuaValue* v = findVar(n->str);
                return v ? *v : LuaValue();
            }
            case Node::Un: {
                LuaValue a = eval(n->kids[0]);
                if(n->str == "-") return LuaValue(-a.toNumber());
                return LuaValue(!a.truthy());
            }
            case Node::Call: {
                std::vector<LuaValue> args;
                for(size_t i = 0; i < n->kids.size(); ++i) args.push_back(eval(n->kids[i]));
                return call(n->str, args);
            }
            case Node::Bin: {
                const std::string& op = n->str;
                if(op == "and"){
                    LuaValue a = eval(n->kids[0]);
                    return a.truthy() ? eval(n->kids[1]) : a;
                }
                if(op == "or"){
                    LuaValue a = eval(n->kids[0]);
                    return a.truthy() ? a : eval(n->kids[1]);
                }
                LuaValue a = eval(n->kids[0]);
                LuaValue b = eval(n->kids[1]);
                if(op == "..") return LuaValue(a.toString() + b.toString());
                if(op == "=="){
                    if(a.type == LuaValue::Str || b.type == LuaValue::Str) return LuaValue(a.toString() == b.toString());
                    return LuaValue(a.toNumber() == b.toNumber());
                }
                if(op == "~="){
                    if(a.type == LuaValue::Str || b.type == LuaValue::Str) return LuaValue(a.toString() != b.toString());
                    return LuaValue(a.toNumber() != b.toNumber());
                }
                double x = a.toNumber(), y = b.toNumber();
                if(op == "+") return LuaValue(x + y);
                if(op == "-") return LuaValue(x - y);
                if(op == "*") return LuaValue(x * y);
                if(op == "/") return LuaValue(y == 0 ? 0.0 : x / y);
                if(op == "%") return LuaValue(y == 0 ? 0.0 : std::fmod(x, y));
                if(op == "^") return LuaValue(std::pow(x, y));
                if(op == "<") return LuaValue(x < y);
                if(op == ">") return LuaValue(x > y);
                if(op == "<=") return LuaValue(x <= y);
                if(op == ">=") return LuaValue(x >= y);
                throw LuaError{"bad operator '" + op + "'"};
            }
            default: break;
        }
        throw LuaError{"bad expression"};
    }
    void execBlock(Node* b, Flow& flow, LuaValue& ret){
        scopes.push_back(std::map<std::string, LuaValue>());
        for(size_t i = 0; i < b->kids.size(); ++i){
            exec(b->kids[i], flow, ret);
            if(flow != Normal) break;
        }
        scopes.pop_back();
    }
    void exec(Node* n, Flow& flow, LuaValue& ret){
        switch(n->k){
            case Node::Block: execBlock(n, flow, ret); return;
            case Node::Local: scopes.back()[n->str] = eval(n->kids[0]); return;
            case Node::Assign: setVar(n->str, eval(n->kids[0])); return;
            case Node::ExprStat: eval(n->kids[0]); return;
            case Node::Return:
                ret = n->kids.empty() ? LuaValue() : eval(n->kids[0]);
                flow = Ret;
                return;
            case Node::Break: flow = Brk; return;
            case Node::FuncDef: {
                FuncDecl fd;
                fd.params = n->params;
                fd.body = n->kids[0];
                funcs[n->str] = fd;
                return;
            }
            case Node::If: {
                size_t i = 0;
                for(; i + 1 < n->kids.size(); i += 2){
                    if(eval(n->kids[i]).truthy()){ execBlock(n->kids[i + 1], flow, ret); return; }
                }
                if(i < n->kids.size()) execBlock(n->kids[i], flow, ret);
                return;
            }
            case Node::While: {
                while(eval(n->kids[0]).truthy()){
                    if(++loopGuard > 5000000) throw LuaError{"infinite loop guard"};
                    Flow f = Normal;
                    execBlock(n->kids[1], f, ret);
                    if(f == Brk) break;
                    if(f == Ret){ flow = Ret; return; }
                }
                return;
            }
            case Node::NumFor: {
                double start = eval(n->kids[0]).toNumber();
                double limit = eval(n->kids[1]).toNumber();
                double stepv = eval(n->kids[2]).toNumber();
                if(stepv == 0) throw LuaError{"for step is zero"};
                for(double v = start; stepv > 0 ? v <= limit : v >= limit; v += stepv){
                    if(++loopGuard > 5000000) throw LuaError{"infinite loop guard"};
                    scopes.push_back(std::map<std::string, LuaValue>());
                    scopes.back()[n->str] = LuaValue(v);
                    Flow f = Normal;
                    execBlock(n->kids[3], f, ret);
                    scopes.pop_back();
                    if(f == Brk) break;
                    if(f == Ret){ flow = Ret; return; }
                }
                return;
            }
            default:
                eval(n);
                return;
        }
    }
};

static double argN(const std::vector<LuaValue>& a, size_t i, double dflt = 0)
{ return i < a.size() ? a[i].toNumber() : dflt; }

LuaVM::LuaVM()
{
    impl_ = new Impl();
    Impl* im = impl_;
    im->globals["math.pi"] = LuaValue(3.14159265358979323846);

    im->natives["print"] = [im](const std::vector<LuaValue>& a) -> LuaValue {
        std::string line;
        for(size_t i = 0; i < a.size(); ++i){ if(i) line += " "; line += a[i].toString(); }
        im->out += line;
        im->out += "\n";
        return LuaValue();
    };
    im->natives["tostring"] = [](const std::vector<LuaValue>& a) -> LuaValue {
        return LuaValue(a.empty() ? std::string("nil") : a[0].toString());
    };
    im->natives["tonumber"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(argN(a, 0)); };
    im->natives["math.sin"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::sin(argN(a, 0))); };
    im->natives["math.cos"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::cos(argN(a, 0))); };
    im->natives["math.tan"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::tan(argN(a, 0))); };
    im->natives["math.sqrt"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::sqrt(std::fabs(argN(a, 0)))); };
    im->natives["math.abs"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::fabs(argN(a, 0))); };
    im->natives["math.floor"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::floor(argN(a, 0))); };
    im->natives["math.ceil"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::ceil(argN(a, 0))); };
    im->natives["math.min"] = [](const std::vector<LuaValue>& a) -> LuaValue {
        double m = argN(a, 0);
        for(size_t i = 1; i < a.size(); ++i) m = std::min(m, a[i].toNumber());
        return LuaValue(m);
    };
    im->natives["math.max"] = [](const std::vector<LuaValue>& a) -> LuaValue {
        double m = argN(a, 0);
        for(size_t i = 1; i < a.size(); ++i) m = std::max(m, a[i].toNumber());
        return LuaValue(m);
    };
    im->natives["math.atan2"] = [](const std::vector<LuaValue>& a) -> LuaValue { return LuaValue(std::atan2(argN(a, 0), argN(a, 1, 1))); };
    im->natives["math.random"] = [im](const std::vector<LuaValue>& a) -> LuaValue {
        im->rng = im->rng * 1664525u + 1013904223u;
        double u = (double)(im->rng >> 8) / 16777216.0;
        if(a.empty()) return LuaValue(u);
        if(a.size() == 1) return LuaValue(std::floor(u * a[0].toNumber()) + 1.0);
        return LuaValue(std::floor(a[0].toNumber() + u * (a[1].toNumber() - a[0].toNumber() + 1.0)));
    };
    im->natives["material_count"] = [](const std::vector<LuaValue>&) -> LuaValue {
        return LuaValue((double)MaterialLibrary::instance().count());
    };
    im->natives["material_name"] = [](const std::vector<LuaValue>& a) -> LuaValue {
        return LuaValue(MaterialLibrary::instance().byIndex((int)argN(a, 0)).name);
    };
}

LuaVM::~LuaVM(){ delete impl_; }

void LuaVM::bindWorld(World& w)
{
    Impl* im = impl_;
    im->world = &w;
    World* wp = &w;
    const MaterialLibrary& lib = MaterialLibrary::instance();
    (void)lib;

    im->natives["set_gravity"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        wp->setGravity(Vec2(argN(a, 0), argN(a, 1, -9.81)));
        return LuaValue();
    };
    im->natives["body_count"] = [wp](const std::vector<LuaValue>&) -> LuaValue {
        return LuaValue((double)wp->bodyCount());
    };
    im->natives["spawn_box"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.shape = Shape::box(argN(a, 2, 1.0), argN(a, 3, 1.0));
        bd.material = MaterialLibrary::instance().physics((int)argN(a, 4, 23));
        bd.position = Vec2(argN(a, 0), argN(a, 1));
        return LuaValue((double)wp->createBody(bd));
    };
    im->natives["spawn_circle"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.shape = Shape::circle(argN(a, 2, 0.5));
        bd.material = MaterialLibrary::instance().physics((int)argN(a, 3, 28));
        bd.position = Vec2(argN(a, 0), argN(a, 1));
        return LuaValue((double)wp->createBody(bd));
    };
    im->natives["spawn_static"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        BodyDef bd;
        bd.type = BodyType::Static;
        bd.shape = Shape::box(argN(a, 2, 10.0), argN(a, 3, 1.0));
        bd.material = MaterialLibrary::instance().physics((int)argN(a, 4, 12));
        bd.position = Vec2(argN(a, 0), argN(a, 1));
        return LuaValue((double)wp->createBody(bd));
    };
    im->natives["body_x"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        const RigidBody* b = wp->body((BodyId)(uint32_t)argN(a, 0));
        return LuaValue(b ? (double)b->position.x : 0.0);
    };
    im->natives["body_y"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        const RigidBody* b = wp->body((BodyId)(uint32_t)argN(a, 0));
        return LuaValue(b ? (double)b->position.y : 0.0);
    };
    im->natives["body_speed"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        const RigidBody* b = wp->body((BodyId)(uint32_t)argN(a, 0));
        if(!b) return LuaValue(0.0);
        return LuaValue(std::sqrt((double)(b->velocity.x * b->velocity.x + b->velocity.y * b->velocity.y)));
    };
    im->natives["apply_force"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        RigidBody* b = wp->body((BodyId)(uint32_t)argN(a, 0));
        if(b){ b->wake(); b->applyForce(Vec2(argN(a, 1), argN(a, 2))); }
        return LuaValue();
    };
    im->natives["apply_impulse"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        RigidBody* b = wp->body((BodyId)(uint32_t)argN(a, 0));
        if(b){ b->wake(); b->applyImpulse(Vec2(argN(a, 1), argN(a, 2))); }
        return LuaValue();
    };
    im->natives["remove_body"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        wp->destroyBody((BodyId)(uint32_t)argN(a, 0));
        return LuaValue();
    };
    im->natives["step"] = [wp](const std::vector<LuaValue>& a) -> LuaValue {
        real dt = (real)argN(a, 0, 1.0 / 60.0);
        int n = (int)argN(a, 1, 1);
        for(int i = 0; i < n; ++i) wp->step(dt);
        return LuaValue((double)n);
    };
}

void LuaVM::registerFunction(const std::string& name, LuaNative fn){ impl_->natives[name] = fn; }

bool LuaVM::doString(const std::string& code, std::string* error)
{
    try {
        Parser ps;
        ps.toks = lex(code);
        ps.p = 0;
        ps.pool = &impl_->pool;
        Node* chunk = ps.parseChunk();
        // hoist top-level function declarations
        for(size_t i = 0; i < chunk->kids.size(); ++i){
            Node* s = chunk->kids[i];
            if(s->k != Node::FuncDef) continue;
            FuncDecl fd;
            fd.params = s->params;
            fd.body = s->kids[0];
            impl_->funcs[s->str] = fd;
        }
        impl_->loopGuard = 0;
        Impl::Flow flow = Impl::Normal;
        LuaValue ret;
        impl_->execBlock(chunk, flow, ret);
        return true;
    } catch(const LuaError& e){
        if(error) *error = std::string("script ") + e.msg;
        return false;
    } catch(const std::exception& e){
        if(error) *error = std::string("script error: ") + e.what();
        return false;
    }
}
bool LuaVM::doFile(const std::string& path, std::string* error)
{
    std::ifstream f(path.c_str());
    if(!f.good()){
        if(error) *error = "cannot open " + path;
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return doString(ss.str(), error);
}
bool LuaVM::hasFunction(const std::string& name) const
{ return impl_->funcs.find(name) != impl_->funcs.end() || impl_->natives.find(name) != impl_->natives.end(); }
LuaValue LuaVM::callFunction(const std::string& name, const std::vector<LuaValue>& args)
{
    try { return impl_->call(name, args); }
    catch(const LuaError&) { return LuaValue(); }
}
void LuaVM::setGlobal(const std::string& name, const LuaValue& v){ impl_->globals[name] = v; }
LuaValue LuaVM::getGlobal(const std::string& name) const
{
    std::map<std::string, LuaValue>::const_iterator it = impl_->globals.find(name);
    return it == impl_->globals.end() ? LuaValue() : it->second;
}
const std::string& LuaVM::output() const { return impl_->out; }
void LuaVM::clearOutput(){ impl_->out.clear(); }
int LuaVM::nativeCount() const { return (int)impl_->natives.size(); }

} // namespace nova
} // namespace phys2d
