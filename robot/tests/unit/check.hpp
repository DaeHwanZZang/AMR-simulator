// 단위 테스트용 최소 도구 (프레임워크 없음). TEST(이름) { CHECK(...); } 로 쓰고 main.cpp 가 전부 돌린다.
#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace ut {

struct Case {
    const char* name;
    void (*fn)();
};
std::vector<Case>& cases();
struct Registrar {
    Registrar(const char* name, void (*fn)()) { cases().push_back({name, fn}); }
};
void count_check();
void fail(const char* file, int line, const std::string& what);

}  // namespace ut

#define TEST(name)                                   \
    static void name();                              \
    static ut::Registrar registrar_##name(#name, name); \
    static void name()

#define CHECK(cond)                                         \
    do {                                                    \
        ut::count_check();                                  \
        if (!(cond)) ut::fail(__FILE__, __LINE__, #cond);   \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        ut::count_check();                                                                             \
        const double ut_a = (a), ut_b = (b);                                                           \
        if (!(std::abs(ut_a - ut_b) <= (tol)))                                                         \
            ut::fail(__FILE__, __LINE__,                                                               \
                     std::string(#a " ~= " #b) + "  (" + std::to_string(ut_a) + " vs " + std::to_string(ut_b) + ")"); \
    } while (0)

#define CHECK_THROWS(expr, Type)                                               \
    do {                                                                       \
        ut::count_check();                                                     \
        bool ut_thrown = false;                                                \
        try { (void)(expr); } catch (const Type&) { ut_thrown = true; }        \
        if (!ut_thrown) ut::fail(__FILE__, __LINE__, #expr " should throw " #Type); \
    } while (0)
