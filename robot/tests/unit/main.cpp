// 단위 테스트 실행기. 리포 루트에서 `make -C robot test` (또는 build/bin/unit_tests [이름 일부...]).
// 맵은 maps/ 를 상대 경로로 읽으므로 리포 루트에서 실행해야 한다. 실패가 있으면 종료 코드 1.
#include <cstdio>
#include <cstring>
#include <exception>

#include "check.hpp"
#include "log.hpp"

namespace ut {

namespace {
int g_checks = 0, g_failed = 0;
}

std::vector<Case>& cases() {
    static std::vector<Case> all;
    return all;
}
void count_check() { ++g_checks; }
void fail(const char* file, int line, const std::string& what) {
    ++g_failed;
    const char* base = std::strrchr(file, '/');
    std::printf("    FAIL %s:%d  %s\n", base ? base + 1 : file, line, what.c_str());
}

}  // namespace ut

int main(int argc, char** argv) {
    fms::log::set_level(fms::log::Level::ERROR);  // 모듈 로그(info)는 시험 출력에 섞이지 않게
    int ran = 0, bad_cases = 0;
    for (auto& c : ut::cases()) {
        bool wanted = argc < 2;
        for (int i = 1; i < argc; ++i) wanted = wanted || std::strstr(c.name, argv[i]) != nullptr;
        if (!wanted) continue;
        const int before = ut::g_failed;
        try {
            c.fn();
        } catch (const std::exception& e) {
            ut::fail(c.name, 0, std::string("unexpected exception: ") + e.what());
        }
        ++ran;
        const bool ok = ut::g_failed == before;
        if (!ok) ++bad_cases;
        std::printf("%s %s\n", ok ? "PASS" : "FAIL", c.name);
    }
    std::printf("\n%d/%d cases PASS (%d checks, %d failed)\n", ran - bad_cases, ran, ut::g_checks, ut::g_failed);
    return bad_cases == 0 && ran > 0 ? 0 : 1;
}
