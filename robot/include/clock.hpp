// 시계 추상화 (robot_client/clock.py 포팅). 실시간 모드만 존재한다 —
// "로봇 1대 = 프로세스 1개" 라 전역 시계 공유가 불가능하기 때문 (CLAUDE.md 결정 4).
#pragma once

#include <atomic>
#include <chrono>
#include <thread>

namespace fms {

class RealtimeClock {
public:
    double now() const {
        return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
    void sleep(double seconds) const {
        if (seconds > 0) std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    }
};

// 일정 주기로 dt 를 뽑아내는 루프 헬퍼. 밀리면 기준을 다시 잡아 spiral of death 를 막는다.
class Ticker {
public:
    // period = 시뮬 시간 기준 tick 주기. scale = 시간 배속 (--time-scale): 벽시계로는 period/scale 마다 돌고
    // 돌려주는 dt 는 시뮬 시간(벽시계 경과 x scale)이다. 1 이면 실시간.
    Ticker(RealtimeClock& clock, double period, double scale = 1.0)
        : clock_(clock), period_(period / (scale > 0 ? scale : 1.0)), scale_(scale > 0 ? scale : 1.0) {}

    void stop() { running_ = false; }
    bool running() const { return running_; }

    // 다음 tick 까지 대기하고 dt(시뮬 시간)를 돌려준다. running() 이 false 가 되면 멈춰야 한다.
    double next() {
        clock_.sleep(next_at_ - clock_.now());
        double now = clock_.now();
        double dt = (now - last_) * scale_;
        last_ = now;
        next_at_ += period_;
        if (next_at_ < now) next_at_ = now + period_;
        return dt;
    }

    void reset() {
        next_at_ = clock_.now() + period_;
        last_ = clock_.now();
    }

private:
    RealtimeClock& clock_;
    double period_;   // 벽시계 기준 주기 (= 시뮬 주기 / scale)
    double scale_;
    std::atomic<bool> running_{true};
    double next_at_ = 0.0, last_ = 0.0;
};

}  // namespace fms
