//
// Created by biao on 24-9-18.
//


#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H
#include <controller_common/common/enumClass.h>
#include <unitree_guide_controller/common/mathTypes.h>

class WaveGenerator {
public:
    WaveGenerator(double period, double st_ratio, const Vec4 &bias);

    ~WaveGenerator() = default;

    /**
     * 보행 위상을 진행시킨다.
     *
     * @param now_s 현재 **sim 시간**(초). 컨트롤러 update() 가 받는 time 을
     *              그대로 넘긴다. 하드웨어 플러그인이 _info.simTime 으로
     *              컨트롤러 매니저를 돌리고 use_sim_time 을 강제하므로
     *              (gz_quadruped_plugin.cpp) 그 값은 시뮬 시간이다.
     *
     * 원래는 인자 없이 std::chrono::system_clock 을 읽었다. 그러면 보행 주기가
     * **벽시계**로 흘러서, RTF 가 1 아래로 떨어지면 걸음만 제 속도로 가고
     * 물리는 뒤처진다 — 평지 직진에서도 휘청이고 경사에서는 넘어진다.
     * 실제로 자세를 0.5초마다 읽는 스레드 하나를 띄웠을 뿐인데 보행이
     * 무너졌다. 로봇 3대에 센서까지 올리면(우리 pipeline 은 RTF 23%)
     * 그 상태로는 이 제어기를 쓸 수 없다.
     */
    void update(double now_s);

    [[nodiscard]] double get_t_stance() const { return period_ * st_ratio_; }
    [[nodiscard]] double get_t_swing() const { return period_ * (1 - st_ratio_); }
    [[nodiscard]] double get_t() const { return period_; }

    Vec4 phase_;
    VecInt4 contact_;
    WaveStatus status_{};

private:
    /**
     * Update phase, contact and status based on current time.
     * @param phase foot phase
     * @param contact foot contact
     * @param status Wave Status
     */
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

    double period_{};
    double st_ratio_{}; // stance phase ratio
    Vec4 bias_;

    Vec4 normal_t_; // normalize time [0,1)
    Vec4 phase_past_; // foot phase
    VecInt4 contact_past_; // foot contact
    VecInt4 switch_status_;
    WaveStatus status_past_;

    // 첫 update() 에서 정한다. 생성자에서 잡으면 안 된다 — 컨트롤러가
    // 활성화되기 전이라 sim 시계가 아직 0 이거나 안 돌고 있을 수 있다.
    double start_t_{-1.0};
    double now_t_{};        // 이번 update 의 sim 시간(초)
};


#endif //WAVEGENERATOR_H
