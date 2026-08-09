//
// Created by biao on 24-9-18.
//

#include "unitree_guide_controller/gait/FeetEndCalc.h"

#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>

FeetEndCalc::FeetEndCalc(CtrlComponent &ctrl_component)
    : ctrl_component_(ctrl_component),
      robot_model_(ctrl_component.robot_model_),
      estimator_(ctrl_component.estimator_) {
    k_x_ = 0.005;
    k_y_ = 0.005;
    k_yaw_ = 0.005;
}

void FeetEndCalc::init() {
    t_stance_ = ctrl_component_.wave_generator_->get_t_stance();
    t_swing_ = ctrl_component_.wave_generator_->get_t_swing();

    Vec34 feet_pos_body = estimator_->getFeetPos2Body();
    // Vec34 feet_pos_body = robot_model_.feet_pos_normal_stand_;

    // getFeetPos2Body() 는 이름과 달리 몸통 좌표계가 아니라 **월드 정렬** 벡터를 준다:
    //     getFootPos(i) - body_pos  ==  rotation_ * p_body_i
    // 그래서 여기서 나온 각도에는 이 함수가 불린 순간의 몸통 yaw 가 이미 섞여 있다.
    // 그런데 calcFootPos() 가 cos(yaw + feet_init_angle_ + next_yaw) 로 yaw 를
    // **또** 더한다. yaw 가 두 번 들어간다.
    //
    // init() 은 트로트 진입 때(GaitGenerator::restart) 한 번 불리므로, 그 순간의
    // yaw 를 yaw0 라 하면 발 놓는 자리가 몸통 기준으로 항상 yaw0 만큼 돌아간
    // 곳이 된다. yaw0 = 0 (스폰 방향 그대로) 이면 아무 일도 안 일어나서 오래
    // 눈에 안 띈다. 로봇을 돌려놓고 걷기 시작해야 드러나고, 90도면 앞다리가
    // 뒷다리 자리로 뻗는다. upstream unitree_guide 부터 있던 버그다.
    //
    // yaw0 를 빼서 몸통 기준 각도로 되돌린다. calcFootPos() 의 yaw 덧셈이
    // 그제서야 정확히 한 번이 된다.
    const double yaw0 = estimator_->getYaw();
    for (int i(0); i < 4; ++i) {
        feet_radius_(i) =
                sqrt(pow(feet_pos_body(0, i), 2) + pow(feet_pos_body(1, i), 2));
        feet_init_angle_(i) = atan2(feet_pos_body(1, i), feet_pos_body(0, i)) - yaw0;
    }
}

Vec3 FeetEndCalc::calcFootPos(const int index, Vec2 vxy_goal_global, const double d_yaw_global, const double phase) {
    Vec3 body_vel_global = estimator_->getVelocity();
    Vec3 next_step;

    next_step(0) = body_vel_global(0) * (1 - phase) * t_swing_ +
                   body_vel_global(0) * t_stance_ / 2 +
                   k_x_ * (body_vel_global(0) - vxy_goal_global(0));
    next_step(1) = body_vel_global(1) * (1 - phase) * t_swing_ +
                   body_vel_global(1) * t_stance_ / 2 +
                   k_y_ * (body_vel_global(1) - vxy_goal_global(1));
    next_step(2) = 0;

    const double yaw = estimator_->getYaw();
    const double d_yaw = estimator_->getDYaw();
    const double next_yaw = d_yaw * (1 - phase) * t_swing_ + d_yaw * t_stance_ / 2 +
                            k_yaw_ * (d_yaw_global - d_yaw);

    next_step(0) +=
            feet_radius_(index) * cos(yaw + feet_init_angle_(index) + next_yaw);
    next_step(1) +=
            feet_radius_(index) * sin(yaw + feet_init_angle_(index) + next_yaw);

    Vec3 foot_pos = estimator_->getPosition() + next_step;
    foot_pos(2) = 0.0;

    return foot_pos;
}
