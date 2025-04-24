#pragma once
#include "undercarriage/fry_lib/math.hpp"

namespace Constants::Undercarriage
{
    //ロボットの限界加速度、速度
    constexpr float PRESCRIBED_MAX_ACCELERATION = 1000.0f;
    constexpr float PRESCRIBED_MAX_SPEED = 1500.0f;

    //実際にロボットが動くときの最大速度
    constexpr float ROBOT_VELOCITY = 900.0f;
    constexpr float ROBOT_ROTATION_VELOCITY = 200.0f;

    //setting frameの定数
    //motor id
    constexpr bool c620 = false;
    //mode
    constexpr uint8_t temp = 50;
    constexpr float velkp = 0.15;
    constexpr float velki = 9;
    constexpr float poskp = 0.5;
    constexpr float tyoku_vel_target = 0;
    constexpr float tyoku_pos_target = 0;
    constexpr float stable_pos_limit_vel = 0;
} 
namespace Constants::Math
{
    constexpr float cos30 = FRY::sqrt(3) / 2; 
}
