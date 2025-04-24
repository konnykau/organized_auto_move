#include "undercarriage/fry_lib/vector.hpp"
#include "undercarriage/fry_lib/math.hpp"
#include "undercarriage/constants.hpp"
#include "rclcpp/rclcpp.hpp"



class Motor
{
private:
    const FRY::Vec2d DIRECTION;
    float last_speed;
    rclcpp::Time last_time;
    float dt;
    const float MAX_ACCELERATION;
    const float MAX_SPEED;
    const uint8_t motor_id;

public:
    Motor(float x,float y,uint8_t id,rclcpp::Time now_time)//規定値最大加速度、速度を使用する場合のコンストラクタ
    : DIRECTION(FRY::Vec2d::make(x,y)), last_speed(0),last_time(now_time)
    ,MAX_ACCELERATION(Constants::Undercarriage::PRESCRIBED_MAX_ACCELERATION)
    ,MAX_SPEED(Constants::Undercarriage::PRESCRIBED_MAX_SPEED)
    , motor_id(id)
    {
        dt = 0;
        if(id < 9 ){
        }
        else{
            RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Invalid motor ID: %d", id);
            throw std::invalid_argument("Motor ID must be between 0 and 8.");
        }
    }
    Motor(float x,float y,uint8_t id,rclcpp::Time now_time,float max_acceleration, float max_speed)//最大加速度、速度を指定する場合のコンストラクタ
    : DIRECTION(FRY::Vec2d::make(x,y)), last_speed(0),last_time(now_time)
    ,MAX_ACCELERATION(max_acceleration)
    ,MAX_SPEED(max_speed), motor_id(id)
    {
        dt = 0;
        if(id < 9 ){
        }
        else{
            RCLCPP_ERROR(rclcpp::get_logger("Motor"), "Invalid motor ID: %d", id);
            throw std::invalid_argument("Motor ID must be between 0 and 8.");
        }
    }
    uint8_t get_motor_id(){
        return motor_id;
    }
    FRY::Vec2d get_vec2d(){
        return DIRECTION;
    }

    float update(float target_speed, const rclcpp::Time current_time)
    {
        dt = (current_time - last_time).seconds();
        last_time = current_time;
        if (dt <= 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("Motor"), "Invalid time difference: %f", dt);
            return last_speed;
        }
        if (target_speed > MAX_SPEED)
        {
            target_speed = MAX_SPEED;
        }
        else if (target_speed < -MAX_SPEED)
        {
            target_speed = -MAX_SPEED;
        }
        float acceleration = (target_speed - last_speed) / dt;
        if (acceleration > MAX_ACCELERATION)
        {
            acceleration = MAX_ACCELERATION;
        }
        else if (acceleration < -MAX_ACCELERATION)
        {
            acceleration = -MAX_ACCELERATION;
        }
        last_speed += acceleration * dt;

        return last_speed;

    }

};

